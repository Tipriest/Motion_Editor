import { Quaternion, Vector3 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';
import type { FootSide } from './FootGroundAlignmentService';
import { solveLinearSystem } from './FootGroundAlignmentService';
import type { FootSoleDefinition } from './GroundContactService';

const LEG_JOINT_NAMES: Record<FootSide, string[]> = {
  left: [
    'hip_pitch_l_joint',
    'hip_roll_l_joint',
    'hip_yaw_l_joint',
    'knee_pitch_l_joint',
    'ankle_pitch_l_joint',
    'ankle_roll_l_joint',
  ],
  right: [
    'hip_pitch_r_joint',
    'hip_roll_r_joint',
    'hip_yaw_r_joint',
    'knee_pitch_r_joint',
    'ankle_pitch_r_joint',
    'ankle_roll_r_joint',
  ],
};

export interface FootLockTarget {
  position: { x: number; y: number; z: number };
  rotation: { x: number; y: number; z: number; w: number };
}

export interface FootLockResult {
  success: boolean;
  jointValues: Record<string, number>;
  iterations: number;
  positionError: number;
  orientationError: number;
  reason?: string;
}

export function buildFootLockHeightCandidates(
  previousOffset: number,
  maximumOffset = 0.15,
  step = 0.01,
): number[] {
  const safeMaximum = Math.max(0, Math.abs(maximumOffset));
  const safeStep = Math.max(0.001, Math.abs(step));
  const candidates = [0];
  for (let magnitude = safeStep; magnitude <= safeMaximum + 1e-9; magnitude += safeStep) {
    candidates.push(-magnitude, magnitude);
  }
  return candidates.sort((left, right) => {
    const leftScore =
      Math.abs(left - previousOffset) * 0.65 + Math.abs(left) * 0.35;
    const rightScore =
      Math.abs(right - previousOffset) * 0.65 + Math.abs(right) * 0.35;
    return leftScore - rightScore || Math.abs(left) - Math.abs(right) || left - right;
  });
}

export function smoothFootLockHeightOffsets(
  offsets: readonly number[],
  radius = 3,
): number[] {
  if (offsets.length === 0) {
    return [];
  }
  const safeRadius = Math.max(0, Math.floor(radius));
  if (safeRadius === 0) {
    return [...offsets];
  }
  const output = new Array<number>(offsets.length).fill(0);
  let fullWeight = 0;
  for (let delta = -safeRadius; delta <= safeRadius; delta += 1) {
    fullWeight += safeRadius + 1 - Math.abs(delta);
  }
  for (let index = 0; index < offsets.length; index += 1) {
    let weighted = 0;
    for (let delta = -safeRadius; delta <= safeRadius; delta += 1) {
      const sourceIndex = index + delta;
      if (sourceIndex < 0 || sourceIndex >= offsets.length) {
        continue;
      }
      const weight = safeRadius + 1 - Math.abs(delta);
      weighted += offsets[sourceIndex] * weight;
    }
    output[index] = weighted / fullWeight;
  }
  return output;
}

function quaternionError(target: any, current: any): any {
  const delta = target.clone().multiply(current.clone().invert()).normalize();
  if (delta.w < 0) {
    delta.set(-delta.x, -delta.y, -delta.z, -delta.w);
  }
  const halfSin = Math.hypot(delta.x, delta.y, delta.z);
  if (halfSin < 1e-10) {
    return new Vector3();
  }
  const angle = 2 * Math.atan2(halfSin, Math.max(0, delta.w));
  return new Vector3(delta.x, delta.y, delta.z)
    .multiplyScalar(angle / halfSin);
}

export class FootLockService {
  captureTarget(
    robot: UrdfRobotLike,
    sole: FootSoleDefinition,
  ): FootLockTarget {
    robot.updateMatrixWorld?.(true);
    const position = new Vector3(
      sole.localCenter.x,
      sole.localCenter.y,
      sole.localCenter.z,
    ).applyMatrix4(sole.link.matrixWorld);
    const rotation = new Quaternion().setFromRotationMatrix(sole.link.matrixWorld);
    return {
      position: { x: position.x, y: position.y, z: position.z },
      rotation: {
        x: rotation.x,
        y: rotation.y,
        z: rotation.z,
        w: rotation.w,
      },
    };
  }

  lock(
    robot: UrdfRobotLike,
    side: FootSide,
    sole: FootSoleDefinition,
    target: FootLockTarget,
  ): FootLockResult {
    const jointNames = LEG_JOINT_NAMES[side];
    const joints = jointNames.map((name) => (robot.joints as any)?.[name]);
    if (joints.some((joint) => !joint?.jointValue || typeof joint.setJointValue !== 'function')) {
      return {
        success: false,
        jointValues: {},
        iterations: 0,
        positionError: Number.POSITIVE_INFINITY,
        orientationError: Number.POSITIVE_INFINITY,
        reason: `The ${side} leg is missing one or more required joints.`,
      };
    }

    const solePoint = new Vector3(
      sole.localCenter.x,
      sole.localCenter.y,
      sole.localCenter.z,
    );
    const targetPosition = new Vector3(
      target.position.x,
      target.position.y,
      target.position.z,
    );
    const targetRotation = new Quaternion(
      target.rotation.x,
      target.rotation.y,
      target.rotation.z,
      target.rotation.w,
    ).normalize();
    const originalValues = joints.map((joint) => Number(joint.jointValue[0] ?? 0));
    const values = [...originalValues];
    const readPose = (): { position: any; rotation: any } => {
      robot.updateMatrixWorld?.(true);
      return {
        position: solePoint.clone().applyMatrix4(sole.link.matrixWorld),
        rotation: new Quaternion().setFromRotationMatrix(sole.link.matrixWorld),
      };
    };
    const applyJoint = (index: number, value: number): number => {
      const joint = joints[index];
      const lower =
        joint.jointType === 'continuous'
          ? Number.NEGATIVE_INFINITY
          : Number(joint.limit?.lower);
      const upper =
        joint.jointType === 'continuous'
          ? Number.POSITIVE_INFINITY
          : Number(joint.limit?.upper);
      const clamped = Math.max(
        Number.isFinite(lower) ? lower : Number.NEGATIVE_INFINITY,
        Math.min(Number.isFinite(upper) ? upper : Number.POSITIVE_INFINITY, value),
      );
      joint.setJointValue(clamped);
      const applied = Number(joint.jointValue?.[0]);
      return Number.isFinite(applied) ? applied : clamped;
    };

    const epsilon = 1e-4;
    const damping = 0.025;
    const orientationWeight = 0.3;
    let iterations = 0;
    let positionError = Number.POSITIVE_INFINITY;
    let orientationError = Number.POSITIVE_INFINITY;
    let solvedValues: number[] | null = null;
    try {
      for (iterations = 0; iterations < 80; iterations += 1) {
        const pose = readPose();
        const positionDelta = targetPosition.clone().sub(pose.position);
        const rotationDelta = quaternionError(targetRotation, pose.rotation);
        positionError = positionDelta.length();
        orientationError = rotationDelta.length();
        if (positionError < 0.0015 && orientationError < 0.012) {
          solvedValues = [...values];
          break;
        }
        const error = [
          positionDelta.x,
          positionDelta.y,
          positionDelta.z,
          rotationDelta.x * orientationWeight,
          rotationDelta.y * orientationWeight,
          rotationDelta.z * orientationWeight,
        ];
        const jacobian = Array.from({ length: 6 }, () => Array(6).fill(0));
        for (let jointIndex = 0; jointIndex < joints.length; jointIndex += 1) {
          let perturbedValue = applyJoint(
            jointIndex,
            values[jointIndex] + epsilon,
          );
          if (Math.abs(perturbedValue - values[jointIndex]) < epsilon * 0.5) {
            perturbedValue = applyJoint(
              jointIndex,
              values[jointIndex] - epsilon,
            );
          }
          const jointDelta = perturbedValue - values[jointIndex];
          if (Math.abs(jointDelta) < 1e-10) {
            applyJoint(jointIndex, values[jointIndex]);
            continue;
          }
          const perturbed = readPose();
          const rotationDerivative = quaternionError(
            perturbed.rotation,
            pose.rotation,
          ).multiplyScalar(orientationWeight / jointDelta);
          jacobian[0][jointIndex] =
            (perturbed.position.x - pose.position.x) / jointDelta;
          jacobian[1][jointIndex] =
            (perturbed.position.y - pose.position.y) / jointDelta;
          jacobian[2][jointIndex] =
            (perturbed.position.z - pose.position.z) / jointDelta;
          jacobian[3][jointIndex] = rotationDerivative.x;
          jacobian[4][jointIndex] = rotationDerivative.y;
          jacobian[5][jointIndex] = rotationDerivative.z;
          applyJoint(jointIndex, values[jointIndex]);
        }
        const normalMatrix = Array.from({ length: 6 }, (_, row) =>
          Array.from({ length: 6 }, (_, column) => {
            let value = 0;
            for (let joint = 0; joint < 6; joint += 1) {
              value += jacobian[row][joint] * jacobian[column][joint];
            }
            return value + (row === column ? damping * damping : 0);
          }),
        );
        const intermediate = solveLinearSystem(normalMatrix, error);
        if (!intermediate) {
          break;
        }
        for (let joint = 0; joint < 6; joint += 1) {
          let delta = 0;
          for (let row = 0; row < 6; row += 1) {
            delta += jacobian[row][joint] * intermediate[row];
          }
          delta = Math.max(-0.12, Math.min(0.12, delta));
          values[joint] = applyJoint(joint, values[joint] + delta);
        }
      }
      const finalPose = readPose();
      positionError = finalPose.position.distanceTo(targetPosition);
      orientationError = quaternionError(
        targetRotation,
        finalPose.rotation,
      ).length();
      if (
        !solvedValues &&
        positionError < 0.003 &&
        orientationError < 0.025
      ) {
        solvedValues = [...values];
      }
    } finally {
      originalValues.forEach((value, index) => applyJoint(index, value));
      robot.updateMatrixWorld?.(true);
    }

    if (!solvedValues) {
      const limitedJoints = jointNames.filter((_, index) => {
        const lower = Number(joints[index].limit?.lower);
        const upper = Number(joints[index].limit?.upper);
        return (
          (Number.isFinite(lower) && Math.abs(values[index] - lower) < 1e-4) ||
          (Number.isFinite(upper) && Math.abs(values[index] - upper) < 1e-4)
        );
      });
      return {
        success: false,
        jointValues: {},
        iterations,
        positionError,
        orientationError,
        reason:
          `Foot lock target is unreachable at this frame. Position drift: ${(positionError * 1_000).toFixed(1)} mm; ` +
          `rotation drift: ${((orientationError * 180) / Math.PI).toFixed(2)}°.` +
          (limitedJoints.length > 0
            ? ` Joints at limits: ${limitedJoints.join(', ')}.`
            : ''),
      };
    }
    return {
      success: true,
      jointValues: Object.fromEntries(
        jointNames.map((name, index) => [name, solvedValues[index]]),
      ),
      iterations,
      positionError,
      orientationError,
    };
  }
}
