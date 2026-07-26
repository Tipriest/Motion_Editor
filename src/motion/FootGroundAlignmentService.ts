import { Vector3 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';
import type { FootSoleDefinition } from './GroundContactService';

export type FootSide = 'left' | 'right';

export interface FootAlignmentResult {
  success: boolean;
  bestEffort?: boolean;
  jointValues: Record<string, number>;
  iterations: number;
  positionError: number;
  orientationError: number;
  heightError?: number;
  horizontalError?: number;
  forwardTiltError?: number;
  sideTiltError?: number;
  reason?: string;
}

export interface FootAlignmentTarget {
  position?: { x: number; y: number; z: number };
  normal?: { x: number; y: number; z: number };
}

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

export function solveLinearSystem(matrix: number[][], vector: number[]): number[] | null {
  const size = vector.length;
  const augmented = matrix.map((row, index) => [...row, vector[index]]);
  for (let column = 0; column < size; column += 1) {
    let pivot = column;
    for (let row = column + 1; row < size; row += 1) {
      if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) {
        pivot = row;
      }
    }
    if (Math.abs(augmented[pivot][column]) < 1e-10) {
      return null;
    }
    [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];
    const divisor = augmented[column][column];
    for (let index = column; index <= size; index += 1) {
      augmented[column][index] /= divisor;
    }
    for (let row = 0; row < size; row += 1) {
      if (row === column) {
        continue;
      }
      const factor = augmented[row][column];
      for (let index = column; index <= size; index += 1) {
        augmented[row][index] -= factor * augmented[column][index];
      }
    }
  }
  return augmented.map((row) => row[size]);
}

export class FootGroundAlignmentService {
  align(
    robot: UrdfRobotLike,
    side: FootSide,
    sole: FootSoleDefinition,
    groundHeight: number,
    target?: FootAlignmentTarget,
  ): FootAlignmentResult {
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

    const originalValues = joints.map((joint) => Number(joint.jointValue[0] ?? 0));
    const currentValues = [...originalValues];
    const solePoint = new Vector3(
      sole.localCenter.x,
      sole.localCenter.y,
      sole.localCenter.z,
    );
    const localNormal = new Vector3(0, 0, 1);
    const readPose = (): { position: any; normal: any } => {
      robot.updateMatrixWorld?.(true);
      return {
        position: solePoint.clone().applyMatrix4(sole.link.matrixWorld),
        normal: localNormal.clone().transformDirection(sole.link.matrixWorld).normalize(),
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

    let iterations = 0;
    let positionError = Number.POSITIVE_INFINITY;
    let orientationError = Number.POSITIVE_INFINITY;
    let heightError = Number.POSITIVE_INFINITY;
    let horizontalError = Number.POSITIVE_INFINITY;
    let forwardTiltError = Number.POSITIVE_INFINITY;
    let sideTiltError = Number.POSITIVE_INFINITY;
    let solvedValues: number[] | null = null;
    let bestEffortValues: number[] | null = null;
    let bestEffortHeight = Number.POSITIVE_INFINITY;
    let bestEffortHorizontal = Number.POSITIVE_INFINITY;
    let bestEffortOrientation = Number.POSITIVE_INFINITY;
    let initialFootHeight = Number.POSITIVE_INFINITY;
    const targetPosition = new Vector3();
    const targetNormal = new Vector3(0, 1, 0);
    try {
      const initialPose = readPose();
      initialFootHeight = initialPose.position.y;
      targetPosition.set(
        target?.position?.x ?? initialPose.position.x,
        target?.position?.y ?? groundHeight,
        target?.position?.z ?? initialPose.position.z,
      );
      if (target?.normal) {
        targetNormal
          .set(target.normal.x, target.normal.y, target.normal.z)
          .normalize();
      }
      const orientationWeight = 0.18;
      const horizontalWeight = 0.35;
      const epsilon = 1e-4;
      const damping = 0.035;

      for (iterations = 0; iterations < 60; iterations += 1) {
        const pose = readPose();
        const heightDelta = targetPosition.y - pose.position.y;
        const horizontalDeltaX = targetPosition.x - pose.position.x;
        const horizontalDeltaZ = targetPosition.z - pose.position.z;
        horizontalError = Math.hypot(horizontalDeltaX, horizontalDeltaZ);
        heightError = Math.abs(heightDelta);
        const normalCorrection = new Vector3().crossVectors(pose.normal, targetNormal);
        positionError = Math.hypot(heightDelta, horizontalError);
        orientationError = Math.acos(
          Math.max(-1, Math.min(1, pose.normal.dot(targetNormal))),
        );
        sideTiltError = Math.abs(normalCorrection.z);
        forwardTiltError = Math.abs(normalCorrection.x);
        const improvesHeight =
          heightError < bestEffortHeight - 1e-9 &&
          pose.position.y < initialFootHeight - 1e-9;
        if (improvesHeight) {
          bestEffortValues = [...currentValues];
          bestEffortHeight = heightError;
          bestEffortHorizontal = horizontalError;
          bestEffortOrientation = orientationError;
        }
        if (
          Math.abs(heightDelta) < 0.0015 &&
          horizontalError < 0.003 &&
          orientationError < 0.012
        ) {
          solvedValues = [...currentValues];
          break;
        }
        const error = [
          heightDelta,
          normalCorrection.x * orientationWeight,
          normalCorrection.z * orientationWeight,
          horizontalDeltaX * horizontalWeight,
          horizontalDeltaZ * horizontalWeight,
        ];
        const jacobian = Array.from({ length: 5 }, () => Array(6).fill(0));
        for (let jointIndex = 0; jointIndex < joints.length; jointIndex += 1) {
          let perturbedValue = applyJoint(
            jointIndex,
            currentValues[jointIndex] + epsilon,
          );
          if (Math.abs(perturbedValue - currentValues[jointIndex]) < epsilon * 0.5) {
            perturbedValue = applyJoint(
              jointIndex,
              currentValues[jointIndex] - epsilon,
            );
          }
          const jointDelta = perturbedValue - currentValues[jointIndex];
          if (Math.abs(jointDelta) < 1e-10) {
            applyJoint(jointIndex, currentValues[jointIndex]);
            continue;
          }
          const perturbed = readPose();
          const normalDerivative = new Vector3()
            .crossVectors(pose.normal, perturbed.normal)
            .multiplyScalar(1 / jointDelta);
          jacobian[0][jointIndex] =
            (perturbed.position.y - pose.position.y) / jointDelta;
          jacobian[1][jointIndex] =
            normalDerivative.x * orientationWeight;
          jacobian[2][jointIndex] =
            normalDerivative.z * orientationWeight;
          jacobian[3][jointIndex] =
            ((perturbed.position.x - pose.position.x) * horizontalWeight) /
            jointDelta;
          jacobian[4][jointIndex] =
            ((perturbed.position.z - pose.position.z) * horizontalWeight) /
            jointDelta;
          applyJoint(jointIndex, currentValues[jointIndex]);
        }
        const normalMatrix = Array.from({ length: 5 }, (_, row) =>
          Array.from({ length: 5 }, (_, column) => {
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
          for (let row = 0; row < 5; row += 1) {
            delta += jacobian[row][joint] * intermediate[row];
          }
          delta = Math.max(-0.12, Math.min(0.12, delta));
          currentValues[joint] = applyJoint(joint, currentValues[joint] + delta);
        }
      }
      const finalPose = readPose();
      heightError = Math.abs(finalPose.position.y - targetPosition.y);
      horizontalError = Math.hypot(
        finalPose.position.x - targetPosition.x,
        finalPose.position.z - targetPosition.z,
      );
      positionError = Math.hypot(heightError, horizontalError);
      orientationError = Math.acos(
        Math.max(-1, Math.min(1, finalPose.normal.dot(targetNormal))),
      );
      const finalNormalCorrection = new Vector3().crossVectors(
        finalPose.normal,
        targetNormal,
      );
      sideTiltError = Math.abs(finalNormalCorrection.z);
      forwardTiltError = Math.abs(finalNormalCorrection.x);
      const finalImprovesHeight =
        heightError < bestEffortHeight - 1e-9 &&
        finalPose.position.y < initialFootHeight - 1e-9;
      if (finalImprovesHeight) {
        bestEffortValues = [...currentValues];
        bestEffortHeight = heightError;
        bestEffortHorizontal = horizontalError;
        bestEffortOrientation = orientationError;
      }
      if (
        !solvedValues &&
        heightError < 0.006 &&
        horizontalError < 0.012 &&
        orientationError < 0.035
      ) {
        solvedValues = [...currentValues];
      }
    } finally {
      originalValues.forEach((value, index) => applyJoint(index, value));
      robot.updateMatrixWorld?.(true);
    }

    if (!solvedValues) {
      const fallbackValues = bestEffortValues;
      if (fallbackValues) {
        fallbackValues.forEach((value, index) => applyJoint(index, value));
        const fallbackPose = readPose();
        heightError = Math.abs(fallbackPose.position.y - targetPosition.y);
        horizontalError = bestEffortHorizontal;
        orientationError = bestEffortOrientation;
        positionError = Math.hypot(heightError, horizontalError);
        const fallbackNormalCorrection = new Vector3().crossVectors(
          fallbackPose.normal,
          targetNormal,
        );
        sideTiltError = Math.abs(fallbackNormalCorrection.z);
        forwardTiltError = Math.abs(fallbackNormalCorrection.x);
        originalValues.forEach((value, index) => applyJoint(index, value));
        robot.updateMatrixWorld?.(true);
      }
      const failures: string[] = [];
      if (heightError >= 0.006) {
        failures.push(
          `Foot height error: ${(heightError * 1_000).toFixed(1)} mm (required ≤ 6.0 mm)`,
        );
      }
      if (horizontalError >= 0.012) {
        failures.push(
          `Foot projection drift: ${(horizontalError * 1_000).toFixed(1)} mm (required ≤ 12.0 mm)`,
        );
      }
      if (orientationError >= 0.035) {
        failures.push(
          `Forward tilt: ${((forwardTiltError * 180) / Math.PI).toFixed(2)}°; side tilt: ${((sideTiltError * 180) / Math.PI).toFixed(2)}° (combined required ≤ 2.01°)`,
        );
      }
      const reportedValues = fallbackValues ?? currentValues;
      const limitedJoints = jointNames.filter((_, index) => {
        const joint = joints[index];
        const lower = Number(joint.limit?.lower);
        const upper = Number(joint.limit?.upper);
        return (
          (Number.isFinite(lower) && Math.abs(reportedValues[index] - lower) < 1e-4) ||
          (Number.isFinite(upper) && Math.abs(reportedValues[index] - upper) < 1e-4)
        );
      });
      if (limitedJoints.length > 0) {
        failures.push(`Joints at limits: ${limitedJoints.join(', ')}`);
      }
      return {
        success: false,
        bestEffort: fallbackValues !== null,
        jointValues: fallbackValues
          ? Object.fromEntries(
              jointNames.map((name, index) => [name, fallbackValues[index]]),
            )
          : {},
        iterations,
        positionError,
        orientationError,
        heightError,
        horizontalError,
        forwardTiltError,
        sideTiltError,
        reason:
          failures.length > 0
            ? `Foot alignment constraints not met:\n• ${failures.join('\n• ')}${
                fallbackValues
                  ? '\n\nThe closest reachable lowered-foot pose will be applied.'
                  : ''
              }`
            : 'The foot IK solver did not converge. Try increasing the smoothing range or adjusting the pose first.',
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
      heightError,
      horizontalError,
      forwardTiltError,
      sideTiltError,
    };
  }
}
