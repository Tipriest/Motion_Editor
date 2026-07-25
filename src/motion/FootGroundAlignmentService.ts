import { Vector3 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';
import type { FootSoleDefinition } from './GroundContactService';

export type FootSide = 'left' | 'right';

export interface FootAlignmentResult {
  success: boolean;
  jointValues: Record<string, number>;
  iterations: number;
  positionError: number;
  orientationError: number;
  reason?: string;
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
      return clamped;
    };

    let iterations = 0;
    let positionError = Number.POSITIVE_INFINITY;
    let orientationError = Number.POSITIVE_INFINITY;
    let solvedValues: number[] | null = null;
    try {
      const initialPose = readPose();
      const up = new Vector3(0, 1, 0);
      const orientationWeight = 0.18;
      const horizontalWeight = 0.35;
      const epsilon = 1e-4;
      const damping = 0.035;

      for (iterations = 0; iterations < 60; iterations += 1) {
        const pose = readPose();
        const heightDelta = groundHeight - pose.position.y;
        const horizontalDeltaX = initialPose.position.x - pose.position.x;
        const horizontalDeltaZ = initialPose.position.z - pose.position.z;
        const horizontalError = Math.hypot(horizontalDeltaX, horizontalDeltaZ);
        const normalCorrection = new Vector3().crossVectors(pose.normal, up);
        positionError = Math.hypot(heightDelta, horizontalError);
        orientationError = Math.acos(Math.max(-1, Math.min(1, pose.normal.dot(up))));
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
          applyJoint(jointIndex, currentValues[jointIndex] + epsilon);
          const perturbed = readPose();
          const normalDerivative = new Vector3()
            .crossVectors(pose.normal, perturbed.normal)
            .multiplyScalar(1 / epsilon);
          jacobian[0][jointIndex] =
            (perturbed.position.y - pose.position.y) / epsilon;
          jacobian[1][jointIndex] =
            normalDerivative.x * orientationWeight;
          jacobian[2][jointIndex] =
            normalDerivative.z * orientationWeight;
          jacobian[3][jointIndex] =
            ((perturbed.position.x - pose.position.x) * horizontalWeight) / epsilon;
          jacobian[4][jointIndex] =
            ((perturbed.position.z - pose.position.z) * horizontalWeight) / epsilon;
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
      const finalHeightError = Math.abs(finalPose.position.y - groundHeight);
      const finalHorizontalError = Math.hypot(
        finalPose.position.x - initialPose.position.x,
        finalPose.position.z - initialPose.position.z,
      );
      positionError = Math.hypot(finalHeightError, finalHorizontalError);
      orientationError = Math.acos(
        Math.max(-1, Math.min(1, finalPose.normal.dot(new Vector3(0, 1, 0)))),
      );
      if (
        !solvedValues &&
        finalHeightError < 0.006 &&
        finalHorizontalError < 0.012 &&
        orientationError < 0.035
      ) {
        solvedValues = [...currentValues];
      }
    } finally {
      originalValues.forEach((value, index) => applyJoint(index, value));
      robot.updateMatrixWorld?.(true);
    }

    if (!solvedValues) {
      return {
        success: false,
        jointValues: {},
        iterations,
        positionError,
        orientationError,
        reason: 'The selected foot target could not be reached within the joint limits.',
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
