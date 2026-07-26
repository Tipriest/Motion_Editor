import { describe, expect, it } from 'vitest';
import { Euler, Matrix4, Quaternion, Vector3 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';
import {
  buildFootLockHeightCandidates,
  FootLockService,
  smoothFootLockHeightOffsets,
} from './FootLockService';

const LEFT_JOINT_NAMES = [
  'hip_pitch_l_joint',
  'hip_roll_l_joint',
  'hip_yaw_l_joint',
  'knee_pitch_l_joint',
  'ankle_pitch_l_joint',
  'ankle_roll_l_joint',
];

function createSyntheticRobot(initialValues: number[]) {
  const values = [...initialValues];
  const joints = Object.fromEntries(
    LEFT_JOINT_NAMES.map((name, index) => [
      name,
      {
        jointType: 'revolute',
        jointValue: [values[index]],
        limit: { lower: -1, upper: 1 },
        setJointValue: (value: number) => {
          values[index] = value;
          joints[name].jointValue[0] = value;
          return true;
        },
      },
    ]),
  ) as any;
  const footLink = { matrixWorld: new Matrix4() };
  const robot = {
    name: 'synthetic-foot-lock',
    joints,
    links: { ankle_roll_l_link: footLink },
    traverse: () => undefined,
    updateMatrixWorld: () => {
      footLink.matrixWorld.compose(
        new Vector3(values[0], values[1], values[2]),
        new Quaternion().setFromEuler(
          new Euler(values[3], values[4], values[5], 'XYZ'),
        ),
        new Vector3(1, 1, 1),
      );
    },
  } as unknown as UrdfRobotLike;
  robot.updateMatrixWorld?.(true);
  return { robot, footLink, values };
}

describe('FootLockService', () => {
  it('prioritizes small height corrections near the previous solution', () => {
    expect(buildFootLockHeightCandidates(0, 0.03, 0.01)[0]).toBe(0);
    expect(buildFootLockHeightCandidates(-0.03, 0.05, 0.01)[0]).toBeCloseTo(
      -0.03,
      8,
    );
  });

  it('smooths height corrections and tapers interval boundaries toward zero', () => {
    const smoothed = smoothFootLockHeightOffsets([0, -0.04, -0.04, -0.04], 1);

    expect(smoothed[0]).toBeCloseTo(-0.01, 8);
    expect(smoothed[1]).toBeCloseTo(-0.03, 8);
    expect(smoothed[2]).toBeCloseTo(-0.04, 8);
    expect(smoothed[3]).toBeCloseTo(-0.03, 8);
  });

  it('solves and restores a full six-dimensional foot target', () => {
    const initialValues = [0.04, -0.03, 0.02, 0.05, -0.04, 0.03];
    const { robot, footLink, values } = createSyntheticRobot(initialValues);
    const service = new FootLockService();
    const result = service.lock(
      robot,
      'left',
      {
        footName: 'ankle_roll_l_link',
        link: footLink,
        localCenter: { x: 0, y: 0, z: 0 },
      },
      {
        position: { x: 0, y: 0, z: 0 },
        rotation: { x: 0, y: 0, z: 0, w: 1 },
      },
    );

    expect(result.success).toBe(true);
    expect(result.positionError).toBeLessThan(0.003);
    expect(result.orientationError).toBeLessThan(0.025);
    expect(values).toEqual(initialValues);
    for (const value of Object.values(result.jointValues)) {
      expect(value).toBeCloseTo(0, 2);
    }
  });

  it('captures the current sole center and complete rotation', () => {
    const { robot, footLink } = createSyntheticRobot([
      0.1,
      0.2,
      0.3,
      0,
      0,
      Math.PI / 4,
    ]);
    const target = new FootLockService().captureTarget(robot, {
      footName: 'ankle_roll_l_link',
      link: footLink,
      localCenter: { x: 0, y: 0, z: 0 },
    });

    expect(target.position).toEqual({ x: 0.1, y: 0.2, z: 0.3 });
    expect(target.rotation.z).toBeCloseTo(Math.sin(Math.PI / 8), 6);
    expect(target.rotation.w).toBeCloseTo(Math.cos(Math.PI / 8), 6);
  });
});
