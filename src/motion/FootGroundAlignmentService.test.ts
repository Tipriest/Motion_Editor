import { describe, expect, it } from 'vitest';
import { Matrix4, Quaternion, Vector3 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';
import {
  FootGroundAlignmentService,
  solveLinearSystem,
} from './FootGroundAlignmentService';

describe('FootGroundAlignmentService', () => {
  it('solves the damped IK normal equation system', () => {
    const solution = solveLinearSystem(
      [
        [4, 1, 0],
        [1, 3, 1],
        [0, 1, 2],
      ],
      [1, 2, 3],
    );

    expect(solution).not.toBeNull();
    expect(solution?.[0]).toBeCloseTo(2 / 9, 6);
    expect(solution?.[1]).toBeCloseTo(1 / 9, 6);
    expect(solution?.[2]).toBeCloseTo(13 / 9, 6);
  });

  it('returns null for a singular system', () => {
    expect(
      solveLinearSystem(
        [
          [1, 2],
          [2, 4],
        ],
        [1, 2],
      ),
    ).toBeNull();
  });

  it('aligns sole height and normal without constraining horizontal pose', () => {
    const names = [
      'hip_pitch_l_joint',
      'hip_roll_l_joint',
      'hip_yaw_l_joint',
      'knee_pitch_l_joint',
      'ankle_pitch_l_joint',
      'ankle_roll_l_joint',
    ];
    const values = [0.05, 0.1, -0.1, 0, 0, 0];
    const joints = Object.fromEntries(
      names.map((name, index) => [
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
      name: 'synthetic',
      joints,
      links: {},
      traverse: () => undefined,
      updateMatrixWorld: () => {
        const base = new Quaternion().setFromAxisAngle(
          new Vector3(1, 0, 0),
          -Math.PI / 2,
        );
        const tiltX = new Quaternion().setFromAxisAngle(
          new Vector3(1, 0, 0),
          values[1],
        );
        const tiltZ = new Quaternion().setFromAxisAngle(
          new Vector3(0, 0, 1),
          values[2],
        );
        footLink.matrixWorld.compose(
          new Vector3(
            values[3] + values[1] * 0.2,
            values[0],
            values[4] + values[2] * 0.2,
          ),
          tiltX.multiply(tiltZ).multiply(base),
          new Vector3(1, 1, 1),
        );
      },
    } as unknown as UrdfRobotLike;
    robot.updateMatrixWorld?.(true);

    const result = new FootGroundAlignmentService().align(
      robot,
      'left',
      {
        footName: 'ankle_roll_l_link',
        link: footLink,
        localCenter: { x: 0, y: 0, z: 0 },
      },
      0,
    );

    expect(result.success).toBe(true);
    expect(result.positionError).toBeLessThan(0.006);
    expect(result.orientationError).toBeLessThan(0.035);
    const alignedX =
      result.jointValues.knee_pitch_l_joint +
      result.jointValues.hip_roll_l_joint * 0.2;
    const alignedZ =
      result.jointValues.ankle_pitch_l_joint +
      result.jointValues.hip_yaw_l_joint * 0.2;
    expect(alignedX).toBeCloseTo(0.02, 2);
    expect(alignedZ).toBeCloseTo(-0.02, 2);
  });

  it('reports each constraint that remains outside tolerance', () => {
    const names = [
      'hip_pitch_l_joint',
      'hip_roll_l_joint',
      'hip_yaw_l_joint',
      'knee_pitch_l_joint',
      'ankle_pitch_l_joint',
      'ankle_roll_l_joint',
    ];
    const joints = Object.fromEntries(
      names.map((name) => [
        name,
        {
          jointType: 'revolute',
          jointValue: [0],
          limit: { lower: -1, upper: 1 },
          setJointValue: () => false,
        },
      ]),
    );
    const footLink = {
      matrixWorld: new Matrix4().makeRotationY(Math.PI / 2).setPosition(0, 0.1, 0),
    };
    const robot = {
      name: 'unreachable',
      joints,
      links: {},
      traverse: () => undefined,
      updateMatrixWorld: () => undefined,
    } as unknown as UrdfRobotLike;

    const result = new FootGroundAlignmentService().align(
      robot,
      'left',
      {
        footName: 'ankle_roll_l_link',
        link: footLink,
        localCenter: { x: 0, y: 0, z: 0 },
      },
      0,
    );

    expect(result.success).toBe(false);
    expect(result.reason).toContain('Foot height error');
    expect(result.reason).toContain('tilt');
  });

  it('returns the lowest reachable pose as a best-effort fallback', () => {
    const names = [
      'hip_pitch_l_joint',
      'hip_roll_l_joint',
      'hip_yaw_l_joint',
      'knee_pitch_l_joint',
      'ankle_pitch_l_joint',
      'ankle_roll_l_joint',
    ];
    const values = Array(6).fill(0);
    const joints = Object.fromEntries(
      names.map((name, index) => [
        name,
        {
          jointType: 'revolute',
          jointValue: [0],
          limit: index === 0 ? { lower: -0.05, upper: 0 } : { lower: 0, upper: 0 },
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
      name: 'limited-height',
      joints,
      links: {},
      traverse: () => undefined,
      updateMatrixWorld: () => {
        footLink.matrixWorld.compose(
          new Vector3(0, 0.1 + values[0], 0),
          new Quaternion().setFromAxisAngle(
            new Vector3(1, 0, 0),
            -Math.PI / 2,
          ),
          new Vector3(1, 1, 1),
        );
      },
    } as unknown as UrdfRobotLike;
    robot.updateMatrixWorld?.(true);

    const result = new FootGroundAlignmentService().align(
      robot,
      'left',
      {
        footName: 'ankle_roll_l_link',
        link: footLink,
        localCenter: { x: 0, y: 0, z: 0 },
      },
      0,
    );

    expect(result.success).toBe(false);
    expect(result.bestEffort).toBe(true);
    expect(result.jointValues.hip_pitch_l_joint).toBeCloseTo(-0.05, 6);
    expect(result.heightError).toBeCloseTo(0.05, 6);
    expect(result.reason).toContain('closest reachable lowered-foot pose');
  });
});
