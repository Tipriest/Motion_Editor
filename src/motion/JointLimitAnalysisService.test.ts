import { describe, expect, it } from 'vitest';

import type { MotionClip, UrdfRobotLike } from '../types/viewer';
import { JointLimitAnalysisService } from './JointLimitAnalysisService';

function createClip(): MotionClip {
  const jointNames = ['hip_joint', 'slide_joint', 'continuous_joint'];
  const stride = 7 + jointNames.length;
  const data = new Float32Array(4 * stride);
  const values = [
    [0, 0.05, 100],
    [0.97, 0.099, 100],
    [1.2, 0.05, 100],
    [-0.95, -0.01, 100],
  ];
  for (let frame = 0; frame < values.length; frame += 1) {
    data[frame * stride + 6] = 1;
    data.set(values[frame], frame * stride + 7);
  }
  return {
    name: 'limits',
    sourcePath: 'limits.csv',
    fps: 50,
    frameCount: values.length,
    stride,
    schema: {
      rootJointName: 'root',
      rootComponentCount: 7,
      jointNames,
    },
    csvMode: 'ordered',
    sourceColumnCount: stride,
    data,
  };
}

function createRobot(): UrdfRobotLike {
  return {
    name: 'limited',
    joints: {
      hip_joint: {
        jointType: 'revolute',
        jointValue: [1],
        limit: { lower: -1, upper: 1 },
      },
      slide_joint: {
        jointType: 'prismatic',
        limit: { lower: 0, upper: 0.1 },
      },
      continuous_joint: {
        jointType: 'continuous',
        limit: { lower: -1, upper: 1 },
      },
    } as any,
    traverse: () => undefined,
  };
}

describe('JointLimitAnalysisService', () => {
  it('classifies raw clip values as near or beyond URDF limits', () => {
    const result = new JointLimitAnalysisService().analyze(
      createClip(),
      createRobot(),
    );

    expect(result.violationFrameCount).toBe(2);
    expect(result.nearFrameCount).toBe(1);
    expect(result.byFrame.get(1)?.map(({ status }) => status)).toEqual([
      'near',
      'near',
    ]);
    expect(result.byFrame.get(2)?.[0]).toMatchObject({
      jointName: 'hip_joint',
      status: 'violation',
      boundary: 'upper',
    });
    expect(result.byFrame.get(2)?.[0].distance).toBeCloseTo(0.2, 5);
    expect(result.byFrame.get(3)?.[0]).toMatchObject({
      jointName: 'slide_joint',
      status: 'violation',
      boundary: 'lower',
    });
  });

  it('ignores continuous joints even when their raw values are large', () => {
    const result = new JointLimitAnalysisService().analyze(
      createClip(),
      createRobot(),
    );

    expect(
      [...result.byFrame.values()]
        .flat()
        .some(({ jointName }) => jointName === 'continuous_joint'),
    ).toBe(false);
  });
});
