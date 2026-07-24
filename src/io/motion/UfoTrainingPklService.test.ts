import { describe, expect, it } from 'vitest';
import { Matrix4 } from 'three';
import type { MotionClip, UrdfRobotLike } from '../../types/viewer';
import type { UfoFrameSampler, UfoReferenceData } from './UfoReferenceNpzService';
import {
  UFO_POLICY_BODY_NAMES,
  UFO_POLICY_JOINT_NAMES,
} from './UfoReferenceNpzService';
import {
  buildUfoTrainingMotionRecord,
  buildUfoTrainingMotionRecordFromClip,
} from './UfoTrainingPklService';

function createReferenceData(): UfoReferenceData {
  const frameCount = 2;
  const bodyCount = UFO_POLICY_BODY_NAMES.length;
  const jointCount = UFO_POLICY_JOINT_NAMES.length;
  const bodyQuatW = new Float32Array(frameCount * bodyCount * 4);
  for (let index = 0; index < frameCount * bodyCount; index += 1) {
    bodyQuatW[index * 4] = 1;
  }
  const jointPos = new Float32Array(frameCount * jointCount);
  jointPos[0] = 0.25;
  jointPos[1] = 0.5;
  jointPos[2] = 0.75;
  const bodyPosW = new Float32Array(frameCount * bodyCount * 3);
  bodyPosW.set([1, 2, 3], 0);
  bodyPosW.set([4, 5, 6], bodyCount * 3);
  return {
    fps: new Float32Array([50]),
    jointPos,
    jointVel: new Float32Array(frameCount * jointCount),
    bodyPosW,
    bodyQuatW,
    bodyLinVelW: new Float32Array(frameCount * bodyCount * 3),
    bodyAngVelW: new Float32Array(frameCount * bodyCount * 3),
  };
}

describe('UfoTrainingPklService', () => {
  it('builds MotionLib root, body-axis and DOF arrays', () => {
    const record = buildUfoTrainingMotionRecord(createReferenceData(), 2, 'test_motion');

    expect(record.root_trans_offset.shape).toEqual([2, 3]);
    expect(Array.from(record.root_trans_offset.data)).toEqual([1, 2, 3, 4, 5, 6]);
    expect(record.pose_aa.shape).toEqual([2, 30, 3]);
    expect(Array.from(record.pose_aa.data.slice(3, 12))).toEqual([
      0,
      0.25,
      0,
      0.5,
      0,
      0,
      0,
      0,
      0.75,
    ]);
    expect(record.dof_pos.shape).toEqual([2, 29]);
    expect(record.pose_quat_global.shape).toEqual([2, 30, 4]);
    expect(record.fps).toBe(50);
    expect(record.motion_key).toBe('test_motion');
  });

  it('builds training fields directly from a clip without velocity data', () => {
    const frameCount = 2;
    const stride = 7 + UFO_POLICY_JOINT_NAMES.length;
    const data = new Float32Array(frameCount * stride);
    data[3] = 0;
    data[6] = 1;
    data[stride + 3] = 0;
    data[stride + 6] = 1;
    data[7] = 0.25;
    data[stride + 7] = 0.5;
    const clip: MotionClip = {
      name: 'direct',
      sourcePath: 'direct.npz',
      fps: 50,
      frameCount,
      stride,
      schema: {
        rootJointName: 'root',
        rootComponentCount: 7,
        jointNames: [...UFO_POLICY_JOINT_NAMES],
      },
      csvMode: 'ordered',
      sourceColumnCount: stride,
      data,
    };
    const links = Object.fromEntries(
      UFO_POLICY_BODY_NAMES.map((name) => [name, { matrixWorld: new Matrix4() }]),
    );
    const robot = {
      parent: { matrixWorld: new Matrix4() },
      links,
    } as unknown as UrdfRobotLike;
    const sampler: UfoFrameSampler = {
      sampleClipFrames: (_clip, visitor) => {
        for (let frame = 0; frame < frameCount; frame += 1) {
          visitor(frame, robot);
        }
      },
    };

    const record = buildUfoTrainingMotionRecordFromClip(clip, sampler, 'direct');

    expect(Array.from(record.dof_pos.data.slice(0, 1))).toEqual([0.25]);
    expect(Array.from(record.dof_pos.data.slice(UFO_POLICY_JOINT_NAMES.length, UFO_POLICY_JOINT_NAMES.length + 1))).toEqual([0.5]);
    expect(Array.from(record.pose_quat_global.data.slice(0, 4))).toEqual([0, 0, 0, 1]);
    expect(record.root_trans_offset.shape).toEqual([2, 3]);
    expect(record.fps).toBe(50);
  });
});
