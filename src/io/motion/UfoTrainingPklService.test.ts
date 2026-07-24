import { describe, expect, it } from 'vitest';
import type { UfoReferenceData } from './UfoReferenceNpzService';
import {
  UFO_POLICY_BODY_NAMES,
  UFO_POLICY_JOINT_NAMES,
} from './UfoReferenceNpzService';
import { buildUfoTrainingMotionRecord } from './UfoTrainingPklService';

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
});
