import { describe, expect, it } from 'vitest';
import type { DroppedFileMap } from '../../types/viewer';
import { writeFloat32Npz } from './NpzWriter';
import {
  DEX_EVT_JOINT_NAMES_29,
  UfoReferenceMotionService,
} from './UfoReferenceMotionService';

function createDexEvtFile(): File {
  const frameCount = 2;
  const bodyCount = 39;
  const jointPos = new Float32Array(frameCount * 29);
  jointPos[29] = 0.5;
  const bodyPos = new Float32Array(frameCount * bodyCount * 3);
  bodyPos.set([1, 2, 3], 0);
  bodyPos.set([4, 5, 6], bodyCount * 3);
  const bodyQuat = new Float32Array(frameCount * bodyCount * 4);
  for (let index = 0; index < frameCount * bodyCount; index += 1) {
    bodyQuat[index * 4] = 1;
  }
  const zeros = new Float32Array(frameCount * bodyCount * 3);
  const bytes = writeFloat32Npz({
    fps: { data: new Float32Array([50]), shape: [] },
    joint_pos: { data: jointPos, shape: [frameCount, 29] },
    joint_vel: { data: new Float32Array(jointPos.length), shape: [frameCount, 29] },
    body_pos_w: { data: bodyPos, shape: [frameCount, bodyCount, 3] },
    body_quat_w: { data: bodyQuat, shape: [frameCount, bodyCount, 4] },
    body_lin_vel_w: { data: zeros, shape: [frameCount, bodyCount, 3] },
    body_ang_vel_w: { data: zeros, shape: [frameCount, bodyCount, 3] },
  });
  return new File([new Uint8Array(bytes)], 'getup.npz');
}

describe('UfoReferenceMotionService', () => {
  it('loads DEX EVT 39-body reference NPZ into a URDF motion clip', async () => {
    const fileMap: DroppedFileMap = new Map([['getup.npz', createDexEvtFile()]]);
    const result = await new UfoReferenceMotionService().loadFromDroppedFiles(fileMap);

    expect(result.clip.fps).toBe(50);
    expect(result.clip.frameCount).toBe(2);
    expect(result.clip.schema.jointNames).toEqual([...DEX_EVT_JOINT_NAMES_29]);
    expect(Array.from(result.clip.data.slice(0, 7))).toEqual([1, 2, 3, 0, 0, 0, 1]);
    expect(Array.from(result.clip.data.slice(result.clip.stride, result.clip.stride + 7))).toEqual([
      4,
      5,
      6,
      0,
      0,
      0,
      1,
    ]);
    expect(result.clip.data[result.clip.stride + 7]).toBeCloseTo(0.5);
  });
});
