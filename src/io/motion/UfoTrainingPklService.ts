import type { MotionClip } from '../../types/viewer';
import {
  buildUfoReferenceData,
  UFO_POLICY_BODY_NAMES,
  UFO_POLICY_JOINT_NAMES,
  type UfoFrameSampler,
  type UfoReferenceData,
} from './UfoReferenceNpzService';
import {
  float32PickleArray,
  type PickleFloat32Array,
  writeNumpyPickle,
} from './NumpyPickleWriter';

export interface UfoTrainingMotionRecord {
  root_trans_offset: PickleFloat32Array;
  pose_aa: PickleFloat32Array;
  pose_quat_global: PickleFloat32Array;
  root_rot: PickleFloat32Array;
  dof_pos: PickleFloat32Array;
  fps: number;
  joint_names: string[];
  body_names: string[];
  motion_key: string;
}

function quaternionWxyzToAxisAngle(
  wValue: number,
  xValue: number,
  yValue: number,
  zValue: number,
): [number, number, number] {
  const norm = Math.hypot(wValue, xValue, yValue, zValue);
  if (norm < 1e-8) {
    return [0, 0, 0];
  }
  let w = wValue / norm;
  let x = xValue / norm;
  let y = yValue / norm;
  let z = zValue / norm;
  if (w < 0) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }
  w = Math.max(-1, Math.min(1, w));
  const vectorNorm = Math.hypot(x, y, z);
  if (vectorNorm < 1e-8) {
    return [0, 0, 0];
  }
  const scale = (2 * Math.atan2(vectorNorm, w)) / vectorNorm;
  return [x * scale, y * scale, z * scale];
}

function jointAxisAngle(jointName: string, angle: number): [number, number, number] {
  if (jointName.includes('_roll_')) {
    return [angle, 0, 0];
  }
  if (jointName.includes('_pitch_')) {
    return [0, angle, 0];
  }
  if (jointName.includes('_yaw_')) {
    return [0, 0, angle];
  }
  throw new Error(`Cannot infer UFO joint axis for "${jointName}".`);
}

export function sanitizeUfoMotionKey(name: string): string {
  const withoutExtension = name.replace(/\.[^.]+$/, '');
  const sanitized = withoutExtension.replace(/[^a-zA-Z0-9_-]+/g, '_').replace(/^_+|_+$/g, '');
  return sanitized || 'modified_motion';
}

export function buildUfoTrainingMotionRecord(
  data: UfoReferenceData,
  frameCount: number,
  motionKey: string,
): UfoTrainingMotionRecord {
  const bodyCount = UFO_POLICY_BODY_NAMES.length;
  const jointCount = UFO_POLICY_JOINT_NAMES.length;
  if (
    data.bodyPosW.length !== frameCount * bodyCount * 3 ||
    data.bodyQuatW.length !== frameCount * bodyCount * 4 ||
    data.jointPos.length !== frameCount * jointCount
  ) {
    throw new Error('UFO training PKL source arrays have inconsistent shapes.');
  }

  const rootTranslation = new Float32Array(frameCount * 3);
  const rootRotationXyzw = new Float32Array(frameCount * 4);
  const globalQuaternionXyzw = new Float32Array(frameCount * bodyCount * 4);
  const poseAxisAngle = new Float32Array(frameCount * bodyCount * 3);

  for (let frame = 0; frame < frameCount; frame += 1) {
    const pelvisPositionBase = frame * bodyCount * 3;
    rootTranslation.set(data.bodyPosW.subarray(pelvisPositionBase, pelvisPositionBase + 3), frame * 3);

    for (let body = 0; body < bodyCount; body += 1) {
      const sourceQuaternionBase = (frame * bodyCount + body) * 4;
      const targetQuaternionBase = sourceQuaternionBase;
      const w = data.bodyQuatW[sourceQuaternionBase];
      const x = data.bodyQuatW[sourceQuaternionBase + 1];
      const y = data.bodyQuatW[sourceQuaternionBase + 2];
      const z = data.bodyQuatW[sourceQuaternionBase + 3];
      globalQuaternionXyzw[targetQuaternionBase] = x;
      globalQuaternionXyzw[targetQuaternionBase + 1] = y;
      globalQuaternionXyzw[targetQuaternionBase + 2] = z;
      globalQuaternionXyzw[targetQuaternionBase + 3] = w;
      if (body === 0) {
        rootRotationXyzw.set([x, y, z, w], frame * 4);
        poseAxisAngle.set(quaternionWxyzToAxisAngle(w, x, y, z), frame * bodyCount * 3);
      } else {
        const jointIndex = body - 1;
        const angle = data.jointPos[frame * jointCount + jointIndex];
        poseAxisAngle.set(
          jointAxisAngle(UFO_POLICY_JOINT_NAMES[jointIndex], angle),
          (frame * bodyCount + body) * 3,
        );
      }
    }
  }

  return {
    root_trans_offset: float32PickleArray(rootTranslation, [frameCount, 3]),
    pose_aa: float32PickleArray(poseAxisAngle, [frameCount, bodyCount, 3]),
    pose_quat_global: float32PickleArray(globalQuaternionXyzw, [frameCount, bodyCount, 4]),
    root_rot: float32PickleArray(rootRotationXyzw, [frameCount, 4]),
    dof_pos: float32PickleArray(data.jointPos.slice(), [frameCount, jointCount]),
    fps: data.fps[0],
    joint_names: [...UFO_POLICY_JOINT_NAMES],
    body_names: [...UFO_POLICY_BODY_NAMES],
    motion_key: motionKey,
  };
}

export function exportUfoTrainingPkl(clip: MotionClip, sampler: UfoFrameSampler): Uint8Array {
  const motionKey = sanitizeUfoMotionKey(clip.name);
  const record = buildUfoTrainingMotionRecord(
    buildUfoReferenceData(clip, sampler),
    clip.frameCount,
    motionKey,
  );
  return writeNumpyPickle({ [motionKey]: record });
}

export interface UfoTrainingPklBatchItem {
  clip: MotionClip;
  motionKey: string;
}

export function exportUfoTrainingPklBatch(
  items: readonly UfoTrainingPklBatchItem[],
  sampler: UfoFrameSampler,
): Uint8Array {
  if (items.length === 0) {
    throw new Error('Select at least one motion for UFO training PKL export.');
  }
  const records: Record<string, UfoTrainingMotionRecord> = {};
  for (const item of items) {
    const motionKey = sanitizeUfoMotionKey(item.motionKey);
    if (records[motionKey]) {
      throw new Error(`Duplicate UFO training motion key: ${motionKey}.`);
    }
    records[motionKey] = buildUfoTrainingMotionRecord(
      buildUfoReferenceData(item.clip, sampler),
      item.clip.frameCount,
      motionKey,
    );
  }
  return writeNumpyPickle(records);
}
