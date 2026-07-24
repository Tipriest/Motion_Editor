import { Matrix4, Quaternion, Vector3 } from 'three';
import type { MotionClip, UrdfRobotLike } from '../../types/viewer';
import {
  UFO_POLICY_BODY_NAMES,
  UFO_POLICY_JOINT_NAMES,
  type UfoFrameSampler,
  type UfoReferenceData,
} from './UfoReferenceNpzService';
import {
  float32PickleArray,
  NumpyPickleDictionaryWriter,
  type PickleFloat32Array,
  writeNumpyPickle,
  writeNumpyPickleParts,
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

interface TransformNode {
  matrixWorld: any;
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

export function buildUfoTrainingMotionRecordFromClip(
  clip: MotionClip,
  sampler: UfoFrameSampler,
  motionKey: string,
): UfoTrainingMotionRecord {
  const frameCount = clip.frameCount;
  const bodyCount = UFO_POLICY_BODY_NAMES.length;
  const jointCount = UFO_POLICY_JOINT_NAMES.length;
  const jointIndices = UFO_POLICY_JOINT_NAMES.map((jointName) => {
    const index = clip.schema.jointNames.indexOf(jointName);
    if (index < 0) {
      throw new Error(`UFO training PKL requires joint "${jointName}".`);
    }
    return index;
  });
  const rootTranslation = new Float32Array(frameCount * 3);
  const rootRotationXyzw = new Float32Array(frameCount * 4);
  const globalQuaternionXyzw = new Float32Array(frameCount * bodyCount * 4);
  const poseAxisAngle = new Float32Array(frameCount * bodyCount * 3);
  const dofPosition = new Float32Array(frameCount * jointCount);

  for (let frame = 0; frame < frameCount; frame += 1) {
    const sourceBase = frame * clip.stride + clip.schema.rootComponentCount;
    for (let joint = 0; joint < jointCount; joint += 1) {
      const angle = clip.data[sourceBase + jointIndices[joint]];
      dofPosition[frame * jointCount + joint] = angle;
      const poseBase = (frame * bodyCount + joint + 1) * 3;
      const jointName = UFO_POLICY_JOINT_NAMES[joint];
      if (jointName.includes('_roll_')) {
        poseAxisAngle[poseBase] = angle;
      } else if (jointName.includes('_pitch_')) {
        poseAxisAngle[poseBase + 1] = angle;
      } else if (jointName.includes('_yaw_')) {
        poseAxisAngle[poseBase + 2] = angle;
      }
    }
  }

  const inverseModelRoot = new Matrix4();
  const localMatrix = new Matrix4();
  const position = new Vector3();
  const quaternion = new Quaternion();
  const scale = new Vector3();
  sampler.sampleClipFrames(clip, (frameIndex: number, robot: UrdfRobotLike) => {
    const modelRoot = robot.parent as TransformNode | null;
    if (!modelRoot?.matrixWorld) {
      throw new Error('UFO training PKL could not resolve the robot model-root transform.');
    }
    inverseModelRoot.copy(modelRoot.matrixWorld).invert();
    for (let body = 0; body < bodyCount; body += 1) {
      const bodyName = UFO_POLICY_BODY_NAMES[body];
      const bodyNode = robot.links?.[bodyName] as TransformNode | undefined;
      if (!bodyNode?.matrixWorld) {
        throw new Error(`UFO training PKL requires body "${bodyName}".`);
      }
      localMatrix.multiplyMatrices(inverseModelRoot, bodyNode.matrixWorld);
      localMatrix.decompose(position, quaternion, scale);
      quaternion.normalize();
      const quaternionBase = (frameIndex * bodyCount + body) * 4;
      if (frameIndex > 0) {
        const previousBase = ((frameIndex - 1) * bodyCount + body) * 4;
        const dot =
          globalQuaternionXyzw[previousBase] * quaternion.x +
          globalQuaternionXyzw[previousBase + 1] * quaternion.y +
          globalQuaternionXyzw[previousBase + 2] * quaternion.z +
          globalQuaternionXyzw[previousBase + 3] * quaternion.w;
        if (dot < 0) {
          quaternion.set(-quaternion.x, -quaternion.y, -quaternion.z, -quaternion.w);
        }
      }
      globalQuaternionXyzw[quaternionBase] = quaternion.x;
      globalQuaternionXyzw[quaternionBase + 1] = quaternion.y;
      globalQuaternionXyzw[quaternionBase + 2] = quaternion.z;
      globalQuaternionXyzw[quaternionBase + 3] = quaternion.w;
      if (body === 0) {
        const translationBase = frameIndex * 3;
        rootTranslation[translationBase] = position.x;
        rootTranslation[translationBase + 1] = position.y;
        rootTranslation[translationBase + 2] = position.z;
        const rootRotationBase = frameIndex * 4;
        rootRotationXyzw[rootRotationBase] = quaternion.x;
        rootRotationXyzw[rootRotationBase + 1] = quaternion.y;
        rootRotationXyzw[rootRotationBase + 2] = quaternion.z;
        rootRotationXyzw[rootRotationBase + 3] = quaternion.w;
        poseAxisAngle.set(
          quaternionWxyzToAxisAngle(
            quaternion.w,
            quaternion.x,
            quaternion.y,
            quaternion.z,
          ),
          frameIndex * bodyCount * 3,
        );
      }
    }
  });

  return {
    root_trans_offset: float32PickleArray(rootTranslation, [frameCount, 3]),
    pose_aa: float32PickleArray(poseAxisAngle, [frameCount, bodyCount, 3]),
    pose_quat_global: float32PickleArray(globalQuaternionXyzw, [frameCount, bodyCount, 4]),
    root_rot: float32PickleArray(rootRotationXyzw, [frameCount, 4]),
    dof_pos: float32PickleArray(dofPosition, [frameCount, jointCount]),
    fps: clip.fps,
    joint_names: [...UFO_POLICY_JOINT_NAMES],
    body_names: [...UFO_POLICY_BODY_NAMES],
    motion_key: motionKey,
  };
}

export function exportUfoTrainingPkl(clip: MotionClip, sampler: UfoFrameSampler): Uint8Array {
  const motionKey = sanitizeUfoMotionKey(clip.name);
  const record = buildUfoTrainingMotionRecordFromClip(clip, sampler, motionKey);
  return writeNumpyPickle({ [motionKey]: record });
}

export interface UfoTrainingPklBatchItem {
  clip: MotionClip;
  motionKey: string;
}

function buildUfoTrainingPklBatchRecords(
  items: readonly UfoTrainingPklBatchItem[],
  sampler: UfoFrameSampler,
): Record<string, UfoTrainingMotionRecord> {
  if (items.length === 0) {
    throw new Error('Select at least one motion for UFO training PKL export.');
  }
  const records: Record<string, UfoTrainingMotionRecord> = {};
  for (const item of items) {
    const motionKey = sanitizeUfoMotionKey(item.motionKey);
    if (records[motionKey]) {
      throw new Error(`Duplicate UFO training motion key: ${motionKey}.`);
    }
    records[motionKey] = buildUfoTrainingMotionRecordFromClip(item.clip, sampler, motionKey);
  }
  return records;
}

export function exportUfoTrainingPklBatch(
  items: readonly UfoTrainingPklBatchItem[],
  sampler: UfoFrameSampler,
): Uint8Array {
  return writeNumpyPickle(buildUfoTrainingPklBatchRecords(items, sampler));
}

export function exportUfoTrainingPklBatchParts(
  items: readonly UfoTrainingPklBatchItem[],
  sampler: UfoFrameSampler,
): Uint8Array[] {
  return writeNumpyPickleParts(buildUfoTrainingPklBatchRecords(items, sampler));
}

export async function exportUfoTrainingPklBatchPartsAsync(
  items: readonly UfoTrainingPklBatchItem[],
  sampler: UfoFrameSampler,
  options: {
    transformClip?: (clip: MotionClip) => MotionClip;
    onProgress?: (completed: number, total: number, motionKey: string) => void;
  } = {},
): Promise<Uint8Array[]> {
  if (items.length === 0) {
    throw new Error('Select at least one motion for UFO training PKL export.');
  }

  const writer = new NumpyPickleDictionaryWriter();
  const motionKeys = new Set<string>();
  for (let index = 0; index < items.length; index += 1) {
    const item = items[index];
    const motionKey = sanitizeUfoMotionKey(item.motionKey);
    if (motionKeys.has(motionKey)) {
      throw new Error(`Duplicate UFO training motion key: ${motionKey}.`);
    }
    motionKeys.add(motionKey);
    const clip = options.transformClip?.(item.clip) ?? item.clip;
    writer.add(
      motionKey,
      buildUfoTrainingMotionRecordFromClip(clip, sampler, motionKey),
    );
    options.onProgress?.(index + 1, items.length, motionKey);
    await new Promise<void>((resolve) => globalThis.setTimeout(resolve, 0));
  }
  return writer.finish();
}
