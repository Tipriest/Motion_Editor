import { Matrix4, Quaternion, Vector3 } from 'three';
import type { MotionClip, UrdfRobotLike } from '../../types/viewer';
import { writeFloat32Npz } from './NpzWriter';
import {
  DEX_EVT_BODY_NAMES_39,
  DEX_EVT_JOINT_NAMES_29,
  type RobotTrackingLayoutId,
} from './RobotTrackingLayouts';

export const UFO_POLICY_JOINT_NAMES = [
  'hip_pitch_l_joint',
  'hip_roll_l_joint',
  'hip_yaw_l_joint',
  'knee_pitch_l_joint',
  'ankle_pitch_l_joint',
  'ankle_roll_l_joint',
  'hip_pitch_r_joint',
  'hip_roll_r_joint',
  'hip_yaw_r_joint',
  'knee_pitch_r_joint',
  'ankle_pitch_r_joint',
  'ankle_roll_r_joint',
  'waist_yaw_joint',
  'waist_roll_joint',
  'waist_pitch_joint',
  'shoulder_pitch_l_joint',
  'shoulder_roll_l_joint',
  'shoulder_yaw_l_joint',
  'elbow_pitch_l_joint',
  'elbow_yaw_l_joint',
  'wrist_pitch_l_joint',
  'wrist_roll_l_joint',
  'shoulder_pitch_r_joint',
  'shoulder_roll_r_joint',
  'shoulder_yaw_r_joint',
  'elbow_pitch_r_joint',
  'elbow_yaw_r_joint',
  'wrist_pitch_r_joint',
  'wrist_roll_r_joint',
] as const;

export const UFO_POLICY_BODY_NAMES = [
  'pelvis',
  'hip_pitch_l_link',
  'hip_roll_l_link',
  'hip_yaw_l_link',
  'knee_pitch_l_link',
  'ankle_pitch_l_link',
  'ankle_roll_l_link',
  'hip_pitch_r_link',
  'hip_roll_r_link',
  'hip_yaw_r_link',
  'knee_pitch_r_link',
  'ankle_pitch_r_link',
  'ankle_roll_r_link',
  'waist_yaw_link',
  'waist_roll_link',
  'waist_pitch_link',
  'shoulder_pitch_l_link',
  'shoulder_roll_l_link',
  'shoulder_yaw_l_link',
  'elbow_pitch_l_link',
  'elbow_yaw_l_link',
  'wrist_pitch_l_link',
  'wrist_roll_l_link',
  'shoulder_pitch_r_link',
  'shoulder_roll_r_link',
  'shoulder_yaw_r_link',
  'elbow_pitch_r_link',
  'elbow_yaw_r_link',
  'wrist_pitch_r_link',
  'wrist_roll_r_link',
] as const;

export interface UfoFrameSampler {
  sampleClipFrames(
    clip: MotionClip,
    visitor: (frameIndex: number, robot: UrdfRobotLike) => void,
  ): void;
}

export interface UfoReferenceData {
  jointCount: number;
  bodyCount: number;
  fps: Float32Array;
  jointPos: Float32Array;
  jointVel: Float32Array;
  bodyPosW: Float32Array;
  bodyQuatW: Float32Array;
  bodyLinVelW: Float32Array;
  bodyAngVelW: Float32Array;
}

export interface UfoReferenceExportOptions {
  layout?: RobotTrackingLayoutId;
}

interface TransformNode {
  matrixWorld: any;
}

function assertFinite(label: string, values: Float32Array): void {
  for (let index = 0; index < values.length; index += 1) {
    if (!Number.isFinite(values[index])) {
      throw new Error(`${label} contains a non-finite value at index ${index}.`);
    }
  }
}

function differentiate(values: Float32Array, frameCount: number, width: number, fps: number): Float32Array {
  const output = new Float32Array(values.length);
  if (frameCount <= 1) {
    return output;
  }
  for (let frame = 0; frame < frameCount; frame += 1) {
    const fromFrame = frame === 0 ? 0 : frame - 1;
    const toFrame = frame === frameCount - 1 ? frameCount - 1 : frame + 1;
    const scale = fps / Math.max(1, toFrame - fromFrame);
    for (let component = 0; component < width; component += 1) {
      output[frame * width + component] =
        (values[toFrame * width + component] - values[fromFrame * width + component]) * scale;
    }
  }
  return output;
}

function calculateAngularVelocities(
  quaternionsXyzw: Float32Array,
  frameCount: number,
  bodyCount: number,
  fps: number,
): Float32Array {
  const output = new Float32Array(frameCount * bodyCount * 3);
  if (frameCount <= 1) {
    return output;
  }
  const fromQuaternion = new Quaternion();
  const toQuaternion = new Quaternion();
  const delta = new Quaternion();

  for (let frame = 0; frame < frameCount; frame += 1) {
    const fromFrame = frame === 0 ? 0 : frame - 1;
    const toFrame = frame === frameCount - 1 ? frameCount - 1 : frame + 1;
    const deltaTime = Math.max(1, toFrame - fromFrame) / fps;
    for (let body = 0; body < bodyCount; body += 1) {
      const fromBase = (fromFrame * bodyCount + body) * 4;
      const toBase = (toFrame * bodyCount + body) * 4;
      fromQuaternion
        .set(
          quaternionsXyzw[fromBase],
          quaternionsXyzw[fromBase + 1],
          quaternionsXyzw[fromBase + 2],
          quaternionsXyzw[fromBase + 3],
        )
        .normalize();
      toQuaternion
        .set(
          quaternionsXyzw[toBase],
          quaternionsXyzw[toBase + 1],
          quaternionsXyzw[toBase + 2],
          quaternionsXyzw[toBase + 3],
        )
        .normalize();
      delta.copy(toQuaternion).multiply(fromQuaternion.clone().invert()).normalize();
      if (delta.w < 0) {
        delta.set(-delta.x, -delta.y, -delta.z, -delta.w);
      }
      const vectorLength = Math.hypot(delta.x, delta.y, delta.z);
      const outputBase = (frame * bodyCount + body) * 3;
      if (vectorLength < 1e-8) {
        continue;
      }
      const angle = 2 * Math.atan2(vectorLength, Math.max(-1, Math.min(1, delta.w)));
      const scale = angle / (vectorLength * deltaTime);
      output[outputBase] = delta.x * scale;
      output[outputBase + 1] = delta.y * scale;
      output[outputBase + 2] = delta.z * scale;
    }
  }
  return output;
}

export function buildUfoReferenceData(
  clip: MotionClip,
  sampler: UfoFrameSampler,
  options: UfoReferenceExportOptions = {},
): UfoReferenceData {
  if (!Number.isFinite(clip.fps) || clip.fps <= 0 || clip.frameCount <= 0) {
    throw new Error('UFO NPZ export requires a non-empty clip with positive FPS.');
  }

  const layout = options.layout ?? 'tiangong3';
  const jointNames =
    layout === 'dex-evt' ? DEX_EVT_JOINT_NAMES_29 : UFO_POLICY_JOINT_NAMES;
  const bodyNames =
    layout === 'dex-evt' ? DEX_EVT_BODY_NAMES_39 : UFO_POLICY_BODY_NAMES;
  const jointIndices = jointNames.map((jointName) => {
    const index = clip.schema.jointNames.indexOf(jointName);
    if (index < 0) {
      throw new Error(`UFO NPZ export requires joint "${jointName}".`);
    }
    return index;
  });
  const frameCount = clip.frameCount;
  const jointCount = jointNames.length;
  const bodyCount = bodyNames.length;
  const jointPos = new Float32Array(frameCount * jointCount);
  const bodyPosW = new Float32Array(frameCount * bodyCount * 3);
  const bodyQuatXyzw = new Float32Array(frameCount * bodyCount * 4);
  const inverseModelRoot = new Matrix4();
  const localMatrix = new Matrix4();
  const position = new Vector3();
  const quaternion = new Quaternion();
  const scale = new Vector3();

  for (let frame = 0; frame < frameCount; frame += 1) {
    const sourceBase = frame * clip.stride + clip.schema.rootComponentCount;
    for (let joint = 0; joint < jointCount; joint += 1) {
      jointPos[frame * jointCount + joint] = clip.data[sourceBase + jointIndices[joint]];
    }
  }

  sampler.sampleClipFrames(clip, (frameIndex, robot) => {
    const modelRoot = robot.parent as TransformNode | null;
    if (!modelRoot?.matrixWorld) {
      throw new Error('UFO NPZ export could not resolve the robot model-root transform.');
    }
    inverseModelRoot.copy(modelRoot.matrixWorld).invert();

    for (let bodyIndex = 0; bodyIndex < bodyCount; bodyIndex += 1) {
      const bodyName = bodyNames[bodyIndex];
      const body = robot.links?.[bodyName] as TransformNode | undefined;
      if (!body?.matrixWorld) {
        throw new Error(`UFO NPZ export requires body "${bodyName}".`);
      }
      localMatrix.multiplyMatrices(inverseModelRoot, body.matrixWorld);
      localMatrix.decompose(position, quaternion, scale);
      quaternion.normalize();

      const positionBase = (frameIndex * bodyCount + bodyIndex) * 3;
      bodyPosW[positionBase] = position.x;
      bodyPosW[positionBase + 1] = position.y;
      bodyPosW[positionBase + 2] = position.z;

      const quaternionBase = (frameIndex * bodyCount + bodyIndex) * 4;
      if (frameIndex > 0) {
        const previousBase = ((frameIndex - 1) * bodyCount + bodyIndex) * 4;
        const dot =
          bodyQuatXyzw[previousBase] * quaternion.x +
          bodyQuatXyzw[previousBase + 1] * quaternion.y +
          bodyQuatXyzw[previousBase + 2] * quaternion.z +
          bodyQuatXyzw[previousBase + 3] * quaternion.w;
        if (dot < 0) {
          quaternion.set(-quaternion.x, -quaternion.y, -quaternion.z, -quaternion.w);
        }
      }
      bodyQuatXyzw[quaternionBase] = quaternion.x;
      bodyQuatXyzw[quaternionBase + 1] = quaternion.y;
      bodyQuatXyzw[quaternionBase + 2] = quaternion.z;
      bodyQuatXyzw[quaternionBase + 3] = quaternion.w;
    }
  });

  const jointVel = differentiate(jointPos, frameCount, jointCount, clip.fps);
  const bodyLinVelW = differentiate(bodyPosW, frameCount, bodyCount * 3, clip.fps);
  const bodyAngVelW = calculateAngularVelocities(bodyQuatXyzw, frameCount, bodyCount, clip.fps);
  const bodyQuatW = new Float32Array(bodyQuatXyzw.length);
  for (let index = 0; index < bodyCount * frameCount; index += 1) {
    const base = index * 4;
    bodyQuatW[base] = bodyQuatXyzw[base + 3];
    bodyQuatW[base + 1] = bodyQuatXyzw[base];
    bodyQuatW[base + 2] = bodyQuatXyzw[base + 1];
    bodyQuatW[base + 3] = bodyQuatXyzw[base + 2];
  }

  for (const [label, values] of Object.entries({
    joint_pos: jointPos,
    joint_vel: jointVel,
    body_pos_w: bodyPosW,
    body_quat_w: bodyQuatW,
    body_lin_vel_w: bodyLinVelW,
    body_ang_vel_w: bodyAngVelW,
  })) {
    assertFinite(label, values);
  }

  return {
    jointCount,
    bodyCount,
    fps: new Float32Array([clip.fps]),
    jointPos,
    jointVel,
    bodyPosW,
    bodyQuatW,
    bodyLinVelW,
    bodyAngVelW,
  };
}

export function encodeUfoReferenceNpz(data: UfoReferenceData, frameCount: number): Uint8Array {
  const { jointCount, bodyCount } = data;
  return writeFloat32Npz({
    fps: { data: data.fps, shape: [] },
    joint_pos: { data: data.jointPos, shape: [frameCount, jointCount] },
    joint_vel: { data: data.jointVel, shape: [frameCount, jointCount] },
    body_pos_w: { data: data.bodyPosW, shape: [frameCount, bodyCount, 3] },
    body_quat_w: { data: data.bodyQuatW, shape: [frameCount, bodyCount, 4] },
    body_lin_vel_w: { data: data.bodyLinVelW, shape: [frameCount, bodyCount, 3] },
    body_ang_vel_w: { data: data.bodyAngVelW, shape: [frameCount, bodyCount, 3] },
  });
}

export function exportUfoReferenceNpz(
  clip: MotionClip,
  sampler: UfoFrameSampler,
  options: UfoReferenceExportOptions = {},
): Uint8Array {
  return encodeUfoReferenceNpz(
    buildUfoReferenceData(clip, sampler, options),
    clip.frameCount,
  );
}
