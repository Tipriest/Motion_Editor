import type { MotionClip } from '../../types/viewer';
import { writeFloat32Npz } from './NpzWriter';
import { UFO_POLICY_JOINT_NAMES } from './UfoReferenceNpzService';

export function exportGmrNpz(clip: MotionClip): Uint8Array {
  const jointIndices = UFO_POLICY_JOINT_NAMES.map((jointName) => {
    const index = clip.schema.jointNames.indexOf(jointName);
    if (index < 0) {
      throw new Error(
        `GMR NPZ export requires joint "${jointName}" in the motion schema.`,
      );
    }
    return index;
  });
  const frameCount = clip.frameCount;
  const rootPosition = new Float32Array(frameCount * 3);
  const rootRotation = new Float32Array(frameCount * 4);
  const jointPosition = new Float32Array(
    frameCount * UFO_POLICY_JOINT_NAMES.length,
  );
  for (let frame = 0; frame < frameCount; frame += 1) {
    const sourceBase = frame * clip.stride;
    rootPosition.set(
      clip.data.subarray(sourceBase, sourceBase + 3),
      frame * 3,
    );
    rootRotation.set(
      clip.data.subarray(sourceBase + 3, sourceBase + 7),
      frame * 4,
    );
    const jointBase = sourceBase + clip.schema.rootComponentCount;
    for (let joint = 0; joint < jointIndices.length; joint += 1) {
      jointPosition[frame * jointIndices.length + joint] =
        clip.data[jointBase + jointIndices[joint]];
    }
  }
  return writeFloat32Npz({
    fps: {
      data: new Float32Array([clip.fps]),
      shape: [],
    },
    root_pos: {
      data: rootPosition,
      shape: [frameCount, 3],
    },
    root_rot: {
      data: rootRotation,
      shape: [frameCount, 4],
    },
    dof_pos: {
      data: jointPosition,
      shape: [frameCount, UFO_POLICY_JOINT_NAMES.length],
    },
  });
}
