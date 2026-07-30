import type { MotionClip } from '../../types/viewer';

export interface MotionTimeReverseOptions {
  rebaseRootPosition?: boolean;
}

export function reverseMotionClipTime(
  clip: MotionClip,
  options: MotionTimeReverseOptions = {},
): MotionClip {
  const data = new Float32Array(clip.data.length);
  for (let targetFrame = 0; targetFrame < clip.frameCount; targetFrame += 1) {
    const sourceFrame = clip.frameCount - 1 - targetFrame;
    const sourceOffset = sourceFrame * clip.stride;
    data.set(
      clip.data.subarray(sourceOffset, sourceOffset + clip.stride),
      targetFrame * clip.stride,
    );
  }

  if (
    (options.rebaseRootPosition ?? true) &&
    clip.frameCount > 0 &&
    clip.schema.rootComponentCount >= 3
  ) {
    const sourceStart = 0;
    const reversedStart = (clip.frameCount - 1) * clip.stride;
    const offsets = [
      clip.data[sourceStart] - clip.data[reversedStart],
      clip.data[sourceStart + 1] - clip.data[reversedStart + 1],
      clip.data[sourceStart + 2] - clip.data[reversedStart + 2],
    ];
    for (let frame = 0; frame < clip.frameCount; frame += 1) {
      const frameOffset = frame * clip.stride;
      data[frameOffset] += offsets[0];
      data[frameOffset + 1] += offsets[1];
      data[frameOffset + 2] += offsets[2];
    }
  }

  return {
    ...clip,
    data,
  };
}
