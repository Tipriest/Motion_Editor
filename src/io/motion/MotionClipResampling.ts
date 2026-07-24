import type { MotionClip } from '../../types/viewer';

const ROOT_QUATERNION_OFFSET = 3;
const ROOT_QUATERNION_COMPONENT_COUNT = 4;

function normalizeQuaternion(
  x: number,
  y: number,
  z: number,
  w: number,
): [number, number, number, number] {
  const length = Math.hypot(x, y, z, w);
  if (length < 1e-10) {
    return [0, 0, 0, 1];
  }
  return [x / length, y / length, z / length, w / length];
}

function slerpQuaternion(
  from: readonly number[],
  to: readonly number[],
  alpha: number,
): [number, number, number, number] {
  let [toX, toY, toZ, toW] = to;
  let dot = from[0] * toX + from[1] * toY + from[2] * toZ + from[3] * toW;

  // q and -q represent the same rotation. Use the shorter interpolation arc.
  if (dot < 0) {
    dot = -dot;
    toX = -toX;
    toY = -toY;
    toZ = -toZ;
    toW = -toW;
  }

  if (dot > 0.9995) {
    return normalizeQuaternion(
      from[0] + alpha * (toX - from[0]),
      from[1] + alpha * (toY - from[1]),
      from[2] + alpha * (toZ - from[2]),
      from[3] + alpha * (toW - from[3]),
    );
  }

  const theta = Math.acos(Math.min(1, Math.max(-1, dot)));
  const sinTheta = Math.sin(theta);
  const fromWeight = Math.sin((1 - alpha) * theta) / sinTheta;
  const toWeight = Math.sin(alpha * theta) / sinTheta;
  return normalizeQuaternion(
    from[0] * fromWeight + toX * toWeight,
    from[1] * fromWeight + toY * toWeight,
    from[2] * fromWeight + toZ * toWeight,
    from[3] * fromWeight + toW * toWeight,
  );
}

export function resampleMotionClip(clip: MotionClip, targetFps: number): MotionClip {
  if (!Number.isFinite(targetFps) || targetFps <= 0) {
    throw new Error(`Target FPS must be positive, received ${targetFps}.`);
  }
  if (!Number.isFinite(clip.fps) || clip.fps <= 0) {
    throw new Error(`Source FPS must be positive, received ${clip.fps}.`);
  }
  if (clip.frameCount <= 1 || Math.abs(clip.fps - targetFps) < 1e-8) {
    return {
      ...clip,
      fps: targetFps,
      data: clip.data.slice(),
    };
  }

  const targetFrameCount = Math.max(
    2,
    Math.round(((clip.frameCount - 1) * targetFps) / clip.fps) + 1,
  );
  const output = new Float32Array(targetFrameCount * clip.stride);

  for (let targetFrame = 0; targetFrame < targetFrameCount; targetFrame += 1) {
    const sourcePosition =
      targetFrameCount === 1
        ? 0
        : (targetFrame * (clip.frameCount - 1)) / (targetFrameCount - 1);
    const fromFrame = Math.floor(sourcePosition);
    const toFrame = Math.min(fromFrame + 1, clip.frameCount - 1);
    const alpha = sourcePosition - fromFrame;
    const fromBase = fromFrame * clip.stride;
    const toBase = toFrame * clip.stride;
    const targetBase = targetFrame * clip.stride;

    for (let component = 0; component < clip.stride; component += 1) {
      const fromValue = clip.data[fromBase + component];
      const toValue = clip.data[toBase + component];
      output[targetBase + component] = fromValue + (toValue - fromValue) * alpha;
    }

    const fromQuaternion = Array.from(
      clip.data.subarray(
        fromBase + ROOT_QUATERNION_OFFSET,
        fromBase + ROOT_QUATERNION_OFFSET + ROOT_QUATERNION_COMPONENT_COUNT,
      ),
    );
    const toQuaternion = Array.from(
      clip.data.subarray(
        toBase + ROOT_QUATERNION_OFFSET,
        toBase + ROOT_QUATERNION_OFFSET + ROOT_QUATERNION_COMPONENT_COUNT,
      ),
    );
    const quaternion = slerpQuaternion(fromQuaternion, toQuaternion, alpha);
    output.set(quaternion, targetBase + ROOT_QUATERNION_OFFSET);
  }

  return {
    ...clip,
    fps: targetFps,
    frameCount: targetFrameCount,
    data: output,
  };
}

export function retimeMotionClip(
  clip: MotionClip,
  targetFps: number,
  playbackSpeed: number,
): MotionClip {
  if (!Number.isFinite(playbackSpeed) || playbackSpeed <= 0) {
    throw new Error(`Playback speed must be positive, received ${playbackSpeed}.`);
  }
  return resampleMotionClip(
    {
      ...clip,
      fps: clip.fps * playbackSpeed,
    },
    targetFps,
  );
}
