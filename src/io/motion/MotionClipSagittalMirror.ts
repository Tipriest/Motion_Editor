import type { MotionClip, UrdfRobotLike } from '../../types/viewer';

export interface MotionMirrorOptions {
  robot?: UrdfRobotLike | null;
}

function mirroredPartnerName(jointName: string): string | null {
  const replacements: Array<[RegExp, string]> = [
    [/(^|_)left(?=_|$)/i, '$1right'],
    [/(^|_)right(?=_|$)/i, '$1left'],
    [/(^|_)l(?=_|$)/i, '$1r'],
    [/(^|_)r(?=_|$)/i, '$1l'],
  ];
  for (const [pattern, replacement] of replacements) {
    if (pattern.test(jointName)) {
      return jointName.replace(pattern, replacement);
    }
  }
  return null;
}

function readAxis(robot: UrdfRobotLike | null | undefined, jointName: string): number[] | null {
  const axis = (robot?.joints as any)?.[jointName]?.axis;
  if (!axis) {
    return null;
  }
  if (Array.isArray(axis)) {
    return axis.slice(0, 3).map(Number);
  }
  const values = [Number(axis.x), Number(axis.y), Number(axis.z)];
  return values.every(Number.isFinite) ? values : null;
}

function mirrorJointSign(
  jointName: string,
  robot: UrdfRobotLike | null | undefined,
): number {
  const axis = readAxis(robot, jointName);
  if (axis) {
    const [x, y, z] = axis.map((value) => Math.abs(value));
    return y >= x && y >= z ? 1 : -1;
  }
  const normalized = jointName.toLowerCase();
  if (normalized.includes('pitch')) {
    return 1;
  }
  if (normalized.includes('roll') || normalized.includes('yaw')) {
    return -1;
  }
  throw new Error(
    `Cannot infer sagittal mirror sign for joint "${jointName}". Add a URDF axis or use pitch/roll/yaw naming.`,
  );
}

export function mirrorMotionClipSagittal(
  clip: MotionClip,
  options: MotionMirrorOptions = {},
): MotionClip {
  if (clip.schema.rootComponentCount < 7) {
    throw new Error('Sagittal mirroring requires root XYZ plus XYZW quaternion data.');
  }
  const jointIndexByName = new Map(
    clip.schema.jointNames.map((jointName, index) => [jointName, index]),
  );
  const mapping = clip.schema.jointNames.map((jointName) => {
    const partnerName = mirroredPartnerName(jointName);
    const sourceIndex =
      partnerName === null ? jointIndexByName.get(jointName) : jointIndexByName.get(partnerName);
    if (sourceIndex === undefined) {
      throw new Error(
        `Cannot mirror joint "${jointName}" because partner "${partnerName}" is missing from the motion schema.`,
      );
    }
    return {
      sourceIndex,
      sign: mirrorJointSign(jointName, options.robot),
    };
  });

  const data = new Float32Array(clip.data.length);
  for (let frame = 0; frame < clip.frameCount; frame += 1) {
    const base = frame * clip.stride;
    data.set(clip.data.subarray(base, base + clip.stride), base);
    data[base] = clip.data[base];
    data[base + 1] = -clip.data[base + 1];
    data[base + 2] = clip.data[base + 2];
    const qx = clip.data[base + 3];
    const qy = clip.data[base + 4];
    const qz = clip.data[base + 5];
    const qw = clip.data[base + 6];
    const norm = Math.hypot(qx, qy, qz, qw);
    if (norm < 1e-8) {
      throw new Error(`Cannot mirror invalid root quaternion at frame ${frame + 1}.`);
    }
    data[base + 3] = -qx / norm;
    data[base + 4] = qy / norm;
    data[base + 5] = -qz / norm;
    data[base + 6] = qw / norm;
    const jointBase = base + clip.schema.rootComponentCount;
    for (let targetIndex = 0; targetIndex < mapping.length; targetIndex += 1) {
      const { sourceIndex, sign } = mapping[targetIndex];
      data[jointBase + targetIndex] =
        clip.data[jointBase + sourceIndex] * sign;
    }
  }
  return {
    ...clip,
    data,
  };
}
