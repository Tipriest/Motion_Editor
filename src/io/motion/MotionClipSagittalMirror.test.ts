import { describe, expect, it } from 'vitest';

import type { MotionClip } from '../../types/viewer';
import { mirrorMotionClipSagittal } from './MotionClipSagittalMirror';

function createClip(): MotionClip {
  const jointNames = [
    'hip_pitch_l_joint',
    'hip_pitch_r_joint',
    'hip_roll_l_joint',
    'hip_roll_r_joint',
    'waist_yaw_joint',
    'waist_pitch_joint',
  ];
  const stride = 7 + jointNames.length;
  const yaw = Math.PI / 3;
  const data = new Float32Array(stride);
  data.set(
    [
      1,
      2,
      3,
      0,
      0,
      Math.sin(yaw / 2),
      Math.cos(yaw / 2),
      0.1,
      0.2,
      0.3,
      0.4,
      0.5,
      0.6,
    ],
  );
  return {
    name: 'turn_right',
    sourcePath: 'turn_right.pkl',
    fps: 50,
    frameCount: 1,
    stride,
    schema: {
      rootJointName: 'root',
      rootComponentCount: 7,
      jointNames,
    },
    csvMode: 'ordered',
    sourceColumnCount: stride,
    data,
  };
}

describe('mirrorMotionClipSagittal', () => {
  it('reflects root pose and swaps signed left-right joints', () => {
    const mirrored = mirrorMotionClipSagittal(createClip());

    expect(Array.from(mirrored.data.slice(0, 3))).toEqual([1, -2, 3]);
    expect(mirrored.data[5]).toBeCloseTo(-0.5, 6);
    expect(mirrored.data[6]).toBeCloseTo(Math.cos(Math.PI / 6), 6);
    expect(Array.from(mirrored.data.slice(7))).toEqual([
      expect.closeTo(0.2, 6),
      expect.closeTo(0.1, 6),
      expect.closeTo(-0.4, 6),
      expect.closeTo(-0.3, 6),
      expect.closeTo(-0.5, 6),
      expect.closeTo(0.6, 6),
    ]);
  });

  it('returns to the source after mirroring twice', () => {
    const clip = createClip();
    const restored = mirrorMotionClipSagittal(
      mirrorMotionClipSagittal(clip),
    );

    expect(restored.data).not.toBe(clip.data);
    expect(Array.from(restored.data)).toEqual(
      Array.from(clip.data).map((value) => expect.closeTo(value, 6)),
    );
  });
});
