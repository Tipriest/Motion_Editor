import { describe, expect, it } from 'vitest';
import type { MotionClip } from '../../types/viewer';
import { resampleMotionClip } from './MotionClipResampling';

function buildClip(): MotionClip {
  const stride = 8;
  const frameCount = 5;
  const data = new Float32Array(frameCount * stride);
  for (let frame = 0; frame < frameCount; frame += 1) {
    const base = frame * stride;
    data[base] = frame;
    data[base + 3] = 0;
    data[base + 4] = 0;
    data[base + 5] = Math.sin((frame * Math.PI) / 16);
    data[base + 6] = Math.cos((frame * Math.PI) / 16);
    data[base + 7] = frame * 10;
  }
  return {
    name: 'test',
    sourcePath: 'test.pkl',
    fps: 100,
    frameCount,
    stride,
    schema: {
      rootJointName: 'floating_base_joint',
      rootComponentCount: 7,
      jointNames: ['joint_0'],
    },
    csvMode: 'ordered',
    sourceColumnCount: stride,
    data,
  };
}

describe('resampleMotionClip', () => {
  it('resamples values while preserving the selected endpoints', () => {
    const source = buildClip();
    const result = resampleMotionClip(source, 50);

    expect(result.fps).toBe(50);
    expect(result.frameCount).toBe(3);
    expect(result.data[0]).toBeCloseTo(0);
    expect(result.data[result.stride]).toBeCloseTo(2);
    expect(result.data[result.stride * 2]).toBeCloseTo(4);
    expect(result.data[result.stride + 7]).toBeCloseTo(20);

    for (let frame = 0; frame < result.frameCount; frame += 1) {
      const base = frame * result.stride + 3;
      const quaternionLength = Math.hypot(
        result.data[base],
        result.data[base + 1],
        result.data[base + 2],
        result.data[base + 3],
      );
      expect(quaternionLength).toBeCloseTo(1);
    }
  });

  it('returns an independent data buffer when the FPS is unchanged', () => {
    const source = buildClip();
    const result = resampleMotionClip(source, source.fps);

    expect(result.frameCount).toBe(source.frameCount);
    expect(result.data).not.toBe(source.data);
    expect(Array.from(result.data)).toEqual(Array.from(source.data));
  });

  it('rejects invalid target frame rates', () => {
    expect(() => resampleMotionClip(buildClip(), 0)).toThrow(/Target FPS must be positive/);
  });
});
