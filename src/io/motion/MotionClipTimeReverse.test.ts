import { describe, expect, it } from 'vitest';

import type { MotionClip } from '../../types/viewer';
import { reverseMotionClipTime } from './MotionClipTimeReverse';

function createClip(): MotionClip {
  const stride = 8;
  return {
    name: 'forward.pkl',
    sourcePath: 'forward.pkl',
    fps: 50,
    frameCount: 3,
    stride,
    schema: {
      rootJointName: 'root',
      rootComponentCount: 7,
      jointNames: ['joint'],
    },
    csvMode: 'ordered',
    sourceColumnCount: stride,
    data: new Float32Array([
      1, 2, 3, 0, 0, 0, 1, 10,
      3, 2, 3, 0, 0, 0.5, 0.5, 20,
      6, 4, 3, 0, 0, 1, 0, 30,
    ]),
  };
}

describe('reverseMotionClipTime', () => {
  it('reverses every pose while rebasing root translation to the source start', () => {
    const reversed = reverseMotionClipTime(createClip());

    expect(Array.from(reversed.data.slice(0, 8))).toEqual([
      1, 2, 3, 0, 0, 1, 0, 30,
    ]);
    expect(Array.from(reversed.data.slice(8, 16))).toEqual([
      -2, 0, 3, 0, 0, 0.5, 0.5, 20,
    ]);
    expect(Array.from(reversed.data.slice(16, 24))).toEqual([
      -4, 0, 3, 0, 0, 0, 1, 10,
    ]);
  });

  it('returns to the source after reversing twice', () => {
    const clip = createClip();
    const restored = reverseMotionClipTime(reverseMotionClipTime(clip));

    expect(Array.from(restored.data)).toEqual(Array.from(clip.data));
  });

  it('can preserve the raw reversed world trajectory', () => {
    const reversed = reverseMotionClipTime(createClip(), {
      rebaseRootPosition: false,
    });

    expect(Array.from(reversed.data.slice(0, 3))).toEqual([6, 4, 3]);
    expect(Array.from(reversed.data.slice(16, 19))).toEqual([1, 2, 3]);
  });
});
