import { describe, expect, it } from 'vitest';
import { parseNpzFile } from './NumpyIO';
import { writeFloat32Npz } from './NpzWriter';

describe('writeFloat32Npz', () => {
  it('writes NumPy-compatible float32 arrays and scalars', async () => {
    const bytes = writeFloat32Npz({
      motion: {
        data: new Float32Array([1, 2, 3, 4, 5, 6]),
        shape: [2, 3],
      },
      fps: {
        data: new Float32Array([50]),
        shape: [],
      },
    });
    const fileBytes = new Uint8Array(bytes);
    const archive = await parseNpzFile(
      new File([fileBytes], 'motion.npz', { type: 'application/octet-stream' }),
    );

    expect(archive.listFileNames().sort()).toEqual(['fps.npy', 'motion.npy']);
    const motion = await archive.readNpy('motion.npy');
    expect(motion.descr).toBe('<f4');
    expect(motion.shape).toEqual([2, 3]);
    expect(Array.from(motion.toNumberArray())).toEqual([1, 2, 3, 4, 5, 6]);

    const fps = await archive.readNpy('fps.npy');
    expect(fps.shape).toEqual([]);
    expect(fps.toScalarNumber()).toBe(50);
  });

  it('rejects entries whose value count does not match the shape', () => {
    expect(() =>
      writeFloat32Npz({
        invalid: {
          data: new Float32Array([1, 2]),
          shape: [2, 2],
        },
      }),
    ).toThrow(/expected 4/);
  });
});
