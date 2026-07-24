import { describe, expect, it } from 'vitest';
import {
  float32PickleArray,
  NumpyPickleDictionaryWriter,
  writeNumpyPickle,
  writeNumpyPickleParts,
} from './NumpyPickleWriter';

describe('NumpyPickleWriter', () => {
  it('produces identical protocol bytes in contiguous and chunked modes', () => {
    const value = {
      motion: {
        data: float32PickleArray(new Float32Array(40_000), [20_000, 2]),
        fps: 50,
      },
    };
    const contiguous = writeNumpyPickle(value);
    const parts = writeNumpyPickleParts(value);
    const joined = new Uint8Array(parts.reduce((sum, part) => sum + part.length, 0));
    let offset = 0;
    for (const part of parts) {
      joined.set(part, offset);
      offset += part.length;
    }

    expect(parts.length).toBeGreaterThan(1);
    expect(joined).toEqual(contiguous);
  });

  it('streams dictionary entries without changing the pickle bytes', () => {
    const entries = {
      first: { fps: 50, data: float32PickleArray(new Float32Array([1, 2]), [1, 2]) },
      second: { fps: 30, data: float32PickleArray(new Float32Array([3, 4]), [1, 2]) },
    };
    const writer = new NumpyPickleDictionaryWriter();
    for (const [key, value] of Object.entries(entries)) {
      writer.add(key, value);
    }
    const parts = writer.finish();
    const joined = new Uint8Array(parts.reduce((sum, part) => sum + part.length, 0));
    let offset = 0;
    for (const part of parts) {
      joined.set(part, offset);
      offset += part.length;
    }

    expect(joined).toEqual(writeNumpyPickle(entries));
  });
});
