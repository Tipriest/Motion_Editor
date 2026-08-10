import { describe, expect, it } from 'vitest';

import { writeStoredZip } from './ZipWriter';

describe('writeStoredZip', () => {
  it('stores multiple named binary files in a valid classic ZIP layout', () => {
    const bytes = writeStoredZip([
      { name: 'first.pkl', data: new Uint8Array([1, 2, 3]) },
      { name: 'folder/second.pkl', data: new Uint8Array([4, 5]) },
    ]);
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const decoder = new TextDecoder();

    expect(view.getUint32(0, true)).toBe(0x04034b50);
    const firstNameLength = view.getUint16(26, true);
    expect(decoder.decode(bytes.subarray(30, 30 + firstNameLength))).toBe(
      'first.pkl',
    );
    expect(Array.from(bytes.subarray(30 + firstNameLength, 33 + firstNameLength))).toEqual([
      1, 2, 3,
    ]);
    expect(view.getUint32(bytes.length - 22, true)).toBe(0x06054b50);
    expect(view.getUint16(bytes.length - 14, true)).toBe(2);
  });

  it('rejects duplicate archive names', () => {
    expect(() =>
      writeStoredZip([
        { name: 'motion.pkl', data: new Uint8Array([1]) },
        { name: 'motion.pkl', data: new Uint8Array([2]) },
      ]),
    ).toThrow('duplicate');
  });
});
