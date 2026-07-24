export interface Float32NpzEntry {
  data: Float32Array;
  shape: number[];
}

const textEncoder = new TextEncoder();

function assertEntry(name: string, entry: Float32NpzEntry): void {
  if (!name || name.includes('\0')) {
    throw new Error(`Invalid NPZ entry name: ${name}`);
  }
  if (entry.shape.some((dimension) => !Number.isInteger(dimension) || dimension < 0)) {
    throw new Error(`NPZ entry ${name} has an invalid shape.`);
  }
  const expectedLength = entry.shape.reduce((product, dimension) => product * dimension, 1);
  if (entry.data.length !== expectedLength) {
    throw new Error(
      `NPZ entry ${name} has ${entry.data.length} values, expected ${expectedLength} for shape [${entry.shape.join(', ')}].`,
    );
  }
}

function encodeFloat32Npy(entry: Float32NpzEntry): Uint8Array {
  const shapeText =
    entry.shape.length === 0
      ? ''
      : entry.shape.length === 1
        ? `${entry.shape[0]},`
        : entry.shape.join(', ');
  const headerBase = `{'descr': '<f4', 'fortran_order': False, 'shape': (${shapeText}), }`;
  const preambleLength = 10;
  const paddingLength = (16 - ((preambleLength + headerBase.length + 1) % 16)) % 16;
  const header = textEncoder.encode(`${headerBase}${' '.repeat(paddingLength)}\n`);
  if (header.length > 0xffff) {
    throw new Error('NPY v1 header exceeds 65535 bytes.');
  }

  const output = new Uint8Array(preambleLength + header.length + entry.data.length * 4);
  output.set([0x93, 0x4e, 0x55, 0x4d, 0x50, 0x59, 0x01, 0x00], 0);
  const view = new DataView(output.buffer);
  view.setUint16(8, header.length, true);
  output.set(header, preambleLength);

  let offset = preambleLength + header.length;
  for (const value of entry.data) {
    view.setFloat32(offset, value, true);
    offset += 4;
  }
  return output;
}

const CRC_TABLE = (() => {
  const table = new Uint32Array(256);
  for (let index = 0; index < table.length; index += 1) {
    let value = index;
    for (let bit = 0; bit < 8; bit += 1) {
      value = (value & 1) !== 0 ? 0xedb88320 ^ (value >>> 1) : value >>> 1;
    }
    table[index] = value >>> 0;
  }
  return table;
})();

function crc32(bytes: Uint8Array): number {
  let crc = 0xffffffff;
  for (const byte of bytes) {
    crc = CRC_TABLE[(crc ^ byte) & 0xff] ^ (crc >>> 8);
  }
  return (crc ^ 0xffffffff) >>> 0;
}

interface EncodedZipEntry {
  name: Uint8Array;
  data: Uint8Array;
  crc: number;
  localOffset: number;
}

export function writeFloat32Npz(entries: Record<string, Float32NpzEntry>): Uint8Array {
  const encodedEntries: EncodedZipEntry[] = [];
  let localSize = 0;

  for (const [rawName, entry] of Object.entries(entries)) {
    const name = rawName.endsWith('.npy') ? rawName : `${rawName}.npy`;
    assertEntry(name, entry);
    const encoded: EncodedZipEntry = {
      name: textEncoder.encode(name),
      data: encodeFloat32Npy(entry),
      crc: 0,
      localOffset: localSize,
    };
    encoded.crc = crc32(encoded.data);
    localSize += 30 + encoded.name.length + encoded.data.length;
    encodedEntries.push(encoded);
  }

  if (encodedEntries.length === 0) {
    throw new Error('Cannot create an empty NPZ archive.');
  }

  const centralSize = encodedEntries.reduce((size, entry) => size + 46 + entry.name.length, 0);
  const output = new Uint8Array(localSize + centralSize + 22);
  const view = new DataView(output.buffer);
  let offset = 0;

  for (const entry of encodedEntries) {
    view.setUint32(offset, 0x04034b50, true);
    view.setUint16(offset + 4, 20, true);
    view.setUint16(offset + 6, 0, true);
    view.setUint16(offset + 8, 0, true);
    view.setUint32(offset + 14, entry.crc, true);
    view.setUint32(offset + 18, entry.data.length, true);
    view.setUint32(offset + 22, entry.data.length, true);
    view.setUint16(offset + 26, entry.name.length, true);
    view.setUint16(offset + 28, 0, true);
    output.set(entry.name, offset + 30);
    output.set(entry.data, offset + 30 + entry.name.length);
    offset += 30 + entry.name.length + entry.data.length;
  }

  const centralOffset = offset;
  for (const entry of encodedEntries) {
    view.setUint32(offset, 0x02014b50, true);
    view.setUint16(offset + 4, 20, true);
    view.setUint16(offset + 6, 20, true);
    view.setUint16(offset + 8, 0, true);
    view.setUint16(offset + 10, 0, true);
    view.setUint32(offset + 16, entry.crc, true);
    view.setUint32(offset + 20, entry.data.length, true);
    view.setUint32(offset + 24, entry.data.length, true);
    view.setUint16(offset + 28, entry.name.length, true);
    view.setUint16(offset + 30, 0, true);
    view.setUint16(offset + 32, 0, true);
    view.setUint16(offset + 34, 0, true);
    view.setUint16(offset + 36, 0, true);
    view.setUint32(offset + 38, 0, true);
    view.setUint32(offset + 42, entry.localOffset, true);
    output.set(entry.name, offset + 46);
    offset += 46 + entry.name.length;
  }

  view.setUint32(offset, 0x06054b50, true);
  view.setUint16(offset + 4, 0, true);
  view.setUint16(offset + 6, 0, true);
  view.setUint16(offset + 8, encodedEntries.length, true);
  view.setUint16(offset + 10, encodedEntries.length, true);
  view.setUint32(offset + 12, centralSize, true);
  view.setUint32(offset + 16, centralOffset, true);
  view.setUint16(offset + 20, 0, true);
  return output;
}
