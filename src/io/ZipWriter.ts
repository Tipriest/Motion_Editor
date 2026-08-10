export interface StoredZipEntry {
  name: string;
  data: Uint8Array;
}

const textEncoder = new TextEncoder();

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

export function writeStoredZip(entries: readonly StoredZipEntry[]): Uint8Array {
  if (entries.length === 0) {
    throw new Error('Cannot create an empty ZIP archive.');
  }
  if (entries.length > 0xffff) {
    throw new Error('ZIP archive contains too many files.');
  }

  const names = new Set<string>();
  const encoded = entries.map((entry) => {
    const normalizedName = entry.name.replace(/\\/g, '/').replace(/^\/+/, '');
    if (
      !normalizedName ||
      normalizedName.includes('\0') ||
      normalizedName.split('/').includes('..') ||
      names.has(normalizedName)
    ) {
      throw new Error(`Invalid or duplicate ZIP entry name: ${entry.name}`);
    }
    names.add(normalizedName);
    const name = textEncoder.encode(normalizedName);
    if (name.length > 0xffff || entry.data.length > 0xffffffff) {
      throw new Error(`ZIP entry is too large: ${normalizedName}`);
    }
    return {
      name,
      data: entry.data,
      crc: crc32(entry.data),
      localOffset: 0,
    };
  });

  let localSize = 0;
  for (const entry of encoded) {
    entry.localOffset = localSize;
    localSize += 30 + entry.name.length + entry.data.length;
  }
  const centralSize = encoded.reduce(
    (size, entry) => size + 46 + entry.name.length,
    0,
  );
  if (localSize + centralSize + 22 > 0xffffffff) {
    throw new Error('ZIP archive exceeds the classic ZIP size limit.');
  }

  const output = new Uint8Array(localSize + centralSize + 22);
  const view = new DataView(output.buffer);
  let offset = 0;
  for (const entry of encoded) {
    view.setUint32(offset, 0x04034b50, true);
    view.setUint16(offset + 4, 20, true);
    view.setUint16(offset + 6, 0x0800, true);
    view.setUint16(offset + 8, 0, true);
    view.setUint32(offset + 14, entry.crc, true);
    view.setUint32(offset + 18, entry.data.length, true);
    view.setUint32(offset + 22, entry.data.length, true);
    view.setUint16(offset + 26, entry.name.length, true);
    output.set(entry.name, offset + 30);
    output.set(entry.data, offset + 30 + entry.name.length);
    offset += 30 + entry.name.length + entry.data.length;
  }

  const centralOffset = offset;
  for (const entry of encoded) {
    view.setUint32(offset, 0x02014b50, true);
    view.setUint16(offset + 4, 20, true);
    view.setUint16(offset + 6, 20, true);
    view.setUint16(offset + 8, 0x0800, true);
    view.setUint16(offset + 10, 0, true);
    view.setUint32(offset + 16, entry.crc, true);
    view.setUint32(offset + 20, entry.data.length, true);
    view.setUint32(offset + 24, entry.data.length, true);
    view.setUint16(offset + 28, entry.name.length, true);
    view.setUint32(offset + 42, entry.localOffset, true);
    output.set(entry.name, offset + 46);
    offset += 46 + entry.name.length;
  }

  view.setUint32(offset, 0x06054b50, true);
  view.setUint16(offset + 8, encoded.length, true);
  view.setUint16(offset + 10, encoded.length, true);
  view.setUint32(offset + 12, centralSize, true);
  view.setUint32(offset + 16, centralOffset, true);
  return output;
}
