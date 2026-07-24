export interface PickleFloat32Array {
  data: Float32Array;
  shape: number[];
}

const NATIVE_LITTLE_ENDIAN =
  new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;
const PICKLE_CHUNK_SIZE = 1024 * 1024;

export function float32PickleArray(
  data: Float32Array,
  shape: number[],
): PickleFloat32Array {
  const expectedLength = shape.reduce((product, dimension) => product * dimension, 1);
  if (
    shape.some((dimension) => !Number.isInteger(dimension) || dimension < 0) ||
    expectedLength !== data.length
  ) {
    throw new Error(`Invalid float32 pickle array shape [${shape.join(', ')}].`);
  }
  return { data, shape };
}

function isFloat32ArrayValue(value: unknown): value is PickleFloat32Array {
  if (!value || typeof value !== 'object') {
    return false;
  }
  const candidate = value as Partial<PickleFloat32Array>;
  return candidate.data instanceof Float32Array && Array.isArray(candidate.shape);
}

class Protocol4PickleWriter {
  private readonly chunks: Uint8Array[] = [];
  private currentChunk = new Uint8Array(PICKLE_CHUNK_SIZE);
  private currentOffset = 0;
  private totalLength = 0;
  private readonly encoder = new TextEncoder();

  write(value: unknown): Uint8Array {
    const chunks = this.writeParts(value);
    const output = new Uint8Array(this.totalLength);
    let offset = 0;
    for (const chunk of chunks) {
      output.set(chunk, offset);
      offset += chunk.length;
    }
    return output;
  }

  writeParts(value: unknown): Uint8Array[] {
    this.byte(0x80);
    this.byte(0x04);
    this.writeValue(value);
    this.byte(0x2e);
    this.flushChunk();
    return this.chunks;
  }

  beginDictionary(): void {
    this.byte(0x80);
    this.byte(0x04);
    this.byte(0x7d);
    this.byte(0x28);
  }

  writeDictionaryEntry(key: string, value: unknown): void {
    this.unicode(key);
    this.writeValue(value);
  }

  finishDictionaryParts(): Uint8Array[] {
    this.byte(0x75);
    this.byte(0x2e);
    this.flushChunk();
    return this.chunks;
  }

  private flushChunk(): void {
    if (this.currentOffset === 0) {
      return;
    }
    const chunk =
      this.currentOffset === this.currentChunk.length
        ? this.currentChunk
        : this.currentChunk.slice(0, this.currentOffset);
    this.chunks.push(chunk);
    this.totalLength += chunk.length;
    this.currentChunk = new Uint8Array(PICKLE_CHUNK_SIZE);
    this.currentOffset = 0;
  }

  private byte(value: number): void {
    if (this.currentOffset >= this.currentChunk.length) {
      this.flushChunk();
    }
    this.currentChunk[this.currentOffset] = value & 0xff;
    this.currentOffset += 1;
  }

  private uint32(value: number): void {
    this.byte(value);
    this.byte(value >>> 8);
    this.byte(value >>> 16);
    this.byte(value >>> 24);
  }

  private int32(value: number): void {
    this.uint32(value >>> 0);
  }

  private raw(value: Uint8Array): void {
    let sourceOffset = 0;
    while (sourceOffset < value.length) {
      if (this.currentOffset >= this.currentChunk.length) {
        this.flushChunk();
      }
      const writable = Math.min(
        value.length - sourceOffset,
        this.currentChunk.length - this.currentOffset,
      );
      this.currentChunk.set(
        value.subarray(sourceOffset, sourceOffset + writable),
        this.currentOffset,
      );
      this.currentOffset += writable;
      sourceOffset += writable;
    }
  }

  private global(moduleName: string, name: string): void {
    this.byte(0x63);
    this.raw(this.encoder.encode(`${moduleName}\n${name}\n`));
  }

  private unicode(value: string): void {
    const encoded = this.encoder.encode(value);
    this.byte(0x58);
    this.uint32(encoded.length);
    this.raw(encoded);
  }

  private binary(value: Uint8Array): void {
    this.byte(0x42);
    this.uint32(value.length);
    this.raw(value);
  }

  private integer(value: number): void {
    if (value >= 0 && value <= 0xff) {
      this.byte(0x4b);
      this.byte(value);
      return;
    }
    this.byte(0x4a);
    this.int32(value);
  }

  private tuple(values: readonly number[]): void {
    if (values.length > 3) {
      this.byte(0x28);
      for (const value of values) {
        this.integer(value);
      }
      this.byte(0x74);
      return;
    }
    for (const value of values) {
      this.integer(value);
    }
    if (values.length === 0) {
      this.byte(0x29);
    } else if (values.length === 1) {
      this.byte(0x85);
    } else if (values.length === 2) {
      this.byte(0x86);
    } else if (values.length === 3) {
      this.byte(0x87);
    }
  }

  private float32Binary(values: Float32Array): void {
    this.byte(0x42);
    this.uint32(values.length * 4);
    if (NATIVE_LITTLE_ENDIAN) {
      this.raw(new Uint8Array(values.buffer, values.byteOffset, values.byteLength));
      return;
    }
    const valuesPerChunk = 16 * 1024;
    for (let start = 0; start < values.length; start += valuesPerChunk) {
      const count = Math.min(valuesPerChunk, values.length - start);
      const buffer = new ArrayBuffer(count * 4);
      const view = new DataView(buffer);
      for (let index = 0; index < count; index += 1) {
        view.setFloat32(index * 4, values[start + index], true);
      }
      this.raw(new Uint8Array(buffer));
    }
  }

  private writeFloat32Array(value: PickleFloat32Array): void {
    float32PickleArray(value.data, value.shape);
    this.global('numpy.core.multiarray', '_reconstruct');
    this.global('numpy', 'ndarray');
    this.integer(0);
    this.byte(0x85);
    this.binary(this.encoder.encode('b'));
    this.byte(0x87);
    this.byte(0x52);

    this.byte(0x28);
    this.integer(1);
    this.tuple(value.shape);
    this.global('numpy', 'dtype');
    this.unicode('f4');
    this.byte(0x89);
    this.byte(0x88);
    this.byte(0x87);
    this.byte(0x52);
    this.byte(0x28);
    this.integer(3);
    this.unicode('<');
    this.byte(0x4e);
    this.byte(0x4e);
    this.byte(0x4e);
    this.integer(-1);
    this.integer(-1);
    this.integer(0);
    this.byte(0x74);
    this.byte(0x62);
    this.byte(0x89);
    this.float32Binary(value.data);
    this.byte(0x74);
    this.byte(0x62);
  }

  private writeValue(value: unknown): void {
    if (isFloat32ArrayValue(value)) {
      this.writeFloat32Array(value);
      return;
    }
    if (value === null || value === undefined) {
      this.byte(0x4e);
      return;
    }
    if (typeof value === 'string') {
      this.unicode(value);
      return;
    }
    if (typeof value === 'boolean') {
      this.byte(value ? 0x88 : 0x89);
      return;
    }
    if (typeof value === 'number') {
      if (!Number.isFinite(value)) {
        throw new Error('Pickle export does not support non-finite numbers.');
      }
      this.byte(0x47);
      const buffer = new ArrayBuffer(8);
      new DataView(buffer).setFloat64(0, value, false);
      this.raw(new Uint8Array(buffer));
      return;
    }
    if (Array.isArray(value)) {
      this.byte(0x5d);
      if (value.length > 0) {
        this.byte(0x28);
        for (const item of value) {
          this.writeValue(item);
        }
        this.byte(0x65);
      }
      return;
    }
    if (typeof value === 'object') {
      this.byte(0x7d);
      const entries = Object.entries(value as Record<string, unknown>);
      if (entries.length > 0) {
        this.byte(0x28);
        for (const [key, item] of entries) {
          this.unicode(key);
          this.writeValue(item);
        }
        this.byte(0x75);
      }
      return;
    }
    throw new Error(`Unsupported pickle value type: ${typeof value}.`);
  }
}

export function writeNumpyPickle(value: unknown): Uint8Array {
  return new Protocol4PickleWriter().write(value);
}

export function writeNumpyPickleParts(value: unknown): Uint8Array[] {
  return new Protocol4PickleWriter().writeParts(value);
}

export class NumpyPickleDictionaryWriter {
  private readonly writer = new Protocol4PickleWriter();

  constructor() {
    this.writer.beginDictionary();
  }

  add(key: string, value: unknown): void {
    this.writer.writeDictionaryEntry(key, value);
  }

  finish(): Uint8Array[] {
    return this.writer.finishDictionaryParts();
  }
}
