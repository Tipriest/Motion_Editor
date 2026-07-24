export interface PickleFloat32Array {
  data: Float32Array;
  shape: number[];
}

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
  private readonly bytes: number[] = [0x80, 0x04];
  private readonly encoder = new TextEncoder();

  write(value: unknown): Uint8Array {
    this.writeValue(value);
    this.byte(0x2e);
    return new Uint8Array(this.bytes);
  }

  private byte(value: number): void {
    this.bytes.push(value & 0xff);
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
    for (const byte of value) {
      this.byte(byte);
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

  private float32Bytes(values: Float32Array): Uint8Array {
    const output = new Uint8Array(values.length * 4);
    const view = new DataView(output.buffer);
    for (let index = 0; index < values.length; index += 1) {
      view.setFloat32(index * 4, values[index], true);
    }
    return output;
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
    this.binary(this.float32Bytes(value.data));
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
