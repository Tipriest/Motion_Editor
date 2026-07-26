import type { DroppedFileMap, MotionClip } from '../../types/viewer';
import { getBaseName, normalizePath } from '../urdf/pathResolver';
import { DEFAULT_ROOT_COMPONENT_COUNT, DEFAULT_ROOT_JOINT_NAME } from './MotionSchema';
import {
  parsePickleNdarrayFloat64,
  parsePythonPickleBuffer,
} from './PythonPickleIO';
import { UFO_POLICY_JOINT_NAMES } from './UfoReferenceNpzService';

const REQUIRED_RECORD_KEYS = [
  'root_trans_offset',
  'root_rot',
  'dof_pos',
  'fps',
] as const;

interface UfoTrainingRecordSelection {
  motionKey: string;
  record: Record<string, unknown>;
  availableMotionKeys: string[];
}

export interface UfoTrainingPklMotionLoadResult {
  clip: MotionClip;
  selectedMotionPath: string;
  selectedMotionKey: string;
  availableMotionKeys: string[];
  warnings: string[];
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function isUfoTrainingRecord(value: unknown): value is Record<string, unknown> {
  return (
    isRecord(value) &&
    REQUIRED_RECORD_KEYS.every((key) => value[key] !== undefined)
  );
}

function selectRecord(
  parsed: unknown,
  preferredMotionKey?: string,
): UfoTrainingRecordSelection {
  if (!isRecord(parsed)) {
    throw new Error('UFO Training PKL root object must be a dictionary.');
  }
  if (isUfoTrainingRecord(parsed)) {
    const motionKey =
      typeof parsed.motion_key === 'string' && parsed.motion_key.trim()
        ? parsed.motion_key.trim()
        : preferredMotionKey || 'motion';
    return {
      motionKey,
      record: parsed,
      availableMotionKeys: [motionKey],
    };
  }
  const entries = Object.entries(parsed)
    .filter((entry): entry is [string, Record<string, unknown>] =>
      isUfoTrainingRecord(entry[1]),
    )
    .sort(([left], [right]) => left.localeCompare(right));
  if (entries.length === 0) {
    throw new Error(
      'PKL does not contain a UFO Training MotionLib record with root_trans_offset, root_rot, dof_pos, and fps.',
    );
  }
  const selected =
    (preferredMotionKey
      ? entries.find(([motionKey]) => motionKey === preferredMotionKey)
      : undefined) ?? entries[0];
  return {
    motionKey: selected[0],
    record: selected[1],
    availableMotionKeys: entries.map(([motionKey]) => motionKey),
  };
}

function parseFps(value: unknown): number {
  let fps = Number(value);
  if (!Number.isFinite(fps)) {
    try {
      const array = parsePickleNdarrayFloat64(value, 'fps');
      fps = array.values[0];
    } catch {
      // The validation below provides the stable public error.
    }
  }
  if (!Number.isFinite(fps) || fps <= 0) {
    throw new Error(`UFO Training PKL has invalid fps: ${String(value)}.`);
  }
  return fps;
}

function parseJointNames(value: unknown, jointCount: number): string[] {
  if (value === undefined || value === null) {
    if (jointCount !== UFO_POLICY_JOINT_NAMES.length) {
      throw new Error(
        `UFO Training PKL has ${jointCount} DOF columns and no joint_names metadata.`,
      );
    }
    return [...UFO_POLICY_JOINT_NAMES];
  }
  if (!Array.isArray(value)) {
    throw new Error('UFO Training PKL joint_names must be an array.');
  }
  const jointNames = value.map((item) => String(item ?? '').trim());
  if (
    jointNames.length !== jointCount ||
    jointNames.some((name) => !name) ||
    new Set(jointNames).size !== jointNames.length
  ) {
    throw new Error(
      `UFO Training PKL joint_names must contain ${jointCount} unique non-empty names.`,
    );
  }
  return jointNames;
}

function assertShape(
  shape: readonly number[],
  expected: readonly number[],
  label: string,
): void {
  if (
    shape.length !== expected.length ||
    expected.some((size, index) => shape[index] !== size)
  ) {
    throw new Error(
      `${label} must have shape [${expected.join(', ')}], got [${shape.join(', ')}].`,
    );
  }
}

function assertFinite(values: Float64Array, label: string): void {
  for (let index = 0; index < values.length; index += 1) {
    if (!Number.isFinite(values[index])) {
      throw new Error(`${label} contains a non-finite value at index ${index}.`);
    }
  }
}

function buildClip(
  selection: UfoTrainingRecordSelection,
  sourcePath: string,
): MotionClip {
  const rootPosition = parsePickleNdarrayFloat64(
    selection.record.root_trans_offset,
    'root_trans_offset',
  );
  const rootRotation = parsePickleNdarrayFloat64(
    selection.record.root_rot,
    'root_rot',
  );
  const jointPosition = parsePickleNdarrayFloat64(
    selection.record.dof_pos,
    'dof_pos',
  );
  const frameCount = rootPosition.shape[0] ?? 0;
  if (frameCount <= 0) {
    throw new Error('UFO Training PKL motion contains no frames.');
  }
  if (jointPosition.shape.length !== 2 || jointPosition.shape[0] !== frameCount) {
    throw new Error(
      `dof_pos must have shape [${frameCount}, jointCount], got [${jointPosition.shape.join(', ')}].`,
    );
  }
  const jointCount = jointPosition.shape[1] ?? 0;
  assertShape(rootPosition.shape, [frameCount, 3], 'root_trans_offset');
  assertShape(rootRotation.shape, [frameCount, 4], 'root_rot');
  const jointNames = parseJointNames(selection.record.joint_names, jointCount);
  assertFinite(rootPosition.values, 'root_trans_offset');
  assertFinite(rootRotation.values, 'root_rot');
  assertFinite(jointPosition.values, 'dof_pos');

  const stride = DEFAULT_ROOT_COMPONENT_COUNT + jointCount;
  const data = new Float32Array(frameCount * stride);
  for (let frame = 0; frame < frameCount; frame += 1) {
    const targetBase = frame * stride;
    data.set(rootPosition.values.subarray(frame * 3, frame * 3 + 3), targetBase);
    const quaternionBase = frame * 4;
    const x = rootRotation.values[quaternionBase];
    const y = rootRotation.values[quaternionBase + 1];
    const z = rootRotation.values[quaternionBase + 2];
    const w = rootRotation.values[quaternionBase + 3];
    const norm = Math.hypot(x, y, z, w);
    if (norm < 1e-8) {
      throw new Error(`root_rot contains an invalid quaternion at frame ${frame + 1}.`);
    }
    data[targetBase + 3] = x / norm;
    data[targetBase + 4] = y / norm;
    data[targetBase + 5] = z / norm;
    data[targetBase + 6] = w / norm;
    data.set(
      jointPosition.values.subarray(
        frame * jointCount,
        frame * jointCount + jointCount,
      ),
      targetBase + DEFAULT_ROOT_COMPONENT_COUNT,
    );
  }

  return {
    name: selection.motionKey,
    sourcePath,
    fps: parseFps(selection.record.fps),
    frameCount,
    stride,
    schema: {
      rootJointName: DEFAULT_ROOT_JOINT_NAME,
      rootComponentCount: DEFAULT_ROOT_COMPONENT_COUNT,
      jointNames,
    },
    csvMode: 'ordered',
    sourceColumnCount: stride,
    data,
  };
}

async function parseFile(
  file: File,
  preferredMotionKey?: string,
): Promise<UfoTrainingRecordSelection> {
  return selectRecord(
    parsePythonPickleBuffer(await file.arrayBuffer()),
    preferredMotionKey,
  );
}

export class UfoTrainingPklMotionService {
  async findCompatiblePaths(
    fileMap: DroppedFileMap,
    maxResults = Number.POSITIVE_INFINITY,
  ): Promise<string[]> {
    const compatible: string[] = [];
    const paths = [...fileMap.keys()]
      .map((path) => normalizePath(path))
      .filter((path) => path.toLowerCase().endsWith('.pkl'))
      .sort((left, right) => left.localeCompare(right));
    for (const path of paths) {
      const file = fileMap.get(path);
      if (!file) {
        continue;
      }
      try {
        await parseFile(file);
        compatible.push(path);
        if (compatible.length >= maxResults) {
          break;
        }
      } catch {
        // Other PKL formats are handled by their own loaders.
      }
    }
    return compatible;
  }

  async loadFromDroppedFiles(
    fileMap: DroppedFileMap,
    preferredPath?: string,
    preferredMotionKey?: string,
  ): Promise<UfoTrainingPklMotionLoadResult> {
    const selectedMotionPath =
      preferredPath && fileMap.has(normalizePath(preferredPath))
        ? normalizePath(preferredPath)
        : (await this.findCompatiblePaths(fileMap, 1))[0];
    if (!selectedMotionPath) {
      throw new Error('No UFO Training MotionLib PKL was found.');
    }
    const file = fileMap.get(selectedMotionPath);
    if (!file) {
      throw new Error(`UFO Training PKL is missing: ${selectedMotionPath}.`);
    }
    const selection = await parseFile(file, preferredMotionKey);
    const warnings = [
      'Loaded UFO Training MotionLib PKL using root_rot quaternion order XYZW.',
    ];
    if (selection.availableMotionKeys.length > 1) {
      warnings.push(
        `PKL contains ${selection.availableMotionKeys.length} motions; loaded "${selection.motionKey}".`,
      );
    }
    const clip = buildClip(selection, selectedMotionPath);
    if (!clip.name) {
      clip.name = getBaseName(selectedMotionPath) || 'motion.pkl';
    }
    return {
      clip,
      selectedMotionPath,
      selectedMotionKey: selection.motionKey,
      availableMotionKeys: selection.availableMotionKeys,
      warnings,
    };
  }
}
