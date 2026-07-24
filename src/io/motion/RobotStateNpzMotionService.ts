import type { DroppedFileMap, MotionClip } from '../../types/viewer';
import { DEFAULT_ROOT_COMPONENT_COUNT, DEFAULT_ROOT_JOINT_NAME } from './MotionSchema';
import { parseNpzFile, type ParsedNpyArray } from './NumpyIO';
import { UFO_POLICY_JOINT_NAMES } from './UfoReferenceNpzService';

const REQUIRED_KEYS = ['fps.npy', 'root_pos.npy', 'root_rot.npy', 'dof_pos.npy'] as const;

export interface RobotStateNpzMotionLoadResult {
  clip: MotionClip;
  selectedMotionPath: string;
  warnings: string[];
}

function assertShape(array: ParsedNpyArray, expected: readonly number[], label: string): void {
  if (
    array.shape.length !== expected.length ||
    expected.some((dimension, index) => array.shape[index] !== dimension)
  ) {
    throw new Error(`${label} must have shape [${expected.join(', ')}], got [${array.shape.join(', ')}].`);
  }
}

function assertFinite(values: Float64Array, label: string): void {
  for (let index = 0; index < values.length; index += 1) {
    if (!Number.isFinite(values[index])) {
      throw new Error(`${label} contains a non-finite value at index ${index}.`);
    }
  }
}

export class RobotStateNpzMotionService {
  async findCompatiblePaths(
    fileMap: DroppedFileMap,
    maxResults = Number.POSITIVE_INFINITY,
  ): Promise<string[]> {
    const compatible: string[] = [];
    const paths = [...fileMap.keys()]
      .filter((path) => path.toLowerCase().endsWith('.npz'))
      .sort((left, right) => left.localeCompare(right));
    for (const path of paths) {
      const file = fileMap.get(path);
      if (!file) {
        continue;
      }
      try {
        const archive = await parseNpzFile(file);
        if (REQUIRED_KEYS.every((key) => archive.hasFile(key))) {
          compatible.push(path);
          if (compatible.length >= maxResults) {
            break;
          }
        }
      } catch {
        // Other NPZ formats are handled by their own loaders.
      }
    }
    return compatible;
  }

  async loadFromDroppedFiles(
    fileMap: DroppedFileMap,
    preferredPath?: string,
  ): Promise<RobotStateNpzMotionLoadResult> {
    const compatiblePaths =
      preferredPath && fileMap.has(preferredPath)
        ? [preferredPath]
        : await this.findCompatiblePaths(fileMap, 1);
    const selectedMotionPath = compatiblePaths[0];
    if (!selectedMotionPath) {
      throw new Error('No complete robot-state NPZ was found.');
    }
    const file = fileMap.get(selectedMotionPath);
    if (!file) {
      throw new Error(`Robot-state NPZ is missing: ${selectedMotionPath}.`);
    }

    const archive = await parseNpzFile(file);
    const [fpsArray, rootPosArray, rootRotArray, dofPosArray] = await Promise.all([
      archive.readNpy('fps.npy'),
      archive.readNpy('root_pos.npy'),
      archive.readNpy('root_rot.npy'),
      archive.readNpy('dof_pos.npy'),
    ]);
    const frameCount = rootPosArray.shape[0] ?? 0;
    if (frameCount <= 0) {
      throw new Error('Robot-state NPZ contains no frames.');
    }
    assertShape(rootPosArray, [frameCount, 3], 'root_pos');
    assertShape(rootRotArray, [frameCount, 4], 'root_rot');
    assertShape(dofPosArray, [frameCount, UFO_POLICY_JOINT_NAMES.length], 'dof_pos');

    const fps = fpsArray.toNumberArray()[0];
    if (!Number.isFinite(fps) || fps <= 0) {
      throw new Error(`Robot-state NPZ has invalid fps: ${fps}.`);
    }
    const rootPos = rootPosArray.toNumberArray();
    const rootRotXyzw = rootRotArray.toNumberArray();
    const dofPos = dofPosArray.toNumberArray();
    assertFinite(rootPos, 'root_pos');
    assertFinite(rootRotXyzw, 'root_rot');
    assertFinite(dofPos, 'dof_pos');

    const jointNames = [...UFO_POLICY_JOINT_NAMES];
    const stride = DEFAULT_ROOT_COMPONENT_COUNT + jointNames.length;
    const data = new Float32Array(frameCount * stride);
    for (let frame = 0; frame < frameCount; frame += 1) {
      const targetBase = frame * stride;
      data.set(rootPos.subarray(frame * 3, frame * 3 + 3), targetBase);
      const quaternionBase = frame * 4;
      const x = rootRotXyzw[quaternionBase];
      const y = rootRotXyzw[quaternionBase + 1];
      const z = rootRotXyzw[quaternionBase + 2];
      const w = rootRotXyzw[quaternionBase + 3];
      const norm = Math.hypot(x, y, z, w);
      if (norm < 1e-8) {
        throw new Error(`root_rot contains an invalid quaternion at frame ${frame + 1}.`);
      }
      data[targetBase + 3] = x / norm;
      data[targetBase + 4] = y / norm;
      data[targetBase + 5] = z / norm;
      data[targetBase + 6] = w / norm;
      for (let joint = 0; joint < jointNames.length; joint += 1) {
        data[targetBase + DEFAULT_ROOT_COMPONENT_COUNT + joint] =
          dofPos[frame * jointNames.length + joint];
      }
    }

    const fileName = selectedMotionPath.split('/').pop() ?? selectedMotionPath;
    return {
      selectedMotionPath,
      warnings: [
        'Loaded complete 29-DOF robot-state NPZ using root_rot quaternion order XYZW.',
      ],
      clip: {
        name: fileName,
        sourcePath: selectedMotionPath,
        fps,
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
      },
    };
  }
}
