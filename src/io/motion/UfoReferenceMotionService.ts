import type { DroppedFileMap, MotionClip } from '../../types/viewer';
import { DEFAULT_ROOT_COMPONENT_COUNT, DEFAULT_ROOT_JOINT_NAME } from './MotionSchema';
import { parseNpzFile, type ParsedNpyArray } from './NumpyIO';
import {
  UFO_POLICY_BODY_NAMES,
  UFO_POLICY_JOINT_NAMES,
} from './UfoReferenceNpzService';
import { DEX_EVT_JOINT_NAMES_29 } from './RobotTrackingLayouts';

export { DEX_EVT_JOINT_NAMES_29 } from './RobotTrackingLayouts';

const REQUIRED_KEYS = ['fps.npy', 'joint_pos.npy', 'body_pos_w.npy', 'body_quat_w.npy'] as const;

export interface UfoReferenceMotionLoadResult {
  clip: MotionClip;
  selectedMotionPath: string;
  warnings: string[];
}

function assertShape(array: ParsedNpyArray, expected: readonly (number | null)[], label: string): void {
  if (
    array.shape.length !== expected.length ||
    expected.some((dimension, index) => dimension !== null && array.shape[index] !== dimension)
  ) {
    throw new Error(
      `${label} must have shape [${expected.map((value) => value ?? 'T').join(', ')}], got [${array.shape.join(', ')}].`,
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

export class UfoReferenceMotionService {
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
        // Other NPZ families are handled by the existing SMPL scanner.
      }
    }
    return compatible;
  }

  async loadFromDroppedFiles(
    fileMap: DroppedFileMap,
    preferredPath?: string,
  ): Promise<UfoReferenceMotionLoadResult> {
    const compatiblePaths =
      preferredPath && fileMap.has(preferredPath)
        ? [preferredPath]
        : await this.findCompatiblePaths(fileMap, 1);
    const selectedMotionPath = compatiblePaths[0];
    if (!selectedMotionPath) {
      throw new Error('No compatible UFO reference NPZ was found.');
    }
    const file = fileMap.get(selectedMotionPath);
    if (!file) {
      throw new Error(`UFO reference NPZ is missing from the dropped files: ${selectedMotionPath}.`);
    }

    const archive = await parseNpzFile(file);
    const [fpsArray, jointPosArray, bodyPosArray, bodyQuatArray] = await Promise.all([
      archive.readNpy('fps.npy'),
      archive.readNpy('joint_pos.npy'),
      archive.readNpy('body_pos_w.npy'),
      archive.readNpy('body_quat_w.npy'),
    ]);
    assertShape(jointPosArray, [null, 29], 'joint_pos');
    const frameCount = jointPosArray.shape[0] ?? 0;
    const bodyCount = bodyPosArray.shape[1] ?? 0;
    if (bodyCount !== 30 && bodyCount !== 39) {
      throw new Error(`body_pos_w must contain 30 or 39 bodies, got ${bodyCount}.`);
    }
    assertShape(bodyPosArray, [frameCount, bodyCount, 3], 'body_pos_w');
    assertShape(bodyQuatArray, [frameCount, bodyCount, 4], 'body_quat_w');
    if (frameCount <= 0) {
      throw new Error('UFO reference NPZ contains no motion frames.');
    }

    const fpsValues = fpsArray.toNumberArray();
    const fps = fpsValues[0];
    if (!Number.isFinite(fps) || fps <= 0) {
      throw new Error(`UFO reference NPZ has invalid fps: ${fps}.`);
    }
    const jointPos = jointPosArray.toNumberArray();
    const bodyPos = bodyPosArray.toNumberArray();
    const bodyQuatWxyz = bodyQuatArray.toNumberArray();
    assertFinite(jointPos, 'joint_pos');
    assertFinite(bodyPos, 'body_pos_w');
    assertFinite(bodyQuatWxyz, 'body_quat_w');

    const jointNames =
      bodyCount === 39 ? [...DEX_EVT_JOINT_NAMES_29] : [...UFO_POLICY_JOINT_NAMES];
    const stride = DEFAULT_ROOT_COMPONENT_COUNT + jointNames.length;
    const data = new Float32Array(frameCount * stride);
    for (let frame = 0; frame < frameCount; frame += 1) {
      const targetBase = frame * stride;
      const rootPositionBase = frame * bodyCount * 3;
      data[targetBase] = bodyPos[rootPositionBase];
      data[targetBase + 1] = bodyPos[rootPositionBase + 1];
      data[targetBase + 2] = bodyPos[rootPositionBase + 2];

      const rootQuaternionBase = frame * bodyCount * 4;
      const w = bodyQuatWxyz[rootQuaternionBase];
      const x = bodyQuatWxyz[rootQuaternionBase + 1];
      const y = bodyQuatWxyz[rootQuaternionBase + 2];
      const z = bodyQuatWxyz[rootQuaternionBase + 3];
      const quaternionNorm = Math.hypot(w, x, y, z);
      if (quaternionNorm < 1e-8) {
        throw new Error(`body_quat_w contains an invalid pelvis quaternion at frame ${frame + 1}.`);
      }
      data[targetBase + 3] = x / quaternionNorm;
      data[targetBase + 4] = y / quaternionNorm;
      data[targetBase + 5] = z / quaternionNorm;
      data[targetBase + 6] = w / quaternionNorm;

      const jointSourceBase = frame * jointNames.length;
      for (let joint = 0; joint < jointNames.length; joint += 1) {
        data[targetBase + DEFAULT_ROOT_COMPONENT_COUNT + joint] =
          jointPos[jointSourceBase + joint];
      }
    }

    const warnings =
      bodyCount === 39
        ? ['Loaded DEX EVT 39-body UFO reference NPZ and mapped its 29 joints by canonical name.']
        : [`Loaded Tiangong3 ${UFO_POLICY_BODY_NAMES.length}-body UFO reference NPZ.`];
    const fileName = selectedMotionPath.split('/').pop() ?? selectedMotionPath;
    return {
      selectedMotionPath,
      warnings,
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
