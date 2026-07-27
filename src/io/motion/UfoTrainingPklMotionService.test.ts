import { describe, expect, it, vi } from 'vitest';

import type { DroppedFileMap } from '../../types/viewer';
import {
  float32PickleArray,
  writeNumpyPickle,
} from './NumpyPickleWriter';
import { UFO_POLICY_JOINT_NAMES } from './UfoReferenceNpzService';
import { UfoTrainingPklMotionService } from './UfoTrainingPklMotionService';

function createRecord(jointNames = [...UFO_POLICY_JOINT_NAMES]) {
  const frameCount = 2;
  const jointCount = jointNames.length;
  const dofPosition = new Float32Array(frameCount * jointCount);
  dofPosition[0] = 0.25;
  dofPosition[jointCount] = 0.5;
  return {
    root_trans_offset: float32PickleArray(
      new Float32Array([1, 2, 3, 4, 5, 6]),
      [frameCount, 3],
    ),
    root_rot: float32PickleArray(
      new Float32Array([0, 0, 0, 2, 0, 0, 1, 0]),
      [frameCount, 4],
    ),
    dof_pos: float32PickleArray(dofPosition, [frameCount, jointCount]),
    fps: 50,
    joint_names: jointNames,
    motion_key: 'walk',
  };
}

function createFile(payload: unknown, name = 'training.pkl'): File {
  return new File([new Uint8Array(writeNumpyPickle(payload))], name);
}

describe('UfoTrainingPklMotionService', () => {
  it('loads a nested MotionLib record as an editable motion clip', async () => {
    const file = createFile({ walk: createRecord() });
    const fileMap: DroppedFileMap = new Map([[file.name, file]]);
    const result = await new UfoTrainingPklMotionService().loadFromDroppedFiles(
      fileMap,
    );

    expect(result.selectedMotionKey).toBe('walk');
    expect(result.clip.fps).toBe(50);
    expect(result.clip.frameCount).toBe(2);
    expect(result.clip.schema.jointNames).toEqual(UFO_POLICY_JOINT_NAMES);
    expect(Array.from(result.clip.data.slice(0, 8))).toEqual([
      1,
      2,
      3,
      0,
      0,
      0,
      1,
      0.25,
    ]);
    expect(
      Array.from(
        result.clip.data.slice(
          result.clip.stride,
          result.clip.stride + 7,
        ),
      ),
    ).toEqual([4, 5, 6, 0, 0, 1, 0]);
  });

  it('selects a deterministic record from a batch PKL and reports all keys', async () => {
    const file = createFile({
      z_motion: createRecord(),
      a_motion: createRecord(),
    });
    const fileMap: DroppedFileMap = new Map([[file.name, file]]);
    const service = new UfoTrainingPklMotionService();
    const arrayBufferSpy = vi.spyOn(file, 'arrayBuffer');
    const result = await service.loadFromDroppedFiles(fileMap);
    const second = await service.loadFromDroppedFiles(
      fileMap,
      file.name,
      'a_motion',
    );
    const restoredFirst = await service.loadFromDroppedFiles(
      fileMap,
      file.name,
      'z_motion',
    );

    expect(await service.findCompatiblePaths(fileMap)).toEqual([file.name]);
    expect(result.selectedMotionKey).toBe('z_motion');
    expect(second.selectedMotionKey).toBe('a_motion');
    expect(second.clip).not.toBe(result.clip);
    expect(restoredFirst.clip).toBe(result.clip);
    expect(result.availableMotionKeys).toEqual(['z_motion', 'a_motion']);
    expect(arrayBufferSpy).toHaveBeenCalledTimes(1);
    expect(result.warnings.join(' ')).toContain('contains 2 motions');
  });

  it('rejects flat non-UFO PKL dictionaries', async () => {
    const file = createFile({
      fps: 50,
      root_pos: [[0, 0, 0]],
      root_rot: [[0, 0, 0, 1]],
      dof_pos: [[0]],
    });
    const fileMap: DroppedFileMap = new Map([[file.name, file]]);

    expect(
      await new UfoTrainingPklMotionService().findCompatiblePaths(fileMap),
    ).toEqual([]);
  });
});
