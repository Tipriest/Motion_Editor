import { describe, expect, it } from 'vitest';
import type { DroppedFileMap } from '../../types/viewer';
import { writeFloat32Npz } from './NpzWriter';
import { RobotStateNpzMotionService } from './RobotStateNpzMotionService';

function createRobotStateFile(includeRoot = true): File {
  const frameCount = 2;
  const entries: Parameters<typeof writeFloat32Npz>[0] = {
    fps: { data: new Float32Array([100]), shape: [] },
    dof_pos: { data: new Float32Array(frameCount * 29), shape: [frameCount, 29] },
    dof_vel: { data: new Float32Array(frameCount * 29), shape: [frameCount, 29] },
    endpoint_pos_BCS: {
      data: new Float32Array(frameCount * 4 * 3),
      shape: [frameCount, 4, 3],
    },
  };
  if (includeRoot) {
    entries.root_pos = { data: new Float32Array([1, 2, 3, 4, 5, 6]), shape: [2, 3] };
    entries.root_rot = {
      data: new Float32Array([0, 0, 0, 1, 0, 0, 1, 0]),
      shape: [2, 4],
    };
  }
  const bytes = writeFloat32Npz(entries);
  return new File([new Uint8Array(bytes)], includeRoot ? 'slow_walk.npz' : 'run.npz');
}

describe('RobotStateNpzMotionService', () => {
  it('loads complete root and 29-DOF trajectories', async () => {
    const file = createRobotStateFile();
    const fileMap: DroppedFileMap = new Map([[file.name, file]]);
    const result = await new RobotStateNpzMotionService().loadFromDroppedFiles(fileMap);

    expect(result.clip.fps).toBe(100);
    expect(result.clip.frameCount).toBe(2);
    expect(result.clip.schema.jointNames).toHaveLength(29);
    expect(Array.from(result.clip.data.slice(0, 7))).toEqual([1, 2, 3, 0, 0, 0, 1]);
    expect(Array.from(result.clip.data.slice(result.clip.stride, result.clip.stride + 7))).toEqual([
      4,
      5,
      6,
      0,
      0,
      1,
      0,
    ]);
  });

  it('does not accept joint-only trajectories without root pose', async () => {
    const file = createRobotStateFile(false);
    const fileMap: DroppedFileMap = new Map([[file.name, file]]);
    expect(await new RobotStateNpzMotionService().findCompatiblePaths(fileMap)).toEqual([]);
  });
});
