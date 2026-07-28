import { describe, expect, it } from 'vitest';

import type { DroppedFileMap, MotionClip } from '../../types/viewer';
import { RobotStateNpzMotionService } from './RobotStateNpzMotionService';
import { exportGmrNpz } from './GmrNpzService';
import { DEX_EVT_JOINT_NAMES_29 } from './RobotTrackingLayouts';
import { UFO_POLICY_JOINT_NAMES } from './UfoReferenceNpzService';

describe('GMR NPZ import and export modes', () => {
  it('exports a file that the Robot-State import mode recognizes and loads', async () => {
    const jointNames = [...DEX_EVT_JOINT_NAMES_29];
    const stride = 7 + jointNames.length;
    const data = new Float32Array(2 * stride);
    data.set([1, 2, 3, 0, 0, 0, 1], 0);
    data.set([4, 5, 6, 0, 0, 1, 0], stride);
    for (let frame = 0; frame < 2; frame += 1) {
      for (let joint = 0; joint < jointNames.length; joint += 1) {
        data[frame * stride + 7 + joint] = frame * 100 + joint;
      }
    }
    const clip: MotionClip = {
      name: 'gmr',
      sourcePath: 'gmr.pkl',
      fps: 50,
      frameCount: 2,
      stride,
      schema: {
        rootJointName: 'root',
        rootComponentCount: 7,
        jointNames,
      },
      csvMode: 'ordered',
      sourceColumnCount: stride,
      data,
    };
    const file = new File([new Uint8Array(exportGmrNpz(clip))], 'gmr.npz');
    const fileMap: DroppedFileMap = new Map([[file.name, file]]);
    const importer = new RobotStateNpzMotionService();

    expect(await importer.findCompatiblePaths(fileMap)).toEqual(['gmr.npz']);
    const loaded = await importer.loadFromDroppedFiles(fileMap);

    expect(loaded.clip.fps).toBe(50);
    expect(Array.from(loaded.clip.data.slice(0, 7))).toEqual([
      1,
      2,
      3,
      0,
      0,
      0,
      1,
    ]);
    expect(loaded.clip.data[7]).toBe(
      jointNames.indexOf(UFO_POLICY_JOINT_NAMES[0]),
    );
    expect(loaded.clip.data[loaded.clip.stride + 7]).toBe(
      100 + jointNames.indexOf(UFO_POLICY_JOINT_NAMES[0]),
    );
  });
});
