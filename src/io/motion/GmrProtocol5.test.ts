import { existsSync, readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { GmrMotionService } from './GmrMotionService';
import { parsePickleNdarrayFloat64, parsePythonPickleBuffer } from './PythonPickleIO';
import { TIANGONG3_JOINT_NAMES_31 } from './RobotTrackingLayouts';

const schema = {
  rootJointName: 'floating_base_joint',
  rootComponentCount: 7,
  jointNames: [...TIANGONG3_JOINT_NAMES_31],
};

describe('GMR NumPy protocol 5 PKL', () => {
  // Fixtures preserve two frames from the reported MonkeySunIP sample and add
  // a column-major float32 array. Both NumPy module spellings occur in PKL.
  it.each(['gmr-protocol5.pkl', 'gmr-protocol5-numpy1.pkl'])(
    'imports NumPy protocol 5 arrays: %s',
    async (name) => {
      const bytes = readFileSync(new URL(`./fixtures/${name}`, import.meta.url));
      const file = new File([new Uint8Array(bytes)], name);
      const result = await new GmrMotionService().loadFromDroppedFiles(
        new Map([[name, file]]), schema, name,
      );
      expect(result.clip.frameCount).toBe(2);
      expect(result.clip.fps).toBe(120);
      expect(result.clip.sourceJointCount).toBe(31);
      expect(result.clip.stride).toBe(38);
      expect(result.clip.data[0]).toBeCloseTo(1.691706554);
      expect(result.clip.data[3]).toBeCloseTo(0.02834595087);
      expect(result.clip.data[6]).toBeCloseTo(0.9562460628);
      expect(result.clip.data[7]).toBeCloseTo(0.1172544952);
      expect(result.clip.data[38]).toBeCloseTo(1.691732884);
      expect(result.warnings.join(' ')).toContain('joint order could not be verified');

      const parsed = parsePythonPickleBuffer(await file.arrayBuffer()) as Record<string, unknown>;
      const columnMajor = parsePickleNdarrayFloat64(parsed.fortran_test, 'fortran_test');
      expect(columnMajor.shape).toEqual([2, 3]);
      expect(Array.from(columnMajor.values)).toEqual([1, 2, 3, 4, 5, 6]);
    },
  );

  const samplePath = new URL('../../../assets/motions/dataset/MonkeySunIP/11_25_highdynamic/其它动态动作/侧转/tak_002_1_Skeleton0dex_evt2_Refined_Denoised.pkl', import.meta.url);
  it.skipIf(!existsSync(samplePath))('imports all 1017 frames of the local sample', async () => {
    const file = new File([new Uint8Array(readFileSync(samplePath))], 'sample.pkl');
    const { clip } = await new GmrMotionService().loadFromDroppedFiles(
      new Map([[file.name, file]]), schema, file.name,
    );
    expect(clip.frameCount).toBe(1017);
    expect(clip.data.length).toBe(1017 * 38);
    expect(clip.data.every(Number.isFinite)).toBe(true);
    for (let frame = 0; frame < clip.frameCount; frame += 1) {
      expect(Math.hypot(...clip.data.slice(frame * 38 + 3, frame * 38 + 7))).toBeCloseTo(1);
    }
  });
});
