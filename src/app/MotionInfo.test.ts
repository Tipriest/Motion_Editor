import { describe, expect, it } from 'vitest';
import { buildMotionInfoRows, type MotionInfo } from './MotionInfo';

const motion: MotionInfo = {
  kind: 'gmr',
  name: 'left_punch',
  sourcePath: 'motions/left_punch.pkl',
  fps: 50,
  frameCount: 358,
  jointCount: 31,
  sourceJointCount: 29,
};

describe('motion information', () => {
  it('distinguishes the source DOF from the mapped playback DOF', () => {
    const rows = Object.fromEntries(buildMotionInfoRows(motion));
    expect(rows['Source joint DOF']).toBe('29 DOF');
    expect(rows['Playback joint DOF']).toBe('31 DOF (excluding floating base)');
    expect(rows['Playback rate']).toBe('50 FPS (Hz)');
    expect(rows['Duration (first to last frame)']).toBe('7.140 s');
  });

  it('identifies the motion family separately from its file extension', () => {
    const gmr = Object.fromEntries(buildMotionInfoRows(motion));
    const mimic = Object.fromEntries(buildMotionInfoRows({ ...motion, kind: 'mimickit' }));
    expect(gmr.Format).toBe('GMR');
    expect(mimic.Format).toBe('MimicKit');
    expect(gmr['File type']).toBe(mimic['File type']);
    expect(Object.fromEntries(buildMotionInfoRows({ ...motion, sourcePath: 'new_motion' }))['File type'])
      .toBe('In memory');
  });

  it('shows animated joints for skeleton motions without claiming robot DOF', () => {
    const rows = Object.fromEntries(buildMotionInfoRows({
      ...motion, kind: 'smpl', sourceJointCount: 52, jointCount: 24,
    }));
    expect(rows['Pose joints in file']).toBe('52');
    expect(rows['Animated joints']).toBe('24');
    expect(rows['Playback joint DOF']).toBeUndefined();
    const bvh = Object.fromEntries(buildMotionInfoRows({
      ...motion, kind: 'bvh', sourceJointCount: undefined, jointCount: 22, linearUnit: 'cm',
    }));
    expect(bvh['BVH source unit']).toBe('cm');
    expect(bvh['Animated joints']).toBe('22');
  });

  it('handles single-frame CSV clips without inventing a source DOF', () => {
    const rows = Object.fromEntries(buildMotionInfoRows({
      ...motion, kind: 'csv', sourceJointCount: undefined, frameCount: 1, csvMode: 'header',
    }));
    expect(rows['Source joint DOF']).toBeUndefined();
    expect(rows['Duration (first to last frame)']).toBe('0.000 s');
    expect(rows['CSV layout']).toBe('Named columns');
  });
});
