import type { MotionCsvMode } from '../types/viewer';
import type { UrdfMotionKind } from './MotionFormatModes';

export interface MotionInfo {
  kind: UrdfMotionKind | 'bvh' | 'smpl';
  name: string;
  sourcePath: string;
  fps: number;
  frameCount: number;
  jointCount: number;
  sourceJointCount?: number;
  model?: string;
  linearUnit?: string;
  csvMode?: MotionCsvMode;
}

const FORMAT_NAMES: Record<MotionInfo['kind'], string> = {
  csv: 'CSV',
  mimickit: 'MimicKit',
  gmr: 'GMR',
  'ufo-reference': 'UFO Tracking (Reference)',
  'ufo-training': 'UFO Training (MotionLib)',
  'robot-state': 'Robot State',
  bvh: 'BVH',
  smpl: 'SMPL',
};

export function buildMotionInfoRows(info: MotionInfo): Array<[string, string]> {
  const extension = info.sourcePath.split('/').pop()?.match(/\.([^.]+)$/)?.[1].toUpperCase();
  const rows: Array<[string, string]> = [
    ['Motion', info.name],
    ['Format', FORMAT_NAMES[info.kind]],
    ['File type', extension ? `.${extension.toLowerCase()}` : 'In memory'],
    ['Playback rate', `${Number(info.fps.toFixed(3))} FPS (Hz)`],
    ['Frames', String(info.frameCount)],
    ['Duration (first to last frame)', `${(Math.max(0, info.frameCount - 1) / info.fps).toFixed(3)} s`],
  ];
  if (info.kind === 'bvh' || info.kind === 'smpl') {
    if (info.sourceJointCount !== undefined) {
      rows.push(['Pose joints in file', String(info.sourceJointCount)]);
    }
    rows.push(['Animated joints', String(info.jointCount)]);
  } else {
    if (info.sourceJointCount !== undefined) {
      rows.push(['Source joint DOF', `${info.sourceJointCount} DOF`]);
    }
    rows.push(['Playback joint DOF', `${info.jointCount} DOF (excluding floating base)`]);
  }
  if (info.model) rows.push(['Model', info.model]);
  if (info.linearUnit) rows.push(['BVH source unit', info.linearUnit]);
  if (info.kind === 'csv' && info.csvMode) {
    rows.push(['CSV layout', info.csvMode === 'header' ? 'Named columns' : 'Ordered columns']);
  }
  rows.push(['Source path', info.sourcePath]);
  return rows;
}
