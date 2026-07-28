export type UrdfMotionKind =
  | 'csv'
  | 'mimickit'
  | 'gmr'
  | 'ufo-reference'
  | 'ufo-training'
  | 'robot-state';

export type ExportFormatValue =
  | 'source'
  | 'gmr-npz'
  | 'ufo-pkl'
  | 'ufo-npz'
  | 'dex-mosaic-npz';

export interface ExportFormatOption {
  value: ExportFormatValue;
  label: string;
}

export type RobotExportTarget = 'tiangong3' | 'dex-evt' | 'other';

export function detectRobotExportTarget(
  robotName?: string | null,
  urdfPath?: string | null,
): RobotExportTarget {
  const identity = `${robotName ?? ''} ${urdfPath ?? ''}`.toLowerCase();
  if (
    identity.includes('dex_evt') ||
    identity.includes('evt2') ||
    identity.includes('tiangong2dex')
  ) {
    return 'dex-evt';
  }
  if (identity.includes('tiangong3')) {
    return 'tiangong3';
  }
  return 'other';
}

function sourceFormat(kind: UrdfMotionKind): ExportFormatOption {
  switch (kind) {
    case 'csv':
      return { value: 'source', label: 'CSV' };
    case 'mimickit':
      return { value: 'source', label: 'MimicKit PKL' };
    case 'ufo-reference':
      return { value: 'ufo-npz', label: 'UFO Tracking NPZ (Reference)' };
    case 'ufo-training':
      return { value: 'ufo-pkl', label: 'UFO Training PKL (MotionLib)' };
    case 'robot-state':
      return { value: 'gmr-npz', label: 'GMR NPZ (root + 29-DOF)' };
    case 'gmr':
      return { value: 'source', label: 'GMR PKL' };
  }
}

export function getExportFormatOptions(
  kind: UrdfMotionKind,
  target: RobotExportTarget,
): ExportFormatOption[] {
  const options = [sourceFormat(kind)];
  if (target === 'other') {
    return options;
  }
  const robotFormats: ExportFormatOption[] = [
    { value: 'gmr-npz', label: 'GMR NPZ (root + 29-DOF)' },
    { value: 'ufo-npz', label: 'UFO Tracking NPZ (Reference)' },
  ];
  if (target === 'tiangong3') {
    robotFormats.splice(1, 0, {
      value: 'ufo-pkl',
      label: 'UFO Training PKL (MotionLib)',
    });
    robotFormats.push({
      value: 'dex-mosaic-npz',
      label: 'DEX_MOSAIC NPZ (39-body)',
    });
  }
  for (const option of robotFormats) {
    if (!options.some(({ value }) => value === option.value)) {
      options.push(option);
    }
  }
  return options;
}
