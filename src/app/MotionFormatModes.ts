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
  | 'ufo-npz';

export interface ExportFormatOption {
  value: ExportFormatValue;
  label: string;
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
  isTiangong3: boolean,
): ExportFormatOption[] {
  const options = [sourceFormat(kind)];
  if (!isTiangong3) {
    return options;
  }
  const tiangongFormats: ExportFormatOption[] = [
    { value: 'gmr-npz', label: 'GMR NPZ (root + 29-DOF)' },
    { value: 'ufo-pkl', label: 'UFO Training PKL (MotionLib)' },
    { value: 'ufo-npz', label: 'UFO Tracking NPZ (Reference)' },
  ];
  for (const option of tiangongFormats) {
    if (!options.some(({ value }) => value === option.value)) {
      options.push(option);
    }
  }
  return options;
}
