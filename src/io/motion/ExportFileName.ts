function sanitizeBaseName(motionName: string): string {
  const fileName = motionName.split(/[\\/]/).pop() ?? motionName;
  const withoutExtension = fileName.replace(/\.[^.]+$/, '');
  const sanitized = withoutExtension
    .replace(/[^a-zA-Z0-9_-]+/g, '_')
    .replace(/^_+|_+$/g, '');
  return sanitized || 'motion';
}

function formatSpeed(playbackSpeed: number): string {
  if (!Number.isFinite(playbackSpeed) || playbackSpeed <= 0) {
    throw new Error(`Export speed must be positive, received ${playbackSpeed}.`);
  }
  return `${Number(playbackSpeed.toFixed(4))}x`;
}

export function buildMotionExportFileName(
  motionName: string,
  playbackSpeed: number,
  formatSuffix: string | null,
  extension: string,
  options: { mirrored?: boolean } = {},
): string {
  const mirrorSuffix = options.mirrored ? '_mirror' : '';
  const suffix = formatSuffix ? `_${formatSuffix}` : '';
  return `${sanitizeBaseName(motionName)}_${formatSpeed(playbackSpeed)}${mirrorSuffix}${suffix}.${extension}`;
}
