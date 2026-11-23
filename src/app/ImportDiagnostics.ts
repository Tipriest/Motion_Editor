import type { MotionClip, ViewerState } from '../types/viewer';

export type ImportStatus = 'idle' | 'loading' | 'success' | 'error';
export interface ImportSummary {
  rows: Array<[string, string]>;
  warnings: string[];
}
export interface ImportReport extends ImportSummary {
  status: ImportStatus;
  source: string;
  errors: string[];
}

/** Stores the last import independently of playback and recoverable viewer errors. */
export class ImportDiagnostics {
  report: ImportReport = this.emptyReport();
  private outcome: 'success' | 'error' | null = null;
  private revision = 0;

  constructor(private readonly onChange: () => void = () => {}) {}

  private emptyReport(): ImportReport {
    return { status: 'idle', source: '', rows: [], warnings: [], errors: [] };
  }

  reset(): void {
    this.revision += 1;
    this.outcome = null;
    this.report = this.emptyReport();
    this.onChange();
  }

  setSource(source: string): void {
    if (this.report.status !== 'loading') return;
    this.report.source = source;
    this.onChange();
  }

  observeState(
    state: ViewerState,
    details: { title?: string; detail?: string; dropHint?: string; warnings?: string[] },
  ): void {
    if (this.report.status !== 'loading') return;
    if (state === 'error') {
      this.outcome = 'error';
      this.report.errors = [...new Set(
        [details.title, details.detail, details.dropHint].filter((value): value is string => Boolean(value)),
      )];
    } else if (state === 'model_ready' || state === 'playing') {
      this.outcome = 'success';
      this.report.errors = [];
      this.report.warnings = [...new Set(details.warnings ?? [])];
    }
  }

  async run(
    source: string,
    action: () => Promise<void>,
    summarize: () => ImportSummary,
  ): Promise<void> {
    // A second input must not overwrite an import that is still being parsed.
    if (this.report.status === 'loading') return;
    const revision = ++this.revision;
    this.outcome = null;
    this.report = { ...this.emptyReport(), status: 'loading', source };
    this.onChange();
    try {
      await action();
      if (revision !== this.revision) return;
      if (this.outcome === 'success') {
        const summary = summarize();
        this.report.rows = summary.rows.map(([label, value]) => [label, value]);
        this.report.warnings = [...new Set([...this.report.warnings, ...summary.warnings])];
        this.report.status = 'success';
      } else {
        this.report.status = this.outcome === 'error' ? 'error' : 'idle';
      }
    } catch (error) {
      if (revision !== this.revision) return;
      this.report.status = 'error';
      this.report.rows = [];
      this.report.errors.push(error instanceof Error ? error.message : String(error));
    }
    this.onChange();
  }
}

/** Checks the decoded robot data without changing the loaded motion. Frames are 1-based in messages. */
export function inspectMotionData(clip: MotionClip): ImportSummary {
  const rows: Array<[string, string]> = [
    ['fps', String(clip.fps)],
    ['frameCount', String(clip.frameCount)],
    ['data（解析后）', `[${clip.frameCount}, ${clip.stride}]`],
    ['joint positions（解析后）', `[${clip.frameCount}, ${clip.schema.jointNames.length}]`],
  ];
  if (clip.schema.rootComponentCount === 7) {
    rows.push(
      ['root position（解析后）', `[${clip.frameCount}, 3] · x, y, z`],
      ['root rotation（解析后）', `[${clip.frameCount}, 4] · x, y, z, w`],
    );
  }
  const warnings: string[] = [];
  if (!Number.isFinite(clip.fps) || clip.fps <= 0) warnings.push('fps 必须为大于 0 的有限数值。');
  if (!Number.isInteger(clip.frameCount) || clip.frameCount <= 0) warnings.push('frameCount 必须为大于 0 的整数。');
  const expectedStride = clip.schema.rootComponentCount + clip.schema.jointNames.length;
  if (clip.stride !== expectedStride || clip.data.length !== clip.frameCount * clip.stride) {
    warnings.push(`data 维度不匹配：每帧应有 ${expectedStride} 项，实际 stride=${clip.stride}；数据共 ${clip.data.length} 项。`);
  }
  let invalidCount = 0;
  let firstInvalid = -1;
  for (let index = 0; index < clip.data.length; index += 1) {
    if (!Number.isFinite(clip.data[index])) {
      invalidCount += 1;
      if (firstInvalid < 0) firstInvalid = index;
    }
  }
  if (invalidCount > 0) {
    const frame = Math.floor(firstInvalid / clip.stride) + 1;
    const column = firstInvalid % clip.stride;
    const field = ['root.x', 'root.y', 'root.z', 'root.qx', 'root.qy', 'root.qz', 'root.qw'][column] ??
      clip.schema.jointNames[column - clip.schema.rootComponentCount] ?? `column ${column + 1}`;
    warnings.push(`发现 ${invalidCount} 个 NaN/Infinity；首处位于第 ${frame} 帧，字段 ${field}（第 ${column + 1} 列）。`);
  }
  const names = clip.schema.jointNames;
  if (names.some((name) => !name.trim()) || new Set(names).size !== names.length) {
    warnings.push('关节名称包含空值或重复项，请确认关节映射。');
  }
  return { rows, warnings };
}
