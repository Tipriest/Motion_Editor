import { describe, expect, it, vi } from 'vitest';
import type { MotionClip } from '../types/viewer';
import { ImportDiagnostics, inspectMotionData } from './ImportDiagnostics';

const summary = { rows: [['fps', '50']] as Array<[string, string]>, warnings: ['fps defaulted'] };

describe('import report lifetime', () => {
  it('records success, deduplicates warnings, and freezes the imported metadata', async () => {
    const diagnostics = new ImportDiagnostics();
    const rows: Array<[string, string]> = [['fps', '50']];
    await diagnostics.run('motion.pkl', async () => {
      diagnostics.observeState('loading', {});
      diagnostics.observeState('playing', { warnings: ['fps defaulted'] });
    }, () => ({ rows, warnings: ['fps defaulted'] }));
    rows[0][1] = '100';
    expect(diagnostics.report).toMatchObject({
      status: 'success', source: 'motion.pkl', rows: [['fps', '50']], warnings: ['fps defaulted'], errors: [],
    });
    diagnostics.observeState('model_ready', { warnings: ['later playback warning'] });
    diagnostics.observeState('error', { detail: 'unrelated editing error' });
    expect(diagnostics.report.status).toBe('success');
    expect(diagnostics.report.warnings).toEqual(['fps defaulted']);
  });

  it('preserves a failed import after the viewer recovers to its previous motion', async () => {
    const diagnostics = new ImportDiagnostics();
    const summarize = vi.fn(() => summary);
    await diagnostics.run('broken.pkl', async () => {
      diagnostics.observeState('error', { title: 'Load Failed', detail: 'missing root_pos', dropHint: 'Check fields' });
    }, summarize);
    diagnostics.observeState('playing', {});
    diagnostics.observeState('drag_over', {});
    expect(diagnostics.report.status).toBe('error');
    expect(diagnostics.report.errors).toContain('missing root_pos');
    expect(diagnostics.report.rows).toEqual([]);
    expect(summarize).not.toHaveBeenCalled();
  });

  it('replaces old errors and warnings on the next import', async () => {
    const diagnostics = new ImportDiagnostics();
    await diagnostics.run('bad.pkl', async () => { throw new Error('missing fps'); }, () => summary);
    expect(diagnostics.report.errors).toEqual(['missing fps']);
    await diagnostics.run('good.csv', async () => {
      diagnostics.observeState('playing', {});
    }, () => ({ rows: [], warnings: [] }));
    expect(diagnostics.report).toMatchObject({ status: 'success', source: 'good.csv', warnings: [], errors: [] });
    await diagnostics.run('bad.csv', async () => {
      diagnostics.observeState('error', { detail: 'missing joint_a' });
    }, () => summary);
    expect(diagnostics.report).toMatchObject({ status: 'error', rows: [], warnings: [], errors: ['missing joint_a'] });
  });

  it('does not report success when selecting a mode without importing a file', async () => {
    const diagnostics = new ImportDiagnostics();
    await diagnostics.run('BVH preview', async () => {
      diagnostics.observeState('idle', {});
    }, () => summary);
    expect(diagnostics.report.status).toBe('idle');
  });

  it('keeps the active report when another input arrives while loading', async () => {
    const diagnostics = new ImportDiagnostics();
    let finish!: () => void;
    const pending = diagnostics.run('first.csv', () => new Promise<void>((resolve) => { finish = resolve; }), () => summary);
    const secondAction = vi.fn();
    await diagnostics.run('second.csv', secondAction, () => summary);
    expect(secondAction).not.toHaveBeenCalled();
    expect(diagnostics.report.source).toBe('first.csv');
    diagnostics.observeState('playing', {});
    finish();
    await pending;
    expect(diagnostics.report.status).toBe('success');
  });

  it('ignores a late completion after reset', async () => {
    const diagnostics = new ImportDiagnostics();
    let finish!: () => void;
    const pending = diagnostics.run('motion.pkl', () => new Promise<void>((resolve) => { finish = resolve; }), () => summary);
    diagnostics.reset();
    diagnostics.observeState('playing', {});
    finish();
    await pending;
    expect(diagnostics.report).toEqual({ status: 'idle', source: '', rows: [], warnings: [], errors: [] });
  });
});

function clip(): MotionClip {
  return {
    name: 'motion', sourcePath: 'motion.pkl', fps: 50, frameCount: 2, stride: 8,
    schema: { rootJointName: 'root', rootComponentCount: 7, jointNames: ['knee'] },
    data: new Float32Array([0, 0, 1, 0, 0, 0, 1, 0.5, 0, 0, 1, 0, 0, 0, 1, 0.5]),
    csvMode: 'ordered', sourceColumnCount: 8,
  };
}

describe('decoded robot motion data inspection', () => {
  it('reports the parsed dimensions without modifying the motion', () => {
    const motion = clip();
    const original = motion.data.slice();
    const result = inspectMotionData(motion);
    expect(result.warnings).toEqual([]);
    expect(Object.fromEntries(result.rows)['data（解析后）']).toBe('[2, 8]');
    expect(Object.fromEntries(result.rows)['root rotation（解析后）']).toContain('[2, 4]');
    expect(motion.data).toEqual(original);
  });

  it('counts non-finite numbers and locates the first affected frame and joint', () => {
    const motion = clip();
    motion.data[15] = NaN;
    motion.data[7] = Infinity;
    const result = inspectMotionData(motion);
    expect(result.warnings.join(' ')).toMatch(/2 个 NaN\/Infinity.*第 1 帧.*knee.*第 8 列/);
  });

  it('detects invalid metadata, incomplete frames, and duplicate or empty joints', () => {
    const motion = clip();
    motion.fps = 0;
    motion.frameCount = 0;
    motion.schema.jointNames = ['knee', 'knee', ''];
    const warnings = inspectMotionData(motion).warnings.join(' ');
    expect(warnings).toContain('fps');
    expect(warnings).toContain('frameCount');
    expect(warnings).toContain('维度不匹配');
    expect(warnings).toContain('空值或重复项');
  });
});
