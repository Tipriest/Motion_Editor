import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import type { MotionClip, ViewerState } from '../types/viewer';
import { GmrMotionService } from '../io/motion/GmrMotionService';
import { CsvMotionService } from '../io/motion/CsvMotionService';
import { MimicKitMotionService } from '../io/motion/MimicKitMotionService';
import { SmplMotionService } from '../io/motion/SmplMotionService';
import { UfoReferenceMotionService } from '../io/motion/UfoReferenceMotionService';
import { RobotStateNpzMotionService } from '../io/motion/RobotStateNpzMotionService';
import { UfoTrainingPklMotionService } from '../io/motion/UfoTrainingPklMotionService';
import { UrdfLoadService } from '../io/urdf/UrdfLoadService';
import { pickle } from 'picklefriend';
import { AppController } from './App';
import { ImportDiagnostics } from './ImportDiagnostics';

interface AppHarness {
  handleDrop(transfer: DataTransfer): Promise<void>;
  handlePickedFiles(files: FileList | null): Promise<void>;
  loadMotionOptionByKey(key: string): Promise<void>;
  loadModelOptionByKey(key: string): Promise<void>;
  loadPresetById(key: string): Promise<void>;
  setState(state: ViewerState, details?: { title?: string; detail?: string }): void;
  lastLoadResult: unknown;
  currentMotionClip: MotionClip | null;
  currentMotionKind: string | null;
  currentMotionSourcePath: string | null;
  motionWarnings: string[];
}

let app: AppHarness;
let diagnostics: ImportDiagnostics;

beforeEach(() => {
  diagnostics = new ImportDiagnostics();
  // Exercise real input routing, parsers, state transitions and report snapshots.
  // Rendering and Three.js playback are replaced so these checks require no WebGL context.
  app = Object.create(AppController.prototype) as AppHarness;
  Object.assign(app, {
    importDiagnosticsPanel: { diagnostics }, viewerState: 'idle', warnings: [], motionWarnings: [],
    currentMotionClip: null, currentMotionKind: null, currentMotionSourcePath: null,
    lastLoadResult: {
      robotName: 'test', selectedUrdfPath: 'robot.urdf', jointCount: 1, linkCount: 2, warnings: [],
      motionSchema: { rootJointName: 'root', rootComponentCount: 7, jointNames: ['knee'] },
    },
    urdfLoadService: new UrdfLoadService(), csvMotionService: new CsvMotionService(),
    gmrMotionService: new GmrMotionService(), mimicKitMotionService: new MimicKitMotionService(),
    smplMotionService: new SmplMotionService(), ufoReferenceMotionService: new UfoReferenceMotionService(),
    robotStateNpzMotionService: new RobotStateNpzMotionService(), ufoTrainingPklMotionService: new UfoTrainingPklMotionService(),
    bvhMotionService: { getAvailableBvhPaths: () => [] }, objLoadService: { getAvailableObjPaths: () => [] },
    clearRecoverReadyTimer: vi.fn(), scheduleRecoverToReady: vi.fn(), renderState: vi.fn(),
    applyLoadedUrdfMotion: (_robot: unknown, clip: MotionClip, kind: string, path: string, warnings: string[]) => {
      app.currentMotionClip = clip;
      app.currentMotionKind = kind;
      app.currentMotionSourcePath = path;
      app.motionWarnings = warnings;
      app.setState('model_ready', {});
    },
  });
  vi.spyOn(console, 'log').mockImplementation(() => {});
});

afterEach(() => { vi.restoreAllMocks(); });

function transfer(...files: File[]): DataTransfer {
  return { items: [], files } as unknown as DataTransfer;
}

describe('App import diagnostics integration', () => {
  it('reports a successful CSV drop and keeps the imported snapshot when playback changes', async () => {
    await app.handleDrop(transfer(new File(['0,0,1,0,0,0,1,0.25\n'], 'walk.csv')));
    expect(diagnostics.report.status).toBe('success');
    expect(diagnostics.report.source).toBe('walk.csv');
    expect(Object.fromEntries(diagnostics.report.rows)['data（解析后）']).toBe('[1, 8]');
    app.currentMotionClip!.fps = 120;
    app.setState('playing', {});
    expect(Object.fromEntries(diagnostics.report.rows).fps).toBe('30');
  });

  it('reports missing model requirements for motion-only input', async () => {
    app.lastLoadResult = null;
    await app.handleDrop(transfer(new File(['0,0,1,0,0,0,1,0.25'], 'walk.csv')));
    expect(diagnostics.report.status).toBe('error');
    expect(diagnostics.report.errors.join(' ')).toContain('Load a URDF robot first');
    app.setState('model_ready', {});
    expect(diagnostics.report.status).toBe('error');
  });

  it('includes concrete missing-field errors after all PKL format fallbacks fail', async () => {
    const file = new File([pickle.dumps({ fps: 50 })], 'broken.pkl');
    await app.handleDrop(transfer(file));
    expect(diagnostics.report.status).toBe('error');
    const errors = diagnostics.report.errors.join(' ');
    expect(errors).toContain('broken.pkl · GMR:');
    expect(errors).toContain('missing "root_pos", "root_rot", "dof_pos"');
    expect(errors).not.toContain('No Supported Files');
    expect(diagnostics.report.rows).toEqual([]);
  });

  it('uses the GMR fallback successfully and includes its source warnings', async () => {
    await app.handleDrop(transfer(new File([pickle.dumps({
      fps: -1, root_pos: [[0, 0, 1]], root_rot: [[0, 0, 0, 0]], dof_pos: [[0.25]], link_body_list: null,
    })], 'fixed.pkl')));
    expect(app.currentMotionKind).toBe('gmr');
    expect(diagnostics.report.status).toBe('success');
    expect(diagnostics.report.errors).toEqual([]);
    expect(diagnostics.report.warnings.join(' ')).toContain('zero-length');
    expect(diagnostics.report.warnings.join(' ')).toContain('invalid fps');
  });

  it('tracks file picker imports, empty drops and unsupported files, without clearing reports on picker cancellation', async () => {
    await app.handlePickedFiles([new File(['0,0,1,0,0,0,1,0.25'], 'selected.csv')] as unknown as FileList);
    expect(diagnostics.report).toMatchObject({ status: 'success', source: 'selected.csv' });
    await app.handlePickedFiles(null);
    expect(diagnostics.report.status).toBe('success');
    await app.handleDrop(transfer());
    expect(diagnostics.report.errors).toContain('No Files Found');
    await app.handleDrop(transfer(new File(['text'], 'notes.txt')));
    expect(diagnostics.report.status).toBe('error');
    expect(diagnostics.report.errors).toContain('No Supported Files');
  });

  it('wraps model, motion, and preset selections in the same reporting flow', async () => {
    Object.assign(app, {
      getAvailableModelOptions: () => [], getCurrentModelOption: () => null, getPresetById: () => null,
    });
    await app.loadModelOptionByKey('missing.urdf');
    expect(diagnostics.report.errors).toContain('Model Not Found');
    await app.loadMotionOptionByKey('missing.pkl');
    expect(diagnostics.report.errors).toContain('Model Required');
    await app.loadPresetById('missing-preset');
    expect(diagnostics.report.errors).toContain('Preset Not Found');
  });
});
