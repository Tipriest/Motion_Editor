import { describe, expect, it } from 'vitest';

import {
  detectRobotExportTarget,
  getExportFormatOptions,
  type UrdfMotionKind,
} from './MotionFormatModes';

describe('motion import/export mode availability', () => {
  it.each<UrdfMotionKind>([
    'gmr',
    'robot-state',
    'ufo-training',
    'ufo-reference',
  ])('keeps all four Tiangong3 robot export modes for %s input', (inputKind) => {
    const values = getExportFormatOptions(inputKind, 'tiangong3').map(
      ({ value }) => value,
    );

    expect(values).toContain('gmr-npz');
    expect(values).toContain('ufo-pkl');
    expect(values).toContain('ufo-npz');
    expect(values).toContain('dex-mosaic-npz');
    expect(new Set(values).size).toBe(values.length);
  });

  it('does not expose Tiangong3-only conversions for another robot', () => {
    expect(getExportFormatOptions('gmr', 'other')).toEqual([
      { value: 'source', label: 'GMR PKL' },
    ]);
  });

  it('shows DEX_MOSAIC NPZ as the fourth Tiangong3 option for Tracking input', () => {
    expect(getExportFormatOptions('ufo-reference', 'tiangong3')).toEqual([
      { value: 'ufo-npz', label: 'UFO Tracking NPZ (Reference)' },
      { value: 'gmr-npz', label: 'GMR NPZ (root + 29-DOF)' },
      { value: 'ufo-pkl', label: 'UFO Training PKL (MotionLib)' },
      { value: 'dex-mosaic-npz', label: 'DEX_MOSAIC NPZ (39-body)' },
    ]);
  });

  it('keeps DEX EVT GMR and 39-body Tracking exports without Tiangong3 training PKL', () => {
    const values = getExportFormatOptions('ufo-reference', 'dex-evt').map(
      ({ value }) => value,
    );

    expect(values).toEqual(['ufo-npz', 'gmr-npz']);
  });

  it('detects EVT2 URDF paths as the DEX EVT export target', () => {
    expect(
      detectRobotExportTarget(
        'tiangong2dex_31dof',
        'assets/robots/EVT2/urdf/tiangong2dex_31dof.urdf',
      ),
    ).toBe('dex-evt');
  });
});
