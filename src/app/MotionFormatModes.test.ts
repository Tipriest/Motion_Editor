import { describe, expect, it } from 'vitest';

import {
  getExportFormatOptions,
  type UrdfMotionKind,
} from './MotionFormatModes';

describe('motion import/export mode availability', () => {
  it.each<UrdfMotionKind>([
    'gmr',
    'robot-state',
    'ufo-training',
    'ufo-reference',
  ])('keeps all three Tiangong3 export modes for %s input', (inputKind) => {
    const values = getExportFormatOptions(inputKind, true).map(
      ({ value }) => value,
    );

    expect(values).toContain('gmr-npz');
    expect(values).toContain('ufo-pkl');
    expect(values).toContain('ufo-npz');
    expect(new Set(values).size).toBe(values.length);
  });

  it('does not expose Tiangong3-only conversions for another robot', () => {
    expect(getExportFormatOptions('gmr', false)).toEqual([
      { value: 'source', label: 'GMR PKL' },
    ]);
  });
});
