import { describe, expect, it } from 'vitest';
import { buildMotionExportFileName } from './ExportFileName';

describe('buildMotionExportFileName', () => {
  it('uses the source motion name, speed and format suffix', () => {
    expect(buildMotionExportFileName('left punch.pkl', 2, 'ufo_training', 'pkl')).toBe(
      'left_punch_2x_ufo_training.pkl',
    );
    expect(buildMotionExportFileName('motions/left_punch.pkl', 0.5, 'gmr', 'pkl')).toBe(
      'left_punch_0.5x_gmr.pkl',
    );
  });

  it('supports extension-only formats and stable decimal speed labels', () => {
    expect(buildMotionExportFileName('boxing.csv', 1.25, null, 'csv')).toBe(
      'boxing_1.25x.csv',
    );
  });
});
