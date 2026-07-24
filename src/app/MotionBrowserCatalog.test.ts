import { describe, expect, it } from 'vitest';
import { buildMotionBrowserTree, collectTreeAssetIds } from './MotionBrowserCatalog';

describe('MotionBrowserCatalog', () => {
  it('builds a sorted recursive tree and collects folder selections', () => {
    const tree = buildMotionBrowserTree('assets/motions', [
      { id: 'b', path: 'walk/run/b.npz', source: 'builtin' },
      { id: 'a', path: 'walk/a.pkl', source: 'builtin' },
      { id: 'c', path: 'idle.csv', source: 'builtin' },
    ]);

    expect(tree.files.map((file) => file.path)).toEqual(['idle.csv']);
    expect(tree.directories[0].name).toBe('walk');
    expect(tree.directories[0].directories[0].name).toBe('run');
    expect(collectTreeAssetIds(tree.directories[0]).sort()).toEqual(['a', 'b']);
  });
});
