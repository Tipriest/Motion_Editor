import { readdirSync } from 'node:fs';
import { extname, join, relative, resolve } from 'node:path';
import { defineConfig } from 'vitest/config';

const MOTION_ASSET_MODULE_ID = 'virtual:motion-assets';
const RESOLVED_MOTION_ASSET_MODULE_ID = `\0${MOTION_ASSET_MODULE_ID}`;
const SUPPORTED_MOTION_EXTENSIONS = new Set(['.bvh', '.csv', '.pkl']);

function scanMotionAssets(directory: string, root = directory): string[] {
  try {
    return readdirSync(directory, { withFileTypes: true })
      .flatMap((entry) => {
        const absolutePath = join(directory, entry.name);
        if (entry.isDirectory()) {
          return scanMotionAssets(absolutePath, root);
        }
        if (!entry.isFile() || !SUPPORTED_MOTION_EXTENSIONS.has(extname(entry.name).toLowerCase())) {
          return [];
        }
        return [`assets/motions/${relative(root, absolutePath).replace(/\\/g, '/')}`];
      })
      .sort((left, right) => left.localeCompare(right));
  } catch {
    return [];
  }
}

export default defineConfig({
  // Use relative asset paths so dist/ can be hosted under any sub-path.
  base: './',
  plugins: [
    {
      name: 'motion-asset-catalog',
      resolveId(id) {
        return id === MOTION_ASSET_MODULE_ID ? RESOLVED_MOTION_ASSET_MODULE_ID : null;
      },
      load(id) {
        if (id !== RESOLVED_MOTION_ASSET_MODULE_ID) {
          return null;
        }
        const paths = scanMotionAssets(resolve(process.cwd(), 'assets/motions'));
        return `export default ${JSON.stringify(paths)};`;
      },
    },
  ],
  server: {
    host: true,
    port: 5173,
    watch: {
      ignored: [
        // Large asset trees are loaded at runtime or via drag-and-drop, not via Vite HMR.
        '**/ref/**',
        '**/motions/**',
        '**/models/**',
        '**/.cache/**',
        '**/*.lock',
        '**/.venv/**',
        '**/site-packages/**',
      ],
    },
  },
  test: {
    environment: 'node',
    include: ['src/**/*.test.ts'],
  },
});
