export interface MotionBrowserAsset {
  id: string;
  path: string;
  source: 'builtin' | 'imported';
  file?: File;
}

export interface MotionBrowserTreeNode {
  name: string;
  path: string;
  directories: MotionBrowserTreeNode[];
  files: MotionBrowserAsset[];
}

export function buildMotionBrowserTree(
  name: string,
  assets: readonly MotionBrowserAsset[],
): MotionBrowserTreeNode {
  const root: MotionBrowserTreeNode = {
    name,
    path: '',
    directories: [],
    files: [],
  };
  const nodes = new Map<string, MotionBrowserTreeNode>([['', root]]);

  for (const asset of assets) {
    const parts = asset.path.split('/').filter(Boolean);
    const fileName = parts.pop();
    if (!fileName) {
      continue;
    }
    let parent = root;
    let currentPath = '';
    for (const part of parts) {
      currentPath = currentPath ? `${currentPath}/${part}` : part;
      let node = nodes.get(currentPath);
      if (!node) {
        node = {
          name: part,
          path: currentPath,
          directories: [],
          files: [],
        };
        nodes.set(currentPath, node);
        parent.directories.push(node);
      }
      parent = node;
    }
    parent.files.push({ ...asset, path: [...parts, fileName].join('/') });
  }

  const sortNode = (node: MotionBrowserTreeNode): void => {
    node.directories.sort((left, right) => left.name.localeCompare(right.name));
    node.files.sort((left, right) => left.path.localeCompare(right.path));
    node.directories.forEach(sortNode);
  };
  sortNode(root);
  return root;
}

export function collectTreeAssetIds(node: MotionBrowserTreeNode): string[] {
  return [
    ...node.files.map((asset) => asset.id),
    ...node.directories.flatMap(collectTreeAssetIds),
  ];
}
