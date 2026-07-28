import { Box3, Vector3 } from 'three';
import type { UrdfRobotLike } from '../types/viewer';

export type CollisionSeverity = 'near' | 'penetration';
export type CollisionKind = 'self' | 'ground';

export interface CollisionHit {
  frameIndex: number;
  kind: CollisionKind;
  severity: CollisionSeverity;
  linkA: string;
  linkB?: string;
  depth: number;
}

export interface CollisionAnalysisResult {
  byFrame: Map<number, CollisionHit[]>;
  penetrationFrameCount: number;
  nearFrameCount: number;
  selfHitCount: number;
  groundHitCount: number;
}

export interface CollisionAnalysisOptions {
  nearDistance?: number;
  penetrationEpsilon?: number;
  ignoredLinkPairs?: ReadonlySet<string>;
}

interface ColliderEntry {
  linkName: string;
  node: any;
  box: any;
}

const tempDelta = new Vector3();

function pairKey(left: string, right: string): string {
  return left < right ? `${left}\0${right}` : `${right}\0${left}`;
}

function findLinkName(node: any, linkNamesByNode: Map<any, string>): string | null {
  let current: any = node;
  while (current) {
    const mapped = linkNamesByNode.get(current);
    if (mapped) {
      return mapped;
    }
    if (current.isURDFLink && typeof current.name === 'string' && current.name) {
      return current.name;
    }
    current = current.parent;
  }
  return null;
}

function findParentLinkName(linkNode: any, linkNamesByNode: Map<any, string>): string | null {
  let current = linkNode?.parent;
  while (current) {
    const mapped = linkNamesByNode.get(current);
    if (mapped) {
      return mapped;
    }
    current = current.parent;
  }
  return null;
}

function boxDistance(left: any, right: any): number {
  tempDelta.set(
    Math.max(0, left.min.x - right.max.x, right.min.x - left.max.x),
    Math.max(0, left.min.y - right.max.y, right.min.y - left.max.y),
    Math.max(0, left.min.z - right.max.z, right.min.z - left.max.z),
  );
  return tempDelta.length();
}

function boxPenetrationDepth(left: any, right: any): number {
  if (!left.intersectsBox(right)) {
    return 0;
  }
  return Math.max(
    0,
    Math.min(
      left.max.x - right.min.x,
      right.max.x - left.min.x,
      left.max.y - right.min.y,
      right.max.y - left.min.y,
      left.max.z - right.min.z,
      right.max.z - left.min.z,
    ),
  );
}

export class CollisionAnalysisService {
  private robot: UrdfRobotLike | null = null;
  private colliders: ColliderEntry[] = [];
  private ignoredLinkPairs = new Set<string>();
  private linkNamesByNode = new Map<any, string>();

  attachRobot(robot: UrdfRobotLike | null): void {
    this.robot = robot;
    this.colliders = [];
    this.ignoredLinkPairs.clear();
    this.linkNamesByNode.clear();
    if (!robot) {
      return;
    }

    for (const [linkName, linkNode] of Object.entries(robot.links ?? {})) {
      this.linkNamesByNode.set(linkNode, linkName);
    }
    const parentByLink = new Map<string, string>();
    for (const [linkName, linkNode] of Object.entries(robot.links ?? {})) {
      const parentLinkName = findParentLinkName(linkNode, this.linkNamesByNode);
      if (parentLinkName) {
        parentByLink.set(linkName, parentLinkName);
      }
    }
    for (const linkName of parentByLink.keys()) {
      let ancestor = parentByLink.get(linkName);
      for (let distance = 0; distance < 2 && ancestor; distance += 1) {
        this.ignoredLinkPairs.add(pairKey(linkName, ancestor));
        ancestor = parentByLink.get(ancestor);
      }
    }

    robot.traverse((child: unknown) => {
      const node = child as any;
      if (!node?.isURDFCollider) {
        return;
      }
      const linkName = findLinkName(node, this.linkNamesByNode);
      if (!linkName) {
        return;
      }
      this.colliders.push({
        linkName,
        node,
        box: new Box3(),
      });
    });
  }

  getColliderCount(): number {
    return this.colliders.length;
  }

  analyzeCurrentFrame(
    frameIndex: number,
    groundHeight: number,
    partialOptions: CollisionAnalysisOptions = {},
  ): CollisionHit[] {
    if (!this.robot || this.colliders.length === 0) {
      return [];
    }
    const nearDistance = partialOptions.nearDistance ?? 0.008;
    const penetrationEpsilon = partialOptions.penetrationEpsilon ?? 0.001;
    const ignoredPairs = partialOptions.ignoredLinkPairs;
    const boxesByLink = new Map<string, any>();
    for (const collider of this.colliders) {
      collider.box.setFromObject(collider.node, true);
      if (collider.box.isEmpty()) {
        continue;
      }
      const current = boxesByLink.get(collider.linkName);
      if (current) {
        current.union(collider.box);
      } else {
        boxesByLink.set(collider.linkName, collider.box.clone());
      }
    }

    const hits: CollisionHit[] = [];
    const links = [...boxesByLink.entries()];
    for (const [linkName, box] of links) {
      const groundDistance = box.min.y - groundHeight;
      if (groundDistance < -penetrationEpsilon) {
        hits.push({
          frameIndex,
          kind: 'ground',
          severity: 'penetration',
          linkA: linkName,
          depth: -groundDistance,
        });
      }
    }

    for (let leftIndex = 0; leftIndex < links.length; leftIndex += 1) {
      const [leftName, leftBox] = links[leftIndex];
      for (let rightIndex = leftIndex + 1; rightIndex < links.length; rightIndex += 1) {
        const [rightName, rightBox] = links[rightIndex];
        const key = pairKey(leftName, rightName);
        if (this.ignoredLinkPairs.has(key) || ignoredPairs?.has(key)) {
          continue;
        }
        const depth = boxPenetrationDepth(leftBox, rightBox);
        if (depth > penetrationEpsilon) {
          hits.push({
            frameIndex,
            kind: 'self',
            severity: 'penetration',
            linkA: leftName,
            linkB: rightName,
            depth,
          });
          continue;
        }
        const distance = boxDistance(leftBox, rightBox);
        if (distance <= nearDistance) {
          hits.push({
            frameIndex,
            kind: 'self',
            severity: 'near',
            linkA: leftName,
            linkB: rightName,
            depth: Math.max(0, nearDistance - distance),
          });
        }
      }
    }
    hits.sort(
      (left, right) =>
        Number(right.severity === 'penetration') -
          Number(left.severity === 'penetration') ||
        right.depth - left.depth,
    );
    return hits;
  }

  getInfluencingJointNames(linkNames: readonly string[]): string[] {
    if (!this.robot) {
      return [];
    }
    const jointNamesByNode = new Map<any, string>();
    for (const [jointName, jointNode] of Object.entries(this.robot.joints ?? {})) {
      jointNamesByNode.set(jointNode, jointName);
    }
    const result = new Set<string>();
    for (const linkName of linkNames) {
      let current: any = (this.robot.links as any)?.[linkName];
      while (current) {
        const jointName = jointNamesByNode.get(current);
        if (jointName) {
          result.add(jointName);
        }
        current = current.parent;
      }
    }
    return [...result];
  }

  static summarize(byFrame: Map<number, CollisionHit[]>): CollisionAnalysisResult {
    let penetrationFrameCount = 0;
    let nearFrameCount = 0;
    let selfHitCount = 0;
    let groundHitCount = 0;
    for (const hits of byFrame.values()) {
      if (hits.some(({ severity }) => severity === 'penetration')) {
        penetrationFrameCount += 1;
      } else {
        nearFrameCount += 1;
      }
      selfHitCount += hits.filter(({ kind }) => kind === 'self').length;
      groundHitCount += hits.filter(({ kind }) => kind === 'ground').length;
    }
    return {
      byFrame,
      penetrationFrameCount,
      nearFrameCount,
      selfHitCount,
      groundHitCount,
    };
  }
}
