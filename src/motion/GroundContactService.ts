import { Matrix4, Vector3 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';

export interface GroundContactPoint {
  x: number;
  y: number;
  z: number;
  footName: string;
  probeName: string;
  isContact: boolean;
}

export interface FootSoleDefinition {
  footName: string;
  link: any;
  localCenter: { x: number; y: number; z: number };
}

interface FootProbe {
  footName: string;
  probeName: string;
  link: {
    matrixWorld: any;
  };
  localPoint: any;
}

interface OutlinePoint {
  x: number;
  y: number;
}

function isFootLinkName(name: string): boolean {
  return (
    /^ankle_roll_[lr]_link$/i.test(name) ||
    /(?:^|_)(?:foot|toe)_(?:l|r)(?:_|$)/i.test(name)
  );
}

function isColliderNode(node: any): boolean {
  let current = node;
  while (current) {
    if (current.isURDFCollider) {
      return true;
    }
    current = current.parent;
  }
  return false;
}

function cross(origin: OutlinePoint, left: OutlinePoint, right: OutlinePoint): number {
  return (
    (left.x - origin.x) * (right.y - origin.y) -
    (left.y - origin.y) * (right.x - origin.x)
  );
}

function convexHull(points: readonly OutlinePoint[]): OutlinePoint[] {
  const unique = new Map<string, OutlinePoint>();
  for (const point of points) {
    unique.set(`${point.x.toFixed(5)}:${point.y.toFixed(5)}`, point);
  }
  const sorted = [...unique.values()].sort(
    (left, right) => left.x - right.x || left.y - right.y,
  );
  if (sorted.length <= 2) {
    return sorted;
  }
  const lower: OutlinePoint[] = [];
  for (const point of sorted) {
    while (
      lower.length >= 2 &&
      cross(lower[lower.length - 2], lower[lower.length - 1], point) <= 0
    ) {
      lower.pop();
    }
    lower.push(point);
  }
  const upper: OutlinePoint[] = [];
  for (let index = sorted.length - 1; index >= 0; index -= 1) {
    const point = sorted[index];
    while (
      upper.length >= 2 &&
      cross(upper[upper.length - 2], upper[upper.length - 1], point) <= 0
    ) {
      upper.pop();
    }
    upper.push(point);
  }
  lower.pop();
  upper.pop();
  return [...lower, ...upper];
}

function sampleClosedOutline(
  outline: readonly OutlinePoint[],
  spacing = 0.025,
): OutlinePoint[] {
  if (outline.length < 3) {
    return [...outline];
  }
  const lengths: number[] = [];
  let perimeter = 0;
  for (let index = 0; index < outline.length; index += 1) {
    const next = outline[(index + 1) % outline.length];
    const length = Math.hypot(next.x - outline[index].x, next.y - outline[index].y);
    lengths.push(length);
    perimeter += length;
  }
  const sampleCount = Math.max(12, Math.min(32, Math.round(perimeter / spacing)));
  const samples: OutlinePoint[] = [];
  let edgeIndex = 0;
  let edgeStartDistance = 0;
  for (let sampleIndex = 0; sampleIndex < sampleCount; sampleIndex += 1) {
    const distance = (sampleIndex * perimeter) / sampleCount;
    while (
      edgeIndex < lengths.length - 1 &&
      edgeStartDistance + lengths[edgeIndex] < distance
    ) {
      edgeStartDistance += lengths[edgeIndex];
      edgeIndex += 1;
    }
    const from = outline[edgeIndex];
    const to = outline[(edgeIndex + 1) % outline.length];
    const alpha =
      lengths[edgeIndex] > 1e-8
        ? (distance - edgeStartDistance) / lengths[edgeIndex]
        : 0;
    samples.push({
      x: from.x + (to.x - from.x) * alpha,
      y: from.y + (to.y - from.y) * alpha,
    });
  }
  return samples;
}

export class GroundContactService {
  private footProbes: FootProbe[] = [];
  private footLinks: Array<{ footName: string; link: any }> = [];
  private robot: UrdfRobotLike | null = null;
  private readonly tempPoint = new Vector3();
  private readonly localMatrix = new Matrix4();
  private readonly inverseLinkMatrix = new Matrix4();

  attachRobot(robot: UrdfRobotLike | null): void {
    this.robot = robot;
    this.footProbes = [];
    this.footLinks = [];
    if (!robot?.links) {
      return;
    }
    for (const [footName, rawLink] of Object.entries(robot.links)) {
      if (!isFootLinkName(footName)) {
        continue;
      }
      this.footLinks.push({ footName, link: rawLink as any });
    }
    this.buildFootOutlineProbes();
  }

  private buildFootOutlineProbes(): void {
    if (!this.robot || this.footProbes.length > 0) {
      return;
    }
    this.robot.updateMatrixWorld?.(true);
    for (const { footName, link } of this.footLinks) {
      const vertices: Array<{ x: number; y: number; z: number }> = [];
      this.inverseLinkMatrix.copy(link.matrixWorld).invert();
      link.traverse?.((node: any) => {
        const positions = node?.geometry?.getAttribute?.('position');
        if (!node?.isMesh || !positions || isColliderNode(node)) {
          return;
        }
        this.localMatrix.multiplyMatrices(this.inverseLinkMatrix, node.matrixWorld);
        for (let index = 0; index < positions.count; index += 1) {
          this.tempPoint
            .fromBufferAttribute(positions, index)
            .applyMatrix4(this.localMatrix);
          vertices.push({
            x: this.tempPoint.x,
            y: this.tempPoint.y,
            z: this.tempPoint.z,
          });
        }
      });
      if (vertices.length === 0) {
        continue;
      }
      let minimumZ = Number.POSITIVE_INFINITY;
      for (const vertex of vertices) {
        minimumZ = Math.min(minimumZ, vertex.z);
      }
      let bottomVertices = vertices.filter(({ z }) => z <= minimumZ + 0.004);
      if (bottomVertices.length < 3) {
        bottomVertices = vertices.filter(({ z }) => z <= minimumZ + 0.012);
      }
      const outline = convexHull(
        bottomVertices.map(({ x, y }) => ({ x, y })),
      );
      const samples = sampleClosedOutline(outline);
      for (let index = 0; index < samples.length; index += 1) {
        const sample = samples[index];
        this.footProbes.push({
          footName,
          probeName: `${footName}_outline_${index + 1}`,
          link,
          localPoint: new Vector3(sample.x, sample.y, minimumZ),
        });
      }
    }
  }

  getFootLinkCount(): number {
    return this.footLinks.length;
  }

  getProbeCount(): number {
    return this.footProbes.length;
  }

  getFootSole(side: 'left' | 'right'): FootSoleDefinition | null {
    this.buildFootOutlineProbes();
    const sideToken = side === 'left' ? '_l_' : '_r_';
    const probes = this.footProbes.filter(({ footName }) =>
      footName.toLowerCase().includes(sideToken),
    );
    if (probes.length === 0) {
      return null;
    }
    const center = probes.reduce(
      (result, probe) => {
        result.x += probe.localPoint.x;
        result.y += probe.localPoint.y;
        result.z += probe.localPoint.z;
        return result;
      },
      { x: 0, y: 0, z: 0 },
    );
    center.x /= probes.length;
    center.y /= probes.length;
    center.z /= probes.length;
    return {
      footName: probes[0].footName,
      link: probes[0].link,
      localCenter: center,
    };
  }

  compute(
    groundHeight: number,
    contactThreshold = 0.005,
    lowestPointThreshold = 0.003,
  ): GroundContactPoint[] {
    this.buildFootOutlineProbes();
    if (!this.robot || this.footProbes.length === 0) {
      return [];
    }
    this.robot.updateMatrixWorld?.(true);
    const points: Array<GroundContactPoint & { height: number }> = [];
    for (const probe of this.footProbes) {
      this.tempPoint.copy(probe.localPoint).applyMatrix4(probe.link.matrixWorld);
      points.push({
        x: this.tempPoint.x,
        y: this.tempPoint.y,
        z: this.tempPoint.z,
        footName: probe.footName,
        probeName: probe.probeName,
        isContact: false,
        height: this.tempPoint.y,
      });
    }
    const minimumHeightByFoot = new Map<string, number>();
    for (const point of points) {
      minimumHeightByFoot.set(
        point.footName,
        Math.min(
          minimumHeightByFoot.get(point.footName) ?? Number.POSITIVE_INFINITY,
          point.height,
        ),
      );
    }
    return points.map(({ height, ...point }) => ({
      ...point,
      isContact:
        Math.abs(height - groundHeight) <= contactThreshold &&
        height - (minimumHeightByFoot.get(point.footName) ?? height) <=
          lowestPointThreshold,
    }));
  }
}
