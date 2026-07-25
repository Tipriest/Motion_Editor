import { Vector3 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';

export interface CenterOfMassPoint {
  x: number;
  y: number;
  z: number;
}

interface InertialLink {
  node: {
    matrixWorld: any;
  };
  mass: number;
  localCenter: any;
}

function findDirectChild(element: any, tagName: string): any | null {
  const children = Array.from(element?.children ?? []) as any[];
  return (
    children.find((child) => String(child?.tagName ?? '').toLowerCase() === tagName) ?? null
  );
}

function parseVector(value: string | null): CenterOfMassPoint {
  const components = (value ?? '0 0 0')
    .trim()
    .split(/\s+/)
    .map(Number);
  return {
    x: Number.isFinite(components[0]) ? components[0] : 0,
    y: Number.isFinite(components[1]) ? components[1] : 0,
    z: Number.isFinite(components[2]) ? components[2] : 0,
  };
}

export class CenterOfMassService {
  private inertialLinks: InertialLink[] = [];
  private totalMass = 0;
  private readonly tempPoint = new Vector3();
  private readonly weightedCenter = new Vector3();

  attachRobot(robot: UrdfRobotLike | null): void {
    this.inertialLinks = [];
    this.totalMass = 0;
    if (!robot?.links) {
      return;
    }

    for (const link of Object.values(robot.links)) {
      const node = link as any;
      const inertial = findDirectChild(node.urdfNode, 'inertial');
      const massElement = findDirectChild(inertial, 'mass');
      const mass = Number(massElement?.getAttribute?.('value'));
      if (!Number.isFinite(mass) || mass <= 0 || !node.matrixWorld) {
        continue;
      }
      const origin = findDirectChild(inertial, 'origin');
      const localCenter = parseVector(origin?.getAttribute?.('xyz') ?? null);
      this.inertialLinks.push({
        node,
        mass,
        localCenter: new Vector3(localCenter.x, localCenter.y, localCenter.z),
      });
      this.totalMass += mass;
    }
  }

  getLinkCount(): number {
    return this.inertialLinks.length;
  }

  getTotalMass(): number {
    return this.totalMass;
  }

  compute(robot: UrdfRobotLike): CenterOfMassPoint | null {
    if (this.inertialLinks.length === 0 || this.totalMass <= 0) {
      return null;
    }
    robot.updateMatrixWorld?.(true);
    this.weightedCenter.set(0, 0, 0);
    for (const link of this.inertialLinks) {
      this.tempPoint.copy(link.localCenter).applyMatrix4(link.node.matrixWorld);
      this.weightedCenter.addScaledVector(this.tempPoint, link.mass);
    }
    this.weightedCenter.multiplyScalar(1 / this.totalMass);
    return {
      x: this.weightedCenter.x,
      y: this.weightedCenter.y,
      z: this.weightedCenter.z,
    };
  }
}
