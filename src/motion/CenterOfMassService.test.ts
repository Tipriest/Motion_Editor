import { describe, expect, it } from 'vitest';
import { Matrix4 } from 'three';

import type { UrdfRobotLike } from '../types/viewer';
import { CenterOfMassService } from './CenterOfMassService';

function element(tagName: string, attributes: Record<string, string> = {}, children: any[] = []) {
  return {
    tagName,
    children,
    getAttribute: (name: string) => attributes[name] ?? null,
  };
}

function link(mass: number, center: string, matrixWorld: any) {
  return {
    matrixWorld,
    urdfNode: element('link', {}, [
      element('inertial', {}, [
        element('origin', { xyz: center }),
        element('mass', { value: String(mass) }),
      ]),
    ]),
  };
}

describe('CenterOfMassService', () => {
  it('combines URDF inertial origins using link masses', () => {
    const robot = {
      name: 'test',
      links: {
        light: link(1, '1 0 0', new Matrix4()),
        heavy: link(3, '0 0 0', new Matrix4().makeTranslation(3, 0, 0)),
      },
      traverse: () => undefined,
      updateMatrixWorld: () => undefined,
    } as unknown as UrdfRobotLike;
    const service = new CenterOfMassService();
    service.attachRobot(robot);

    expect(service.getLinkCount()).toBe(2);
    expect(service.getTotalMass()).toBe(4);
    expect(service.compute(robot)).toEqual({ x: 2.5, y: 0, z: 0 });
  });
});
