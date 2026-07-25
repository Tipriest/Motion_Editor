import { BufferGeometry, Float32BufferAttribute, Group, Mesh } from 'three';
import { describe, expect, it } from 'vitest';

import type { UrdfRobotLike } from '../types/viewer';
import { GroundContactService } from './GroundContactService';

describe('GroundContactService', () => {
  it('samples the bottom mesh outline and updates its contact states', () => {
    const geometry = new BufferGeometry();
    geometry.setAttribute(
      'position',
      new Float32BufferAttribute(
        [
          -0.1, -0.05, -0.05,
          0.1, -0.05, -0.05,
          0.1, 0.05, -0.05,
          -0.1, 0.05, -0.05,
          -0.1, -0.05, 0.05,
          0.1, -0.05, 0.05,
          0.1, 0.05, 0.05,
          -0.1, 0.05, 0.05,
        ],
        3,
      ),
    );
    const foot = new Group();
    foot.add(new Mesh(geometry));
    foot.rotation.x = -Math.PI / 2;
    foot.updateMatrixWorld(true);
    const robot = {
      name: 'test',
      links: { ankle_roll_l_link: foot },
      traverse: () => undefined,
      updateMatrixWorld: () => foot.updateMatrixWorld(true),
    } as unknown as UrdfRobotLike;
    const service = new GroundContactService();
    service.attachRobot(robot);

    const contacts = service.compute(-0.05);

    expect(service.getFootLinkCount()).toBe(1);
    expect(service.getProbeCount()).toBeGreaterThanOrEqual(12);
    expect(contacts.every(({ isContact }) => isContact)).toBe(true);
    expect(contacts.every((point) => point.footName === 'ankle_roll_l_link')).toBe(true);

    foot.position.y = 0.1;
    foot.updateMatrixWorld(true);
    expect(service.compute(-0.05).every(({ isContact }) => !isContact)).toBe(true);

    foot.position.y = 0;
    foot.rotation.x = -Math.PI / 2 + 0.12;
    foot.updateMatrixWorld(true);
    const tiltedPositions = service.compute(
      0,
      Number.POSITIVE_INFINITY,
      Number.POSITIVE_INFINITY,
    );
    const lowestHeight = Math.min(...tiltedPositions.map(({ y }) => y));
    const tiltedContacts = service.compute(lowestHeight);
    expect(tiltedContacts.some(({ isContact }) => isContact)).toBe(true);
    expect(tiltedContacts.some(({ isContact }) => !isContact)).toBe(true);
  });
});
