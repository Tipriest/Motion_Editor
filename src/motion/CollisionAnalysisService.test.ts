import { BoxGeometry, Group, Mesh } from 'three';
import { describe, expect, it } from 'vitest';

import type { UrdfRobotLike } from '../types/viewer';
import { CollisionAnalysisService } from './CollisionAnalysisService';

function createRobot(separation: number): UrdfRobotLike {
  const robot = new Group() as any;
  robot.name = 'collision-test';
  robot.links = {};
  robot.joints = {};

  for (const [name, x] of [
    ['left_link', 0],
    ['right_link', separation],
  ] as const) {
    const link = new Group() as any;
    link.name = name;
    link.isURDFLink = true;
    link.position.set(x, 0.2, 0);
    const collider = new Group() as any;
    collider.isURDFCollider = true;
    collider.add(new Mesh(new BoxGeometry(0.5, 0.5, 0.5)));
    link.add(collider);
    robot.add(link);
    robot.links[name] = link;
  }
  robot.updateMatrixWorld(true);
  return robot as UrdfRobotLike;
}

describe('CollisionAnalysisService', () => {
  it('detects self collision and ground penetration', () => {
    const robot = createRobot(0.3);
    const service = new CollisionAnalysisService();
    service.attachRobot(robot);

    const hits = service.analyzeCurrentFrame(4, 0);

    expect(
      hits.some(
        ({ kind, severity, linkA, linkB }) =>
          kind === 'self' &&
          severity === 'penetration' &&
          linkA === 'left_link' &&
          linkB === 'right_link',
      ),
    ).toBe(true);
    expect(hits.filter(({ kind }) => kind === 'ground')).toHaveLength(2);
    expect(hits.every(({ frameIndex }) => frameIndex === 4)).toBe(true);
  });

  it('does not report separated links as self collision', () => {
    const robot = createRobot(2);
    const service = new CollisionAnalysisService();
    service.attachRobot(robot);

    expect(
      service
        .analyzeCurrentFrame(0, -1)
        .filter(({ kind }) => kind === 'self'),
    ).toEqual([]);
  });
});
