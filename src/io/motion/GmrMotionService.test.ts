import { pickle } from 'picklefriend';
import { describe, expect, it } from 'vitest';

import type { DroppedFileMap, MotionSchema } from '../../types/viewer';
import { GmrMotionService } from './GmrMotionService';
import {
  TIANGONG3_JOINT_NAMES_29,
  TIANGONG3_JOINT_NAMES_31,
} from './RobotTrackingLayouts';

function createSchema(jointNames: readonly string[]): MotionSchema {
  return {
    rootJointName: 'floating_base_joint',
    rootComponentCount: 7,
    jointNames: [...jointNames],
  };
}

function createGmrFile(
  jointCount: number,
  linkBodyList: readonly string[] | null = null,
  name = 'motion.pkl',
): File {
  const serialized = pickle.dumps({
    fps: 50,
    root_pos: [[1, 2, 3]],
    root_rot: [[0, 0, 0, 1]],
    dof_pos: [Array.from({ length: jointCount }, (_, index) => index + 0.25)],
    link_body_list: linkBodyList ? [...linkBodyList] : null,
  });
  return new File([serialized], name);
}

function createFileMap(...files: File[]): DroppedFileMap {
  return new Map(files.map((file) => [file.name, file]));
}

describe('GmrMotionService Tiangong3 joint layouts', () => {
  it('maps an unnamed 29-DOF motion onto the 31-joint URDF and zeros the head', async () => {
    const service = new GmrMotionService();
    const result = await service.loadFromDroppedFiles(
      createFileMap(createGmrFile(29)),
      createSchema(TIANGONG3_JOINT_NAMES_31),
      'motion.pkl',
    );

    expect(result.clip.schema.jointNames).toEqual([...TIANGONG3_JOINT_NAMES_31]);
    expect(result.clip.stride).toBe(38);
    expect(result.clip.data[7 + 14]).toBeCloseTo(14.25);
    expect(result.clip.data[7 + 15]).toBe(0);
    expect(result.clip.data[7 + 16]).toBe(0);
    expect(result.clip.data[7 + 17]).toBeCloseTo(15.25);
    expect(result.warnings.join(' ')).toMatch(/29-DOF.*head_yaw_joint.*head_pitch_joint/);
    expect(result.warnings.join(' ')).toMatch(/inferred/);
  });

  it('keeps an unnamed 31-DOF motion unchanged on the 31-joint URDF', async () => {
    const service = new GmrMotionService();
    const result = await service.loadFromDroppedFiles(
      createFileMap(createGmrFile(31)),
      createSchema(TIANGONG3_JOINT_NAMES_31),
      'motion.pkl',
    );

    expect(Array.from(result.clip.data.slice(7))).toEqual(
      Array.from({ length: 31 }, (_, index) => index + 0.25),
    );
    expect(result.warnings.some((warning) => warning.includes('Mapped'))).toBe(false);
  });

  it('drops the two head joints when mapping an unnamed 31-DOF motion to 29 DOF', async () => {
    const service = new GmrMotionService();
    const result = await service.loadFromDroppedFiles(
      createFileMap(createGmrFile(31)),
      createSchema(TIANGONG3_JOINT_NAMES_29),
      'motion.pkl',
    );

    expect(result.clip.schema.jointNames).toEqual([...TIANGONG3_JOINT_NAMES_29]);
    expect(result.clip.data[7 + 14]).toBeCloseTo(14.25);
    expect(result.clip.data[7 + 15]).toBeCloseTo(17.25);
    expect(result.warnings.join(' ')).toMatch(/ignored 2 extra joints/);
    expect(result.warnings.join(' ')).toMatch(/inferred/);
  });

  it('auto-selects a convertible motion and still rejects unknown layouts', async () => {
    const service = new GmrMotionService();
    const schema = createSchema(TIANGONG3_JOINT_NAMES_31);
    const result = await service.loadFromDroppedFiles(
      createFileMap(
        createGmrFile(28, null, 'unknown.pkl'),
        createGmrFile(29, null, 'compatible.pkl'),
      ),
      schema,
    );

    expect(result.selectedMotionPath).toBe('compatible.pkl');
    await expect(
      service.loadFromDroppedFiles(
        createFileMap(createGmrFile(28, null, 'unknown.pkl')),
        schema,
        'unknown.pkl',
      ),
    ).rejects.toThrow(/has 28 joints, expected 31/);
  });
});
