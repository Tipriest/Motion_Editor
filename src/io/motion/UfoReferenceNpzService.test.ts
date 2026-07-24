import { Group } from 'three';
import { describe, expect, it } from 'vitest';
import type { MotionClip, UrdfRobotLike } from '../../types/viewer';
import { parseNpzFile } from './NumpyIO';
import {
  buildUfoReferenceData,
  exportUfoReferenceNpz,
  UFO_POLICY_BODY_NAMES,
  UFO_POLICY_JOINT_NAMES,
  type UfoFrameSampler,
} from './UfoReferenceNpzService';

function createClip(frameCount = 3): MotionClip {
  const jointNames = [...UFO_POLICY_JOINT_NAMES, 'head_yaw_joint', 'head_pitch_joint'];
  const stride = 7 + jointNames.length;
  const data = new Float32Array(frameCount * stride);
  for (let frame = 0; frame < frameCount; frame += 1) {
    const base = frame * stride;
    data[base + 6] = 1;
    for (let joint = 0; joint < jointNames.length; joint += 1) {
      data[base + 7 + joint] = frame * 0.01 + joint * 0.001;
    }
  }
  return {
    name: 'ufo-test',
    sourcePath: 'ufo-test.pkl',
    fps: 50,
    frameCount,
    stride,
    schema: {
      rootJointName: 'floating_base_joint',
      rootComponentCount: 7,
      jointNames,
    },
    csvMode: 'ordered',
    sourceColumnCount: stride,
    data,
  };
}

function createSampler(frameCount: number): UfoFrameSampler {
  const modelRoot = new Group();
  modelRoot.rotation.x = -Math.PI / 2;
  const robot = new Group() as any as UrdfRobotLike & {
    position: any;
    rotation: any;
    add: (child: any) => void;
  };
  robot.name = 'tiangong3';
  robot.joints = {};
  robot.links = {};
  modelRoot.add(robot);

  UFO_POLICY_BODY_NAMES.forEach((bodyName, index) => {
    const body = new Group();
    body.name = bodyName;
    body.position.set(0, index * 0.01, 0);
    robot.add(body);
    robot.links![bodyName] = body;
  });

  return {
    sampleClipFrames: (_clip, visitor) => {
      for (let frame = 0; frame < frameCount; frame += 1) {
        robot.position.set(frame, 0, 0);
        robot.rotation.set(0, 0, frame * 0.1);
        modelRoot.updateMatrixWorld(true);
        visitor(frame, robot);
      }
    },
  };
}

describe('UfoReferenceNpzService', () => {
  it('maps policy joints and computes world kinematic velocities', () => {
    const clip = createClip();
    const data = buildUfoReferenceData(clip, createSampler(clip.frameCount));

    expect(data.jointPos.length).toBe(clip.frameCount * 29);
    expect(data.bodyPosW.length).toBe(clip.frameCount * 30 * 3);
    expect(data.jointPos[29]).toBeCloseTo(0.01);
    expect(data.jointVel[0]).toBeCloseTo(0.5);

    const secondPelvisPosition = 30 * 3;
    expect(data.bodyPosW[secondPelvisPosition]).toBeCloseTo(1);
    expect(data.bodyPosW[secondPelvisPosition + 1]).toBeCloseTo(0);
    expect(data.bodyLinVelW[0]).toBeCloseTo(50);
    expect(data.bodyAngVelW[2]).toBeCloseTo(5, 3);

    const secondPelvisQuaternion = 30 * 4;
    expect(data.bodyQuatW[secondPelvisQuaternion]).toBeCloseTo(Math.cos(0.05));
    expect(data.bodyQuatW[secondPelvisQuaternion + 3]).toBeCloseTo(Math.sin(0.05));
  });

  it('writes all xmigcs reference keys with canonical shapes', async () => {
    const clip = createClip();
    const bytes = exportUfoReferenceNpz(clip, createSampler(clip.frameCount));
    const archive = await parseNpzFile(new File([new Uint8Array(bytes)], 'ufo.npz'));

    expect(archive.listFileNames().sort()).toEqual(
      [
        'fps.npy',
        'joint_pos.npy',
        'joint_vel.npy',
        'body_pos_w.npy',
        'body_quat_w.npy',
        'body_lin_vel_w.npy',
        'body_ang_vel_w.npy',
      ].sort(),
    );
    expect((await archive.readNpy('joint_pos.npy')).shape).toEqual([3, 29]);
    expect((await archive.readNpy('body_pos_w.npy')).shape).toEqual([3, 30, 3]);
    expect((await archive.readNpy('body_quat_w.npy')).shape).toEqual([3, 30, 4]);
    expect((await archive.readNpy('fps.npy')).toScalarNumber()).toBe(50);
  });

  it('rejects clips missing a policy joint', () => {
    const clip = createClip();
    clip.schema.jointNames = clip.schema.jointNames.filter((name) => name !== 'wrist_roll_r_joint');
    expect(() => buildUfoReferenceData(clip, createSampler(clip.frameCount))).toThrow(
      /wrist_roll_r_joint/,
    );
  });
});
