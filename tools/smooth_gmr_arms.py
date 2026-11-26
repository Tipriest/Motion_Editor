#!/usr/bin/env python3
"""Create GMR arm-smoothing copies, diagnostics, and URDF skeleton previews.

Requires numpy, scipy, matplotlib; ffmpeg is needed for --preview.
Joint columns must follow the supplied URDF's revolute joint order.
Root rotation follows the viewer's XYZW convention. This is kinematic
smoothing, not a dynamics, collision, or hand-contact optimization.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
import pickle
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy.spatial.transform import Rotation


class NumericPickleReader(pickle.Unpickler):
    def find_class(self, module, name):
        if module in ('numpy.core.numeric', 'numpy._core.numeric') and name == '_frombuffer':
            return np.core.numeric._frombuffer
        if module in ('numpy.core.multiarray', 'numpy._core.multiarray') and name == '_reconstruct':
            return np.core.multiarray._reconstruct
        if module == 'numpy' and name in ('dtype', 'ndarray'):
            return getattr(np, name)
        raise ValueError(f'Unsupported pickle global: {module}.{name}')


def load(path):
    return NumericPickleReader(io.BytesIO(path.read_bytes())).load()


def metrics(q, fps):
    result = {}
    for order, label in [(1, 'velocity'), (2, 'acceleration'), (3, 'jerk')]:
        values = np.diff(q, n=order, axis=0) * fps ** order
        result[label + '_rms'] = float(np.sqrt(np.mean(values ** 2)))
        result[label + '_max'] = float(np.max(np.abs(values)))
    return result


def make_curve_plot(before, after, names, fps, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(7, 2, figsize=(15, 17), sharex=True)
    time = np.arange(len(before)) / fps
    for index, ax in enumerate(axes.T.flat):
        ax.plot(time, np.rad2deg(before[:, index]), color='#db8030', alpha=.7, lw=1, label='Original')
        ax.plot(time, np.rad2deg(after[:, index]), color='#167bb5', lw=1.4, label='Smoothed')
        ax.set_title(names[index], fontsize=10)
        ax.set_ylabel('deg')
        ax.grid(alpha=.2)
    axes[0, 0].legend()
    axes[-1, 0].set_xlabel('Time (s)')
    axes[-1, 1].set_xlabel('Time (s)')
    fig.suptitle('Arm joint curves: original vs smoothed', fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .98))
    fig.savefig(destination, dpi=110)
    plt.close(fig)


def skeleton_positions(payload, robot, names, frames):
    q = payload['dof_pos']
    indices = {name: i for i, name in enumerate(names)}
    joint_defs = []
    for joint in robot.findall('joint'):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        rpy = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ') if origin is not None else np.zeros(3)
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        transform[:3, 3] = xyz
        axis = joint.find('axis')
        vector = np.fromstring(axis.get('xyz', '1 0 0'), sep=' ') if axis is not None else np.array([1., 0, 0])
        joint_defs.append((joint.find('parent').get('link'), joint.find('child').get('link'),
                           transform, vector, indices.get(joint.get('name')),
                           any(part in joint.get('name', '') for part in ('shoulder', 'elbow', 'wrist'))))
    children = {item[1] for item in joint_defs}
    root = next(link.get('name') for link in robot.findall('link') if link.get('name') not in children)
    results = []
    for frame in frames:
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(payload['root_rot'][frame]).as_matrix()
        transform[:3, 3] = payload['root_pos'][frame]
        transforms = {root: transform}
        remaining = list(joint_defs)
        while remaining:
            unresolved = []
            for parent, child, origin, axis, index, arm in remaining:
                if parent not in transforms:
                    unresolved.append((parent, child, origin, axis, index, arm))
                    continue
                rotation = np.eye(4)
                if index is not None:
                    rotation[:3, :3] = Rotation.from_rotvec(axis * q[frame, index]).as_matrix()
                transforms[child] = transforms[parent] @ origin @ rotation
            if len(unresolved) == len(remaining):
                raise ValueError('URDF link tree is disconnected.')
            remaining = unresolved
        segments = np.array([[transforms[p][:3, 3], transforms[c][:3, 3], [np.nan]*3]
                             for p, c, *_ in joint_defs])
        results.append(segments)
    return np.array(results), np.array([item[-1] for item in joint_defs])


def preview(before, after, robot, names, destination):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, FFMpegWriter
    step = max(1, round(before['fps'] / 24))
    frames = np.arange(0, len(before['dof_pos']), step)
    original, arms = skeleton_positions(before, robot, names, frames)
    repaired, _ = skeleton_positions(after, robot, names, frames)
    # Follow the root in the horizontal plane so arm detail remains readable.
    offset = np.zeros((len(frames), 1, 1, 3))
    offset[:, 0, 0, :2] = before['root_pos'][frames, :2]
    original -= offset
    repaired -= offset
    points = np.concatenate((original.reshape(-1, 3), repaired.reshape(-1, 3)))
    center = (np.nanmin(points, axis=0) + np.nanmax(points, axis=0)) / 2
    radius = max(np.nanmax(points[:, :2], axis=0) - np.nanmin(points[:, :2], axis=0)) / 2 + .3
    fig = plt.figure(figsize=(10, 5.4))
    lines = []
    for i, (label, color) in enumerate([('Original', '#db8030'), ('Smoothed arms', '#167bb5')]):
        ax = fig.add_subplot(1, 2, i + 1, projection='3d')
        ax.set(xlim=(center[0]-radius, center[0]+radius), ylim=(center[1]-radius, center[1]+radius),
               zlim=(-.15, max(1.8, np.nanmax(points[:, 2])+.15)), title=label,
               xlabel='Root-relative X (m)', ylabel='Root-relative Y (m)', zlabel='Z (m)')
        ax.view_init(elev=18, azim=-55)
        ax.set_box_aspect((2*radius, 2*radius, 2))
        lines.append((ax.plot([], [], [], color='#89949c', lw=2)[0],
                      ax.plot([], [], [], color=color, lw=3)[0]))
    title = fig.suptitle('URDF skeleton comparison (no mesh / physics simulation)')
    def update(i):
        for poses, (body, arm) in zip((original, repaired), lines):
            body.set_data_3d(poses[i].reshape(-1, 3).T)
            arm.set_data_3d(poses[i, arms].reshape(-1, 3).T)
        title.set_text(f'URDF skeleton comparison | t = {frames[i]/before["fps"]:.2f} s')
        return []
    movie = FuncAnimation(fig, update, frames=len(frames), interval=1000*step/before['fps'])
    movie.save(destination, writer=FFMpegWriter(fps=before['fps']/step, bitrate=1600))
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', nargs='+', type=Path)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--preview', action='store_true')
    args = parser.parse_args()
    robot = ET.parse(args.urdf).getroot()
    joints = [j for j in robot.findall('joint') if j.get('type') in ('revolute', 'continuous')]
    names = [j.get('name') for j in joints]
    arm_indices = [i for i, name in enumerate(names) if any(p in name for p in ('shoulder', 'elbow', 'wrist'))]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    reports = []
    for source in args.inputs:
        source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
        original = load(source)
        q = original['dof_pos']
        fps = float(original['fps'])
        if q.shape[1] != len(names) or len(q) < 5 or not np.isfinite(q).all() or fps <= 0:
            raise ValueError(f'Invalid motion shape, values, or FPS: {source}')
        repaired = q.copy()
        settings = {}
        for i in arm_indices:
            # Symmetric positive Gaussian weights avoid phase delay and overshoot.
            # Larger distal windows suppress wrist/elbow twitches more strongly.
            sigma = .065 if 'shoulder' in names[i] else (.085 if 'elbow' in names[i] else .1)
            filtered = gaussian_filter1d(q[:, i], sigma * fps, mode='nearest', truncate=4)
            # Smoothly retain the first/last pose; taper is flat at both ends.
            distance = np.minimum(np.arange(len(q)), np.arange(len(q))[::-1]) / fps
            phase = np.clip(distance / .25, 0, 1)
            blend = phase**3 * (10 - 15*phase + 6*phase**2)
            repaired[:, i] = q[:, i] + blend * (filtered - q[:, i])
            limit = joints[i].find('limit')
            if limit is not None:
                lower, upper = float(limit.get('lower')), float(limit.get('upper'))
                if np.any(repaired[:, i] < lower - 1e-6) or np.any(repaired[:, i] > upper + 1e-6):
                    raise ValueError(f'Joint limit violation: {names[i]}')
            settings[names[i]] = sigma
        result = dict(original, dof_pos=repaired)
        target = args.output_dir / (source.stem + '_ArmsSmoothed.pkl')
        if target.exists():
            raise FileExistsError(f'Refusing to overwrite {target}')
        target.write_bytes(pickle.dumps(result, protocol=4))
        restored = load(target)
        other = [i for i in range(len(names)) if i not in arm_indices]
        assert np.array_equal(restored['dof_pos'][:, other], q[:, other])
        for key in original.keys() - {'dof_pos'}:
            if isinstance(original[key], np.ndarray):
                assert np.array_equal(original[key], restored[key])
            else:
                assert original[key] == restored[key]
        assert np.array_equal(restored['dof_pos'][[0, -1]], q[[0, -1]])
        assert hashlib.sha256(source.read_bytes()).hexdigest() == source_hash
        delta = repaired[:, arm_indices] - q[:, arm_indices]
        report = dict(source=str(source), output=str(target), source_sha256=source_hash,
                      frames=len(q), fps=fps, joint_order_assumption='URDF revolute order',
                      sigma_seconds=settings, before=metrics(q[:, arm_indices], fps),
                      after=metrics(repaired[:, arm_indices], fps),
                      arm_change_rms_deg=float(np.rad2deg(np.sqrt(np.mean(delta**2)))),
                      arm_change_max_deg=float(np.rad2deg(np.abs(delta).max())),
                      unchanged=['root_pos', 'root_rot', 'non-arm joints', 'first/last pose', 'metadata'])
        reports.append(report)
        make_curve_plot(q[:, arm_indices], repaired[:, arm_indices],
                        [names[i] for i in arm_indices], fps, target.with_suffix('.png'))
        print(json.dumps(report, ensure_ascii=False), flush=True)
        if args.preview:
            preview(original, result, robot, names, target.with_suffix('.mp4'))
    (args.output_dir / 'repair_report.json').write_text(json.dumps(reports, indent=2, ensure_ascii=False)+'\n')


if __name__ == '__main__':
    main()
