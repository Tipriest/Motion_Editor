#!/usr/bin/env python3
"""Stabilize ground-near hand orientation in 31-DOF GMR motions.

Requires numpy and scipy. Uses the supplied URDF and binary STL hand meshes;
columns must follow URDF revolute order and root quaternions must be XYZW.
The optimization preserves root/non-arm data and writes a separate PKL.
This is a kinematic contact repair, not a dynamics simulation.
"""
from smooth_gmr_arms import load
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.ndimage import binary_closing, label
from scipy.spatial import ConvexHull
from pathlib import Path
import xml.etree.ElementTree as ET
import time
import argparse
import hashlib
import json
import pickle

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    src = args.input
    source_hash = hashlib.sha256(src.read_bytes()).hexdigest()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / (src.stem.replace('_ArmsSmoothed', '') + '_ArmsContactStabilized.pkl')
    if output.exists():
        raise FileExistsError(output)
    reports = {}
    d = load(src)
    q = d['dof_pos']
    fps = d['fps']
    N = len(q)
    repaired = q.copy()
    assert q.shape == (N, 31) and N > 24 and (fps > 0) and np.isfinite(q).all()
    robot = ET.parse(args.urdf).getroot()
    joints = robot.findall('joint')
    names = [j.get('name') for j in joints if j.get('type') == 'revolute']
    idx = {n: i for (i, n) in enumerate(names)}

    def vec(s):
        return np.fromstring(s, sep=' ')

    def ori(j):
        o = j.find('origin')
        T = np.eye(4)
        if o is not None:
            T[:3, :3] = R.from_euler('xyz', vec(o.get('rpy', '0 0 0'))).as_matrix()
            T[:3, 3] = vec(o.get('xyz', '0 0 0'))
        return T
    world = {}
    root = next((l.get('name') for l in robot.findall('link') if l.get('name') not in {j.find('child').get('link') for j in joints}))
    T = np.tile(np.eye(4), (N, 1, 1))
    T[:, :3, :3] = R.from_quat(d['root_rot']).as_matrix()
    T[:, :3, 3] = d['root_pos']
    world[root] = T
    pending = list(joints)
    while pending:
        rest = []
        for j in pending:
            (pa, ch) = (j.find('parent').get('link'), j.find('child').get('link'))
            if pa not in world:
                rest.append(j)
                continue
            T = world[pa] @ ori(j)
            if j.get('name') in idx:
                a = vec(j.find('axis').get('xyz'))
                M = np.tile(np.eye(4), (N, 1, 1))
                M[:, :3, :3] = R.from_rotvec(q[:, idx[j.get('name')], None] * a).as_matrix()
                T = T @ M
            world[ch] = T
        if len(rest) == len(pending):
            raise ValueError('URDF link tree is disconnected.')
        pending = rest
    for (side, long) in [('l', 'left'), ('r', 'right')]:
        cols = [i for (i, n) in enumerate(names) if n.endswith('_' + side + '_joint') and any((k in n for k in ['shoulder', 'elbow', 'wrist']))]
        source = q[:, cols]
        chain = [next((j for j in joints if j.get('name') == names[c])) for c in cols]
        base = world[chain[0].find('parent').get('link')]
        origins = [ori(j) for j in chain]
        axes = [vec(j.find('axis').get('xyz')) for j in chain]
        tip = ori(next((j for j in joints if j.get('name') == long + '_tcp_joint')))

        def fk(x):
            T = base.copy()
            for (k, (origin, axis)) in enumerate(zip(origins, axes)):
                T = T @ origin
                M = np.tile(np.eye(4), (N, 1, 1))
                M[:, :3, :3] = R.from_rotvec(x[:, k, None] * axis).as_matrix()
                T = T @ M
            T = T @ tip
            return (T[:, :3, 3], T[:, :3, :3])
        (pos, rot) = fk(source)
        mesh = robot.find(f"link[@name='{long}_tcp_link']/visual/geometry/mesh")
        raw = (args.urdf.parent / mesh.get('filename')).read_bytes()
        count = int.from_bytes(raw[80:84], 'little')
        vertices = np.frombuffer(raw, dtype=np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')]), count=count, offset=84)['v'].reshape(-1, 3)
        vertices = np.unique(vertices, axis=0)
        vertices = vertices[ConvexHull(vertices).vertices]
        h = np.min(np.einsum('nij,vj->nvi', rot[:, 2:3, :], vertices)[:, :, 0] + pos[:, 2, None], axis=1)
        mask = binary_closing(abs(h) < 0.025, structure=np.ones(7))
        (labels, nb) = label(mask)
        weight = np.zeros(N)
        targetp = pos.copy()
        targetr = rot.copy()
        segments = []
        for k in range(1, nb + 1):
            ix = np.where(labels == k)[0]
            if len(ix) < 24:
                continue
            (a, b) = (ix[0], ix[-1])
            expanded = np.arange(max(0, a - 36), min(N, b + 37))
            enter = np.clip((expanded - (a - 36)) / 36, 0, 1)
            leave = np.clip((b + 36 - expanded) / 36, 0, 1)
            phase = np.minimum(enter, leave)
            blend = phase ** 3 * (10 - 15 * phase + 6 * phase ** 2)
            weight[expanded] = np.maximum(weight[expanded], blend)
            mean = R.from_matrix(rot[ix]).mean().as_matrix()
            delta = R.from_matrix(mean @ np.swapaxes(rot[expanded], 1, 2)).as_rotvec()
            targetr[expanded] = R.from_rotvec(delta * blend[:, None]).as_matrix() @ rot[expanded]
            anchor = np.median(pos[ix], axis=0)
            targetp[expanded, :2] = pos[expanded, :2] + 0.35 * blend[:, None] * (anchor[:2] - pos[expanded, :2])
            groundz = -np.min(vertices @ mean[2, :]) + 0.003
            targetp[ix, 2] = np.maximum(pos[ix, 2], groundz)
            segments.append([a / fps, b / fps])
        print(side, 'segments', segments, flush=True)
        lo = []
        hi = []
        velocity_limits = []
        for j in chain:
            limits = j.findall('limit')
            lo.append(max((float(x.get('lower')) for x in limits)))
            hi.append(min((float(x.get('upper')) for x in limits)))
            velocity_limits.append(min(float(x.get('velocity')) for x in limits))
        # Leave a margin because the speed objective is soft, unlike angle bounds.
        speed_threshold = np.minimum(8, np.array(velocity_limits) * .85)
        sizes = [N * 3, N * 3, N * 7, N * 2, N, (N - 1) * 7, (N - 2) * 7, (N - 1) * 7]
        spars = lil_matrix((sum(sizes), N * 7), dtype=int)
        o = 0
        for (count, width, full) in [(N, 3, True), (N, 3, True), (N, 7, False), (N, 2, False), (N, 1, True)]:
            for f in range(count):
                if full:
                    spars[o + f * width:o + (f + 1) * width, f * 7:(f + 1) * 7] = 1
                else:
                    for k in range(width):
                        spars[o + f * width + k, f * 7 + (k if width == 7 else k + 5)] = 1
            o += count * width
        for order in [1, 2, 1]:
            for f in range(N - order):
                for k in range(7):
                    for neighbor in range(order + 1):
                        spars[o + f * 7 + k, (f + neighbor) * 7 + k] = 1
            o += (N - order) * 7
        fidelity = np.full((N, 7), 0.25)
        fidelity[:12] = 20
        fidelity[-12:] = 20

        def residual(flat):
            x = flat.reshape(N, 7)
            (p, r) = fk(x)
            angle = R.from_matrix(targetr @ np.swapaxes(r, 1, 2)).as_rotvec()
            velocity = np.diff(x, axis=0) * fps
            hand_height = np.min(r[:, 2, :] @ vertices.T, axis=1) + p[:, 2]
            return np.concatenate((
                ((p - targetp) * 80).ravel(),
                (angle * 3).ravel(),
                ((x - source) * fidelity).ravel(),
                (x[:, 5:] * weight[:, None] * .1).ravel(),
                (np.maximum(-.002 - hand_height, 0) * 250).ravel(),
                (velocity * .03).ravel(),
                (np.diff(x, n=2, axis=0) * fps**2 * .015).ravel(),
                (np.maximum(abs(velocity) - speed_threshold, 0) * 1.5).ravel(),
            ))
        started = time.time()
        fit = least_squares(residual, np.clip(source, np.array(lo) + 1e-08, np.array(hi) - 1e-08).ravel(), bounds=(np.tile(lo, N), np.tile(hi, N)), jac_sparsity=spars.tocsr(), max_nfev=200, verbose=0, ftol=2e-05)
        x = fit.x.reshape(N, 7)
        (p, r) = fk(x)
        print(side, 'solve', time.time() - started, fit.message, 'poserr', np.max(np.linalg.norm(p - targetp, axis=1)), 'orienterr max deg', np.rad2deg(R.from_matrix(targetr @ np.swapaxes(r, 1, 2)).magnitude()).max(), 'qdiff rms/max deg', np.rad2deg(np.sqrt(np.mean((x - source) ** 2))), np.rad2deg(abs(x - source)).max(), flush=True)
        if not fit.success:
            raise RuntimeError(f'Optimizer did not converge: {side}: {fit.message}')
        endpoint = np.minimum(np.arange(N), np.arange(N)[::-1]) / (0.3 * fps)
        phase = np.clip(endpoint, 0, 1)
        envelope = phase ** 3 * (10 - 15 * phase + 6 * phase ** 2)
        x = source + envelope[:, None] * (x - source)
        if np.any(np.max(abs(np.diff(x, axis=0) * fps), axis=0) > np.array(velocity_limits)):
            raise ValueError(f'{side} arm exceeds a URDF speed limit.')
        if np.any(x < np.array(lo) - 1e-6) or np.any(x > np.array(hi) + 1e-6):
            raise ValueError(f'{side} arm exceeds a URDF angle limit.')
        repaired[:, cols] = x
        (p, r) = fk(x)
        newheight = np.min(r[:, 2, :] @ vertices.T, axis=1) + p[:, 2]
        comparisons = []
        for (start, end) in [(2.5, 2.75), (3.0, 3.5), (4.2, 4.5), (5.0, 5.5)]:
            (i, j) = (round(start * fps), round(end * fps))
            if j >= N:
                continue
            comparisons.append(dict(seconds=[start, end], before_rotation_deg=float(np.rad2deg(R.from_matrix(rot[j] @ rot[i].T).magnitude())), after_rotation_deg=float(np.rad2deg(R.from_matrix(r[j] @ r[i].T).magnitude())), before_xy_displacement_m=float(np.linalg.norm(pos[j, :2] - pos[i, :2])), after_xy_displacement_m=float(np.linalg.norm(p[j, :2] - p[i, :2]))))
        reports[long] = dict(contact_segments_seconds=segments, optimizer=fit.message, max_joint_speed_rad_s=float(abs(np.diff(x, axis=0) * fps).max()), max_joint_acceleration_rad_s2=float(abs(np.diff(x, n=2, axis=0) * fps ** 2).max()), max_angle_change_deg=float(np.rad2deg(abs(x - source)).max()), rms_angle_change_deg=float(np.rad2deg(np.sqrt(np.mean((x - source) ** 2)))), minimum_hand_mesh_z_before_m=float(h.min()), minimum_hand_mesh_z_after_m=float(newheight.min()), comparison_windows=comparisons)
        np.savez_compressed(args.output_dir / (long + '_contact_diagnostics.npz'), before_position=pos, after_position=p, before_rotation=rot, after_rotation=r, before_height=h, after_height=newheight, weight=weight)
    result = dict(d, dof_pos=repaired)
    output.write_bytes(pickle.dumps(result, protocol=4))
    restored = load(output)
    arm_columns = [idx[name] for name in names if any((part in name for part in ['shoulder', 'elbow', 'wrist']))]
    other = [i for i in range(31) if i not in arm_columns]
    assert np.array_equal(restored['dof_pos'][:, other], q[:, other])
    assert np.array_equal(restored['dof_pos'][[0, -1]], q[[0, -1]])
    assert np.isfinite(restored['dof_pos']).all()
    for key in d.keys() - {'dof_pos'}:
        if isinstance(d[key], np.ndarray):
            assert np.array_equal(d[key], restored[key])
        else:
            assert d[key] == restored[key]
    assert hashlib.sha256(src.read_bytes()).hexdigest() == source_hash
    report = dict(source=str(src), output=str(output), source_sha256=source_hash, urdf=str(args.urdf), fps=fps, frames=N, assumptions=['XYZW root quaternions', 'URDF revolute joint column order', 'ground Z=0', 'Contact inferred from hand mesh height; no force labels'], method='Sparse whole-trajectory arm IK with hand orientation stabilization, partial XY anchoring, ground barrier and temporal penalties', parameters=dict(contact_height_m=0.025, minimum_contact_frames=24, transition_seconds=0.3, xy_anchor_fraction=0.35, position_weight=80, rotation_weight=3, posture_weight=0.25, acceleration_weight=0.015, soft_speed_threshold='min(8 rad/s, 85% of each URDF speed limit)', speed_penalty_weight=1.5), unchanged=['input file', 'root_pos', 'root_rot', 'non-arm joints', 'first/last pose', 'fps', 'frame count', 'metadata'], arms=reports, limitations=['Kinematic repair; not validated by a dynamics simulation', 'Some hand sliding remains; ground penetration penalty is soft'])
    (args.output_dir / 'contact_repair_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print('Saved', output, flush=True)
if __name__ == '__main__':
    main()
