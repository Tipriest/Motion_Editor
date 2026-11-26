#!/usr/bin/env python3
"""Soften tak_012's first right-fist touchdown while retaining the contact repair.

Requires numpy and scipy. Timing is tailored to this 120 FPS tak_012 motion:
edit 1.05-2.70 s and plan a smooth descent from 1.32 to 2.14 s. Root motion,
left arm, all other joints, and frames outside the window are preserved.
The input must use URDF revolute joint order and XYZW root quaternions.
"""
import sys
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
    output = args.output_dir / (src.stem + '_SoftRightLanding.pkl')
    if output.exists():
        raise FileExistsError(output)
    d = load(src)
    q = d['dof_pos']
    fps = d['fps']
    N = len(q)
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
        pending = rest
    fullq = q.copy()
    fullN = N
    selection = np.arange(round(1.05 * fps), round(2.7 * fps) + 1)
    N = len(selection)
    times = selection / fps
    cols = [idx[n] for n in names if n.endswith('_r_joint') and any((k in n for k in ['shoulder', 'elbow', 'wrist']))]
    source = fullq[selection[:, None], cols]
    chain = [next((j for j in joints if j.get('name') == names[c])) for c in cols]
    base = world[chain[0].find('parent').get('link')][selection]
    origins = [ori(j) for j in chain]
    axes = [vec(j.find('axis').get('xyz')) for j in chain]
    tip = ori(next((j for j in joints if j.get('name') == 'right_tcp_joint')))

    def fk(x):
        T = base.copy()
        for (k, (origin, axis)) in enumerate(zip(origins, axes)):
            T = T @ origin
            M = np.tile(np.eye(4), (N, 1, 1))
            M[:, :3, :3] = R.from_rotvec(x[:, k, None] * axis).as_matrix()
            T = T @ M
        T = T @ tip
        return (T[:, :3, 3], T[:, :3, :3])
    mesh = robot.find("link[@name='right_tcp_link']/visual/geometry/mesh")
    raw = (args.urdf.parent / mesh.get('filename')).read_bytes()
    count = int.from_bytes(raw[80:84], 'little')
    vertices = np.frombuffer(raw, dtype=np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')]), count=count, offset=84)['v'].reshape(-1, 3)
    vertices = np.unique(vertices, axis=0)
    vertices = vertices[ConvexHull(vertices).vertices]
    (pos, rot) = fk(source)

    def height(p, r):
        return np.min(r[:, 2, :] @ vertices.T, axis=1) + p[:, 2]
    h = height(pos, rot)
    targeth = h.copy()
    targetr = rot.copy()
    targetp = pos.copy()
    (begin, end) = (1.32, 2.14)
    (a, b) = (round(begin * fps) - selection[0], round(end * fps) - selection[0])
    ix = np.arange(a, b + 1)
    u = (ix - a) / (b - a)
    ease = u ** 3 * (10 - 15 * u + 6 * u * u)
    targeth[ix] = h[a] + ease * (0.003 - h[a])
    anchorrot = R.from_matrix(rot[b:b + 24]).mean().as_matrix()
    delta = R.from_matrix(anchorrot @ rot[a].T).as_rotvec()
    targetr[ix] = R.from_rotvec(ease[:, None] * delta).as_matrix() @ rot[a]
    endblend = np.arange(b + 1, min(N, b + 31))
    u = (endblend - b) / 30
    ease = u ** 3 * (10 - 15 * u + 6 * u * u)
    targeth[endblend] = 0.003 + ease * (h[endblend] - 0.003)
    delta = R.from_matrix(rot[endblend] @ anchorrot.T).as_rotvec()
    targetr[endblend] = R.from_rotvec(delta * ease[:, None]).as_matrix() @ anchorrot
    lo = []
    hi = []
    vlim = []
    for j in chain:
        limits = j.findall('limit')
        lo.append(max((float(x.get('lower')) for x in limits)))
        hi.append(min((float(x.get('upper')) for x in limits)))
        vlim.append(min((float(x.get('velocity')) for x in limits)))
    lo = np.tile(lo, (N, 1))
    hi = np.tile(hi, (N, 1))
    lo[:12] = source[:12] - 1e-10
    hi[:12] = source[:12] + 1e-10
    lo[-12:] = source[-12:] - 1e-10
    hi[-12:] = source[-12:] + 1e-10
    blocks = [(N, 2, 0, True), (N, 1, 0, True), (N, 3, 0, True), (N, 7, 0, False), (N, 1, 0, True), (N - 1, 7, 1, False), (N - 2, 7, 2, False), (N - 1, 1, 1, True), (N - 2, 1, 2, True)]
    spars = lil_matrix((sum((count * width for (count, width, _, _) in blocks)), N * 7), dtype=int)
    offset = 0
    for (count, width, order, full) in blocks:
        for f in range(count):
            for k in range(width):
                for neighbor in range(order + 1):
                    if full:
                        spars[offset + f * width + k, (f + neighbor) * 7:(f + neighbor + 1) * 7] = 1
                    else:
                        spars[offset + f * width + k, (f + neighbor) * 7 + k] = 1
        offset += count * width
    vthreshold = np.minimum(7, np.array(vlim) * 0.85)

    def residual(flat):
        x = flat.reshape(N, 7)
        (p, r) = fk(x)
        hh = height(p, r)
        angles = R.from_matrix(targetr @ np.swapaxes(r, 1, 2)).as_rotvec()
        vel = np.diff(x, axis=0) * fps
        return np.concatenate(((p[:, :2] - targetp[:, :2]).ravel() * 35, (hh - targeth) * 120, angles.ravel() * 2, ((x - source) * 0.25).ravel(), np.maximum(-0.001 - hh, 0) * 300, (np.maximum(abs(vel) - vthreshold, 0) * 1.5).ravel(), (np.diff(x, n=2, axis=0) * fps ** 2 * 0.02).ravel(), (np.diff(hh) - np.diff(targeth)) * fps * 1.5, np.diff(hh, n=2) * fps ** 2 * 0.025))
    started = time.time()
    fit = least_squares(residual, np.clip(source, lo + 1e-12, hi - 1e-12).ravel(), bounds=(lo.ravel(), hi.ravel()), jac_sparsity=spars.tocsr(), max_nfev=250, ftol=1e-05)
    x = fit.x.reshape(N, 7)
    (p, r) = fk(x)
    newh = height(p, r)
    print('solve', time.time() - started, fit.message, 'height RMSE', np.sqrt(np.mean((newh - targeth) ** 2)), 'maxheight error', abs(newh - targeth).max(), flush=True)
    for (label, hh) in [('before', h), ('after', newh)]:
        velocity = np.diff(hh) * fps
        acc = np.diff(hh, n=2) * fps ** 2
        near = (hh[:-1] > 0.005) & (hh[:-1] < 0.1) & (times[:-1] < 2.2)
        first = np.where((hh < 0.01) & (times > 1.5))[0][0]
        print(label, 'downward vmax', -velocity.min(), 'max speed within 10cm', -velocity[near].min(), 'maxacc', abs(acc).max(), 'touchdown', times[first], 'speed at 1cm', velocity[first - 1], flush=True)
    print('maxjointvel', abs(np.diff(x, axis=0) * fps).max(), 'speed ratios', np.max(abs(np.diff(x, axis=0) * fps) / vlim), 'posechange maxdeg', np.rad2deg(abs(x - source)).max(), flush=True)
    np.savez('/tmp/touchdown_fit_final.npz', selection=selection, cols=cols, source=source, fit=x, h=h, newh=newh, targeth=targeth, beforepos=pos, afterpos=p, beforerot=rot, afterrot=r, times=times)
    if not fit.success:
        raise RuntimeError(fit.message)
    x[:12] = source[:12]
    x[-12:] = source[-12:]
    repaired = fullq.copy()
    repaired[selection[:, None], cols] = x
    other = [i for i in range(fullq.shape[1]) if i not in cols]
    assert np.array_equal(repaired[:, other], fullq[:, other])
    assert np.array_equal(repaired[:selection[0]], fullq[:selection[0]])
    assert np.array_equal(repaired[selection[-1] + 1:], fullq[selection[-1] + 1:])
    assert np.array_equal(repaired[[0, -1]], fullq[[0, -1]])
    assert np.isfinite(repaired).all()
    all_joints = [j for j in joints if j.get('type') == 'revolute']
    for i in cols:
        for limit in all_joints[i].findall('limit'):
            assert repaired[:, i].min() >= float(limit.get('lower')) - 1e-06, (i, 'lower')
            assert repaired[:, i].max() <= float(limit.get('upper')) + 1e-06, (i, 'upper')
            assert abs(np.diff(repaired[:, i]) * fps).max() <= float(limit.get('velocity')) + 1e-06, (i, 'speed')
    result = dict(d, dof_pos=repaired)
    output.write_bytes(pickle.dumps(result, protocol=4))
    restored = load(output)
    assert np.array_equal(restored['dof_pos'], repaired)
    for key in d.keys() - {'dof_pos'}:
        if isinstance(d[key], np.ndarray):
            assert np.array_equal(d[key], restored[key])
        else:
            assert d[key] == restored[key]
    assert hashlib.sha256(src.read_bytes()).hexdigest() == source_hash
    measurements = {}
    for (name, hh) in [('before', h), ('after', newh)]:
        velocity = np.diff(hh) * fps
        near = (hh[:-1] > 0.005) & (hh[:-1] < 0.1) & (times[:-1] < 2.2)
        first = np.where((hh < 0.01) & (times > 1.5))[0][0]
        measurements[name] = dict(max_downward_speed_m_s=float(-velocity.min()), max_downward_speed_within_10cm_m_s=float(-velocity[near].min()), first_below_1cm_seconds=float(times[first]), downward_speed_at_1cm_m_s=float(-velocity[first - 1]), max_vertical_acceleration_m_s2=float(abs(np.diff(hh, n=2) * fps ** 2).max()), minimum_hand_height_m=float(hh.min()))
    report = dict(source=str(src), output=str(output), source_sha256=source_hash, urdf=str(args.urdf), frames=fullN, fps=fps, edited_window_seconds=[float(times[0]), float(times[-1])], planned_descent_seconds=[begin, end], method='Right arm trajectory IK with eased descent, hand height velocity and acceleration penalties', measurements=measurements, max_joint_change_deg=float(np.rad2deg(abs(x - source)).max()), max_joint_speed_rad_s=float(abs(np.diff(repaired[:, cols], axis=0) * fps).max()), unchanged=['input', 'root_pos', 'root_rot', 'all non-right-arm joints', 'all frames after 2.7 s', 'first/last pose', 'frame count', 'fps', 'metadata'], validation=dict(joint_angle_and_speed_limits=True, serialized_roundtrip=True, source_hash_unchanged=True), limitations=['Contact inferred from geometry; no contact-force simulation', 'First right-hand touchdown is delayed by about 0.16 seconds'])
    (args.output_dir / 'soft_landing_report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print('Saved', output, flush=True)
if __name__ == '__main__':
    main()
