"""Lidar V-marker detection (pure Python + numpy): the pose of a V-shaped docking marker (MiR VL-marker style) in a 2D
scan. Used by bay_marker_detector for the Nav2 docking server's external detection.

Marker frame: origin at the V's apex, x pointing into the marker (the heading of a robot docked in front of it); the
mouth opens towards -x, its corners at (-depth, +-width/2).

detect(points, predicted, width, depth):
  1. region of interest: scan points within `roi` of the predicted apex (the prediction comes from the map pose of
     the marker and the robot's localization, good to ~0.2 m / 10 deg);
  2. line segments by sequential RANSAC over scan-order-contiguous runs (a line must be one unbroken surface);
  3. the pair of segments whose angle matches the V's (180 - included angle) and whose intersection lies at the
     inner ends of both, with the opening facing the robot;
  4. least squares (Gauss-Newton, point-to-line) of the V template to the two segments' points -> (x, y, yaw);
  5. quality gate: points per arm, RMS residual, distance from the prediction.
Returns (x, y, yaw, info) or (None, reason).
"""
import math

import numpy as np


def _fit_line(pts):
    """Total least squares line: (centroid, unit direction)."""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    return c, vt[0]


def segments(pts, tol=0.008, min_pts=6, max_gap=0.04, iters=60, rng=None):
    """Line segments in an ordered point list: [(indices, centroid, direction)]. Sequential RANSAC; each accepted
    segment is the longest scan-order-contiguous run of inliers (gaps <= max_gap)."""
    rng = rng or np.random.default_rng(0)
    free = np.ones(len(pts), bool)
    out = []
    while free.sum() >= min_pts and len(out) < 6:
        idx = np.flatnonzero(free)
        best = None
        for _ in range(iters):
            a, b = rng.choice(idx, 2, replace=False)
            d = pts[b] - pts[a]
            n = math.hypot(d[0], d[1])
            if n < 0.02:
                continue
            normal = np.array([-d[1], d[0]]) / n
            inl = idx[np.abs((pts[idx] - pts[a]) @ normal) < tol]
            if len(inl) < min_pts:
                continue
            # longest contiguous run (consecutive in scan order, neighbours closer than max_gap; the first one on a
            # tie). Vectorized: the per-point Python loop took ~190 ms per scan with the marker 0.25 m from a
            # nanoScan3 (~470 points in the ROI) — the detector then published at ~5 Hz with gaps up to 0.93 s
            # against the docking server's 1 s detection timeout (dock test 2026-10-05)
            d = pts[inl[1:]] - pts[inl[:-1]]
            cut = np.flatnonzero(np.sqrt((d * d).sum(axis=1)) > max_gap) + 1
            starts = np.concatenate(([0], cut))
            ends = np.concatenate((cut, [len(inl)]))
            k = int(np.argmax(ends - starts))
            run = inl[starts[k]:ends[k]]
            if len(run) >= min_pts and (best is None or len(run) > len(best)):
                best = run
        if best is None:
            break
        best = np.array(best)
        c, u = _fit_line(pts[best])
        res = np.abs((pts[best] - c) @ np.array([-u[1], u[0]]))
        best = best[res < tol]
        if len(best) < min_pts:
            free[best] = False
            continue
        out.append((best, *_fit_line(pts[best])))
        free[best] = False
    return out


def _sides(pose, pts):
    """0 for points on the marker's +y side of its x axis (the left arm), 1 for the others."""
    x, y, th = pose
    return ((-math.sin(th) * (pts[:, 0] - x) + math.cos(th) * (pts[:, 1] - y)) < 0).astype(int)


def _residuals(pose, pts, sides, width, depth):
    """Signed point-to-arm-line distances and their Jacobian wrt (x, y, yaw)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    r, J = np.empty(len(pts)), np.empty((len(pts), 3))
    for k, sign in ((0, 1.0), (1, -1.0)):
        m = sides == k
        u = np.array([-depth, sign * width / 2])
        u /= np.linalg.norm(u)
        n = np.array([-u[1], u[0]])                      # arm normal, marker frame
        nw = np.array([c * n[0] - s * n[1], s * n[0] + c * n[1]])
        d = pts[m] - np.array([x, y])
        r[m] = d @ nw
        J[m, 0], J[m, 1] = -nw[0], -nw[1]
        J[m, 2] = d @ np.array([-nw[1], nw[0]])
    return r, J


def detect(points, predicted, width, depth, roi=0.40, angle_tol_deg=6.0, line_tol=0.008, max_rms=0.006,
           min_arm_pts=6, max_offset=0.40):
    """points: (N, 2) scan points in scan order (sensor frame); predicted: (x, y, yaw) of the marker frame in the
    same frame. line_tol / max_rms: set from the lidar's range noise (sigma): ~2.5 sigma / ~1.5 sigma, floors 8 / 6 mm.
    Returns ((x, y, yaw), info) or (None, reason)."""
    pts = np.asarray(points, float)
    px, py, pyaw = predicted
    if len(pts) == 0:
        return None, 'no points'
    near = np.hypot(pts[:, 0] - px, pts[:, 1] - py) < roi
    pts = pts[near]
    if len(pts) < 2 * min_arm_pts:
        return None, f'{len(pts)} points near the predicted marker'
    segs = segments(pts, tol=line_tol, min_pts=min_arm_pts, max_gap=max(0.04, 4 * line_tol))
    half = math.atan2(width / 2, depth)                  # arm angle from the -x axis of the marker frame
    want = math.pi - 2 * half                            # angle between the two arm lines
    best = None
    for i in range(len(segs)):
        for j in range(i + 1, len(segs)):
            (ia, ca, ua), (ib, cb, ub) = segs[i], segs[j]
            ang = math.acos(min(1.0, abs(float(ua @ ub))))
            if abs(ang - want) > math.radians(angle_tol_deg + 300 * line_tol):   # short noisy arms: looser angle
                continue
            A = np.column_stack([ua, -ub])
            if abs(np.linalg.det(A)) < 1e-6:
                continue
            ta, _ = np.linalg.solve(A, cb - ca)
            apex = ca + ta * ua
            # the arms must start at the apex and run away from it (inner ends meet)
            ok = True
            dirs = []
            for idx, c, u in ((ia, ca, ua), (ib, cb, ub)):
                proj = (pts[idx] - apex) @ u
                if proj.mean() < 0:
                    u, proj = -u, -proj
                if proj.min() > 0.05 or proj.max() > math.hypot(width / 2, depth) + 0.03 or proj.min() < -0.02:
                    ok = False
                dirs.append(u)
            if not ok:
                continue
            bis = dirs[0] + dirs[1]                       # points out of the mouth (towards the robot) = -x marker
            yaw = math.atan2(-bis[1], -bis[0])
            if abs(math.remainder(yaw - pyaw, 2 * math.pi)) > math.radians(30):
                continue
            score = len(ia) + len(ib) - 10 * math.hypot(apex[0] - px, apex[1] - py)
            if best is None or score > best[0]:
                best = (score, apex, yaw, ia, ib, dirs)
    if best is None:
        return None, f'no V among {len(segs)} segments'
    _, apex, yaw, ia, ib, dirs = best
    P = np.vstack([pts[ia], pts[ib]])
    pose = np.array([apex[0], apex[1], yaw])
    for it in range(12):
        if it == 4:                                       # drop outliers once roughly converged
            r, _ = _residuals(pose, P, _sides(pose, P), width, depth)
            P = P[np.abs(r) < max(3 * float(np.sqrt(np.mean(r ** 2))), line_tol)]
        # each point belongs to the arm on its side of the V's axis (near the apex a point lies within the RANSAC
        # tolerance of both arm lines, so segment membership alone mislabels some)
        sides = _sides(pose, P)
        r, J = _residuals(pose, P, sides, width, depth)
        step = np.linalg.lstsq(J, -r, rcond=None)[0]
        pose += step
        if np.abs(step).max() < 1e-7:
            break
    sides = _sides(pose, P)
    r, _ = _residuals(pose, P, sides, width, depth)
    rms = float(np.sqrt(np.mean(r ** 2)))
    n_l, n_r = int((sides == 0).sum()), int((sides == 1).sum())
    info = dict(rms=rms, left=n_l, right=n_r)
    if rms > max_rms or min(n_l, n_r) < min_arm_pts:
        return None, f'poor fit (rms {rms * 1000:.1f} mm, {n_l}/{n_r} points)'
    if math.hypot(pose[0] - px, pose[1] - py) > max_offset:
        return None, f'V found {math.hypot(pose[0] - px, pose[1] - py):.2f} m from the prediction'
    return (float(pose[0]), float(pose[1]), math.remainder(float(pose[2]), 2 * math.pi)), info


def template_scan(pose, width, depth, sensor=(0.0, 0.0), angle_min=-1.13, angle_max=1.13, step=math.radians(0.5),
                  face_half=0.25, noise=0.0, rng=None):
    """Synthetic scan of a V marker on a flat face (for tests): ray-cast from `sensor` against the two arms and the
    face beyond the mouth (marker frame face at x=0, |y| in [width/2, face_half]). Returns ordered (N, 2) points."""
    rng = rng or np.random.default_rng(1)
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    to_w = lambda p: np.array([x + c * p[0] - s * p[1], y + s * p[0] + c * p[1]])
    a = to_w((0.0, 0.0))
    segs = [(a, to_w((-depth, width / 2))), (a, to_w((-depth, -width / 2))),
            (to_w((0.0, width / 2)), to_w((0.0, face_half))), (to_w((0.0, -width / 2)), to_w((0.0, -face_half)))]
    o = np.array(sensor, float)
    out = []
    for ang in np.arange(angle_min, angle_max, step):
        d = np.array([math.cos(ang), math.sin(ang)])
        best = None
        for p0, p1 in segs:
            e = p1 - p0
            A = np.column_stack([d, -e])
            if abs(np.linalg.det(A)) < 1e-12:
                continue
            t, u = np.linalg.solve(A, p0 - o)
            if t > 0 and 0.0 <= u <= 1.0 and (best is None or t < best):
                best = t
        if best is not None:
            out.append(o + d * (best + (rng.normal(0, noise) if noise else 0.0)))
    return np.array(out)
