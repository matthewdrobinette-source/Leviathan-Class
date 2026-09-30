"""Interior allocation volumes (Annex B.5-B.11, C.12) as meshes."""
from __future__ import annotations

import math

import numpy as np

import lev_geom as G
from lev_mesh import PatchBuilder, lathe_local, mesh_from_arrays

D2R = G.D2R


def spine_xy(th, u, v):
    s, c = math.sin(th * D2R), math.cos(th * D2R)
    return u * s + v * c, u * c - v * s


def prism(pb, plan, z0, z1, mat=0):
    """Closed prism from a CCW-or-CW plan polygon (list of (x, y))."""
    n = len(plan)
    V = [(x, y, z0) for x, y in plan] + [(x, y, z1) for x, y in plan]
    F = [tuple(range(n))[::-1], tuple(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        F.append((i, j, n + j, n + i))
    pb.add_faces(V, F, mat)


def ring(pb, r0, r1, z0, z1, n_seg=1024, mat=0):
    prof = [(r0, z0), (r1, z0), (r1, z1), (r0, z1), (r0, z0)]
    P = lathe_local(prof, n_seg)
    # lathe_local measures its angle from +X; that is fine for a full ring
    pb.add_grid(P, wrap_u=True, mat=mat)


def volume_plan(v):
    """Plan polygon (x, y) for box, boxR and part-wedge volumes."""
    k, A, th = v["kind"], v["A"], v["th"]
    if k == "box":
        u0, u1, v0, v1 = A
        return [spine_xy(th, u0, v0), spine_xy(th, u1, v0), spine_xy(th, u1, v1), spine_xy(th, u0, v1)]
    if k == "boxR":
        u0, _, v0, v1 = A
        pts = [spine_xy(th, u0, v0)]
        for i in range(21):
            vv = v0 + (v1 - v0) * i / 20
            u = G.R_RIM
            for _ in range(8):
                x, y = spine_xy(th, u, vv)
                t = math.degrees(math.atan2(x, y)) % 360
                rt = G.r_out(t) - v["param"]
                u = math.sqrt(max(0.0, rt * rt - vv * vv))
            pts.append(spine_xy(th, u, vv))
        pts.append(spine_xy(th, u0, v1))
        return pts
    if k == "wedge":
        r0, r1, t0, t1 = A
        n = max(4, int(round((t1 - t0) / 360 * 1024)))
        outer = [G.xy(r1, th + t0 + (t1 - t0) * i / n) for i in range(n + 1)]
        inner = [G.xy(r0, th + t1 - (t1 - t0) * i / n) for i in range(n + 1)] if r0 > 0 else [(0.0, 0.0)]
        return outer + inner
    raise ValueError(k)


def shrunk(v, e):
    """Volume shrunk by e on every face, so touching neighbours leave a bulkhead."""
    if e == 0:
        return v
    v = dict(v)
    k, A = v["kind"], v["A"]
    v["z0"], v["z1"] = v["z0"] + e, v["z1"] - e
    if k in ("box", "boxR"):
        v["A"] = (A[0] + e, A[1] - e, A[2] + e, A[3] - e)
        if k == "boxR":
            v["param"] = v["param"] + e
    elif k == "wedge":
        full = (A[3] - A[2]) >= 359.9
        da = 0 if full else math.degrees(e / max(A[0], 1.0))
        v["A"] = (A[0] + e if A[0] > 0 else 0, A[1] - e, A[2] + da, A[3] - da)
    elif k == "sphere":
        v["A"] = (A[0], A[1] - e)
    return v


def add_volume(pb, v, mat=0, sphere_seg=48):
    k, A = v["kind"], v["A"]
    if k == "sphere":
        rc, rho = A
        x, y = spine_xy(v["th"], rc, 0.0)
        zc = (v["z0"] + v["z1"]) / 2
        prof = [(rho * math.sin(a), zc + rho * math.cos(a)) for a in np.linspace(0, math.pi, sphere_seg // 2 + 1)]
        pb.add_grid(lathe_local(prof, sphere_seg, x, y), wrap_u=True, mat=mat)
        return
    if k == "wedge" and (A[3] - A[2]) >= 359.9:
        ring(pb, A[0], A[1], v["z0"], v["z1"], mat=mat)
        return
    prism(pb, volume_plan(v), v["z0"], v["z1"], mat)


def dedupe(V, F, tol=1e-3):
    """Weld one closed piece: merge coincident vertices, drop collapsed faces."""
    V = np.asarray(V, float)
    key = np.round(V / tol).astype(np.int64)
    _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
    inv = inv.ravel()
    V2 = V[first]
    F2 = []
    for f in F:
        g = []
        for i in f:
            j = int(inv[i])
            if not g or g[-1] != j:
                g.append(j)
        if len(g) > 1 and g[0] == g[-1]:
            g.pop()
        if len(g) >= 3 and len(set(g)) == len(g):
            F2.append(tuple(g))
    return V2, F2


def volumes_mesh(name, vols, mat_of=None, shrink=0.0):
    """One mesh holding every volume as its own welded, closed piece."""
    allV, allF, allM, n = [], [], [], 0
    for v in vols:
        pb = PatchBuilder()
        add_volume(pb, shrunk(v, shrink), 0)
        V = np.concatenate(pb.verts, 0)
        F = []
        for f in pb.faces:
            F.extend([tuple(x) for x in (f.tolist() if isinstance(f, np.ndarray) else f)])
        V, F = dedupe(V, F)
        allV.append(V)
        allF.extend([tuple(i + n for i in f) for f in F])
        allM.extend([mat_of(v) if mat_of else 0] * len(F))
        n += len(V)
    me = mesh_from_arrays(name, np.concatenate(allV, 0), allF, smooth=False)
    me.polygons.foreach_set("material_index", np.asarray(allM, np.int32))
    import bmesh
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    return me


def cutter_sets(variant="model"):
    """Split the allocation into (voids, hard, soft) lists, dropping children
    (containment spheres) from the hull cutters, since their halls already cut."""
    V = G.allocation(variant)
    voids = [v for v in V if v["flags"] & G.VOID]
    hard = [v for v in V if not (v["flags"] & (G.VOID | G.SOFT))]
    soft = [v for v in V if v["flags"] & G.SOFT]
    return voids, hard, soft
