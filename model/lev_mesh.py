"""Mesh construction helpers for the Leviathan build (Blender side).

Surfaces are built as parametric patches (numpy), joined, and welded on their
shared boundaries, which gives watertight solids without booleans wherever
the geometry is a surface of revolution.
"""
from __future__ import annotations

import math

import bmesh
import bpy
import numpy as np

import lev_geom as G

D2R = G.D2R


# ----------------------------------------------------------------- basics
def mesh_from_arrays(name, verts, faces, smooth=True):
    """verts (N,3) float array, faces list of index tuples (tris/quads/ngons)."""
    me = bpy.data.meshes.new(name)
    verts = np.asarray(verts, dtype=np.float64)
    me.vertices.add(len(verts))
    me.vertices.foreach_set("co", verts.astype(np.float32).ravel())
    if isinstance(faces, np.ndarray) and faces.ndim == 2:
        nf, k = faces.shape
        flat = faces.astype(np.int32).ravel()
        starts = (np.arange(nf) * k).astype(np.int32)
    else:
        flat = np.asarray([i for f in faces for i in f], np.int32)
        lens = np.asarray([len(f) for f in faces], np.int32)
        starts = np.concatenate([[0], np.cumsum(lens)[:-1]]).astype(np.int32)
        nf = len(faces)
    me.loops.add(len(flat))
    me.loops.foreach_set("vertex_index", flat)
    me.polygons.add(nf)
    me.polygons.foreach_set("loop_start", starts)
    me.update(calc_edges=True)
    me.validate(clean_customdata=False)
    if smooth:
        me.polygons.foreach_set("use_smooth", np.ones(len(me.polygons), bool))
    return me


def grid_faces(nu, nv, wrap_u=False, offset=0):
    """Quad faces for a (nu x nv) vertex grid indexed [i*nv + j]."""
    fu = nu if wrap_u else nu - 1
    faces = np.empty((fu * (nv - 1), 4), np.int64)
    k = 0
    for i in range(fu):
        i2 = (i + 1) % nu
        for j in range(nv - 1):
            a = i * nv + j
            b = i2 * nv + j
            faces[k] = (a, b, b + 1, a + 1)
            k += 1
    return faces + offset


def weld(me, dist=1e-3, recalc=True):
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=dist)
    bmesh.ops.dissolve_degenerate(bm, dist=dist * 0.5, edges=bm.edges)
    if recalc:
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    me.update()


class PatchBuilder:
    """Accumulates parametric patches into one vertex/face list."""

    def __init__(self):
        self.verts = []
        self.faces = []
        self.mat = []
        self.n = 0

    def add_grid(self, P, wrap_u=False, mat=0, flip=False):
        """P: array (nu, nv, 3)."""
        nu, nv, _ = P.shape
        self.verts.append(P.reshape(-1, 3))
        f = grid_faces(nu, nv, wrap_u, self.n)
        if flip:
            f = f[:, ::-1]
        self.faces.append(f)
        self.mat.append(np.full(len(f), mat, np.int32))
        self.n += nu * nv

    def add_faces(self, V, F, mat=0):
        V = np.asarray(V, float)
        self.verts.append(V)
        F = [tuple(int(i) + self.n for i in f) for f in F]
        self.faces.append(F)
        self.mat.append(np.full(len(F), mat, np.int32))
        self.n += len(V)

    def build(self, name, smooth=True, do_weld=True, weld_dist=1e-3):
        V = np.concatenate(self.verts, 0)
        if all(isinstance(f, np.ndarray) and f.shape[1] == 4 for f in self.faces):
            faces = np.concatenate(self.faces, 0)
        else:
            faces = []
            for f in self.faces:
                faces.extend([tuple(x) for x in (f.tolist() if isinstance(f, np.ndarray) else f)])
        me = mesh_from_arrays(name, V, faces, smooth)
        mats = np.concatenate(self.mat)
        if len(mats) == len(me.polygons):
            me.polygons.foreach_set("material_index", mats)
        if do_weld:
            weld(me, weld_dist)
        return me


def revolve_grid(thetas, prof_fn):
    """prof_fn(theta_deg) -> array (nv, 2) of (r, z). Returns (nu, nv, 3) in ship XYZ."""
    cols = []
    for t in thetas:
        rz = np.asarray(prof_fn(t), float)
        s, c = math.sin(t * D2R), math.cos(t * D2R)
        cols.append(np.stack([rz[:, 0] * s, rz[:, 0] * c, rz[:, 1]], 1))
    return np.stack(cols, 0)


def lathe_local(profile_rz, n_seg, cx=0.0, cy=0.0):
    """Revolve an open (rho, z) polyline about a vertical axis at (cx, cy). Returns (n_seg, nv, 3)."""
    prof = np.asarray(profile_rz, float)
    a = np.linspace(0, 2 * math.pi, n_seg, endpoint=False)
    P = np.empty((n_seg, len(prof), 3))
    P[:, :, 0] = cx + prof[None, :, 0] * np.cos(a)[:, None]
    P[:, :, 1] = cy + prof[None, :, 0] * np.sin(a)[:, None]
    P[:, :, 2] = prof[None, :, 1]
    return P


def new_object(name, me, collection, mats=()):
    ob = bpy.data.objects.new(name, me)
    collection.objects.link(ob)
    for m in mats:
        me.materials.append(m)
    return ob


def box_mesh_verts(center, ax_u, ax_v, ax_w, hu, hv, hw):
    """8 corners of an oriented box. ax_* unit vectors; h* half extents."""
    c = np.asarray(center, float)
    U, V, W = (np.asarray(a, float) for a in (ax_u, ax_v, ax_w))
    pts = []
    for su in (-1, 1):
        for sv in (-1, 1):
            for sw in (-1, 1):
                pts.append(c + su * hu * U + sv * hv * V + sw * hw * W)
    return np.array(pts)


BOX_FACES = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]

