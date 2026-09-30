"""The nine cores (Section 3, Annex B.6), built in their own frames.

Local frame: centre at the origin, +Z north. For a Lance +X points radially
outward from the ship's axis, so its gallery runs along local Y. For
Praetorian +X points at the bow. Core longitude lambda is measured from +X
counter-clockwise, which makes a ship bearing b appear at lambda = theta_core - b.
"""
from __future__ import annotations

import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

import lev_geom as G
from lev_mesh import PatchBuilder, lathe_local, mesh_from_arrays
from lev_materials import MATS

D2R = G.D2R

CORE_MATS = ["Core shell, alusteel", "Core equatorial box, doonium", "Core south cap, refractory ceramic",
             "Core frame ring", "Aperture, dark", "Duranium, bright worn", "Sensor aperture", "Shield emitter",
             "Rothana hazard, yellow and black", "Iris shutter", "Repulsor array", "Engine bell, core drive",
             "Mount shutter", "Seam, dark"]
MI = {m: i for i, m in enumerate(CORE_MATS)}


class CoreType:
    def __init__(self, central):
        self.central = central
        if central:
            self.rc = 348.0
            self.box = (-30.0, 30.0)
            self.decks = (7.6, 18.8)
            self.n_wells, self.well_d = 48, 18.0
            self.nozzles, self.nozzle_d = 12, 45.0
            self.gear, self.gear_wh = 12, (45.0, 30.0)
            self.ramps, self.ramp_wh = 6, (50.0, 18.0)
            self.key_w, self.key_proud = 50.0, 20.0
            self.dogs = 24
            self.sensor_wh = (16.0, 8.0)
            self.ring_d = 60.0
            self.reactor = 100.0
            self.frames = (-30.0, -4.0, 13.0, 30.0)   # clear of decks, belts, dogs, rings
            self.name = "Praetorian"
        else:
            self.rc = 210.0
            self.box = (-30.0, 40.0)
            self.decks = (15.0, 35.0)
            self.n_wells, self.well_d = 32, 12.0
            self.nozzles, self.nozzle_d = 8, 30.0
            self.gear, self.gear_wh = 8, (30.0, 20.0)
            self.ramps, self.ramp_wh = 4, (40.0, 15.0)
            self.key_w, self.key_proud = 30.0, 12.0
            self.dogs = 16
            self.sensor_wh = (12.0, 6.0)
            self.ring_d = 40.0
            self.reactor = 60.0
            self.frames = (-30.0, -3.0, 25.0, 40.0)
            self.name = "Lance"

    # hangar mouth longitudes (local)
    def mouth_lons(self):
        return [45.0 * k for k in range(8)] if self.central else [90.0, 270.0]

    def key_lons(self):
        # clocked to the web longitudes: Praetorian 45n (split clear of its mouths), a Lance at 22.5 + 45n
        return [45.0 * k for k in range(8)] if self.central else [22.5 + 45.0 * k for k in range(8)]

    def key_lat_range(self):
        return (24.0, 60.0) if self.central else (2.0, 60.0)

    def well_lons(self):
        if not self.central:
            return [5.625 + 11.25 * k for k in range(32)]
        # grouped six to each gap between the eight mouths (see verification report)
        half = math.degrees(30.0 / (self.rc * math.cos(7.6 * D2R)))
        wh = math.degrees(self.well_d / 2 / (self.rc * math.cos(8 * D2R)))
        a, b = half + wh + 1.0, 45.0 - half - wh - 1.0
        return [45.0 * m + a + (b - a) * k / 5 for m in range(8) for k in range(6)]


def sph(rc, lat, lon, h=0.0):
    la, lo = lat * D2R, lon * D2R
    return Vector((math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la))) * (rc + h)


def enu(lat, lon):
    la, lo = lat * D2R, lon * D2R
    n = Vector((math.cos(la) * math.cos(lo), math.cos(la) * math.sin(lo), math.sin(la)))
    e = Vector((-math.sin(lo), math.cos(lo), 0.0))
    u = n.cross(e)       # north, tangent
    return e, u, n


def shell_mesh(ct: CoreType):
    """Sphere with the four ring frames of the equatorial box standing 1 m proud."""
    rc = ct.rc
    s0, s1 = ct.box
    frames = ct.frames
    w = math.degrees(6.0 / rc)
    lats = sorted(set(np.round(np.concatenate([np.linspace(-90, 90, 181)] +
                                              [[f - w / 2 - 0.05, f - w / 2, f + w / 2, f + w / 2 + 0.05] for f in frames]), 4)))
    prof = []
    for la in lats:
        bump = 1.0 if any(abs(la - f) <= w / 2 + 1e-6 for f in frames) else 0.0
        rr = rc + bump
        prof.append((rr * math.cos(la * D2R), rr * math.sin(la * D2R)))
    prof[0] = (0.0, -rc)
    prof[-1] = (0.0, rc)
    P = lathe_local(prof, 256 if ct.central else 192)
    pb = PatchBuilder()
    pb.add_grid(P, wrap_u=True)
    me = pb.build(ct.name + " shell", weld_dist=1e-3)
    # materials by latitude
    mats = []
    for poly in me.polygons:
        c = poly.center
        la = math.degrees(math.asin(max(-1, min(1, c.z / c.length))))
        if la < -50:
            m = MI["Core south cap, refractory ceramic"]
        elif any(abs(la - f) <= w / 2 + 0.01 for f in frames):
            m = MI["Core frame ring"]
        elif s0 <= la <= s1:
            m = MI["Core equatorial box, doonium"]
        else:
            m = MI["Core shell, alusteel"]
        mats.append(m)
    me.polygons.foreach_set("material_index", np.asarray(mats, np.int32))
    return me


def oriented_box(pb, center, e, u, n, we, hu, dn_in, dn_out, mat):
    """Box on the surface frame: width we along e, height hu along u, from
    -dn_in to +dn_out along n."""
    V = []
    for a in (-we / 2, we / 2):
        for b in (-hu / 2, hu / 2):
            for c in (-dn_in, dn_out):
                V.append(center + a * e + b * u + c * n)
    F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    pb.add_faces([tuple(v) for v in V], F, mat)


def cutters(ct: CoreType):
    """Cut solids (in the core frame) and their material slots."""
    rc = ct.rc
    pieces = []   # (PatchBuilder-ready closed solids)

    def solid():
        pb = PatchBuilder()
        pieces.append(pb)
        return pb
    # hangar decks
    for lat in ct.decks:
        z = rc * math.sin(lat * D2R)
        pb = solid()
        if not ct.central:
            # gallery through-corridor, 60 x 20 m, along local Y: mouths at both ends
            half = math.sqrt(rc ** 2 - z ** 2) + 30
            oriented_box(pb, Vector((0, 0, z)), Vector((1, 0, 0)), Vector((0, 0, 1)), Vector((0, 1, 0)), 60.0, 20.0, half, half, MI["Aperture, dark"])
        else:
            # eight radial bays to an annular hangar clear of the 100 m containment
            for lon in ct.mouth_lons():
                d = Vector((math.cos(lon * D2R), math.sin(lon * D2R), 0))
                e = Vector((-d.y, d.x, 0))
                L = math.sqrt(rc ** 2 - z ** 2) + 30
                oriented_box(solid(), Vector((0, 0, z)) + d * (140 + L) / 2, e, Vector((0, 0, 1)), d, 60.0, 20.0,
                             (L - 140) / 2, (L - 140) / 2, MI["Aperture, dark"])
            prof = [(120, z - 10), (200, z - 10), (200, z + 10), (120, z + 10), (120, z - 10)]
            pb.add_grid(lathe_local(prof, 96), wrap_u=True, mat=MI["Aperture, dark"])
    # reactor hall around the containment sphere
    pb = solid()
    rh = ct.reactor / 2 + 12
    prof = [(rh * math.sin(a), rh * math.cos(a)) for a in np.linspace(0, math.pi, 25)]
    pb.add_grid(lathe_local(prof, 32), wrap_u=True, mat=MI["Core frame ring"])
    # drive cluster nozzle wells above 60 N
    lat_n = 73.0 if not ct.central else 72.0
    for k in range(ct.nozzles):
        lon = 360.0 * k / ct.nozzles
        e, u, n = enu(lat_n, lon)
        c = sph(rc, lat_n, lon)
        pb = solid()
        R = ct.nozzle_d / 2
        a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
        V = [tuple(c + R * (math.cos(t) * e + math.sin(t) * u) - 22 * n) for t in a] + \
            [tuple(c + R * (math.cos(t) * e + math.sin(t) * u) + 12 * n) for t in a]
        F = [tuple(range(24))[::-1], tuple(range(24, 48))] + [(i, (i + 1) % 24, 24 + (i + 1) % 24, 24 + i) for i in range(24)]
        pb.add_faces(V, F, MI["Aperture, dark"])
    # key channels, meridional, clocked to the web longitudes
    la0, la1 = ct.key_lat_range()
    depth = ct.key_proud + 2.0
    for lon in ct.key_lons():
        pb = solid()
        lats = np.linspace(la0, la1, 40)
        ring = []
        for la in lats:
            hw = math.degrees(math.asin(min(0.99, ct.key_w / 2 / (rc * math.cos(la * D2R)))))
            ring.append([sph(rc, la, lon - hw, -depth), sph(rc, la, lon - hw, 6), sph(rc, la, lon + hw, 6), sph(rc, la, lon + hw, -depth)])
        P = np.array([[tuple(ring[i][j]) for i in range(len(lats))] for j in range(4)])
        pb.add_grid(P, wrap_u=True, mat=MI["Core frame ring"])
        pb.add_faces([tuple(ring[0][j]) for j in range(4)], [(0, 1, 2, 3)], MI["Core frame ring"])
        pb.add_faces([tuple(ring[-1][j]) for j in range(4)], [(3, 2, 1, 0)], MI["Core frame ring"])
    # locking dog sockets on the equatorial box, at the bowl lip
    for k in range(ct.dogs):
        lon = 360.0 * (k + 0.5) / ct.dogs
        e, u, n = enu(1.5, lon)
        oriented_box(solid(), sph(rc, 1.5, lon), e, u, n, 12.0, 7.0, 5.0, 6.0, MI["Duranium, bright worn"])
    return pieces


def pieces_mesh(name, pieces):
    allV, allF, allM, off = [], [], [], 0
    from lev_interior import dedupe
    for pb in pieces:
        V = np.concatenate(pb.verts, 0)
        F, M = [], []
        for f, m in zip(pb.faces, pb.mat):
            fl = f.tolist() if isinstance(f, np.ndarray) else f
            F.extend([tuple(x) for x in fl])
            M.extend(list(m))
        # dedupe keeps face order only for surviving faces, so weld materials by rebuilding per face
        Vd, Fd = dedupe(V, F)
        allV.append(Vd)
        allF.extend([tuple(i + off for i in f) for f in Fd])
        allM.extend([M[0]] * len(Fd))
        off += len(Vd)
    me = mesh_from_arrays(name, np.concatenate(allV, 0), allF, smooth=False)
    me.polygons.foreach_set("material_index", np.asarray(allM, np.int32))
    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(me)
    bm.free()
    return me


def decals(ct: CoreType):
    """Surface fit-out that needs no boolean: thruster well irises, gear bays,
    ramps, southern closures, sensors, shield rings, the repulsor caps, engine
    bells, hazard framing of the hangar mouths. Every vertex is conformed to
    the sphere, so large panels don't stand off it."""
    rc = ct.rc
    pb = PatchBuilder()

    def conf(lat, lon, a, b, h):
        e, u, n = enu(lat, lon)
        p = sph(rc, lat, lon) + a * e + b * u
        return tuple(p.normalized() * (rc + h))

    def cbox(lat, lon, we, hu, h0, h1, mat, da=0.0, db=0.0):
        V = [conf(lat, lon, da + sa * we / 2, db + sb * hu / 2, hh)
             for sa in (-1, 1) for sb in (-1, 1) for hh in (h0, h1)]
        F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        pb.add_faces(V, F, mat)

    def disc(lat, lon, R, mat, h=0.25, n=20, blades=0, blade_mat=None, inner=0.0):
        a = np.linspace(0, 2 * math.pi, n, endpoint=False)
        if inner <= 0:
            V = [conf(lat, lon, 0, 0, h)] + [conf(lat, lon, R * math.cos(t), R * math.sin(t), h) for t in a]
            F = [(0, i + 1, (i + 1) % n + 1) for i in range(n)]
        else:
            V = [conf(lat, lon, inner * math.cos(t), inner * math.sin(t), h) for t in a] + \
                [conf(lat, lon, R * math.cos(t), R * math.sin(t), h) for t in a]
            F = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
        pb.add_faces(V, F, mat)
        for bl in range(blades):
            t = 2 * math.pi * bl / blades
            ca, sa = math.cos(t), math.sin(t)
            w = max(0.2, R * 0.05)
            P = [(0, 0), (R * ca, R * sa), (R * ca - w * sa, R * sa + w * ca), (-w * sa, w * ca)]
            pb.add_faces([conf(lat, lon, x, y, h + 0.06) for x, y in P], [(0, 1, 2, 3)], blade_mat)

    def panel(lat, lon, w, hgt, mat, h=0.3, frame=None):
        cbox(lat, lon, w, hgt, -0.5, h, mat)
        if frame is not None:
            for (a, b, fw, fh) in ((0, hgt / 2 + 1.5, w + 6, 3), (0, -hgt / 2 - 1.5, w + 6, 3),
                                   (w / 2 + 1.5, 0, 3, hgt), (-w / 2 - 1.5, 0, 3, hgt)):
                cbox(lat, lon, fw, fh, -0.5, h * 0.8, frame, a, b)
    # thruster belts at +-8, two-part closures shown flush (iris)
    for lon in ct.well_lons():
        for lat in (8.0, -8.0):
            disc(lat, lon, ct.well_d / 2, MI["Iris shutter"], blades=6, blade_mat=MI["Seam, dark"])
    # landing gear bays, ramps, southern batteries (Rothana hazard framing)
    for k in range(ct.gear):
        panel(-40.0, 360.0 * k / ct.gear, ct.gear_wh[0], ct.gear_wh[1], MI["Mount shutter"], frame=MI["Rothana hazard, yellow and black"])
    for k in range(ct.ramps):
        lon = 360.0 * (k + 0.5) / ct.ramps
        panel(-40.0, lon, ct.ramp_wh[0], ct.ramp_wh[1], MI["Iris shutter"], frame=MI["Rothana hazard, yellow and black"])
    if not ct.central:
        for k in range(4):
            disc(-45.0, 67.5 + 90 * k, 7.5, MI["Mount shutter"], blades=6, blade_mat=MI["Seam, dark"])
            disc(-35.0, 67.5 + 90 * k, 7.5, MI["Mount shutter"], blades=6, blade_mat=MI["Seam, dark"])
    else:
        for k in range(12):
            disc(-47.0, 15.0 + 30 * k, 7.5, MI["Mount shutter"], blades=6, blade_mat=MI["Seam, dark"])
    # sensor apertures and shield rings on the box
    for k in range(8):
        panel(-20.0, 22.5 + 45 * k, ct.sensor_wh[0], ct.sensor_wh[1], MI["Sensor aperture"], frame=MI["Seam, dark"])
    for k in range(4):
        disc(-12.0, 45.0 + 90 * k, ct.ring_d / 2, MI["Shield emitter"], h=0.3, n=48, inner=ct.ring_d / 2 * 0.72)
    # hazard framing round each hangar mouth
    for lat in ct.decks:
        for lon in ct.mouth_lons():
            for (a, b, fw, fh) in ((0, 13.5, 72, 3), (0, -13.5, 72, 3), (34.5, 0, 3, 24), (-34.5, 0, 3, 24)):
                cbox(lat, lon, fw, fh, -1.0, 0.25, MI["Rothana hazard, yellow and black"], a, b)
    # north polar repulsor array (field projector, no aperture)
    for ring_i, (lat, n) in enumerate(((86.0, 8), (82.0, 16))):
        for k in range(n):
            disc(lat, 360.0 * (k + 0.5 * ring_i) / n, rc * 0.018 * (2 - ring_i * 0.4), MI["Repulsor array"], h=0.2, n=6)
    # south polar cap: the ventral repulsor array, a ring of flush projector plates
    for k in range(12):
        disc(-80.0, 30.0 * k, rc * 0.022, MI["Repulsor array"], h=0.15, n=6)
    # engine bells deep in the nozzle wells
    lat_n = 73.0 if not ct.central else 72.0
    for k in range(ct.nozzles):
        lon = 360.0 * k / ct.nozzles
        e, u, nn = enu(lat_n, lon)
        c = sph(rc, lat_n, lon)
        R = ct.nozzle_d / 2 * 0.86
        prof = [(R * 0.35, -21.0), (R * 0.55, -16.0), (R * 0.9, -7.0), (R, -3.0)]
        P = lathe_local(prof, 24)
        M = Matrix((e, u, nn)).transposed()
        P2 = np.array([[tuple(c + M @ Vector(v)) for v in row] for row in P])
        pb.add_grid(P2, wrap_u=True, mat=MI["Engine bell, core drive"])
    me = pb.build(ct.name + " fit-out", smooth=False, do_weld=False)
    return me


def build_core_mesh(ct: CoreType, coll):
    """Shell minus cutters (boolean, applied), joined with the decals."""
    shell = shell_mesh(ct)
    for m in CORE_MATS:
        shell.materials.append(MATS[m])
    ob = bpy.data.objects.new(ct.name + " (build)", shell)
    coll.objects.link(ob)
    cut_me = pieces_mesh(ct.name + " cutters", cutters(ct))
    for m in CORE_MATS:
        cut_me.materials.append(MATS[m])
    cut = bpy.data.objects.new(ct.name + " cutters", cut_me)
    coll.objects.link(cut)
    mod = ob.modifiers.new("cut", "BOOLEAN")
    mod.operation, mod.solver, mod.object = "DIFFERENCE", "EXACT", cut
    mod.use_self = True
    mod.material_mode = "TRANSFER"
    dg = bpy.context.evaluated_depsgraph_get()
    ev = ob.evaluated_get(dg)
    me = bpy.data.meshes.new_from_object(ev)
    me.name = ct.name + " core"
    dec = decals(ct)
    for m in CORE_MATS:
        dec.materials.append(MATS[m])
    dec.name = ct.name + " fit-out"
    me.polygons.foreach_set("use_smooth", np.zeros(len(me.polygons), bool))
    # smooth the shell faces only
    for p in me.polygons:
        if me.materials[p.material_index].name in ("Core shell, alusteel", "Core equatorial box, doonium",
                                                    "Core south cap, refractory ceramic", "Core frame ring") and p.area > 20:
            p.use_smooth = True
    bpy.data.objects.remove(ob)
    bpy.data.objects.remove(cut)
    bpy.data.meshes.remove(shell)
    bpy.data.meshes.remove(cut_me)
    return me, dec
