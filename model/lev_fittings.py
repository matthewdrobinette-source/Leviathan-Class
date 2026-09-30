"""Fittings: mounts, sensors, emitters, apertures, instanced with Geometry Nodes.

Each class of fitting is a point cloud (one per hull slab, so it follows the
de-stack) carrying a 'rot' quaternion and 'scale' vector per point. A shared
node group instances the class's mount geometry on the points, raises it by
Deploy x Stroke along the local normal, shows a flush shutter when stowed
and an open aperture ring when deployed, and deletes points behind the
section plane when the section is on.
"""
from __future__ import annotations

import math

import bpy
import numpy as np
from mathutils import Matrix, Vector

import lev_geom as G
from lev_mesh import PatchBuilder, lathe_local, mesh_from_arrays
from lev_materials import MATS

D2R = G.D2R


# ============================================================ node group
def instancer_group():
    name = "LEV Fitting Instancer"
    if name in bpy.data.node_groups:
        return bpy.data.node_groups[name]
    ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
    it = ng.interface
    it.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    it.new_socket("Mount", in_out="INPUT", socket_type="NodeSocketObject")
    it.new_socket("Stowed", in_out="INPUT", socket_type="NodeSocketObject")
    it.new_socket("Open", in_out="INPUT", socket_type="NodeSocketObject")
    s = it.new_socket("Deploy", in_out="INPUT", socket_type="NodeSocketFloat")
    s.min_value, s.max_value, s.default_value = 0.0, 1.0, 0.0
    s = it.new_socket("Stroke", in_out="INPUT", socket_type="NodeSocketFloat")
    s.default_value = 0.0
    it.new_socket("Section", in_out="INPUT", socket_type="NodeSocketBool")
    s = it.new_socket("Section Normal", in_out="INPUT", socket_type="NodeSocketVector")
    s.default_value = (1.0, 0.0, 0.0)
    it.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    N, L = ng.nodes, ng.links
    gi = N.new("NodeGroupInput")
    go = N.new("NodeGroupOutput")
    # section: delete points with dot(P_world, n) > 0 when enabled
    pos = N.new("GeometryNodeInputPosition")
    selfo = N.new("GeometryNodeSelfObject")
    oinfo = N.new("GeometryNodeObjectInfo")
    oinfo.transform_space = "ORIGINAL"
    L.new(selfo.outputs[0], oinfo.inputs["Object"])
    tp = N.new("FunctionNodeTransformPoint")
    L.new(pos.outputs[0], tp.inputs["Vector"])
    L.new(oinfo.outputs["Transform"], tp.inputs["Transform"])
    dot = N.new("ShaderNodeVectorMath")
    dot.operation = "DOT_PRODUCT"
    L.new(tp.outputs[0], dot.inputs[0])
    L.new(gi.outputs["Section Normal"], dot.inputs[1])
    cmp = N.new("FunctionNodeCompare")
    cmp.data_type = "FLOAT"
    cmp.operation = "GREATER_THAN"
    L.new(dot.outputs["Value"], cmp.inputs[0])
    cmp.inputs[1].default_value = 0.0
    band = N.new("FunctionNodeBooleanMath")
    band.operation = "AND"
    L.new(gi.outputs["Section"], band.inputs[0])
    L.new(cmp.outputs[0], band.inputs[1])
    dele = N.new("GeometryNodeDeleteGeometry")
    dele.domain = "POINT"
    L.new(gi.outputs["Geometry"], dele.inputs["Geometry"])
    L.new(band.outputs[0], dele.inputs["Selection"])
    pts = dele.outputs[0]
    rot = N.new("GeometryNodeInputNamedAttribute")
    rot.data_type = "QUATERNION"
    rot.inputs["Name"].default_value = "rot"
    scl = N.new("GeometryNodeInputNamedAttribute")
    scl.data_type = "FLOAT_VECTOR"
    scl.inputs["Name"].default_value = "scale"

    def inst(obj_socket):
        oi = N.new("GeometryNodeObjectInfo")
        oi.transform_space = "ORIGINAL"
        L.new(obj_socket, oi.inputs["Object"])
        ip = N.new("GeometryNodeInstanceOnPoints")
        L.new(pts, ip.inputs["Points"])
        L.new(oi.outputs["Geometry"], ip.inputs["Instance"])
        L.new(rot.outputs["Attribute"], ip.inputs["Rotation"])
        L.new(scl.outputs["Attribute"], ip.inputs["Scale"])
        return ip
    mount = inst(gi.outputs["Mount"])
    # raise the mount by (Deploy - 1) * Stroke so it sits flush-hidden when stowed
    sub = N.new("ShaderNodeMath")
    sub.operation = "SUBTRACT"
    L.new(gi.outputs["Deploy"], sub.inputs[0])
    sub.inputs[1].default_value = 1.0
    mul = N.new("ShaderNodeMath")
    mul.operation = "MULTIPLY"
    L.new(sub.outputs[0], mul.inputs[0])
    L.new(gi.outputs["Stroke"], mul.inputs[1])
    cxyz = N.new("ShaderNodeCombineXYZ")
    L.new(mul.outputs[0], cxyz.inputs["Z"])
    tr = N.new("GeometryNodeTranslateInstances")
    L.new(mount.outputs[0], tr.inputs["Instances"])
    L.new(cxyz.outputs[0], tr.inputs["Translation"])
    tr.inputs["Local Space"].default_value = True
    stowed = inst(gi.outputs["Stowed"])
    opened = inst(gi.outputs["Open"])
    dep = N.new("FunctionNodeCompare")
    dep.data_type = "FLOAT"
    dep.operation = "GREATER_THAN"
    L.new(gi.outputs["Deploy"], dep.inputs[0])
    dep.inputs[1].default_value = 0.01
    L.new(dep.outputs[0], mount.inputs["Selection"])   # stowed: shutter only, the mount is struck below
    sw = N.new("GeometryNodeSwitch")
    sw.input_type = "GEOMETRY"
    L.new(dep.outputs[0], sw.inputs["Switch"])
    L.new(stowed.outputs[0], sw.inputs["False"])
    L.new(opened.outputs[0], sw.inputs["True"])
    join = N.new("GeometryNodeJoinGeometry")
    L.new(sw.outputs[0], join.inputs[0])
    L.new(tr.outputs[0], join.inputs[0])
    L.new(join.outputs[0], go.inputs[0])
    for i, n in enumerate(ng.nodes):
        n.location = (i * 180, 0)
    return ng


def section_delete_group():
    """Deletes faces of any mesh whose centre lies on the removed side."""
    name = "LEV Section Delete"
    if name in bpy.data.node_groups:
        return bpy.data.node_groups[name]
    ng = bpy.data.node_groups.new(name, "GeometryNodeTree")
    it = ng.interface
    it.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    it.new_socket("Section", in_out="INPUT", socket_type="NodeSocketBool")
    s = it.new_socket("Section Normal", in_out="INPUT", socket_type="NodeSocketVector")
    s.default_value = (1.0, 0.0, 0.0)
    it.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    N, L = ng.nodes, ng.links
    gi, go = N.new("NodeGroupInput"), N.new("NodeGroupOutput")
    pos = N.new("GeometryNodeInputPosition")
    selfo = N.new("GeometryNodeSelfObject")
    oinfo = N.new("GeometryNodeObjectInfo")
    oinfo.transform_space = "ORIGINAL"
    L.new(selfo.outputs[0], oinfo.inputs["Object"])
    tp = N.new("FunctionNodeTransformPoint")
    L.new(pos.outputs[0], tp.inputs["Vector"])
    L.new(oinfo.outputs["Transform"], tp.inputs["Transform"])
    dot = N.new("ShaderNodeVectorMath")
    dot.operation = "DOT_PRODUCT"
    L.new(tp.outputs[0], dot.inputs[0])
    L.new(gi.outputs["Section Normal"], dot.inputs[1])
    cmp = N.new("FunctionNodeCompare")
    cmp.data_type, cmp.operation = "FLOAT", "GREATER_THAN"
    L.new(dot.outputs["Value"], cmp.inputs[0])
    band = N.new("FunctionNodeBooleanMath")
    band.operation = "AND"
    L.new(gi.outputs["Section"], band.inputs[0])
    L.new(cmp.outputs[0], band.inputs[1])
    dele = N.new("GeometryNodeDeleteGeometry")
    dele.domain = "FACE"
    L.new(gi.outputs["Geometry"], dele.inputs["Geometry"])
    L.new(band.outputs[0], dele.inputs["Selection"])
    L.new(dele.outputs[0], go.inputs[0])
    return ng


# ======================================================= fitting geometry
def _cyl(pb, r, z0, z1, n=32, mat=0, cap_top=True, cap_bot=False, cx=0.0, cy=0.0):
    prof = []
    if cap_bot:
        prof.append((0.0, z0))
    prof += [(r, z0), (r, z1)]
    if cap_top:
        prof.append((0.0, z1))
    pb.add_grid(lathe_local(prof, n, cx, cy), wrap_u=True, mat=mat)


def _box(pb, cx, cy, cz, sx, sy, sz, mat=0, rot=None):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    P = []
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            for dz in (-hz, hz):
                v = Vector((dx, dy, dz))
                if rot is not None:
                    v = rot @ v
                P.append((cx + v.x, cy + v.y, cz + v.z))
    F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    pb.add_faces(P, F, mat)


def _barrel(pb, x0, y, z, length, radius, elev_deg, mat, n=12):
    """Barrel along +X from (x0, y, z), elevated."""
    e = elev_deg * D2R
    rot = Matrix.Rotation(-e, 3, "Y")
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    V, F = [], []
    for s in (0.0, length):
        for ang in a:
            v = rot @ Vector((s, radius * math.cos(ang), radius * math.sin(ang)))
            V.append((x0 + v.x, y + v.y, z + v.z))
    for i in range(n):
        j = (i + 1) % n
        F.append((i, j, n + j, n + i))
    F.append(tuple(range(n))[::-1])
    F.append(tuple(range(n, 2 * n)))
    pb.add_faces(V, F, mat)


def _oct_prism(pb, cx, R, z0, z1, top_scale=0.8, mat=0, n=8):
    """Faceted gun house: an n-sided prism, chamfered toward its roof."""
    a = [2 * math.pi * (k + 0.5) / n for k in range(n)]
    lo = [(cx + R * math.cos(t), R * math.sin(t), z0) for t in a]
    mid = [(cx + R * math.cos(t), R * math.sin(t), z0 + (z1 - z0) * 0.6) for t in a]
    hi = [(cx + R * top_scale * math.cos(t), R * top_scale * math.sin(t), z1) for t in a]
    V = lo + mid + hi
    F = [tuple(range(n))[::-1], tuple(range(2 * n, 3 * n))]
    for i in range(n):
        j = (i + 1) % n
        F.append((i, j, n + j, n + i))
        F.append((n + i, n + j, 2 * n + j, 2 * n + i))
    pb.add_faces(V, F, mat)


def mount_object(name, aperture, stroke, coll, twin=True, barrel_len=None, elev=35.0):
    """Gun mount in its local frame: +Z the outward normal, +X the training
    direction. Built with its top at z = +stroke; the instancer lowers it by
    (1 - Deploy) x Stroke, so it sits flush-hidden when stowed and stands the
    full stroke proud when deployed."""
    pb = PatchBuilder()
    R = aperture / 2
    h = stroke
    ring_top = h * 0.35
    _cyl(pb, R * 0.97, -4.0, ring_top, 40, 2, cap_top=True, cap_bot=True)            # barbette trunk and turntable
    _oct_prism(pb, 0.0, R * 0.82, ring_top, h, 0.72, 0)                               # faceted gun house
    bl = barrel_len or aperture * 0.6
    br = max(0.35, aperture * 0.04)
    zc = ring_top + (h - ring_top) * 0.45
    offs = (-R * 0.26, R * 0.26) if twin else (0.0,)
    for yy in offs:
        _barrel(pb, R * 0.55, yy, zc, bl, br, elev, 1)
    me = pb.build(name, smooth=False, do_weld=False)
    for m in ("Turret, quadanium", "Turret barrel", "Core frame ring"):
        me.materials.append(MATS[m])
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def disc_object(name, radius, mats, coll, z=0.12, n=40, ring_inner=None, iris_blades=0):
    """Flush disc (shutter) or annulus (open aperture), optional iris seams."""
    pb = PatchBuilder()
    if ring_inner is None:
        prof = [(0.0, z), (radius, z)]
    else:
        prof = [(ring_inner, z), (radius, z)]
    pb.add_grid(lathe_local(prof, n)[:, ::-1, :], wrap_u=True, mat=0)
    if iris_blades:
        for k in range(iris_blades):
            a = 2 * math.pi * k / iris_blades
            ca, sa = math.cos(a), math.sin(a)
            w = max(0.15, radius * 0.02)
            P = [(0, 0, z + 0.05), (radius * ca, radius * sa, z + 0.05),
                 (radius * ca - w * sa, radius * sa + w * ca, z + 0.05), (-w * sa, w * ca, z + 0.05)]
            pb.add_faces(P, [(0, 1, 2, 3)], 1)
        prof = [(radius - max(0.3, radius * 0.04), z + 0.05), (radius, z + 0.05)]
        pb.add_grid(lathe_local(prof, n)[:, ::-1, :], wrap_u=True, mat=1)
    me = pb.build(name, smooth=False, do_weld=False)
    for m in mats:
        me.materials.append(MATS[m])
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def panel_object(name, sx, sy, mat, coll, z=0.25, frame_mat=None):
    pb = PatchBuilder()
    _box(pb, 0, 0, z / 2, sx, sy, z, 0)
    if frame_mat:
        for (cx, cy, w, h) in ((0, sy / 2, sx + 1, 0.8), (0, -sy / 2, sx + 1, 0.8), (sx / 2, 0, 0.8, sy), (-sx / 2, 0, 0.8, sy)):
            _box(pb, cx, cy, z / 2 + 0.05, w, h, z + 0.1, 1)
    me = pb.build(name, smooth=False, do_weld=False)
    me.materials.append(MATS[mat])
    if frame_mat:
        me.materials.append(MATS[frame_mat])
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def torus_object(name, R, r, mat, coll, n=64, m=12):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    b = np.linspace(0, 2 * math.pi, m, endpoint=False)
    V = []
    for ai in a:
        for bi in b:
            V.append(((R + r * math.cos(bi)) * math.cos(ai), (R + r * math.cos(bi)) * math.sin(ai), r * math.sin(bi)))
    F = []
    for i in range(n):
        for j in range(m):
            i2, j2 = (i + 1) % n, (j + 1) % m
            F.append((i * m + j, i2 * m + j, i2 * m + j2, i * m + j2))
    me = mesh_from_arrays(name, V, F, smooth=True)
    me.materials.append(MATS[mat])
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


def dome_object(name, r, mat, coll):
    prof = [(r * math.sin(a), r * math.cos(a) - r * 0.3) for a in np.linspace(0, math.pi / 2, 9)]
    pb = PatchBuilder()
    pb.add_grid(lathe_local(prof, 24), wrap_u=True)
    me = pb.build(name, smooth=True)
    me.materials.append(MATS[mat])
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


# ======================================================= placement maths
def frame_quat(n, f):
    n = Vector(n).normalized()
    f = Vector(f)
    f = (f - f.dot(n) * n)
    if f.length < 1e-6:
        f = Vector((1, 0, 0)) - n.x * n
    f.normalize()
    y = n.cross(f)
    M = Matrix((f, y, n)).transposed()
    q = M.to_quaternion()
    return (q.w, q.x, q.y, q.z)


def radial(t):
    return Vector((math.sin(t * D2R), math.cos(t * D2R), 0.0))


def tangent(t):
    return Vector((math.cos(t * D2R), -math.sin(t * D2R), 0.0))


def toward_bow(t):
    """Unit tangent pointing toward the bow along the rim at bearing t."""
    tt = t % 360
    return -tangent(t) if tt < 180 else tangent(t)


def cap_normal(r, t):
    r = float(r)
    s = float(G.cap_slope(r))
    return (Vector((0, 0, 1)) - s * radial(t)).normalized()


def dome_normal(r, t):
    r = float(r)
    s = float(G.dome_slope(r))
    return (Vector((0, 0, 1)) - s * radial(t)).normalized()


def crown_normal(r, t):
    r = float(r)
    s = float(G.crown_slope(r))
    return (Vector((0, 0, -1)) + s * radial(t)).normalized()


def knuckle_normal(r, t):
    nr, nz = G.knuckle_normal(float(r))
    return (nr * radial(t) + Vector((0, 0, nz))).normalized()


def layer_for(r, t, z):
    if G.in_aft_arc(t) and r >= G.R_AFT_BLOCK:
        return "aft"
    if r <= 450 and z >= -10:
        return "cap"
    for lid, z0, z1 in SLAB_Z:
        if z0 <= z < z1:
            return lid
    return "dorsal" if z >= -50 else "ventral"


# de-stack slabs (B.12), assembled boundaries as in the HTML datum model
SLAB_Z = [("ventral", -2000, -506), ("citadel", -506, -248), ("lowergallery", -248, -224),
          ("handling", -224, -182), ("uppergallery", -182, -158), ("berthing", -158, -50), ("dorsal", -50, 2000)]


class Cloud:
    """Point cloud per slab for one fitting class."""

    def __init__(self, key):
        self.key = key
        self.pts: dict[str, list] = {}

    def add(self, p, n, f, scale=(1, 1, 1), layer=None):
        x, y, z = p
        r, t = G.polar(x, y)
        lid = layer or layer_for(r, t, z)
        self.pts.setdefault(lid, []).append((tuple(p), frame_quat(n, f), tuple(scale)))

    def count(self):
        return sum(len(v) for v in self.pts.values())


def cloud_mesh(name, items):
    me = bpy.data.meshes.new(name)
    P = np.array([i[0] for i in items], np.float32)
    me.vertices.add(len(P))
    me.vertices.foreach_set("co", P.ravel())
    a = me.attributes.new("rot", "QUATERNION", "POINT")
    a.data.foreach_set("value", np.array([i[1] for i in items], np.float32).ravel())
    s = me.attributes.new("scale", "FLOAT_VECTOR", "POINT")
    s.data.foreach_set("vector", np.array([i[2] for i in items], np.float32).ravel())
    me.update()
    return me


# ======================================================= the fit-out
def build_clouds():
    """All hull fittings as point clouds, from Annex B.8 and Sections 6-8, with
    the placement corrections recorded in the verification report."""
    C = {k: Cloud(k) for k in ("heavy", "medium", "pd", "cavity", "sensor", "sensor_hub", "cavity_array",
                               "shield", "screen_emitter")}
    up = Vector((0, 0, 1))
    # --- heavy rim battery: 197 a row, 40 m pitch, three rows
    for z in G.HEAVY_ROWS_Z:
        for t in G.heavy_thetas():
            p = Vector(G.xy(G.R_RIM, t)) .to_3d()
            p.z = z
            C["heavy"].add(p, radial(t), toward_bow(t))
    # --- medium face batteries (B.8)
    for r, face, n_m in ((G.MED_R_THIRD, "dorsal", 117), (G.MED_R_TWO_THIRDS, "dorsal", 234),
                         (G.MED_R_THIRD, "crown", 117), (G.MED_R_VENTRAL_OUTER, "ventral", 278)):
        for k in range(n_m):
            t = (k + 0.5) * 360.0 / n_m
            x, y = G.xy(r, t)
            if face == "dorsal":
                C["medium"].add((x, y, G.cap_z(r)), cap_normal(r, t), radial(t))
            elif face == "crown":
                C["medium"].add((x, y, G.crown_z(r)), crown_normal(r, t), radial(t))
            else:
                C["medium"].add((x, y, G.Z_VENTRAL), -up, radial(t))
    # --- point defence: 8 m mounts on a 120 m grid across the dorsal face, rim and ventral annulus (~1,280)
    for r in (470.0, 630.0, 750.0, 870.0, 990.0, 1250.0, 1370.0, 1490.0, 1610.0):
        n = int(round(2 * math.pi * r / 120))
        for k in range(n):
            t = (k + 0.5 * (int(r) // 120 % 2)) * 360.0 / n
            x, y = G.xy(r, t)
            C["pd"].add((x, y, G.cap_z(r)), cap_normal(r, t), radial(t))
    for z in (-40.0, -412.0, -580.0):
        n = int(round(G.R_RIM * 1.5 * math.pi / 120.0 * 2))
        for k in range(n):
            t = -134.0 + 268.0 * (k + 0.5) / n
            p = Vector(G.xy(G.R_RIM, t)).to_3d()
            p.z = z
            C["pd"].add(p, radial(t), toward_bow(t))
    for r in (1190.0, 1270.0, 1410.0, 1510.0, 1630.0):
        n = int(round(2 * math.pi * r / 120))
        for k in range(n):
            t = (k + 0.25) * 360.0 / n
            x, y = G.xy(r, t)
            C["pd"].add((x, y, G.Z_VENTRAL), -up, radial(t))
    # --- cavity defence: 640, on the crown, the knuckle inner face and the nine well lips
    cav = []
    for c in G.CORES:
        lip = c.well_r + 14
        n = int(round(2 * math.pi * lip / (40 if c.central else 36)))
        for k in range(n):
            a = 2 * math.pi * (k + 0.5) / n
            x, y = c.x + lip * math.cos(a), c.y + lip * math.sin(a)
            r, t = G.polar(x, y)
            cav.append(((x, y, G.crown_z(r)), crown_normal(r, t), radial(t)))
    for r_k in (1066.0, 1082.0):
        n = 170
        for k in range(n):
            t = (k + 0.5 * (r_k > 1070)) * 360.0 / n
            x, y = G.xy(r_k, t)
            cav.append(((x, y, G.knuckle_z(r_k)), knuckle_normal(r_k, t), -radial(t)))
    for r_c in (400.0, 470.0, 520.0, 700.0, 1000.0):
        n = int(round(2 * math.pi * r_c / 75))
        for k in range(n):
            t = (k + 0.5) * 360.0 / n
            x, y = G.xy(r_c, t)
            if any(math.hypot(x - c.x, y - c.y) < c.well_r + 30 for c in G.LANCES):
                continue
            cav.append(((x, y, G.crown_z(r_c)), crown_normal(r_c, t), radial(t)))
    for p, n, f in cav:
        C["cavity"].add(p, n, f)
    # --- sensors (§7): 96 hull apertures plus 48 cavity arrays
    for k in range(16):
        t = 11.25 + 22.5 * k
        for r in (505.0, 1060.0):            # offset from the battery circles, see report
            x, y = G.xy(r, t)
            C["sensor"].add((x, y, G.cap_z(r)), cap_normal(r, t), radial(t))
    for k in range(24):
        t = -129.375 + 11.25 * k
        p = Vector(G.xy(G.R_RIM, t)).to_3d()
        p.z = -218.0
        C["sensor"].add(p, radial(t), up)
    for t in (144.0, 162.0, 198.0, 216.0):
        r = G.r_out(t) - 90.0
        x, y = G.xy(r, t)
        rake_up = (Vector((0, 0, 1)) * math.cos(25 * D2R) + radial(t) * math.sin(25 * D2R))
        rake_dn = (Vector((0, 0, -1)) * math.cos(25 * D2R) + radial(t) * math.sin(25 * D2R))
        C["sensor"].add((x, y, 0.0), rake_up, radial(t))
        C["sensor"].add((x, y, G.Z_VENTRAL), rake_dn, radial(t))
    for k in range(24):
        t = 7.5 + 15 * k
        x, y = G.xy(1450.0, t)
        C["sensor"].add((x, y, G.Z_VENTRAL), -up, radial(t))
    for k in range(8):
        t = 22.5 + 45 * k
        x, y = G.xy(150.0, t)
        C["sensor_hub"].add((x, y, G.dome_z(150.0)), dome_normal(150.0, t), radial(t))
    for k in range(16):
        t = 11.25 + 22.5 * k
        x, y = G.xy(1070.0, t)
        C["cavity_array"].add((x, y, G.knuckle_z(1070.0)), knuckle_normal(1070.0, t), -radial(t))
        t2 = 22.5 * k
        x, y = G.xy(1000.0, t2) if k % 2 == 0 else G.xy(470.0, t2)
        rr = 1000.0 if k % 2 == 0 else 470.0
        C["cavity_array"].add((x, y, G.crown_z(rr)), crown_normal(rr, t2), radial(t2))
        t3 = 22.5 * k + 11.25
        x, y = G.xy(420.0, t3)
        C["cavity_array"].add((x, y, G.crown_z(420.0)), crown_normal(420.0, t3), radial(t3))
    # --- shield ring emitters (§6, B.8): 8 rim, 8 dorsal, 8 ventral
    for t in G.WEB_LONGITUDES:
        if t == 180.0:
            p = Vector(G.xy(G.r_out(180.0) + 20.5, 180.0)).to_3d()   # on the emitter face, at a segment boundary
            p.z = -315.0
            C["shield"].add(p, radial(180.0), up, layer="aft")
        else:
            tt = {135.0: 133.6, 225.0: 226.4}.get(t, t)
            p = Vector(G.xy(G.R_RIM, tt)).to_3d()
            p.z = -218.0
            C["shield"].add(p, radial(tt), up)
        x, y = G.xy(1175.0, t)
        C["shield"].add((x, y, G.cap_z(1175.0)), cap_normal(1175.0, t), radial(t))
        x, y = G.xy(1560.0, t)
        C["shield"].add((x, y, G.Z_VENTRAL), -up, radial(t))
    # --- cavity screen emitters: eight 90 m apertures on the knuckle at the Lance longitudes
    r, z, _ = G.SCREEN_EMITTER
    for t in G.LANCE_LONGITUDES:
        x, y = G.xy(r, t)
        C["screen_emitter"].add((x, y, G.knuckle_z(r)), knuckle_normal(r, t), -radial(t))
    thin_small_mounts(C)
    return C


def thin_small_mounts(C):
    """Drop point-defence and cavity mounts that would sit on a bigger fitting,
    an opening or a marking: the 120 m grid gives way to everything else."""
    from mathutils.kdtree import KDTree
    big = []
    for key, rad in (("heavy", 24.0), ("medium", 13.0), ("sensor", 12.0), ("sensor_hub", 8.0),
                     ("shield", 31.0), ("screen_emitter", 46.0), ("cavity_array", 4.0)):
        for items in C[key].pts.values():
            for p, _, _ in items:
                big.append((p, rad))
    for t in G.MOUTH_THETAS:                       # launch mouths, hazard framing and chevrons
        for dt in (-2.6, 0.0, 2.6):
            x, y = G.xy(G.MOUTH_R, t + dt)
            big.append(((x, y, G.MOUTH_Z), 52.0))
    for t in G.WEB_LONGITUDES:                     # boom recesses, rim numerals and position lights
        for (r, z) in G.BOOM_ROOTS:
            x, y = G.xy(r, t)
            big.append(((x, y, z), 40.0))
        x, y = G.xy(G.R_RIM, t)
        big.append(((x, y, -412.0), 70.0))
        big.append(((x, y, -40.0), 20.0))
    for c in G.CORES:                              # well mouths
        pass
    kd = KDTree(len(big))
    for i, (p, _) in enumerate(big):
        kd.insert(p, i)
    kd.balance()
    maxr = max(r for _, r in big)
    for key in ("pd", "cavity"):
        for lid, items in C[key].pts.items():
            keep = []
            for it in items:
                ok = True
                for (co, idx, dist) in kd.find_range(it[0], maxr + 6.0):
                    if dist < big[idx][1] + 6.0:
                        ok = False
                        break
                if ok and key == "cavity":
                    x, y, z = it[0]
                    if any(math.hypot(x - c.x, y - c.y) < c.well_r + 8 for c in G.CORES):
                        ok = False
                if ok:
                    keep.append(it)
            C[key].pts[lid] = keep
    # hold the cavity layer to its 640 by even thinning
    items = [(lid, it) for lid, lst in C["cavity"].pts.items() for it in lst]
    if len(items) > 640:
        step = len(items) / 640.0
        chosen = [items[int(i * step)] for i in range(640)]
        C["cavity"].pts = {}
        for lid, it in chosen:
            C["cavity"].pts.setdefault(lid, []).append(it)


FITTING_SPECS = {
    # key: (label, stroke, deploy role)
    "heavy": ("Heavy rim battery, 591 mounts, 30 m", 12.0, "batteries"),
    "medium": ("Medium face batteries, 746 mounts, 15 m", 8.0, "batteries"),
    "pd": ("Point defence, 8 m", 4.0, "batteries"),
    "cavity": ("Cavity defence, 640 mounts, 8 m", 4.0, "batteries"),
    "sensor": ("Sensor apertures, 88 conformal panels 20 x 8 m", 0.0, None),
    "sensor_hub": ("Sensor apertures, hub cap, 8 panels 12 x 6 m", 0.0, None),
    "cavity_array": ("Cavity arrays, 48 domes of 6 m", 0.0, None),
    "shield": ("Shield ring emitters, 24, 60 m toroidal", 8.0, "shields"),
    "screen_emitter": ("Cavity screen emitters, 8 of 90 m", 0.0, None),
}


def build_sources(coll):
    """Instance source objects in a hidden collection."""
    S = {}
    S["heavy"] = (mount_object("SRC heavy mount", 30.0, 12.0, coll, True, 17.0),
                  disc_object("SRC heavy shutter", 15.0, ("Mount shutter", "Seam, dark"), coll, iris_blades=8),
                  disc_object("SRC heavy open", 15.4, ("Aperture, dark",), coll, ring_inner=14.2))
    S["medium"] = (mount_object("SRC medium mount", 15.0, 8.0, coll, True, 9.0),
                   disc_object("SRC medium shutter", 7.5, ("Mount shutter", "Seam, dark"), coll, iris_blades=6),
                   disc_object("SRC medium open", 7.8, ("Aperture, dark",), coll, ring_inner=7.1))
    pdm = mount_object("SRC point-defence mount", 8.0, 4.0, coll, True, 4.5, elev=50.0)
    S["pd"] = (pdm, disc_object("SRC point-defence shutter", 4.0, ("Mount shutter", "Seam, dark"), coll, iris_blades=4),
               disc_object("SRC point-defence open", 4.2, ("Aperture, dark",), coll, ring_inner=3.7))
    S["cavity"] = S["pd"]
    S["sensor"] = (panel_object("SRC sensor panel 20x8", 20.0, 8.0, "Sensor aperture", coll, frame_mat="Seam, dark"), None, None)
    S["sensor_hub"] = (panel_object("SRC sensor panel 12x6", 12.0, 6.0, "Sensor aperture", coll, frame_mat="Seam, dark"), None, None)
    S["cavity_array"] = (dome_object("SRC cavity array dome", 3.0, "Sensor aperture", coll), None, None)
    tor = torus_object("SRC shield ring torus", 26.0, 3.5, "Shield emitter", coll)
    S["shield"] = (tor, disc_object("SRC shield aperture", 30.0, ("Shield emitter", "Seam, dark"), coll, ring_inner=22.0, iris_blades=0),
                   disc_object("SRC shield aperture open", 30.0, ("Aperture, dark",), coll, ring_inner=22.0))
    S["screen_emitter"] = (disc_object("SRC screen emitter 90 m", 45.0, ("Screen emitter", "Seam, dark"), coll, iris_blades=12), None, None)
    return S
