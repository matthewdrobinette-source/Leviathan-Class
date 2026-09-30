"""Collar, shutters, drive emitter, cavity screen, booms, bowl fittings,
opening closures, markings (Annex D.3) and exterior lighting (Annex D.4)."""
from __future__ import annotations

import math

import bpy
import numpy as np
from mathutils import Matrix, Vector

import lev_geom as G
from lev_mesh import PatchBuilder, lathe_local, mesh_from_arrays, revolve_grid
from lev_materials import MATS
from lev_interior import spine_xy

D2R = G.D2R


def _obj(name, me, coll, mats):
    for m in mats:
        me.materials.append(MATS[m])
    ob = bpy.data.objects.new(name, me)
    coll.objects.link(ob)
    return ob


# ================================================================ collar
# Model interpretation (see the verification report): a 50 m tall ring,
# crown +100 when extended on a 60 m travel, so that it stows at +40 under
# the 10 m iris shutters. Vertical glazing band at mid-radius r 350, which
# is where B.9's 137 m pitch (100 m bays, 37 m piers) holds.
COLLAR_TRAVEL = 60.0


def collar(coll):
    pb = PatchBuilder()
    th = np.arange(1024) * 360.0 / 1024
    prof = [(252.0, 50.0), (448.0, 50.0), (448.0, 53.0), (350.4, 82.0), (350.4, 100.0), (252.0, 100.0), (252.0, 50.0)]
    pb.add_grid(revolve_grid(th, lambda t: prof), wrap_u=True, mat=0)
    # lift piers under the ring, visible in section
    for k in range(16):
        t = 11.25 + 22.5 * k
        c = Vector(G.xy(350.0, t)).to_3d()
        rr = Vector(G.xy(1.0, t)).to_3d()
        tt = Vector((rr.y, -rr.x, 0))
        V = []
        for a in (-10, 10):
            for b in (-10, 10):
                for z in (-10.0, 49.5):
                    p = c + a * rr + b * tt
                    V.append((p.x, p.y, z))
        F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
        pb.add_faces(V, F, 0)
    me = pb.build("Bridge collar", smooth=False, do_weld=True)
    col = _obj("Bridge collar", me, coll, ["Collar plate"])
    # glazing: sixteen 100 x 18 m bays, eight on the web longitudes and eight between
    g = PatchBuilder()
    half = math.degrees(50.0 / 350.4)
    for k in range(16):
        c = 22.5 * k
        ts = np.linspace(c - half, c + half, 17)
        g.add_grid(revolve_grid(ts, lambda t: [(350.9, 82.0), (350.9, 100.0)]), wrap_u=False, mat=0)
    gm = g.build("Collar glazing", smooth=True, do_weld=False)
    gl = _obj("Collar glazing", gm, coll, ["Glazing, transparisteel"])
    gl.parent = col
    # crown beacon
    b = _sphere_mesh("Collar beacon", 3.0, 12)
    bo = _obj("Collar beacon", b, coll, ["Light, collar beacon"])
    x, y = G.xy(300.0, 0.0)
    bo.location = (x, y, 102.5)
    bo.parent = col
    return col, gl, bo


def _sphere_mesh(name, r, n=12):
    prof = [(r * math.sin(a), r * math.cos(a)) for a in np.linspace(0, math.pi, n // 2 + 1)]
    pb = PatchBuilder()
    pb.add_grid(lathe_local(prof, n), wrap_u=True)
    return pb.build(name)


def shutters(coll):
    """Sixteen iris leaves carrying the dorsal curve and the full 10 m grade
    across the collar well mouth. Each leaf withdraws radially under the plate."""
    leaves = []
    for k in range(16):
        c = 22.5 * k
        t0, t1 = c - 11.25 + 0.06, c + 11.25 - 0.06
        pb = PatchBuilder()
        ts = np.linspace(t0, t1, 25)
        rs = np.linspace(250.4, 449.6, 9)
        top = np.array([[(*G.xy(r, t), G.cap_z(r) + 0.02) for r in rs] for t in ts])
        bot = np.array([[(*G.xy(r, t), G.cap_z(r) - 10.0) for r in rs[::-1]] for t in ts])
        # closed slab: top, bottom and the four edges as a lathe-like loop
        loop = np.concatenate([top, bot], axis=1)                 # (nt, 18, 3) around the section
        pb.add_grid(np.transpose(loop, (1, 0, 2)), wrap_u=True)   # sides + faces around the section loop
        V = [tuple(p) for p in loop[0]]
        pb.add_faces(V, [tuple(range(len(V)))[::-1]])
        V = [tuple(p) for p in loop[-1]]
        pb.add_faces(V, [tuple(range(len(V)))])
        me = pb.build(f"Iris leaf {k + 1:02d}", smooth=False)
        ob = _obj(f"Iris leaf {k + 1:02d}", me, coll, ["Iris shutter"])
        ob["bearing"] = c
        leaves.append(ob)
    return leaves


# ================================================================ drive emitter
def drive_emitter(coll):
    """Sixteen independently fed segments across the fairing's aft boundary,
    face standing 20 m proud of the structural boundary, full 630 m section."""
    pb = PatchBuilder()
    for k in range(16):
        a = G.AFT_TH0 + 5.625 * k + 0.12
        b = G.AFT_TH0 + 5.625 * (k + 1) - 0.12
        ts = np.linspace(a, b, 13)
        zs = (-622.0, -8.0)
        outer = [[(*G.xy(G.r_out(t) + 20.0, t), z) for z in zs] for t in ts]
        inner = [[(*G.xy(G.r_out(t) - 0.5, t), z) for z in zs[::-1]] for t in ts]
        loop = np.concatenate([np.array(outer), np.array(inner)], axis=1)   # (nt, 4, 3)
        pb.add_grid(np.transpose(loop, (1, 0, 2)), wrap_u=True)
        pb.add_faces([tuple(p) for p in loop[0]], [(3, 2, 1, 0)])
        pb.add_faces([tuple(p) for p in loop[-1]], [(0, 1, 2, 3)])
    me = pb.build("Drive emitter", smooth=False)
    return _obj("Drive emitter, 16 segments", me, coll, ["Drive emitter, Dallorian liners"])


# ================================================================ cavity screen
def cavity_screens(coll):
    th = np.arange(1024) * 360.0 / 1024
    out = {}
    for key, z, mat in (("attenuating", G.Z_VENTRAL - 0.6, "Cavity screen, attenuating"),
                        ("hardened", G.Z_VENTRAL - 10.0, "Cavity screen, hardened")):
        pb = PatchBuilder()
        pb.add_grid(revolve_grid(th, lambda t: [(r, z) for r in np.linspace(G.R_DISH, 0.0, 24)]), wrap_u=True)
        out[key] = _obj(f"Cavity screen, {key}", pb.build(f"screen {key}"), coll, [mat])
    # drawn inboard against the dish: 12 m standoff along the cavity-side normal
    prof = []
    for phi in np.linspace(math.pi / 2, 0, 16):
        r, z = G.knuckle_rz(phi)
        nr, nz = G.knuckle_normal(r)
        prof.append((r + 12 * nr, z + 12 * nz))
    for r in np.linspace(G.R_CROWN, 0, 40)[1:]:
        s = G.crown_slope(r)
        n = np.array([s, -1.0]) / math.hypot(s, 1.0)
        prof.append((r + 12 * n[0], G.crown_z(r) + 12 * n[1]))
    prof[-1] = (0.0, prof[-1][1])
    pb = PatchBuilder()
    pb.add_grid(revolve_grid(th, lambda t: prof), wrap_u=True)
    out["inboard"] = _obj("Cavity screen, drawn inboard", pb.build("screen inboard"), coll, ["Cavity screen, hardened"])
    return out


# ================================================================ booms
def lattice(name, length, width, bay=20.0, member=2.4):
    """Lattice cantilever along +X from x = 0, square section centred on the axis."""
    pb = PatchBuilder()
    h = width / 2
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]

    def bar(p, q, s=member):
        p, q = Vector(p), Vector(q)
        d = q - p
        L = d.length
        if L < 1e-6:
            return
        d.normalize()
        a = Vector((0, 0, 1)) if abs(d.z) < 0.9 else Vector((0, 1, 0))
        u = d.cross(a).normalized() * s / 2
        v = d.cross(u).normalized() * s / 2
        V = [tuple(p + su * u + sv * v + t * (q - p)) for t in (0, 1) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        F = [(0, 1, 2, 3)[::-1], (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
        pb.add_faces(V, F)
    for (y, z) in corners:
        bar((0, y, z), (length, y, z), member * 1.4)
    n = max(1, int(length // bay))
    for i in range(n + 1):
        x = length * i / n
        for j in range(4):
            y0, z0 = corners[j]
            y1, z1 = corners[(j + 1) % 4]
            bar((x, y0, z0), (x, y1, z1))
        if i < n:
            x2 = length * (i + 1) / n
            for j in range(4):
                y0, z0 = corners[j]
                y1, z1 = corners[(j + 1) % 4]
                if (i + j) % 2:
                    bar((x, y0, z0), (x2, y1, z1), member * 0.8)
                else:
                    bar((x, y1, z1), (x2, y0, z0), member * 0.8)
    return pb.build(name, smooth=False, do_weld=False)


def booms(coll):
    """Sixteen booms: stowed tangentially in the outer band (B.7), and a
    deployed set, telescoped to a 900 m reach below the rim plane."""
    stowed, deployed = [], []
    me_s = lattice("Boom lattice, stowed", 380.0, 36.0)
    me_s.materials.append(MATS["Boom lattice"])
    for spine in G.WEB_LONGITUDES:
        for which, (r_root, z_root) in (("upper", G.BOOM_ROOTS[0]), ("lower", G.BOOM_ROOTS[1])):
            # stowed: along the tangent at r 1,155, upper toward decreasing theta, lower toward increasing
            u = 1155.0
            zc = -520.0 if which == "upper" else -598.0
            sgn = -1.0 if which == "upper" else 1.0
            x0, y0 = spine_xy(spine, u, sgn * 10.0)
            x1, y1 = spine_xy(spine, u, sgn * 390.0)
            ob = bpy.data.objects.new(f"Boom {spine:.0f} {which}, stowed", me_s)
            coll.objects.link(ob)
            d = Vector((x1 - x0, y1 - y0, 0)).normalized()
            ob.matrix_world = Matrix.Translation((x0, y0, zc)) @ Vector((1, 0, 0)).rotation_difference(d).to_matrix().to_4x4()
            stowed.append(ob)
            # deployed: articulated at the root, reaching down and inward to 900 m below the rim plane
            rx, ry = G.xy(r_root, spine)
            tip_r = 720.0 if which == "upper" else 930.0
            tx, ty = G.xy(tip_r, spine + (4.0 if which == "upper" else -4.0))
            root = Vector((rx, ry, z_root))
            tip = Vector((tx, ty, G.Z_VENTRAL - 900.0))
            L = (tip - root).length
            d = (tip - root).normalized()
            for i, (w, a, b) in enumerate(((40.0, 0.0, 0.38), (34.0, 0.34, 0.70), (28.0, 0.66, 1.0))):
                m = lattice(f"Boom section {i}", L * (b - a), w)
                m.materials.append(MATS["Boom lattice"])
                o = bpy.data.objects.new(f"Boom {spine:.0f} {which}, deployed, section {i + 1}", m)
                coll.objects.link(o)
                o.matrix_world = Matrix.Translation(root + d * (L * a)) @ Vector((1, 0, 0)).rotation_difference(d).to_matrix().to_4x4()
                deployed.append(o)
    return stowed, deployed


# ================================================================ bowl fittings
def bowl_fittings(coll, cores_ct):
    """Keys (eight, tapered 1:40), locking dogs and the crown repulsor array in
    every bowl: visible once a core has left."""
    import lev_cores as LC
    objs = []
    for c in G.CORES:
        ct = cores_ct[c.central]
        rb = c.bowl_inner
        pb = PatchBuilder()
        la0, la1 = ct.key_lat_range()
        for lon in ct.key_lons():
            lats = np.linspace(la0 + 1.0, la1 - 1.0, 30)
            loop = []
            for i, la in enumerate(lats):
                taper = (la1 - la) / (la1 - la0) * (ct.key_proud * 0.25)   # thinner toward the crown
                proud = ct.key_proud * 0.75 + taper
                hw = math.degrees(math.asin(min(0.99, (ct.key_w / 2 - 2.0) / (rb * math.cos(la * D2R)))))
                loop.append([LC.sph(rb, la, lon - hw, 0.5), LC.sph(rb, la, lon - hw, -proud),
                             LC.sph(rb, la, lon + hw, -proud), LC.sph(rb, la, lon + hw, 0.5)])
            P = np.array([[tuple(loop[i][j]) for i in range(len(lats))] for j in range(4)])
            pb.add_grid(P, wrap_u=True, mat=0)
            pb.add_faces([tuple(v) for v in loop[0]], [(0, 1, 2, 3)], 0)
            pb.add_faces([tuple(v) for v in loop[-1]], [(3, 2, 1, 0)], 0)
        for k in range(ct.dogs):
            lon = 360.0 * (k + 0.5) / ct.dogs
            e, u, n = LC.enu(1.5, lon)
            cc = LC.sph(rb, 1.5, lon)
            V = []
            for a in (-5.0, 5.0):
                for b in (-3.0, 3.0):
                    for h in (-6.0, 1.0):
                        V.append(tuple(cc + a * e + b * u + h * n))
            F = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
            pb.add_faces(V, F, 0)
        # repulsor array on the bowl crown (inner surface, facing the core's polar cap)
        for ring_i, (lat, n) in enumerate(((87.0, 6), (83.0, 14))):
            for k in range(n):
                lon = 360.0 * (k + 0.5 * ring_i) / n
                e, u, nn = LC.enu(lat, lon)
                cc = LC.sph(rb, lat, lon, -0.4)
                R = rb * 0.02
                a = np.linspace(0, 2 * math.pi, 6, endpoint=False)
                V = [tuple(cc)] + [tuple(cc + R * (math.cos(t) * e + math.sin(t) * u)) for t in a]
                pb.add_faces(V, [(0, (i + 1) % 6 + 1, i + 1) for i in range(6)], 1)
        me = pb.build(f"Bowl fittings, {c.name}", smooth=False, do_weld=False)
        ob = _obj(f"Bowl fittings, {c.name}", me, coll, ["Duranium, bright worn", "Repulsor array"])
        ob.location = (c.x, c.y, c.zc)
        ob.rotation_euler = (0, 0, (90.0 - c.th) * D2R if not c.central else 90.0 * D2R)
        objs.append(ob)
    return objs


# ================================================================ openings
def knuckle_point(r, t, h=0.0):
    nr, nz = G.knuckle_normal(r)
    x, y = G.xy(r + h * nr, t)
    return Vector((x, y, G.knuckle_z(r) + h * nz))


def knuckle_patch(t_c, half_w, r_a, r_b, h, nu=16, nv=8):
    """Grid conforming to the knuckle, centred on bearing t_c, tangential half
    width half_w (m) at the patch's mean radius, from radius r_a to r_b."""
    rm = (r_a + r_b) / 2
    dt = math.degrees(half_w / rm)
    P = np.array([[tuple(knuckle_point(r, t, h)) for r in np.linspace(r_a, r_b, nv)]
                  for t in np.linspace(t_c - dt, t_c + dt, nu)])
    return P


def openings(coll):
    """Mouth outer shutters, boom recess shutters, hazard marking and the
    flight-operations lighting of the cavity."""
    out = {}
    # outer armoured shutters over the sixteen mouths, flush with the plate
    pb = PatchBuilder()
    for t in G.MOUTH_THETAS:
        pb.add_grid(knuckle_patch(t, 60.0, G.MOUTH_HEAD[0], G.MOUTH_SILL[0] + 0.5, 0.3), mat=0)
    out["mouth_shutters"] = _obj("Launch mouth outer shutters", pb.build("mouth shutters", smooth=True, do_weld=False), coll, ["Iris shutter"])
    # boom recess shutters (flush, closed) and their stowed-station lights
    pb = PatchBuilder()
    lp = PatchBuilder()
    for t in G.WEB_LONGITUDES:
        for (r, z) in G.BOOM_ROOTS:
            # 60 m tangential by 40 m along the knuckle
            ra, rb = _knuckle_span(r, 40.0)
            pb.add_grid(knuckle_patch(t, 30.0, ra, rb, 0.25, 8, 6), mat=0)
            for (a, b) in ((-29.0, ra), (29.0, ra), (-29.0, rb), (29.0, rb)):
                p = knuckle_point(b, t + math.degrees(a / r), 1.2)
                _add_sphere(lp, p, 1.6)
    out["recess_shutters"] = _obj("Boom recess shutters", pb.build("recess shutters", smooth=True, do_weld=False), coll, ["Mount shutter"])
    out["boom_lights"] = _obj("Boom station lights", lp.build("boom lights", do_weld=False), coll, ["Light, boom station"])
    # hazard frames: mouths, recesses, screen emitters, well lips
    hz = PatchBuilder()
    for t in G.MOUTH_THETAS:
        _frame(hz, t, 64.0, G.MOUTH_HEAD[0] - 3.0, min(1129.6, G.MOUTH_SILL[0] + 2.0), 3.0)
    for t in G.WEB_LONGITUDES:
        for (r, z) in G.BOOM_ROOTS:
            ra, rb = _knuckle_span(r, 46.0)
            _frame(hz, t, 33.0, ra, min(rb, 1129.6), 2.0)
    for c in G.CORES:
        prof = [(c.well_r + 2.0, 0.0), (c.well_r + 7.0, 0.0)]
        P = lathe_local(prof, 256 if c.central else 128, c.x, c.y)
        for i in range(P.shape[0]):
            for j in range(P.shape[1]):
                r, _ = G.polar(P[i, j, 0], P[i, j, 1])
                P[i, j, 2] = G.crown_z(r) - 0.3
        hz.add_grid(P, wrap_u=True)
    out["hazard"] = _obj("Hazard marking, cavity apertures", hz.build("hazard", do_weld=False), coll, ["Hazard marking"])
    # approach ladders framing each mouth, the two of a segment coded left and right
    lad_p, lad_s = PatchBuilder(), PatchBuilder()
    for n_seg in range(8):
        for side, t in ((-1, 45.0 * n_seg - G.MOUTH_DTH), (1, 45.0 * n_seg + G.MOUTH_DTH)):
            tgt = lad_p if side < 0 else lad_s
            for k in range(8):
                r = 1062.0 + k * 5.5
                for s in (-1, 1):
                    p = knuckle_point(r, t + s * math.degrees(56.0 / r), 1.0)
                    _add_sphere(tgt, p, 1.5)
    out["ladder_port"] = _obj("Approach ladders, left mouths", lad_p.build("ladders L", do_weld=False), coll, ["Light, approach ladder, port"])
    out["ladder_stbd"] = _obj("Approach ladders, right mouths", lad_s.build("ladders R", do_weld=False), coll, ["Light, approach ladder, starboard"])
    # approach chevrons painted on the knuckle either side of each mouth
    ch = PatchBuilder()
    for t in G.MOUTH_THETAS:
        for s in (-1, 1):
            for k in range(3):
                r0 = 1066.0 + 10.0 * k
                tc = t + s * math.degrees(72.0 / 1100.0)
                a = knuckle_point(r0, tc - s * math.degrees(6.0 / r0), 0.3)
                b = knuckle_point(r0 + 6.0, tc, 0.3)
                c = knuckle_point(r0, tc + s * math.degrees(6.0 / r0), 0.3)
                d = knuckle_point(r0 + 2.5, tc, 0.3)
                ch.add_faces([tuple(a), tuple(b), tuple(d)], [(0, 1, 2)])
                ch.add_faces([tuple(d), tuple(b), tuple(c)], [(0, 1, 2)])
    out["chevrons"] = _obj("Approach chevrons", ch.build("chevrons", do_weld=False), coll, ["Paint, cavity numerals"])
    # four datum lights on the dish rim marking the holding station
    dl = PatchBuilder()
    for t in (0.0, 90.0, 180.0, 270.0):
        x, y = G.xy(G.R_DISH + 6.0, t)
        _add_sphere(dl, Vector((x, y, G.Z_VENTRAL - 2.0)), 4.0)
    out["datum"] = _obj("Holding station datum lights", dl.build("datum", do_weld=False), coll, ["Light, datum"])
    # position lights at the eight rim longitudes: white over the forward 270 deg, red over the drive arc
    pw, pr = PatchBuilder(), PatchBuilder()
    for t in G.WEB_LONGITUDES:
        red = t in (135.0, 180.0, 225.0)
        if t == 180.0:
            x, y = G.xy(G.r_out(180.0) - 8.0, 180.0)
            p = Vector((x, y, 3.0))
        else:
            tt = {135.0: 133.6, 225.0: 226.4}.get(t, t)
            x, y = G.xy(G.R_RIM + 3.0, tt)
            p = Vector((x, y, -40.0))
        _add_sphere(pr if red else pw, p, 5.0)
    out["nav_white"] = _obj("Position lights, white", pw.build("nav w", do_weld=False), coll, ["Light, position white"])
    out["nav_red"] = _obj("Position lights, red", pr.build("nav r", do_weld=False), coll, ["Light, position red"])
    # cavity floodlight fixtures on the knuckle (the light itself is an area lamp)
    fl = PatchBuilder()
    for k in range(16):
        t = 22.5 * k + 11.25
        for dt in (-1.6, 1.6):
            _add_sphere(fl, knuckle_point(1057.0, t + dt, 1.5), 2.5)
    out["flood_fixtures"] = _obj("Cavity floodlight fixtures", fl.build("floods", do_weld=False), coll, ["Light, cavity amber"])
    return out


def _knuckle_span(r_c, length):
    """Radii either side of r_c that span `length` metres along the knuckle curve."""
    phi_c = G.knuckle_phi_at_r(r_c)
    ds = math.hypot(G.KN_A * math.cos(phi_c), G.KN_B * math.sin(phi_c))
    dphi = length / 2 / ds
    return G.knuckle_rz(max(0.0, phi_c - dphi))[0], G.knuckle_rz(min(math.pi / 2, phi_c + dphi))[0]


def _frame(pb, t, half_w, r_a, r_b, w):
    """Rectangular hazard frame conforming to the knuckle."""
    rm = (r_a + r_b) / 2
    pb.add_grid(knuckle_patch(t, half_w, r_a - w, r_a, 0.35, 12, 2))
    pb.add_grid(knuckle_patch(t, half_w, r_b, min(1129.8, r_b + w), 0.35, 12, 2))
    for s in (-1, 1):
        tc = t + s * math.degrees((half_w - w / 2) / rm)
        pb.add_grid(knuckle_patch(tc, w / 2, r_a, r_b, 0.35, 2, 8))


def _add_sphere(pb, p, r, n=8):
    prof = [(r * math.sin(a), r * math.cos(a)) for a in np.linspace(0, math.pi, n // 2 + 1)]
    P = lathe_local(prof, n)
    P[:, :, 0] += p[0]
    P[:, :, 1] += p[1]
    P[:, :, 2] += p[2]
    pb.add_grid(P, wrap_u=True)


# ================================================================ markings
def text_mesh(body, size, extrude=0.0, align="CENTER"):
    cu = bpy.data.curves.new("txt", "FONT")
    cu.body = body
    cu.size = size
    cu.align_x = align
    cu.align_y = "CENTER"
    cu.extrude = extrude
    cu.resolution_u = 3
    ob = bpy.data.objects.new("txt", cu)
    bpy.context.scene.collection.objects.link(ob)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(ob.evaluated_get(dg))
    bpy.data.objects.remove(ob)
    bpy.data.curves.remove(cu)
    return me


def map_mesh(me, fn):
    """Apply fn((u, v)) -> (x, y, z) to every vertex of a flat text mesh."""
    co = np.zeros(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    out = np.array([fn(u, v) for u, v, _ in co])
    me.vertices.foreach_set("co", out.astype(np.float32).ravel())
    me.update()
    return me


def markings(coll):
    """Annex D.3. Low contrast at range, legible from a ship's length."""
    obs = {}
    # hull name forward at two-thirds radius, division numeral aft
    me = text_mesh("LEVIATHAN", 70.0)
    r0 = G.MED_R_TWO_THIRDS

    def on_dorsal_arc(u, v):
        t, r = math.degrees(u / r0), r0 + v
        x, y = G.xy(r, t)
        return (x, y, G.cap_z(r) + 0.35)
    map_mesh(me, on_dorsal_arc)
    obs["name"] = _obj("Marking, hull name LEVIATHAN", me, coll, ["Paint, low-contrast marking"])
    me = text_mesh("I", 110.0)
    map_mesh(me, lambda u, v: (*G.xy(r0 + v, 180.0 + math.degrees(u / r0)), G.cap_z(r0 + v) + 0.35))
    obs["division"] = _obj("Marking, division numeral I", me, coll, ["Paint, low-contrast marking"])
    # class badge on the fixed cap
    pb = PatchBuilder()

    def dome_ring(r0_, r1_, n=96):
        P = revolve_grid(np.arange(n) * 360.0 / n, lambda t: [(r0_, G.dome_z(r0_) + 0.3), (r1_, G.dome_z(r1_) + 0.3)])
        pb.add_grid(P, wrap_u=True)
    dome_ring(84.0, 90.0)
    dome_ring(56.0, 59.0)
    for k in range(8):
        t = 45.0 * k
        P = np.array([[(*G.xy(r, t + s * math.degrees(2.2 / r)), G.dome_z(r) + 0.3) for r in (62.0, 81.0)] for s in (-1, 1)])
        pb.add_grid(P)
        cx, cy = G.xy(38.0, 22.5 + t)
        P = lathe_local([(0.0, 0.0), (6.0, 0.0)], 16, cx, cy)[:, ::-1, :]
        P[:, :, 2] = [[G.dome_z(math.hypot(p[0], p[1])) + 0.3 for p in row] for row in P]
        pb.add_grid(P, wrap_u=True)
    P = lathe_local([(0.0, 0.0), (14.0, 0.0)], 24)[:, ::-1, :]
    P[:, :, 2] = G.dome_z(0.0) + 0.3
    pb.add_grid(P, wrap_u=True)
    obs["badge"] = _obj("Marking, class badge", pb.build("badge", do_weld=False), coll, ["Paint, low-contrast marking"])
    # sector numerals at the rim on each web longitude, repeated in the cavity at each boom station
    rim, cav = [], []
    for k, t in enumerate(G.WEB_LONGITUDES):
        num = str(k + 1)
        me = text_mesh(num, 100.0)
        if t == 180.0:
            ro = G.r_out(180.0) - 110.0
            map_mesh(me, lambda u, v, ro=ro: (*G.xy(ro - v, 180.0 - math.degrees(u / ro)), 0.35))
        else:
            tt = {135.0: 133.0, 225.0: 227.0}.get(t, t)
            # read from outside the rim: the reader's right is decreasing theta
            map_mesh(me, lambda u, v, tt=tt: (*G.xy(G.R_RIM + 0.35, tt - math.degrees(u / G.R_RIM)), -412.0 + v))
        rim.append(me)
        me = text_mesh(num, 60.0)
        rc = 985.0
        map_mesh(me, lambda u, v, t=t: (*G.xy(rc - v, t + math.degrees(u / rc)), G.crown_z(rc - v) - 0.35))
        cav.append(me)
    obs["rim_numerals"] = _obj("Marking, sector numerals, rim", _join(rim, "rim numerals"), coll, ["Paint, low-contrast marking"])
    obs["cavity_numerals"] = _obj("Marking, sector numerals, cavity", _join(cav, "cavity numerals"), coll, ["Paint, cavity numerals"])
    # launch mouths numbered one to sixteen (clockwise from the bow), above each mouth head
    ms = []
    for i, t in enumerate(G.MOUTH_THETAS):
        me = text_mesh(str(i + 1), 24.0)
        map_mesh(me, lambda u, v, t=t: tuple(knuckle_point(1088.0 - v * 0.55, t + math.degrees(u / 1088.0), 0.35)))
        ms.append(me)
    obs["mouth_numbers"] = _obj("Marking, launch mouth numbers", _join(ms, "mouth numbers"), coll, ["Paint, cavity numerals"])
    # wells lettered for their cores
    ws = []
    for c in G.CORES:
        label = "P" if c.central else "L" + str(G.LANCES.index(c) + 1)
        me = text_mesh(label, 40.0)
        if c.central:
            rr, tt = c.well_r + 40.0, 0.0
        else:
            rr, tt = c.r + c.well_r + 36.0, c.th
        map_mesh(me, lambda u, v, rr=rr, tt=tt: (*G.xy(rr - v, tt + math.degrees(u / rr)), G.crown_z(rr - v) - 0.35))
        ws.append(me)
    obs["well_letters"] = _obj("Marking, well letters", _join(ws, "well letters"), coll, ["Paint, cavity numerals"])
    return obs


def core_designation(name, rc, coll):
    """Painted large on the southern cap (D.3), in the core's own frame."""
    size = rc * 0.16
    me = text_mesh(name.upper(), size)

    def f(u, v):
        p = Vector((u, -v, -rc)).normalized() * (rc + 0.35)
        return (p.x, p.y, p.z)
    map_mesh(me, f)
    return _obj(f"Marking, core designation {name}", me, coll, ["Paint, core designation"])


def _join(meshes, name):
    V, F, off = [], [], 0
    for me in meshes:
        co = np.zeros(len(me.vertices) * 3)
        me.vertices.foreach_get("co", co)
        V.append(co.reshape(-1, 3))
        for p in me.polygons:
            F.append(tuple(i + off for i in p.vertices))
        off += len(me.vertices)
        bpy.data.meshes.remove(me)
    return mesh_from_arrays(name, np.concatenate(V, 0), F, smooth=False)
