"""Hull solid: Annex B.2 surfaces as welded parametric patches."""
from __future__ import annotations

import math

import bmesh
import numpy as np

import lev_geom as G
from lev_mesh import PatchBuilder, revolve_grid, lathe_local

N_THETA = 1024  # B.13: the shell as a lathe at 1,024 segments

# material slots on the hull
M_DORSAL, M_RIM, M_VENTRAL, M_CAVITY, M_WELL, M_BOWL, M_COLLARWELL, M_FAIRING = range(8)


def mark_sharp(me, angle_deg=28.0):
    bm = bmesh.new()
    bm.from_mesh(me)
    lim = math.radians(angle_deg)
    for e in bm.edges:
        if len(e.link_faces) == 2:
            try:
                e.smooth = e.calc_face_angle() < lim
            except ValueError:
                e.smooth = False
        else:
            e.smooth = False
    bm.to_mesh(me)
    bm.free()


def build_hull_mesh(name="Hull", n_theta=N_THETA):
    th_all = np.arange(n_theta) * 360.0 / n_theta
    i0 = int(round(G.AFT_TH0 / 360 * n_theta))
    i1 = int(round(G.AFT_TH1 / 360 * n_theta))
    th_aft = np.arange(i0, i1 + 1) * 360.0 / n_theta
    pb = PatchBuilder()
    p = G.PRAETORIAN

    def add(prof, mat, thetas=th_all, wrap=True):
        pb.add_grid(revolve_grid(thetas, prof), wrap_u=wrap, mat=mat)

    # fixed cap dome
    add(lambda t: [(r, G.dome_z(r)) for r in np.linspace(0, 250, 21)], M_DORSAL)
    # collar well: inner wall, floor, outer wall
    add(lambda t: [(250, G.dome_z(250)), (250, 20), (250, -10)], M_COLLARWELL)
    add(lambda t: [(r, -10.0) for r in np.linspace(250, 450, 5)], M_COLLARWELL)
    add(lambda t: [(450, -10.0), (450, 20), (450, G.cap_z(450))], M_COLLARWELL)
    # dorsal cap to the rim
    add(lambda t: [(r, G.cap_z(r)) for r in np.linspace(450, G.R_RIM, 61)], M_DORSAL)
    # fairing crown (flat) and fairing face, aft only
    add(lambda t: [(G.R_RIM + (G.r_out(t) - G.R_RIM) * k / 6, 0.0) for k in range(7)], M_FAIRING, th_aft, False)
    # rim / fairing face
    add(lambda t: [(G.r_out(t), z) for z in np.linspace(0, G.Z_VENTRAL, 8)], M_RIM)
    # ventral annulus
    add(lambda t: [(G.r_out(t) + (G.R_DISH - G.r_out(t)) * k / 12, G.Z_VENTRAL) for k in range(13)], M_VENTRAL)
    # knuckle, dish rim -> crown edge
    add(lambda t: [G.knuckle_rz(math.pi / 2 * (1 - k / 40)) for k in range(41)], M_CAVITY)
    # crown, 1050 -> Praetorian well
    add(lambda t: [(r, G.crown_z(r)) for r in np.linspace(G.R_CROWN, p.well_r, 49)], M_CAVITY)
    # Praetorian well wall, lip, bowl
    add(lambda t: [(p.well_r, G.crown_z(p.well_r)), (p.well_r, -350), (p.well_r, p.zc)], M_WELL)
    add(lambda t: [(p.well_r, p.zc), (p.bowl_inner, p.zc)], M_WELL)
    add(lambda t: [(p.bowl_inner * math.cos(a), p.zc + p.bowl_inner * math.sin(a))
                   for a in np.linspace(0, math.pi / 2, 73)], M_BOWL)
    me = pb.build(name, weld_dist=1e-3)
    mark_sharp(me)
    return me


def lance_cutter_mesh(name="LanceCavities", n_seg=192, bottom=-545.0):
    """Void each Lance leaves in the hull: bowl at r_c + 2 above the equator,
    4 m lip, well at r_c + 6 below, run out through the crown."""
    pb = PatchBuilder()
    for c in G.LANCES:
        rb, rw = c.bowl_inner, c.well_r
        prof = [(rb * math.sin(a), c.zc + rb * math.cos(a)) for a in np.linspace(0, math.pi / 2, 49)]
        prof += [(rw, c.zc), (rw, c.zc - 80), (rw, bottom), (rw * 0.5, bottom), (0.0, bottom)]
        pb.add_grid(lathe_local(prof, n_seg, c.x, c.y), wrap_u=True)
    me = pb.build(name, weld_dist=1e-3)
    return me
