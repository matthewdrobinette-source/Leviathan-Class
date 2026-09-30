"""Leviathan-class, Revision H: geometry from the Annex B modelling datum set.

Pure Python + numpy, no Blender dependency, so the verification script and the
Blender build share one definition of every surface and station.

Frame (Annex B.1): origin on the ship's axis at the dorsal rim plane, z = 0,
positive dorsal. Radii r from the axis. Longitude theta from the bow at 0 deg,
increasing to starboard. In Blender: +Y is the bow, +X starboard, +Z dorsal,
so x = r sin(theta), y = r cos(theta).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

D2R = math.pi / 180.0

# ---------------------------------------------------------------- B.2 surfaces
R_RIM = 1678.45            # rim radius (diameter 3,356.9 m)
R_CAP = 25638.0            # dorsal spherical cap, radius of curvature
CAP_K = math.sqrt(R_CAP ** 2 - R_RIM ** 2)
DOME_R = 1206.0            # fixed cap dome radius of curvature
DOME_TOP = 80.0            # fixed cap on the axis
DOME_C = DOME_TOP - DOME_R  # dome centre z (-1126)
R_FIXED_CAP = 250.0        # fixed cap / collar well inner wall
R_COLLAR_OUT = 450.0       # collar well outer wall
Z_WELL_FLOOR = -10.0       # collar well floor
CROWN_TOP = -420.0         # ventral crown on the axis
R_CROWN = 1050.0           # crown ends, knuckle starts
Z_CROWN_EDGE = -470.0
KN_A = 80.0                # knuckle semi-axis, radial
KN_B = 160.0               # knuckle semi-axis, vertical
R_DISH = 1130.0            # dish rim / knuckle meets the ventral rim plane
Z_VENTRAL = -630.0         # ventral rim plane
FAIRING_REACH = 180.0      # drive fairing reach at 180 deg
AFT_TH0, AFT_TH1 = 135.0, 225.0
R_AFT_BLOCK = 1000.0       # aft block inboard boundary (B.10)

# ------------------------------------------------------------------ grades
GRADE_DORSAL = 10.0
GRADE_OTHER = 4.0

# ------------------------------------------------------------------ deck grid
DECK0 = -236.0             # lower gallery, n = 0
PITCH = 6.0


def deck_z(n: int) -> float:
    return DECK0 + PITCH * n


# ------------------------------------------------------------------ profiles
def cap_z(r: float) -> float:
    """Dorsal spherical cap; 0 at the rim, +55 on the axis."""
    if r >= R_RIM:
        return 0.0
    return math.sqrt(R_CAP ** 2 - r * r) - CAP_K


def cap_slope(r: float) -> float:
    if r >= R_RIM:
        return 0.0
    return -r / math.sqrt(R_CAP ** 2 - r * r)


def dome_z(r: float) -> float:
    """Fixed cap dome inboard of r = 250."""
    return math.sqrt(DOME_R ** 2 - r * r) + DOME_C


def dome_slope(r: float) -> float:
    return -r / math.sqrt(DOME_R ** 2 - r * r)


def crown_z(r: float) -> float:
    """Ventral crown paraboloid, axis to r = 1,050."""
    q = r / R_CROWN
    return CROWN_TOP - 50.0 * q * q


def crown_slope(r: float) -> float:
    return -100.0 * r / (R_CROWN ** 2)


def knuckle_rz(phi: float) -> tuple[float, float]:
    """Quarter-ellipse knuckle, phi 0 at the crown edge, pi/2 at the dish rim."""
    return R_CROWN + KN_A * math.sin(phi), Z_VENTRAL + KN_B * math.cos(phi)


def knuckle_z(r: float) -> float:
    s = min(1.0, max(0.0, (r - R_CROWN) / KN_A))
    return Z_VENTRAL + KN_B * math.sqrt(max(0.0, 1.0 - s * s))


def knuckle_phi_at_r(r: float) -> float:
    s = min(1.0, max(0.0, (r - R_CROWN) / KN_A))
    return math.asin(s)


def knuckle_normal(r: float) -> tuple[float, float]:
    """Outward (into the cavity) unit normal of the knuckle in the (r, z) plane."""
    s = min(1.0, max(0.0, (r - R_CROWN) / KN_A))
    c = math.sqrt(max(0.0, 1.0 - s * s))
    gr, gz = s / KN_A, c / KN_B          # gradient of the ellipse (points into the hull)
    n = math.hypot(gr, gz)
    return -gr / n, -gz / n


def bottom_z(r: float) -> float:
    """Ventral moulded surface (crown, knuckle, flat annulus)."""
    if r >= R_DISH:
        return Z_VENTRAL
    if r >= R_CROWN:
        return knuckle_z(r)
    return crown_z(r)


def norm_th(t: float) -> float:
    return t % 360.0


def in_aft_arc(t: float) -> bool:
    t = norm_th(t)
    return AFT_TH0 <= t <= AFT_TH1


def r_out(t: float) -> float:
    """Outer boundary: rim circle, plus the drive fairing's cosine lune aft."""
    t = norm_th(t)
    if AFT_TH0 <= t <= AFT_TH1:
        return R_RIM + FAIRING_REACH * math.cos(2.0 * (t - 180.0) * D2R)
    return R_RIM


def dorsal_z(r: float, t: float = 0.0) -> float:
    """Dorsal moulded surface at (r, theta), collar well mouth closed (shutters)."""
    if r < R_FIXED_CAP:
        return dome_z(r)
    if r <= R_RIM:
        return cap_z(r)
    return 0.0  # fairing crown, flat


def xy(r: float, t: float) -> tuple[float, float]:
    """Ship (r, theta) to Blender (x, y): bow +Y, starboard +X."""
    return r * math.sin(t * D2R), r * math.cos(t * D2R)


def polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), norm_th(math.atan2(x, y) / D2R)


# --------------------------------------------------------------------- cores
@dataclass
class Core:
    name: str
    rc: float
    r: float
    th: float
    zc: float
    central: bool
    pair: int = 0          # release pair index (1..4), 5 for Praetorian
    x: float = field(init=False)
    y: float = field(init=False)

    def __post_init__(self):
        self.x, self.y = xy(self.r, self.th)

    @property
    def bowl_inner(self) -> float:   # running clearance 2 m
        return self.rc + 2.0

    @property
    def bowl_moulded(self) -> float:  # + 12 m shell
        return self.rc + 14.0

    @property
    def well_r(self) -> float:
        return self.rc + 6.0


WORDS = ["One", "Two", "Three", "Four", "Five", "Six", "Seven", "Eight"]
CORES: list[Core] = [Core("Praetorian", 348.0, 0.0, 0.0, -282.0, True, 5)]
for _n in range(8):
    CORES.append(Core("Lance " + WORDS[_n], 210.0, 800.0, 22.5 + 45.0 * _n, -290.0, False, (_n % 4) + 1))
LANCES = CORES[1:]
PRAETORIAN = CORES[0]

WEB_LONGITUDES = [45.0 * k for k in range(8)]
LANCE_LONGITUDES = [22.5 + 45.0 * k for k in range(8)]

# ------------------------------------------------------------ B.7 openings
MOUTH_R, MOUTH_Z = 1118.0, -545.0
MOUTH_W, MOUTH_H = 120.0, 60.0
MOUTH_SILL = (1125.0, -575.0)
MOUTH_HEAD = (1106.0, -515.0)
MOUTH_DTH = 6.15
MOUTH_THETAS = sorted(norm_th(45.0 * n + s * MOUTH_DTH) for n in range(8) for s in (-1, 1))
BOOM_ROOTS = [(1108.0, -520.0), (1128.0, -596.0)]
BOOM_RECESS = (60.0, 40.0)
SCREEN_EMITTER = (1097.0, -500.0, 90.0)

# ------------------------------------------------------------ B.8 batteries
HEAVY_ROWS_Z = [-120.0, -315.0, -510.0]
HEAVY_PITCH = 40.0
HEAVY_PER_ROW = 197
MED_R_THIRD = 559.5
MED_R_TWO_THIRDS = 1119.0
MED_R_VENTRAL_OUTER = 1331.0
MED_PITCH = 30.0


def heavy_thetas() -> list[float]:
    """197 mounts, a whole number of 40 m pitches centred on the bow."""
    step = HEAVY_PITCH / R_RIM / D2R
    return [(k - (HEAVY_PER_ROW - 1) / 2.0) * step for k in range(HEAVY_PER_ROW)]


def ring_count(r: float, pitch: float) -> int:
    return int(math.floor(2.0 * math.pi * r / pitch))


def fairing_arc_length(n: int = 20000) -> float:
    """Length of the fairing's aft boundary curve r_out(theta), 135 to 225 deg."""
    total = 0.0
    prev = None
    for i in range(n + 1):
        t = AFT_TH0 + (AFT_TH1 - AFT_TH0) * i / n
        p = xy(r_out(t), t)
        if prev is not None:
            total += math.hypot(p[0] - prev[0], p[1] - prev[1])
        prev = p
    return total


def hull_profile(t: float, collar_well: bool = True, with_praetorian: bool = True,
                 n_dome: int = 16, n_cap: int = 48, n_knuckle: int = 24, n_crown: int = 40,
                 n_bowl: int = 64) -> list[tuple[float, float]]:
    """Closed (r, z) section polygon of the hull at longitude t, counter-clockwise
    starting on the axis at the fixed cap. Includes the collar well and
    Praetorian's bowl and well (both axisymmetric). Lance cavities are not
    axisymmetric and are cut separately."""
    pts: list[tuple[float, float]] = []
    ro = r_out(t)
    # fixed cap dome, axis -> r 250
    for i in range(n_dome + 1):
        r = R_FIXED_CAP * i / n_dome
        pts.append((r, dome_z(r)))
    if collar_well:
        pts.append((R_FIXED_CAP, Z_WELL_FLOOR))
        pts.append((R_COLLAR_OUT, Z_WELL_FLOOR))
    # dorsal cap from the well's outer wall to the rim
    r0 = R_COLLAR_OUT if collar_well else R_FIXED_CAP
    for i in range(n_cap + 1):
        r = r0 + (R_RIM - r0) * i / n_cap
        if collar_well or i > 0:
            pts.append((r, cap_z(r)))
    if ro > R_RIM + 1e-6:
        pts.append((ro, 0.0))       # fairing crown flat at z = 0
    pts.append((ro, Z_VENTRAL))     # rim / fairing face
    pts.append((R_DISH, Z_VENTRAL))  # ventral annulus
    # knuckle, dish rim -> crown edge
    for i in range(1, n_knuckle + 1):
        phi = math.pi / 2 * (1 - i / n_knuckle)
        pts.append(knuckle_rz(phi))
    # crown, r 1050 -> Praetorian well (or axis)
    r_end = PRAETORIAN.well_r if with_praetorian else 0.0
    for i in range(1, n_crown + 1):
        r = R_CROWN + (r_end - R_CROWN) * i / n_crown
        pts.append((r, crown_z(r)))
    if with_praetorian:
        p = PRAETORIAN
        pts.append((p.well_r, p.zc))          # well wall up to the equator
        pts.append((p.bowl_inner, p.zc))      # bowl lip, 4 m ledge
        for i in range(1, n_bowl + 1):        # bowl arc to the axis
            a = math.pi / 2 * i / n_bowl
            pts.append((p.bowl_inner * math.cos(a), p.zc + p.bowl_inner * math.sin(a)))
    return pts


# ================================================================ allocation
# Rev H interior allocation (Annex B.5-B.11, C.12), as volumes.
# kinds: box (spine frame: u radial along the spine at longitude th, v toward
# increasing theta), wedge (r0, r1, dtheta0, dtheta1), sphere (rc, radius),
# boxR (u from u0 out to the fairing curve less a margin).
VOID, SOFT, APERTURE, MATES = 1, 2, 4, 8

# category -> (label, colour) shared by section tints, volumes and legends
CATEGORIES = {
    "gallery": ("Galleries, spurs, lifts, mouths", (0.85, 0.65, 0.23)),
    "void": ("Command voids (bridge, flag plot)", (0.80, 0.72, 0.50)),
    "reactor": ("Reactor and auxiliary halls", (0.72, 0.64, 0.56)),
    "containment": ("Hypermatter containment", (0.66, 0.38, 0.31)),
    "ordnance": ("Ordnance cells", (0.79, 0.58, 0.50)),
    "growing": ("Growing decks", (0.61, 0.69, 0.51)),
    "shieldgen": ("Shield generator halls", (0.56, 0.64, 0.70)),
    "screengen": ("Cavity-screen generators", (0.53, 0.66, 0.72)),
    "trunk": ("Main trunks", (0.49, 0.53, 0.58)),
    "boom": ("Boom stowage", (0.61, 0.56, 0.48)),
    "weapons": ("Battery installations", (0.70, 0.52, 0.45)),
    "stores": ("Recycling and stores", (0.65, 0.68, 0.61)),
    "fabrication": ("Fabrication and magazines", (0.79, 0.64, 0.49)),
    "airgroup": ("Air group handling", (0.72, 0.70, 0.62)),
    "berthing": ("Berthing", (0.80, 0.76, 0.68)),
}


def allocation(variant="model"):
    """variant 'spec': Rev H exactly as written. 'model': the minimal
    corrections the model applies (see docs/verification-report.md)."""
    M = variant == "model"
    V = []

    def add(kind, name, cat, th, copies, step, A, z0, z1, flags=0, parent=None, param=0.0, note=""):
        for k in range(copies):
            V.append(dict(kind=kind, name=name, cat=cat, th=th + step * k, A=A, z0=z0, z1=z1, flags=flags,
                          parent=parent, param=param, note=note))
    R = R_RIM
    add("wedge", "Lower gallery", "gallery", 0, 1, 45, (760, 840, -180, 180), -248, -224, VOID | MATES)
    add("wedge", "Upper gallery", "gallery", 0, 1, 45, (760, 840, -180, 180), -182, -158, VOID | MATES)
    add("box", "Lower spur", "gallery", 0, 8, 45, (340, 1118, -30, 30), -248, -224, VOID | MATES)
    add("box", "Upper spur", "gallery", 0, 8, 45, (340, 1118, -30, 30), -182, -158, VOID | MATES)
    for s in (1, -1):
        add("box", "Launch mouth", "gallery", s * MOUTH_DTH, 8, 45, (1100, 1130, -60, 60), -575, -515, VOID | APERTURE)
        add("box", "Mouth lift trunk", "gallery", s * MOUTH_DTH, 8, 45, (1106, 1130, -20, 20), -515, -236, VOID)
    if M:
        add("box", "Mouth head transfer (assumed)", "gallery", 0, 8, 45, (1096, 1130, -142, 142), -248, -224, VOID | MATES,
            note="not in Rev H: joins each lower spur to its two lift trunks")
    add("box", "Combat bridge", "void", 0, 1, 45, (380, 560, -60, 60), -296, -272, VOID)
    add("wedge", "Flag plot", "void", 0, 1, 45, (210, 246 if M else 250, -180, 180), 16, 40, VOID)
    for th, rc in [(150, 1570), (210, 1570), (165, 1640), (195, 1640)]:
        add("sphere", "Reactor containment", "containment", th, 1, 0, (rc, 90), -407, -227, parent="Reactor hall")
    add("boxR", "Reactor hall", "reactor", 150, 2, 60, (1420, 3000, -150, 150), -566, -68, param=40)
    add("boxR", "Reactor hall", "reactor", 165, 2, 30, (1490, 3000, -150, 150), -566, -68, param=40)
    add("sphere", "Auxiliary containment", "containment", 0, 8, 45, (1400, 40), -340, -260, parent="Auxiliary block")
    add("box", "Auxiliary block", "reactor", 0, 8, 45, (1300, 1500, -100, 100), -450, -150)
    r_ord = 1080 if M else 1050
    add("wedge", "Ordnance cells", "ordnance", 180, 1, 45, (r_ord, 1400, -34, -7.5), -476, -296)
    add("wedge", "Ordnance cells", "ordnance", 180, 1, 45, (r_ord, 1400, 7.5, 34), -476, -296)
    zg1 = -338 if M else -344
    for th, cp in [(0, 3), (270, 2)]:
        add("box", "Growing decks", "growing", th, cp, 45, (1350, 1550, 100, 284), -386, zg1)
        add("box", "Growing decks", "growing", th, cp, 45, (1350, 1550, -284, -100), -386, zg1)
    add("box", "Growing decks", "growing", 135, 1, 45, (1350, 1550, -284, -100), -386, zg1)
    add("box", "Growing decks", "growing", 135, 1, 45, (1350, 1550, -484, -300), -386, zg1)
    add("box", "Growing decks", "growing", 225, 1, 45, (1350, 1550, 100, 284), -386, zg1)
    add("box", "Growing decks", "growing", 225, 1, 45, (1350, 1550, 300, 484), -386, zg1)
    add("box", "Rim shield generator", "shieldgen", 0, 8, 45, (1500, 1674, -100, 100), -288, -148)
    add("box", "Ventral shield generator", "shieldgen", 0, 8, 45, (1500, 1630, -100, 100), -483, -343)
    add("box", "Dorsal shield generator", "shieldgen", 0, 8, 45, (1019, 1219, -100, 100), -150, -30)
    add("box", "Main trunk", "trunk", 0, 8, 45, (1230, 1290, -30, 30), -626, -14)
    ub = (1135, 1175) if M else (1130, 1170)
    add("box", "Upper boom, stowed", "boom", 0, 8, 45, (ub[0], ub[1], -400, 0), -540, -500)
    add("box", "Lower boom, stowed", "boom", 0, 8, 45, (ub[0], ub[1], 0, 400), -618, -578)
    add("box", "Screen generator hall", "screengen", 22.5, 8, 45, (1190, 1310, -60, 60), -290, -170)
    for z in HEAVY_ROWS_Z:
        add("wedge", "Heavy rim battery", "weapons", 0, 1, 45, (1634, 1674, -135, 135), z - 22, z + 22)
        add("wedge", "Heavy battery feed and magazine", "weapons", 0, 1, 45, (1510, 1634, -135, 135), z - 22, z + 22)
    add("wedge", "Medium battery, dorsal one-third", "weapons", 0, 1, 45, (548, 571, -180, 180), cap_z(559.5) - 45, cap_z(559.5) - 10)
    add("wedge", "Medium battery, dorsal two-thirds", "weapons", 0, 1, 45, (1108, 1130, -180, 180), cap_z(1119) - 45, cap_z(1119) - 10)
    add("wedge", "Medium battery, ventral one-third", "weapons", 0, 1, 45, (548, 571, -180, 180), crown_z(559.5) + 4, crown_z(559.5) + 49)
    add("wedge", "Medium battery, ventral outer", "weapons", 0, 1, 45, (1320, 1342, -180, 180), -626, -581)
    add("wedge", "Recycling and stores", "stores", 0, 1, 45, (1150, 1500, -135, 135), -506, -392, SOFT)
    add("wedge", "Fabrication and magazines", "fabrication", 0, 1, 45, (0, 600, -180, 180), -356, -242, SOFT)
    add("wedge", "Air group handling", "airgroup", 0, 1, 45, (0, R, -180, 180), -164, -122, SOFT)
    add("wedge", "Berthing", "berthing", 0, 1, 45, (0, R, -180, 180), -116, 40, SOFT)
    return V
