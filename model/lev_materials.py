"""Materials, after Annex D (finish follows the builder).

Everything procedural, in object coordinates, so the model needs no image
textures and the hull slabs keep their panelling when the stack is opened.
"""
from __future__ import annotations

import bpy

import lev_geom as G

MATS: dict[str, bpy.types.Material] = {}
# (material name, node name, input name, role) -> drivers bound later by lev_scene
EMISSION_HOOKS: list[tuple[str, str, str, str]] = []


class NB:
    """Tiny node-building helper."""

    def __init__(self, tree):
        self.t = tree
        self.nodes = tree.nodes
        self.links = tree.links
        self.x = 0

    def n(self, kind, label=None, **props):
        node = self.nodes.new(kind)
        for k, v in props.items():
            setattr(node, k, v)
        if label:
            node.label = label
            node.name = label
        node.location = (self.x, 0)
        self.x += 40
        return node

    def link(self, a, b):
        self.links.new(a, b)

    def _in(self, node, idx, v):
        if isinstance(v, bpy.types.NodeSocket):
            self.link(v, node.inputs[idx])
        elif v is not None:
            node.inputs[idx].default_value = v

    def math(self, op, a, b=None, c=None, clamp=False):
        m = self.n("ShaderNodeMath", operation=op, use_clamp=clamp)
        self._in(m, 0, a)
        if b is not None:
            self._in(m, 1, b)
        if c is not None:
            self._in(m, 2, c)
        return m.outputs[0]

    def vmath(self, op, a, b=None, out=0):
        m = self.n("ShaderNodeVectorMath", operation=op)
        self._in(m, 0, a)
        if b is not None:
            self._in(m, 1, b)
        return m.outputs[out]

    def mix(self, fac, a, b):
        m = self.n("ShaderNodeMix", data_type="RGBA")
        self._in(m, "Factor", fac)
        self._in(m, 6, a)
        self._in(m, 7, b)
        return m.outputs[2]

    def sep(self, v):
        s = self.n("ShaderNodeSeparateXYZ")
        self.link(v, s.inputs[0])
        return s.outputs[0], s.outputs[1], s.outputs[2]

    def rgb(self, col):
        n = self.n("ShaderNodeRGB")
        n.outputs[0].default_value = (*col, 1.0)
        return n.outputs[0]

    def value(self, v, label=None):
        n = self.n("ShaderNodeValue", label=label)
        n.outputs[0].default_value = v
        return n.outputs[0]


def _new(name):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    m.node_tree.nodes.clear()
    MATS[name] = m
    return m, NB(m.node_tree)


def _out(nb, shader):
    o = nb.n("ShaderNodeOutputMaterial")
    nb.link(shader, o.inputs[0])


def principled(nb, base, rough=0.6, metal=0.3, spec=0.5, **kw):
    p = nb.n("ShaderNodeBsdfPrincipled")
    nb._in(p, "Base Color", base if isinstance(base, bpy.types.NodeSocket) else (*base, 1.0))
    nb._in(p, "Roughness", rough)
    nb._in(p, "Metallic", metal)
    nb._in(p, "Specular IOR Level", spec)
    for k, v in kw.items():
        nb._in(p, k.replace("_", " "), v if isinstance(v, bpy.types.NodeSocket) or not isinstance(v, tuple) or len(v) != 3 else (*v, 1.0))
    return p


def simple(name, col, rough=0.6, metal=0.3, spec=0.5, emit=None, strength=0.0, role=None, alpha=None):
    m, nb = _new(name)
    kw = {}
    if emit is not None:
        kw["Emission_Color"] = (*emit, 1.0)
        kw["Emission_Strength"] = strength
    p = principled(nb, col, rough, metal, spec, **kw)
    p.name = "BSDF"
    if alpha is not None:
        p.inputs["Alpha"].default_value = alpha
    _out(nb, p.outputs[0])
    if role:
        EMISSION_HOOKS.append((name, "BSDF", "Emission Strength", role))
    return m


# ------------------------------------------------------------------ panel grid
def panel_grid(nb, co, nrm):
    """Returns (seam 0..1, cell random 0..1) for the Annex B.4 frame grid:
    rings every 20 m of radius, 64 radial primaries doubling at r 200, 400,
    800 and 1,600, 20 m courses up the rim."""
    x, y, z = nb.sep(co)
    r = nb.math("SQRT", nb.math("ADD", nb.math("MULTIPLY", x, x), nb.math("MULTIPLY", y, y)))
    th = nb.math("DIVIDE", nb.math("ARCTAN2", x, y), 6.283185)
    th = nb.math("FRACT", th)
    k = nb.value(64.0)
    for rb in (200, 400, 800, 1600):
        k = nb.math("MULTIPLY", k, nb.math("ADD", 1.0, nb.math("GREATER_THAN", r, rb)))
    sector = nb.math("MULTIPLY", th, k)
    ring = nb.math("DIVIDE", r, 20.0)
    course = nb.math("DIVIDE", z, 20.0)
    _, _, nz = nb.sep(nrm)
    horiz = nb.math("ABSOLUTE", nz)
    vert = nb.math("SUBTRACT", 1.0, horiz)

    def line(v, width_units):
        d = nb.math("ABSOLUTE", nb.math("SUBTRACT", nb.math("FRACT", nb.math("ADD", v, 0.5)), 0.5))
        return nb.math("SUBTRACT", 1.0, nb.math("DIVIDE", d, width_units), clamp=True)
    # seam widths ~0.35 m; radial seam width in sector units = 0.35 * k / (2 pi r)
    w_sec = nb.math("DIVIDE", nb.math("MULTIPLY", k, 0.35), nb.math("MAXIMUM", nb.math("MULTIPLY", r, 6.283185), 1.0))
    s1 = line(sector, w_sec)
    s2 = nb.math("MULTIPLY", line(ring, 0.0175), horiz)
    s3 = nb.math("MULTIPLY", line(course, 0.0175), vert)
    seam = nb.math("MAXIMUM", s1, nb.math("MAXIMUM", s2, s3))
    cell = nb.n("ShaderNodeCombineXYZ")
    nb.link(nb.math("FLOOR", sector), cell.inputs[0])
    nb.link(nb.math("FLOOR", nb.math("ADD", nb.math("MULTIPLY", ring, horiz), nb.math("MULTIPLY", course, vert))), cell.inputs[1])
    nb.link(nb.math("ROUND", horiz), cell.inputs[2])
    wn = nb.n("ShaderNodeTexWhiteNoise", noise_dimensions="3D")
    nb.link(cell.outputs[0], wn.inputs["Vector"])
    return seam, wn.outputs["Value"], r, th, z


def plate(name, base, rough=0.74, metal=0.35, darken_stern=False, pitting=1.0, seam_strength=0.07, repair=True):
    """Neutronium-impregnated quadanium: matte, faint blue cast, closed ground seams,
    unblended repair plates, micrometeorite pitting."""
    m, nb = _new(name)
    tc = nb.n("ShaderNodeTexCoord")
    seam, rnd, r, th, z = panel_grid(nb, tc.outputs["Object"], tc.outputs["Normal"])
    col = nb.rgb(base)
    var = nb.math("MULTIPLY", nb.math("SUBTRACT", rnd, 0.5), 0.06)
    hsv = nb.n("ShaderNodeHueSaturation")
    hsv.inputs["Value"].default_value = 1.0
    nb.link(nb.math("ADD", 1.0, var), hsv.inputs["Value"])
    nb.link(col, hsv.inputs["Color"])
    c = hsv.outputs[0]
    if repair:
        newer = nb.math("GREATER_THAN", rnd, 0.988)
        c = nb.mix(nb.math("MULTIPLY", newer, 0.55), c, nb.rgb((base[0] * 1.18, base[1] * 1.2, base[2] * 1.26)))
    c = nb.mix(nb.math("MULTIPLY", seam, seam_strength / 0.07 * 0.55), c, nb.rgb((base[0] * 0.55, base[1] * 0.56, base[2] * 0.6)))
    if darken_stern:
        # fairing crown darkened for 200 m forward of the emitter face (D.2)
        cos2 = nb.math("COSINE", nb.math("MULTIPLY", nb.math("SUBTRACT", nb.math("MULTIPLY", th, 6.283185), 3.14159265), 2.0))
        rout = nb.math("ADD", G.R_RIM, nb.math("MULTIPLY", 180.0, cos2))
        aft = nb.math("MULTIPLY", nb.math("GREATER_THAN", th, 0.375), nb.math("LESS_THAN", th, 0.625))
        near = nb.math("SUBTRACT", 1.0, nb.math("DIVIDE", nb.math("SUBTRACT", rout, r), 200.0), clamp=True)
        f = nb.math("MULTIPLY", nb.math("MULTIPLY", aft, near), 0.75)
        c = nb.mix(f, c, nb.rgb((0.10, 0.095, 0.09)))
    p = principled(nb, c, rough, metal, 0.32)
    # pitting and seam relief
    bump = nb.n("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.25 * pitting
    bump.inputs["Distance"].default_value = 0.4
    vn = nb.n("ShaderNodeTexVoronoi", feature="F1")
    vn.inputs["Scale"].default_value = 0.35
    nb.link(tc.outputs["Object"], vn.inputs["Vector"])
    h = nb.math("ADD", nb.math("MULTIPLY", seam, -0.6), nb.math("MULTIPLY", nb.math("LESS_THAN", vn.outputs["Distance"], 0.05), -0.5))
    nb.link(h, bump.inputs["Height"])
    nb.link(bump.outputs[0], p.inputs["Normal"])
    _out(nb, p.outputs[0])
    return m


# ------------------------------------------------------------ section shader
def section_material(name="Section", core=None):
    """Cut faces: soft-zone tint by (r, z), deck lines every 6 m anchored on
    z = -236 with the gallery decks in amber, ring frames every 20 m. Fed by
    object coordinates, so it reads correctly on every slab and core."""
    m, nb = _new(name)
    tc = nb.n("ShaderNodeTexCoord")
    co = tc.outputs["Object"]
    x, y, z = nb.sep(co)
    _, _, nz = nb.sep(tc.outputs["Normal"])
    horiz = nb.math("ABSOLUTE", nz)
    vert = nb.math("SUBTRACT", 1.0, horiz)
    r = nb.math("SQRT", nb.math("ADD", nb.math("MULTIPLY", x, x), nb.math("MULTIPLY", y, y)))
    th = nb.math("FRACT", nb.math("DIVIDE", nb.math("ARCTAN2", x, y), 6.283185))

    def band(v, lo, hi):
        return nb.math("MULTIPLY", nb.math("GREATER_THAN", v, lo), nb.math("LESS_THAN", v, hi))
    base = nb.rgb((0.34, 0.325, 0.30))   # hull structure / compartmented void
    col = base
    if core is None:
        zones = [
            (nb.math("MULTIPLY", band(z, -116, 60), 1.0), (0.86, 0.74, 0.56)),                 # berthing
            (band(z, -164, -122), (0.62, 0.66, 0.44)),                                         # air group handling
            (nb.math("MULTIPLY", band(z, -230, -176), band(r, 0, 1500)), (0.55, 0.62, 0.72)),  # handling and stores
            (nb.math("MULTIPLY", band(z, -356, -242), band(r, 0, 600)), (0.86, 0.55, 0.32)),   # fabrication, magazines
            (nb.math("MULTIPLY", band(z, -506, -392), band(r, 1150, 1500)), (0.50, 0.66, 0.52)),  # recycling and stores
            (band(z, -640, -506), (0.46, 0.52, 0.60)),                                         # ventral traffic
            (nb.math("MULTIPLY", band(r, 1500, 2000), band(z, -532, -142)), (0.70, 0.42, 0.38)),  # weapons band
        ]
        aft = nb.math("MULTIPLY", nb.math("MULTIPLY", band(th, 0.375, 0.625), band(r, 1000, 3000)), 1.0)
        for f, c in zones:
            col = nb.mix(nb.math("MULTIPLY", f, 0.9), col, nb.rgb(tuple(x * 0.55 for x in c)))
        col = nb.mix(nb.math("MULTIPLY", aft, 0.6), col, nb.rgb((0.36, 0.28, 0.25)))
        zdeck = z
    else:
        # core section: bands by latitude in the core's own frame (B.6 internal arrangement)
        rc, anchor = core
        s = nb.math("DIVIDE", z, rc)
        zones = [
            (nb.math("GREATER_THAN", s, 0.866), (0.55, 0.60, 0.66)),                # drive cluster and plant (>60 N)
            (band(s, 0.26, 0.574), G.CATEGORIES["berthing"][1]),                  # between hangar decks
            (band(s, -0.5, 0.0), (0.66, 0.62, 0.52)),                             # vehicle decks, equator to 30 S
            (band(s, -0.77, -0.5), G.CATEGORIES["stores"][1]),                    # stores, water, ramp lobbies
            (nb.math("LESS_THAN", s, -0.77), (0.80, 0.78, 0.72)),                 # southern band and polar cap
        ]
        for f, c in zones:
            col = nb.mix(nb.math("MULTIPLY", f, 0.85), col, nb.rgb(tuple(x * 0.62 for x in c)))
        zdeck = nb.math("ADD", z, anchor)
    # deck lines on vertical cut faces
    d = nb.math("ABSOLUTE", nb.math("SUBTRACT", nb.math("FRACT", nb.math("ADD", nb.math("DIVIDE", nb.math("ADD", zdeck, 236.0), 6.0), 0.5)), 0.5))
    deck = nb.math("MULTIPLY", nb.math("LESS_THAN", d, 0.06), vert)
    col = nb.mix(nb.math("MULTIPLY", deck, 0.55), col, nb.rgb((0.20, 0.20, 0.21)))
    if core is None:
        g1 = nb.math("LESS_THAN", nb.math("ABSOLUTE", nb.math("ADD", z, 236.0)), 1.2)
        g2 = nb.math("LESS_THAN", nb.math("ABSOLUTE", nb.math("ADD", z, 170.0)), 1.2)
        gal = nb.math("MULTIPLY", nb.math("MAXIMUM", g1, g2), vert)
        col = nb.mix(gal, col, nb.rgb((0.95, 0.66, 0.18)))
        # ring frames every 20 m and radial primaries on horizontal cut faces
        dr = nb.math("ABSOLUTE", nb.math("SUBTRACT", nb.math("FRACT", nb.math("ADD", nb.math("DIVIDE", r, 20.0), 0.5)), 0.5))
        rings = nb.math("MULTIPLY", nb.math("LESS_THAN", dr, 0.03), horiz)
        ds = nb.math("ABSOLUTE", nb.math("SUBTRACT", nb.math("FRACT", nb.math("ADD", nb.math("MULTIPLY", th, 64.0), 0.5)), 0.5))
        radials = nb.math("MULTIPLY", nb.math("LESS_THAN", nb.math("MULTIPLY", ds, nb.math("DIVIDE", nb.math("MULTIPLY", r, 6.283185), 64.0)), 0.8), horiz)
        col = nb.mix(nb.math("MULTIPLY", nb.math("MAXIMUM", rings, radials), 0.35), col, nb.rgb((0.25, 0.25, 0.26)))
    p = principled(nb, col, 0.88, 0.05, 0.25)
    _out(nb, p.outputs[0])
    return m


# -------------------------------------------------------------- special ones
def glazing():
    m, nb = _new("Glazing, transparisteel")
    tc = nb.n("ShaderNodeTexCoord")
    x, y, z = nb.sep(tc.outputs["Object"])
    # pane grid 10 x 6 m
    th = nb.math("ARCTAN2", x, y)
    u = nb.math("MULTIPLY", th, 350.0 / 10.0)
    v = nb.math("DIVIDE", z, 6.0)

    def grid(val, w):
        d = nb.math("ABSOLUTE", nb.math("SUBTRACT", nb.math("FRACT", val), 0.5))
        return nb.math("GREATER_THAN", d, 0.5 - w)
    mull = nb.math("MAXIMUM", grid(u, 0.03), grid(v, 0.05))
    glass = principled(nb, (0.02, 0.03, 0.04), 0.05, 0.0, 1.0, Emission_Color=(1.0, 0.78, 0.52, 1), Emission_Strength=0.0)
    glass.name = "BSDF"
    EMISSION_HOOKS.append((m.name, "BSDF", "Emission Strength", "interior"))
    frame = principled(nb, (0.30, 0.31, 0.33), 0.5, 0.6, 0.5)
    mix = nb.n("ShaderNodeMixShader")
    nb.link(mull, mix.inputs[0])
    nb.link(glass.outputs[0], mix.inputs[1])
    nb.link(frame.outputs[0], mix.inputs[2])
    _out(nb, mix.outputs[0])
    return m


def drive_emitter():
    """Dallorian throat liners: heat-tinted bronze -> straw -> blue-grey (D.2).
    Only the throat slots of the linear emitter glow, driven by the throttle."""
    m, nb = _new("Drive emitter, Dallorian liners")
    tc = nb.n("ShaderNodeTexCoord")
    x, y, z = nb.sep(tc.outputs["Object"])
    t = nb.math("ABSOLUTE", nb.math("DIVIDE", nb.math("ADD", z, 315.0), 315.0))   # 0 at mid-depth, 1 at faces
    ramp = nb.n("ShaderNodeValToRGB")
    cr = ramp.color_ramp
    cr.elements[0].position = 0.0
    cr.elements[0].color = (0.30, 0.34, 0.42, 1)   # blue-grey where hottest
    cr.elements[1].position = 1.0
    cr.elements[1].color = (0.40, 0.26, 0.12, 1)   # bronze at the cooler edges
    e = cr.elements.new(0.5)
    e.color = (0.62, 0.50, 0.28, 1)                # straw
    noise = nb.n("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 0.02
    nb.link(tc.outputs["Object"], noise.inputs["Vector"])
    nb.link(nb.math("ADD", t, nb.math("MULTIPLY", nb.math("SUBTRACT", noise.outputs["Fac"], 0.5), 0.35)), ramp.inputs[0])
    # vertical throat slots, one every 12 m of face
    th = nb.math("ARCTAN2", x, y)
    slot = nb.math("ABSOLUTE", nb.math("SUBTRACT", nb.math("FRACT", nb.math("MULTIPLY", th, 1860.0 / 12.0)), 0.5))
    lou = nb.math("GREATER_THAN", slot, 0.43)
    edge = nb.math("MULTIPLY", nb.math("GREATER_THAN", t, 0.03), nb.math("LESS_THAN", t, 0.96))
    glow = nb.math("MULTIPLY", lou, edge)
    col = nb.mix(nb.math("MULTIPLY", lou, 0.85), ramp.outputs[0], nb.rgb((0.03, 0.035, 0.045)))
    g = nb.value(1.0, label="Throttle glow")
    EMISSION_HOOKS.append((m.name, "Throttle glow", "OUT", "drive"))
    p = principled(nb, col, 0.42, 0.8, 0.5, Emission_Color=(0.45, 0.72, 1.0, 1))
    nb.link(nb.math("MULTIPLY", glow, g), p.inputs["Emission Strength"])
    _out(nb, p.outputs[0])
    return m


def field(name, col, role, base_alpha=0.06, hex_scale=0.02):
    """Energy surface (cavity screen, shield bubbles): fresnel-weighted emission
    over transparency, with a faint hexagonal gating grid."""
    m, nb = _new(name)
    tc = nb.n("ShaderNodeTexCoord")
    vor = nb.n("ShaderNodeTexVoronoi", feature="DISTANCE_TO_EDGE")
    vor.inputs["Scale"].default_value = hex_scale
    nb.link(tc.outputs["Object"], vor.inputs["Vector"])
    edge = nb.math("LESS_THAN", vor.outputs["Distance"], 0.04)
    lw = nb.n("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.3
    em = nb.n("ShaderNodeEmission", label="Emission")
    em.inputs["Color"].default_value = (*col, 1)
    em.inputs["Strength"].default_value = 1.0
    EMISSION_HOOKS.append((m.name, "Emission", "Strength", role))
    tr = nb.n("ShaderNodeBsdfTransparent")
    mix = nb.n("ShaderNodeMixShader")
    f = nb.math("ADD", nb.math("MULTIPLY", lw.outputs["Facing"], 0.18), nb.math("ADD", base_alpha, nb.math("MULTIPLY", edge, 0.045)), clamp=True)
    nb.link(f, mix.inputs[0])
    nb.link(tr.outputs[0], mix.inputs[1])
    nb.link(em.outputs[0], mix.inputs[2])
    _out(nb, mix.outputs[0])
    return m


def hazard(name="Hazard marking", c1=(0.62, 0.52, 0.22), c2=(0.12, 0.12, 0.12), scale=0.25, rough=0.7):
    m, nb = _new(name)
    tc = nb.n("ShaderNodeTexCoord")
    x, y, z = nb.sep(tc.outputs["Object"])
    s = nb.math("MULTIPLY", nb.math("ADD", nb.math("ADD", x, y), z), scale)
    st = nb.math("GREATER_THAN", nb.math("FRACT", s), 0.5)
    col = nb.mix(st, nb.rgb(c1), nb.rgb(c2))
    p = principled(nb, col, rough, 0.1, 0.3)
    _out(nb, p.outputs[0])
    return m


def ceramic_cap():
    """Core southern cap: pale chalky refractory ceramic, scorched grey-brown at
    the stagnation region (D.2). Generated z runs 0 (south pole) to 1."""
    m, nb = _new("Core south cap, refractory ceramic")
    tc = nb.n("ShaderNodeTexCoord")
    _, _, gz = nb.sep(tc.outputs["Generated"])
    noise = nb.n("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 6.0
    noise.inputs["Detail"].default_value = 6.0
    nb.link(tc.outputs["Generated"], noise.inputs["Vector"])
    scorch = nb.math("SUBTRACT", 1.0, nb.math("DIVIDE", gz, 0.14), clamp=True)
    scorch = nb.math("MULTIPLY", scorch, nb.math("ADD", 0.55, nb.math("MULTIPLY", noise.outputs["Fac"], 0.6)), clamp=True)
    col = nb.mix(scorch, nb.rgb((0.78, 0.75, 0.69)), nb.rgb((0.30, 0.25, 0.21)))
    p = principled(nb, col, 0.92, 0.0, 0.3)
    _out(nb, p.outputs[0])
    return m


def world_space(strength=1.0):
    w = bpy.data.worlds.new("Deep space")
    w.use_nodes = True
    nt = w.node_tree
    nt.nodes.clear()
    nb = NB(nt)
    tc = nb.n("ShaderNodeTexCoord")
    vor = nb.n("ShaderNodeTexVoronoi", feature="F1")
    vor.inputs["Scale"].default_value = 420.0
    vor.inputs["Randomness"].default_value = 1.0
    nb.link(tc.outputs["Generated"], vor.inputs["Vector"])
    star = nb.math("LESS_THAN", vor.outputs["Distance"], 0.035)
    wn = nb.n("ShaderNodeTexWhiteNoise", noise_dimensions="3D")
    nb.link(vor.outputs["Position"], wn.inputs["Vector"])
    bright = nb.math("POWER", wn.outputs["Value"], 6.0)
    s = nb.math("MULTIPLY", nb.math("MULTIPLY", star, bright), 6.0)
    # faint galactic band
    noise = nb.n("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = 2.2
    noise.inputs["Detail"].default_value = 8
    nb.link(tc.outputs["Generated"], noise.inputs["Vector"])
    _, gy, gz = nb.sep(tc.outputs["Generated"])
    bandf = nb.math("SUBTRACT", 1.0, nb.math("DIVIDE", nb.math("ABSOLUTE", nb.math("ADD", gz, nb.math("MULTIPLY", gy, 0.35))), 0.25), clamp=True)
    glow = nb.math("MULTIPLY", nb.math("MULTIPLY", bandf, nb.math("POWER", noise.outputs["Fac"], 3.0)), 0.02)
    tot = nb.math("ADD", s, glow)
    tot = nb.math("ADD", tot, 0.0004)
    bg = nb.n("ShaderNodeBackground", label="Stars")
    nb.link(tot, bg.inputs["Strength"])
    bg.inputs["Color"].default_value = (0.85, 0.9, 1.0, 1)
    lp = nb.n("ShaderNodeLightPath")
    # stars only to camera rays so they don't light the ship
    amb = nb.n("ShaderNodeBackground", label="Ambient")
    amb.inputs["Color"].default_value = (0.35, 0.40, 0.50, 1)
    amb.inputs["Strength"].default_value = 0.012 * strength
    mix = nb.n("ShaderNodeMixShader")
    nb.link(lp.outputs["Is Camera Ray"], mix.inputs[0])
    nb.link(amb.outputs[0], mix.inputs[1])
    nb.link(bg.outputs[0], mix.inputs[2])
    o = nb.n("ShaderNodeOutputWorld")
    nb.link(mix.outputs[0], o.inputs[0])
    return w


def planet():
    m, nb = _new("Planet surface")
    tc = nb.n("ShaderNodeTexCoord")
    n1 = nb.n("ShaderNodeTexNoise")
    n1.inputs["Scale"].default_value = 3.0
    n1.inputs["Detail"].default_value = 12
    n1.inputs["Roughness"].default_value = 0.62
    nb.link(tc.outputs["Object"], n1.inputs["Vector"])
    n1.inputs["Scale"].default_value = 26.0 / 6.371e6
    land = nb.math("GREATER_THAN", n1.outputs["Fac"], 0.52)
    ramp_l = nb.mix(nb.math("POWER", n1.outputs["Fac"], 2.0), nb.rgb((0.18, 0.22, 0.12)), nb.rgb((0.42, 0.36, 0.26)))
    col = nb.mix(land, nb.rgb((0.02, 0.06, 0.13)), ramp_l)
    n2 = nb.n("ShaderNodeTexNoise")
    n2.inputs["Scale"].default_value = 70.0 / 6.371e6
    n2.inputs["Detail"].default_value = 10
    n2.inputs["Distortion"].default_value = 0.8
    nb.link(tc.outputs["Object"], n2.inputs["Vector"])
    cloud = nb.math("MULTIPLY", nb.math("SUBTRACT", n2.outputs["Fac"], 0.5), 3.0, clamp=True)
    col = nb.mix(cloud, col, nb.rgb((0.85, 0.87, 0.9)))
    p = principled(nb, col, nb.math("SUBTRACT", 0.95, nb.math("MULTIPLY", nb.math("SUBTRACT", 1.0, land), 0.45)), 0.0, 0.35)
    lw = nb.n("ShaderNodeLayerWeight")
    lw.inputs["Blend"].default_value = 0.08
    em = nb.n("ShaderNodeEmission")
    em.inputs["Color"].default_value = (0.35, 0.6, 1.0, 1)
    em.inputs["Strength"].default_value = 0.6
    add = nb.n("ShaderNodeMixShader")
    nb.link(nb.math("POWER", lw.outputs["Fresnel"], 3.0), add.inputs[0])
    nb.link(p.outputs[0], add.inputs[1])
    nb.link(em.outputs[0], add.inputs[2])
    _out(nb, add.outputs[0])
    return m


def build_all():
    # exterior plate: Kuat practice, flat, machined and anonymous
    plate("Hull plate, dorsal", (0.205, 0.218, 0.24), rough=0.8, darken_stern=True)
    plate("Hull plate, rim", (0.20, 0.212, 0.23), rough=0.76)
    plate("Hull plate, ventral", (0.215, 0.222, 0.235), rough=0.72)
    plate("Hull plate, cavity", (0.25, 0.252, 0.255), rough=0.62, pitting=0.1)
    plate("Collar well, alusteel", (0.30, 0.305, 0.31), rough=0.5, metal=0.55, repair=False)
    plate("Bowl and well, alusteel", (0.40, 0.41, 0.42), rough=0.42, metal=0.62, pitting=0.2, repair=False)
    section_material("Section")
    section_material("Section, Lance", core=(210.0, -290.0))
    section_material("Section, Praetorian", core=(348.0, -282.0))
    for key, (label, col) in G.CATEGORIES.items():
        simple(f"Volume, {label}", tuple(c * 0.7 for c in col), 0.8, 0.05, 0.3)
    simple("Gallery interior, alusteel joiner", (0.40, 0.40, 0.39), 0.45, 0.3, 0.5,
           emit=(1.0, 0.92, 0.8), strength=0.0, role="interior")
    glazing()
    drive_emitter()
    simple("Collar plate", (0.21, 0.225, 0.245), 0.7, 0.4, 0.35)
    simple("Iris shutter", (0.19, 0.205, 0.225), 0.76, 0.35, 0.32)
    simple("Aperture, dark", (0.012, 0.013, 0.015), 0.9, 0.2, 0.2)
    simple("Seam, dark", (0.05, 0.05, 0.055), 0.8, 0.3, 0.2)
    simple("Sensor aperture", (0.03, 0.05, 0.06), 0.12, 0.5, 0.9, emit=(0.2, 0.7, 0.85), strength=0.0, role="sensor")
    simple("Shield emitter", (0.08, 0.09, 0.11), 0.35, 0.7, 0.5, emit=(0.35, 0.62, 1.0), strength=0.0, role="shield")
    simple("Screen emitter", (0.07, 0.10, 0.13), 0.3, 0.7, 0.5, emit=(0.25, 0.65, 1.0), strength=0.0, role="screen_emitter")
    field("Cavity screen, attenuating", (0.3, 0.62, 1.0), "screen_att", base_alpha=0.02, hex_scale=0.012)
    field("Cavity screen, hardened", (0.35, 0.68, 1.0), "screen_hard", base_alpha=0.10, hex_scale=0.018)
    simple("Turret, quadanium", (0.17, 0.18, 0.195), 0.55, 0.6, 0.45)
    simple("Turret barrel", (0.08, 0.085, 0.09), 0.45, 0.75, 0.5)
    simple("Mount shutter", (0.175, 0.19, 0.21), 0.72, 0.4, 0.3)
    simple("Duranium, bright worn", (0.72, 0.72, 0.70), 0.28, 0.95, 0.6)
    simple("Repulsor array", (0.14, 0.16, 0.19), 0.35, 0.75, 0.5, emit=(0.4, 0.55, 1.0), strength=0.0, role="repulsor")
    simple("Core shell, alusteel", (0.34, 0.35, 0.36), 0.5, 0.6, 0.45)
    simple("Core equatorial box, doonium", (0.20, 0.21, 0.225), 0.6, 0.55, 0.4)
    simple("Core frame ring", (0.15, 0.155, 0.165), 0.55, 0.6, 0.4)
    ceramic_cap()
    hazard("Hazard marking")
    hazard("Rothana hazard, yellow and black", (0.85, 0.66, 0.05), (0.03, 0.03, 0.03), scale=0.12, rough=0.55)
    simple("Paint, low-contrast marking", (0.105, 0.11, 0.12), 0.85, 0.1, 0.3)
    simple("Paint, cavity numerals", (0.78, 0.76, 0.70), 0.7, 0.1, 0.3)
    simple("Paint, core designation", (0.17, 0.16, 0.15), 0.85, 0.0, 0.3)
    simple("Boom lattice", (0.45, 0.46, 0.47), 0.45, 0.8, 0.5)
    simple("Light, position white", (0.9, 0.9, 0.9), 0.3, 0.0, 0.5, emit=(1.0, 0.96, 0.9), strength=0.0, role="nav")
    simple("Light, position red", (0.9, 0.1, 0.1), 0.3, 0.0, 0.5, emit=(1.0, 0.08, 0.05), strength=0.0, role="nav")
    simple("Light, collar beacon", (0.9, 0.1, 0.1), 0.3, 0.0, 0.5, emit=(1.0, 0.1, 0.05), strength=0.0, role="beacon")
    simple("Light, cavity amber", (0.9, 0.6, 0.2), 0.3, 0.0, 0.5, emit=(1.0, 0.62, 0.22), strength=0.0, role="flood")
    simple("Light, approach ladder, port", (0.9, 0.6, 0.2), 0.3, 0.0, 0.5, emit=(1.0, 0.55, 0.12), strength=0.0, role="flightops")
    simple("Light, approach ladder, starboard", (0.9, 0.9, 0.9), 0.3, 0.0, 0.5, emit=(0.85, 0.92, 1.0), strength=0.0, role="flightops")
    simple("Light, boom station", (0.2, 0.9, 0.3), 0.3, 0.0, 0.5, emit=(0.15, 1.0, 0.3), strength=0.0, role="boom")
    simple("Light, datum", (0.9, 0.9, 0.9), 0.3, 0.0, 0.5, emit=(1.0, 1.0, 1.0), strength=0.0, role="flightops")
    simple("Engine bell, core drive", (0.10, 0.10, 0.11), 0.35, 0.9, 0.5, emit=(0.4, 0.7, 1.0), strength=0.0, role="core_drive")
    planet()
    return MATS
