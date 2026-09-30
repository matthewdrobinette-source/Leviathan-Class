"""Checks that every control on the built .blend drives what it should.

    python3 verification/test_blend_controls.py blender/Leviathan_RevH.blend
"""
import math
import sys

import bpy

path = sys.argv[1] if len(sys.argv) > 1 else "blender/Leviathan_RevH.blend"
bpy.ops.wm.open_mainfile(filepath=path)
C = bpy.data.objects["LEVIATHAN controls"]
O = bpy.data.objects
FAIL = []


def setc(**kw):
    for k, v in kw.items():
        C[k] = type(C[k])(v)
    C.update_tag()
    bpy.context.view_layer.update()


def ev(name):
    return O[name].evaluated_get(bpy.context.evaluated_depsgraph_get())


def check(label, ok, detail=""):
    print(("PASS " if ok else "FAIL ") + label + (f"  ({detail})" if detail else ""))
    if not ok:
        FAIL.append(label)


def fit_count(prefix):
    """Instances the fitting clouds produce, summed over slabs."""
    n = 0
    dg = bpy.context.evaluated_depsgraph_get()
    for inst in dg.object_instances:
        if inst.is_instance and inst.parent and inst.parent.original.name.startswith(prefix):
            n += 1
    return n


setc(collar_extended=1, batteries_deployed=0, core_release=0, destack=0, section=0, screen_state=1, booms_deployed=0, mouths_open=1)
z_ext = ev("Bridge collar").matrix_world.translation.z
setc(collar_extended=0)
z_sto = ev("Bridge collar").matrix_world.translation.z
check("collar travels 60 m", abs((z_ext - z_sto) - 60.0) < 0.01, f"{z_ext:.1f} -> {z_sto:.1f}")
leaf = ev("Iris leaf 01").matrix_world.translation
check("iris leaves closed when stowed", leaf.length < 0.01, f"{tuple(round(v, 1) for v in leaf)}")
setc(collar_extended=1)
leaf = ev("Iris leaf 01").matrix_world.translation
check("iris leaves withdrawn when extended", leaf.length > 200, f"{leaf.length:.0f} m")

for key, n in (("Fittings, Heavy rim battery", 591 * 2), ("Fittings, Medium face batteries", 746 * 2),
               ("Fittings, Cavity defence", 640 * 2), ("Fittings, Sensor apertures, 88", 88)):
    got = fit_count(key)
    check(f"{key}: {n} instances (mount + shutter or open ring)", got == n, str(got))

z0 = ev("Lance One").matrix_world.translation.z
setc(core_release=0.2)
z1 = ev("Lance One").matrix_world.translation.z
z5 = ev("Lance Five").matrix_world.translation.z
z2 = ev("Lance Two").matrix_world.translation.z
zp = ev("Praetorian").matrix_world.translation.z
check("pair One/Five released first", z1 < z0 - 1400 and z5 < z0 - 1400 and abs(z2 - z0) < 0.01 and abs(zp + 282) < 0.01,
      f"One {z1:.0f}, Five {z5:.0f}, Two {z2:.0f}, Praetorian {zp:.0f}")
setc(core_release=1.0)
check("Praetorian released last", ev("Praetorian").matrix_world.translation.z < -1700)
setc(core_release=0.0, destack=1.0)
check("de-stack: dorsal slab lifts 900 m", abs(ev("Slab, dorsal shell and battery band").matrix_world.translation.z - 900) < 0.01)
check("de-stack: aft block draws 600 m aft", abs(ev("Slab, aft block (draws aft)").matrix_world.translation.y + 600) < 0.01)
check("de-stack: assembled hull hidden, slabs shown", O["Hull (assembled)"].hide_render and not O["Slab, lower gallery"].hide_render)
check("de-stack: cores drop 900 m", abs(ev("Lance Three").matrix_world.translation.z - (-290 - 900)) < 0.01)
setc(destack=0.0)
for q, name in ((1, "Cavity screen, attenuating"), (2, "Cavity screen, hardened"), (3, "Cavity screen, drawn inboard")):
    setc(screen_state=q)
    vis = [n for n in ("Cavity screen, attenuating", "Cavity screen, hardened", "Cavity screen, drawn inboard") if not O[n].hide_render]
    check(f"screen state {q} shows only '{name}'", vis == [name], str(vis))
setc(screen_state=1, mouths_open=0)
check("mouth shutters close", not O["Launch mouth outer shutters"].hide_render)
setc(mouths_open=1, booms_deployed=1)
check("booms deploy", not O["Boom 0 upper, deployed, section 3"].hide_render and O["Boom 0 upper, stowed"].hide_render)
setc(booms_deployed=0, section=1, section_longitude=22.5)
me = ev("Lance One").to_mesh()
n1 = len(me.polygons)
ev("Lance One").to_mesh_clear()
me = ev("Lance Three").to_mesh()
n3 = len(me.polygons)
ev("Lance Three").to_mesh_clear()
check("section at 22.5 deg cuts Lance One in half and removes Lance Three", 0 < n1 < 36512 and n3 == 0, f"{n1}, {n3}")
setc(section=0, section_longitude=0.0)
print(f"\n{'ALL PASS' if not FAIL else str(len(FAIL)) + ' FAILED'}")
sys.exit(1 if FAIL else 0)
