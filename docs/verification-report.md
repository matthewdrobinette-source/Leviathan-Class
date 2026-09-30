# Leviathan-class Build Specification, Revision H: verification and model report

Sources checked: `spec/leviathan-class-build-specification_RevH.pdf` (44 pages) and
`spec/leviathan-datum-model_RevH.html` (the three.js datum model, supplied as
"Revision_G_datum_model.html").

## Summary

The specification holds up very well. I recomputed every figure it derives from other
figures: dimensions, volumes, masses, densities, flotation, masking angles, counts and
complement arithmetic. **145 of 159 reproduce within tolerance.** The Annex B datum set
is complete enough to build the ship parametrically without a single hand-placed
surface. That build (Blender) agrees with the independent calculation of the hull
volume to four significant figures (4.791 billion m³ both ways).

The problems fall into four groups:

1. **The drive fairing was not carried everywhere.** When it was added (198 M m³), the
   density and hull-volume figures in §1, §20 and A.8 were updated. §2.7 and A.8's
   flotation figures were not, so they describe a hull without a fairing.
2. **Praetorian inherits Lance rules that don't fit it.** Its keys and hangar mouths
   share the same eight longitudes. Its thruster wells can't be evenly spaced. Its own
   deck grid can't carry the galleries. Its command deck is not "one level below" the
   flag plot. Its gear-leg lengths are the Lance figures.
3. **A few places where two datums claim the same ground.** Three fittings sit on the
   r 1,119 circle. The ordnance cells break through the dish. The lower stowed boom
   runs into the knuckle plating. A 45 m collar travel leaves the stowed crown inside
   the shutters. The growing decks hold 14 tiers, not 16.
4. **Arithmetic slips in Annex C.** The mount volume, machinery budget, laundry rate,
   berthing share and locker comparison are off, and the volume budget does not sum
   to 100 %.

None of these threatens the concept. Each has a small, local fix, listed below. The
Blender model applies the fixes it needs to be geometrically consistent, and records
every one (see *Modelling decisions*).

The Annex B.13 self-check ("152 volumes, 271,000 points … no clash, no envelope
violation") is not quite right. Measured along the surface normal at ≤ 4 m spacing, the
as-written allocation has three envelope violations: the ordnance cells, the lower
stowed boom and the flag plot. Pairwise, there are no hard-volume overlaps and no void
runs through a hard volume, so that part of the claim stands. With the fixes applied,
the modelled allocation passes all four tests across 5 million sample points.

## Method

- `model/lev_geom.py` encodes Annex B once: every profile curve, station and
  allocation volume. The verification script and the Blender build both import it,
  so they can't drift apart.
- `verification/verify_spec.py` recomputes each derived figure from the datums.
  Volumes come from revolving the actual (r, z) section polygons, with the fairing
  integrated across its 90° arc. Flotation uses bisection on the displaced volume.
  Cores use sphere-segment integrals.
- The clash check samples every volume at ≤ 8 m (≤ 4 m for thin ones). It tests them
  against each other and against the plated hull envelope: 10 m under dorsal faces,
  12 m on Praetorian's bowl shell, 4 m elsewhere, **measured along the surface
  normal**, plus the Lance bowl and well keep-outs.
- Full numeric results: `verification/results.md` (table) and `verification/results.json`.

---

## Significant findings

### 1. Flotation figures leave out the drive fairing (§2.7, A.8)
The displacements of 4.87 and 5.62 billion m³ reproduce exactly, but only without the
fairing. With the fairing (198 M m³ of volume, 0.315 M m² of waterplane) they are
**5.06 and 5.82 billion m³**. The immersed densities become 0.950 and 0.825 t/m³. The
drafts become:

| Medium | Flooded (stated → with fairing) | Screen hardened (stated → with fairing) |
|---|---|---|
| Fresh water | 622 → 602 m | 538 → 519 m |
| Seawater | 609 → 589 m | 524 → 506 m |
| Dense brine | 517 → 501 m | 433 → 417 m |

§2.7 also uses 0.96 t/m³ for the hull, the pre-fairing figure (§1, §20 and A.8 give
0.93). Its "7.4 MPa" for an emplaced hull can't be reproduced: the mated weight over
the footprint is 5.3 MPa, which is §2.7's own figure one sentence later.

### 2. The Lance flotation sentence contradicts itself (§2.7)
With the waterline at 9° N, pole down, the southern band (30°–50° S) is 110–160 m
under water. That band holds the gear bays, the ramps and the southern batteries. The
sentence says the band stays dry, then puts the gear under. *Suggested:* "…leaves every
hangar mouth dry and puts the southern band, the gear, the entry face and the shield
face under."

### 3. Praetorian's keys and hangar mouths share the same eight longitudes (§3.1, B.6)
All keys are clocked to the web longitudes. Praetorian's sixteen hangar mouths are "on
the spur longitudes", which are the same eight. Each 50 m key channel would run through
two 60 m mouths, and each spur would enter the bowl through a key. A Lance doesn't have
this problem, because its keys are 22.5° from its gallery mouths. *Suggested:* split
Praetorian's keys above 22° N (the model does this), or clock them to 22.5° + 45n.

### 4. Praetorian's thruster wells can't be evenly spaced (§3.3, B.6)
Forty-eight wells on an even 7.5° pitch put the two nearest each spur longitude 3.75°
off it. Those two fall inside the 60 m mouth at 7.6° N, which spans ±4.98° of
longitude, so they would share a longitude with a mouth, which §3.3 forbids.
*Suggested:* group the wells six to each 35° gap between mouths (≈ 6.0° pitch). The
model does this.

### 5. Praetorian's own deck grid can't carry the galleries (B.6, B.3)
B.6 anchors each core's grid on its own equator. For a Lance (equator z −290) the
galleries sit 54 m and 120 m above it, which is 9 and 20 levels. For Praetorian
(equator z −282) they sit 46 m and 112 m above it, which is not a whole number of levels
(2 m remainder). The centre can't move, because the south cap must stay flush at
z −630. *Suggested:* anchor Praetorian's grid on the hull grid, with a level at z −284.

### 6. The flag command deck is not below the flag plot (B.9, §15)
The plot spans n = 42–46 (z +16 to +40). The command deck at z +28 is n = 44, in the
middle of that span, not "one level below". z +28 is also 63° N on the core, inside
the northern cap that B.6 gives to the drive cluster (above 60° N). *Suggested:* put
the command deck at z +10 (n = 41, 57.5° N).

### 7. Three fittings claim the dorsal r 1,119 circle (§6, §7, B.8)
234 medium mounts on a 30 m pitch use 7,020 m of the 7,031 m circumference. The
8 dorsal shield rings (60 m apertures) and 16 dorsal sensors are also specified at
two-thirds radius. The one-third circle has the same problem: 117 mounts use 3,510 m of
3,515 m, and 16 more sensors are specified there. The HTML datum model draws them all
on top of each other. *Model:* emitters at r 1,175 (inside their generator hall), and
sensors at r 1,060 and r 505.

### 8. The ordnance cells break through the dish (B.8, B.10)
The cells run from r 1,050 with their floor at n = −40 (z −476). At r 1,050 the ventral
surface is z −470, and it doesn't drop below −476 until r ≈ 1,072. So the inboard 22 m
of the bottom level lies outside the hull, opening the magazines onto the cavity. That
is the most heavily protected volume on the ship, facing the bearing it is most exposed
on. *Model:* the cells start at r 1,080 (floor clear of the 4 m grade from r ≈ 1,078).

### 9. The growing decks' z range holds 14 tiers, not 16 (B.3, B.11)
n = −25 to −18 and z −386 to −344 span 42 m. That is seven pitches, room for fourteen
3 m tiers. Sixteen tiers, or "eight primary bays", need 48 m. C.12's ordnance-block
volume confirms that a range "n = a to b" means deck planes a to b. At 14 tiers the
growing area is 7.21 M m², short of §12's 8.25 M m². *Model:* the blocks run to
z −338 (n = −25 to −17).

### 10. The r = 1,500 rule is broken by volumes the spec itself places (§14, B.11)
"Nothing but weapons … outboard of r = 1,500 between z −142 and −532" is contradicted
by three volumes:
- The growing decks, which run to r 1,550.
- The rim shield generator halls (r 1,500–1,674).
- The ventral shield generator halls (r 1,500–1,630).

The halls belong there, "set in the gaps between battery rows", so the rule's wording
needs to change. The growing decks should move in to r 1,300–1,500.

### 11. A 45 m collar travel leaves the stowed crown inside the shutters (§2.2, B.9, D.8)
A crown at +100 lowered 45 m stands at +55. The dorsal surface across the well mouth is
at +53.8 (inner) to +51.1 (outer), and the iris shutters carry the full 10 m grade
under it. A crown at +55 is 1–4 m proud of the surface and 11–14 m into the shutters.
The well can't be deepened to fix this, because Praetorian's bowl shell reaches r 250
at z −20. *Model:* the collar body is 50 m tall (+50 to +100 extended, −10 to +40
stowed) on 60 m of travel.

---

## Minor findings

| Ref | Finding | Suggested resolution |
|---|---|---|
| §2.2, B.2 | The collar-well argument uses the bare core radius. Against the bowl shell (r_c + 14) the core "reaches r 250" at z −20, not −40, so the margin under the well floor is 10 m, not 30. §2.2 says 65 m of depth; B.2 says 64 (63.8). | State the margin against the bowl shell. |
| B.2 | The knuckle is specified with a horizontal tangent at the crown, but the crown slopes 5.4° at r 1,050, so the dish has a crease there. | Accept it (the model keeps it), or tilt the ellipse. |
| §3.3, D.8 | "Gear legs of 50 to 105 m" stand a 210 m sphere on its pole. Praetorian needs ≈ 174 m and ≈ 81 m. | Give Praetorian's legs separately. |
| §3.3 | The Lance well mouth falls at 49.2° S (crown at z −449.0), not 49.6°. | Trivial. |
| B.3, B.7 | The mouth sill is at z −575 in B.7 but at n = −57 (z −578) in B.3. | Put the sill on the grid. |
| B.8, D.2 | No rim shield-emitter height is given. The rim rings at 135° and 225° fall on the emitter boundary, and the 180° ring falls on the emitter face (D.2 counts five on the battery rim). | *Model:* z −218, between battery rows; the 180° ring on the emitter face at a segment boundary. Please confirm. |
| B.8 | Emitter segments are "about 169 m". The fairing's aft boundary is 2,845 m long, which gives 178 m a segment. | Use 178 m. |
| B.8 | "Forty a segment" (640) and "three a segment, 48 in all" imply 16 segments. Everywhere else a segment is one of eight. | Say eighty and six, or define a half-segment. |
| B.8, C.12 | Heavy ready magazines (40 × 40 × 30 m) sit where the 40 m mount pitch has shrunk to about 36.6 m. With 40 m tangential they overlap. | Orient the 30 m side tangentially. |
| B.7 | The lower boom stows at r 1,130, where the near-vertical knuckle's 4 m plating reaches r 1,133.8. The B.13 check misses this because it measures the grade vertically. | *Model:* stowage at r 1,135–1,175. |
| B.9, B.3 | The flag plot's outer wall (r 250) is the collar well's moulded wall, leaving no room for its 4 m plate. If n = 46 is usable, it fits under the cap only inboard of r ≈ 219. | *Model:* plot at r 210–246. |
| §14, B.3, B.9 | The "citadel mid-plane" at z −290 is 100 m below the hub's geometric mid-plane (z −190 at r 470). The combat bridge sits 144 m above the cavity crown and 339 m below the dorsal face. | Say it aligns with Praetorian's equator, or raise it. |
| §7 | The hub cap is "the highest point in either collar state", but the extended collar crown (+100) stands 20 m above it. | "Highest point with the collar stowed." |
| B.2, B.13, B.11 | B.2 says seven of eight segments are identical (plus two aft: 9). B.13 says six. The aft block spans half of the 135° and 225° segments, which B.11 treats differently, so five are identical. | Five identical, one mirrored pair, one stern segment. |
| B.7 | "Toward the bow" has no meaning on the 0° and 180° spines, and reverses between port and starboard. | State a rotational sense. The model stows upper booms toward decreasing θ. |
| B.3, B.12 | B.3 reads the gallery z as a floor, B.12 as a centre (±12 m). B.12's citadel and handling layers overlap the gallery bands. | Pick one. The model follows B.12. |
| D.3, §3.3 | The designation face "is uppermost when a core is landed", but cores land on their southern cap. | "The face turned to the ground on descent." |
| §13.3 | "4.8 billion tonnes … twelve powerplants": mated there are 21 (4 + 8 + 9). | Twenty-one, or 4.5 Bt. |
| C.3 | 40,300 m³ of lockers is "larger than the citadel". The combat bridge hall alone is 518,000 m³. | Drop the comparison. |
| C.3 | Berthing is "a third of one percent" of deck. 263,000 of 886 M m² is 0.03 %. | Three hundredths of one percent. |
| C.6 | 1,193 t of laundry a week implies 10.9 kg a person. The stated rates (5.4 and 3.2 kg) give 351–592 t. | Recompute. |
| C.12 | The mount stack as stated sums to about 102 M m³ (heavy 78, medium 20, close-in 4). 53 M m³ is roughly the total without barbettes. | Recompute, and the 2.6 % share with it. |
| C.12 | The ventral outer band is 278 mounts in B.8 (30 m pitch at r 1,331) but 261 in C.12. | 278. |
| C.14 | Shares sum to 99.6 % (4,804 of 4,830 M m³). Reactor and auxiliary halls alone are 275 M m³ of the 290 allowed for machinery. With plausible generator halls it is about 395. | Rebalance. |

| §1, B.8 | "Overall length, over drive fairing, 3,536.9 m" is to the fairing's structural boundary. The emitter face stands 20 m proud of it (B.8), so the ship measures 3,556.9 m end to end. The render measures 3,559 m. | State both, or say the length excludes the emitter standoff. |

Checks that pass include:
- Every principal dimension and the dorsal cap geometry.
- The fixed-cap / collar-well construction.
- The dish masking angles.
- The fairing lune (0.315 M m²).
- Core stations and hangar-deck latitudes (every deck lands on z −236 / −170).
- The launch mouth and boom-root datums on the knuckle.
- Battery counts (197/591, 117, 234, 278).
- All personnel, air group and task force arithmetic (109,592; 49,812; 183,340; 2,640).
- Growing area (8.25 M m²).
- Plating mass (0.71 Bt dorsal).
- The hull density (0.93 t/m³ on the A.8 method).
- Core flotation (244 m, 404 m).
- The Annex C fixture, messing, recreation, medical and ventilation figures.

## Rendered geometry measured against the spec

`model/annotate_drawings.py` measures the hull silhouette in the orthographic renders.
It samples lines clear of every light, so bloom can't widen the silhouette, and compares
each measurement with the datums:

| Measurement | Expected | Measured from pixels |
|---|---:|---:|
| Beam view at z −420: rim to emitter face (3,536.9 + 20 m standoff) | 3,556.9 m | 3,558.8 m |
| Beam view at y −350: collar crown to ventral rim plane | 730.0 m | 731.2 m |
| Plan view: diameter fitted to four chords | 3,356.9 m | 3,359.7 m |
| Plan view along x = +300: bow rim to emitter face | 3,496.6 m | 3,496.0 m |

All four agree within 3 m, about two pixels at 1.6–1.9 m per pixel.
`verification/test_blend_controls.py` separately checks that every control on the
.blend drives what it should. It covers collar travel, iris leaves, instance counts,
release order, de-stack offsets, screen states, mouth shutters, booms and the section
(22 checks, all passing).

## The HTML datum model

- **Revision labels.** The file is named *Revision G*, titled *Revision H*, and its
  toggle reads "Rev F as written / Rev H as specified", keyed internally as `G`.
- **The drawn fittings lag the allocation data in the same file.**
  - Heavy battery: it draws 198 a row (594), not 197 (591).
  - Medium battery: the ventral outer band is drawn at r 1,119 on the knuckle, not
    at r 1,331 with 278 mounts. The legend says "702 medium" (B.8 gives 746).
  - Boom recesses: drawn at r 1,078 and 1,130, not at the Rev H roots
    (r 1,108 / z −520 and r 1,128 / z −596).
- **Fittings drawn on top of one another.**
  - Rim shield emitters at z −315, on the middle battery row.
  - Dorsal emitters and sensors on the battery circles.
  - Praetorian's key channels across its mouths.
  - Evenly spaced thruster wells overlapping the mouths.
- **The in-page clash check.**
  - It offsets the plating grade vertically (`bottomZ(r) + 4`), which is wrong on the
    near-vertical knuckle.
  - It samples cell centres at up to 40 m, too coarse to catch the ordnance-cell
    breach (6 m) or the boom intrusion (3.4 m).

The HTML's Rev H allocation data itself is faithful to the PDF, and the Blender model
reuses it.

---

## Modelling decisions

Where the specification leaves a choice open, or two datums conflict, the Blender
model does the following. All of it is in `model/lev_geom.py` and `model/*.py`.

| Item | Decision |
|---|---|
| Ordnance cells | From r 1,080 (spec: 1,050). |
| Lower stowed boom | r 1,135–1,175 (spec: 1,130–1,170); upper matched. |
| Growing decks | z −386 to −338, sixteen 3 m tiers (spec range: to −344). |
| Flag plot | r 210–246 (spec: 210–250). |
| Mouth head transfer | Not in Rev H. A 24 m passage at the lower gallery joins each outward spur to its two lift trunks (the spur ends 100 m short of them). |
| Bridge collar | 50 m ring body, crown +100 extended, 60 m travel to +40 stowed under 10 m iris shutters. The glazing band is vertical at mid-radius (r 350), where B.9's 137 m pitch gives 100 m bays and 37 m piers. A sloped skirt runs out to r 448. Sixteen lift piers stand inside the well. |
| Iris shutters | Sixteen leaves, 10 m thick, following the dorsal curve. They drop 16 m and withdraw 212 m radially under the plate. |
| Dorsal fittings | Shield emitters at r 1,175; two-thirds sensors at r 1,060; one-third sensors at r 505. |
| Rim fittings | Shield emitters and rim sensors at z −218, between battery rows. Emitters for 135° and 225° at 133.6° / 226.4°; the 180° emitter on the drive-emitter face. |
| Praetorian | Keys split to 24°–60° N, clear of its mouths. Thruster wells grouped six per gap between mouths. Hangar decks as eight radial bays into an annular hangar clear of its 100 m containment. |
| Lances | Gallery through-corridors (60 × 20 m) run the full chord at 15° N and 35° N, with the mouths at both ends. Reactor hall at the centre. |
| Core fit-out | Thruster wells, drive nozzles (with engine bells), key channels, dog sockets, gear bays, ramps, southern batteries, sensors, shield rings, repulsor arrays, Rothana hazard framing, designations painted on the southern cap. |
| Bowls | Eight tapered keys, locking dogs at the lip, and a crown repulsor array in each. Visible when the core has left. |
| Point defence | 1,238 mounts on a 120 m grid across the dorsal face, rim and ventral annulus. They give way to any larger fitting, opening, marking or light (spec: "about 1,280"). |
| Cavity defence | 640 mounts on the nine well lips, two knuckle rings and five crown rings. |
| Drive emitter | Sixteen segments standing 20 m proud across the full section. Dallorian heat tint (bronze → straw → blue-grey) with glowing throat slots. |
| Cavity screen | Four states: off; attenuating at z −630.6; hardened at z −640; drawn inboard 12 m off the dish. |
| Booms | Sixteen lattice cantilevers, 36 m square, stowed tangentially. A deployed set telescopes in three sections (40/34/28 m) to 900 m below the rim plane. |
| Markings (D.3) | "LEVIATHAN" at two-thirds radius forward, division numeral "I" aft, class badge on the fixed cap. Sector numerals 1–8 on the rim (sector 5 on the fairing crown) and in the cavity at each boom station. Mouths numbered 1–16 clockwise from the bow, with approach chevrons. Wells lettered P and L1–L8. Core designations on the southern caps. |
| Lighting (D.4) | Position lights (white forward, red over the drive arc), a collar beacon, amber cavity floods, approach ladders coded left and right, green boom-station lights, and four datum lights on the dish rim. All are extinguished in the Action state. |
| Materials (D.1, D.2) | Kuat plate: matte mid-grey with a faint blue cast, B.4 frame-grid seams, unblended repair plates, pitting. Fairing crown darkened within 200 m of the emitter face. Alusteel bowls and wells. Duranium keys and dogs. Rothana hazard stripes on the cores. Refractory ceramic south caps, scorched at the pole. |
