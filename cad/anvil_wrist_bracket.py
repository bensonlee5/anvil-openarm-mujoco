"""Anvil OpenARM 2.0 wrist support bracket — placement wrapper.

The bracket geometry itself is the user-authored CAD in
anvil_openarm2_wrist_bracket_source.step (an assembly of two lap-jointed arm
plates with Ø3 pivot lugs 80 mm apart, an integral Ø10 x 10 spacer under the
strap-side lug, and a strap ending in a two-bolt Ø3 foot). This wrapper only
PLACES that part into the sim's LEFT link5 (forearm) frame and re-exports
it; edit the source STEP in CAD, not here.

The bracket is rigid to the FOREARM: the foot bolts into the J6 motor case
(link5 structure) and the far lug pivots at a forearm standoff, while the
strap-side lug is an outboard bearing seat ON the J6 axis — the gimbal shaft
rotates within it, so the bracket must not follow link6.

Placement (derived from the part's mate features, in mm; link6 sits at
link5 z = -120.5):
  - strap-side lug bore (CAD x=-5.2, y=-57.25, axis z) -> the J6 axis at
    link5 (0, y, -120.5), with the integral spacer's inboard end landing
    exactly on the gimbal hub face at y = 37.5
  - arm direction (CAD +y) -> link5 +z, so the far lug (80 mm away) sits at
    link5 z = -40.5 beside the forearm, where the forearm standoff meets it
  - foot bolt axes (CAD x) -> link5 x, the two bolts straddling the J6 motor
    case at link5 z = -110.5/-130.5, pointing inboard

That is the proper rotation Rot(z,180)*Rot(x,90) plus translation
(-5.2, 27.13, -63.25). MuJoCo loads the exported STL with scale 0.001 for
the left arm and a y-negated scale for the right arm.

Validation targets:
  - 3 solids (arm plate x2, spacer+strap+foot), volumes preserved
  - AABB approx x [-14.0, 11.0], y [7.5, 51.5], z [-135.5, -35.5]
"""

from pathlib import Path

from build123d import Compound, Pos, Rot, import_step

SOURCE = Path(__file__).parent / "anvil_openarm2_wrist_bracket_source.step"

# Strap-side lug bore centre in the source CAD frame (mm).
LUG_B = (-5.2, -57.25)
# Gimbal hub face the integral spacer lands on (y, mm; same value in link5
# and link6 frames); the spacer's inboard end is at CAD z = 10.37.
HUB_FACE_Y = 37.5
SPACER_INBOARD_CAD_Z = 10.37
# The link6 body (and the J6 axis) sits at link5 z = -120.5 mm.
LINK6_Z_IN_LINK5 = -120.5


def gen_step():
    part = import_step(str(SOURCE))
    solids = part.solids()
    # Rot(z,180)*Rot(x,90) maps CAD (x, y, z) -> (-x, z, y); the translation
    # then puts the lug bore on the J6 axis and the spacer on the hub face.
    loc = (
        Pos(
            LUG_B[0],
            HUB_FACE_Y - SPACER_INBOARD_CAD_Z,
            -LUG_B[1] + LINK6_Z_IN_LINK5,
        )
        * Rot(0, 0, 180)
        * Rot(90, 0, 0)
    )
    placed = Compound(children=[loc * solid for solid in solids])
    placed.label = "anvil_wrist_bracket_left"
    return placed
