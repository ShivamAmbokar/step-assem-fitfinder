"""Fit class from the clearances c of the matched face pairs (mm):
    opposed planes c = s + s'   plane parallel to an axis c = h + h'   round faces c = 2 (r_bore - r_shaft)
    any c < -EXACT_GAP -> interference;  max |c| <= EXACT_GAP -> exact;  max c <= CLEARANCE_MAX -> clearance
"""
from __future__ import annotations

from . import config as C


def classify(values: list):
    """Fit class and the deciding clearance (mm) from the clearances of all matched face pairs.
    The tightest pair decides a press fit, the loosest pair a clearance fit."""
    vals = [v for v in values if v is not None]
    if not vals:
        return "exact", 0.0
    lo, hi = min(vals), max(vals)
    if lo < -C.EXACT_GAP:
        return ("interference", lo) if lo >= -C.INTERFERENCE_MAX - 1e-6 else ("too small", lo)
    if hi <= C.EXACT_GAP + 1e-6:
        return "exact", hi
    if hi <= C.CLEARANCE_MAX + 1e-6:
        return "clearance", hi
    return "too loose", hi


def behaviour(fit: str, joint: str) -> str:
    """Plain words for a fit and its joint, e.g. 'sliding fit' or 'coincident, can turn'."""
    moving = joint not in ("fixed",)
    if fit in ("exact", "interference"):
        if fit == "interference":
            return "press fit (coincident)"
        verb = {"prismatic": "slide", "revolute": "turn", "cylindrical": "slide and turn",
                "planar": "slide in the plane", "spherical": "swivel"}.get(joint, "move")
        return f"coincident, can {verb}" if moving else "coincident, fully located"
    if fit == "clearance":
        if joint in ("prismatic", "cylindrical", "planar"):
            return "sliding fit"
        if joint in ("revolute", "spherical"):
            return "running fit"
        return "clearance fit"
    return fit
