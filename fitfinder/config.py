"""All tolerances in one place (mm, degrees)."""
import math

GEOM_TOL = 1e-3            # "same point / same surface"
ANGLE_TOL = math.radians(0.5)
EXACT_GAP = 0.02           # |clearance| <= this  -> exact (coincident) fit
INTERFERENCE_MAX = 0.05    # clearance down to -this -> interference (press) fit, still coincident
CLEARANCE_MAX = 1.0        # clearance up to this     -> sliding / clearance fit
RELAX = 2.0                # the search window is wider by this, so near-misses can be explained
MIN_PAIR_OVERLAP = 0.2     # each mated face pair must really touch over >= 20 % of the smaller face
MIN_MEAN_OVERLAP = 0.5
COLLISION_VOL = 1e-2       # mm^3 of interpenetration tolerated (numerical noise)
SLIDE_STEPS = 25          # positions tried inside the range where the faces overlap (verify.slide_range)
ROT_STEPS = 12             # angles tried about a free rotation axis
INPLACE_GAP = 1.0          # in-place mode: faces within this distance count as touching
NEAR_MISS_GROUPS = 50       # loose face groups examined per part with no real fit, to explain why
MIN_AREA_RATIO = 0.02        # a mating face must be >= 2 % of the selected face's area (no CAD slivers)
MESH_DEFLECTION = 0.05     # mm, chord error of the meshes used by the fast collision pre-check
MESH_DEPTH = 0.25          # mm, the part's test points lie at least this deep inside it (> chord error + press fit)
MESH_GRID_POINTS = 20000   # grid points spread over the part's bounding box for that test
MESH_REACH = 0.1           # mm, another part's surface this close to a test point runs through the part
MESH_MIN_POINTS = 3        # this many test points inside another part = a sure collision
MESH_TEST_POINTS = 1500    # at most this many of them are tested per check (spread evenly)
