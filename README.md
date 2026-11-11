# step-assem-prj: auto-mating in a STEP assembly

Pick a few faces on a part, for example the four flats on a chuck key's square drive. The tool finds every place in a STEP assembly where that group of faces fits, and for each one reports:

- which part it is on, for every instance of that part;
- the matching face IDs on both sides;
- the fit: **exact**, **clearance** (with the gap in mm) or **interference**;
- the motion still allowed (the joint): fixed, prismatic, revolute, cylindrical, planar or spherical.

Supported faces are planes, cylinders, cones and spheres, including B-spline faces that are really one of these. Nothing is hard-coded per feature: a key, a pin and a taper bush all go through the same code.

`HOWTO.md` is the quick guide to the three ways of running it (browser with a part file, browser with the part picked inside the assembly, command line). `WRITEUP.md` explains what is matched and why, what breaks, and what I'd do next.

## How it works

A single face says little: a lathe has hundreds of flat faces. What makes a mate is a *group* of faces in a fixed arrangement. So the tool works in four steps:

1. **Describe** each selected face: flat, cylinder, cone or sphere; which way it points; its size; shaft or hole.
2. **Find groups** of faces in the assembly that are arranged the same way: same angles, same distances, facing the opposite way. These measurements don't change when a part is moved or turned, so no position has to be guessed.
3. **Place** the part: work out the position that puts the selected faces onto the group. The directions it can still move in give the joint.
4. **Check** on the real shapes: the faces must really touch over an area, and the part must not run into anything. Then the gap is measured: up to 0.02 mm is exact, up to 1 mm is clearance.

If the part already sits in the assembly, the tool also reports what its selected faces touch right now ("in place"), as a cross-check.

The code follows these steps one to one. Start reading at `fitfinder/match.py`.

## Install

You need Python 3.10–3.12.

```bash
git clone <this repo> && cd step-assem-prj
python -m venv .venv
source .venv/bin/activate              # Windows: .venv\Scripts\activate
pip install -r requirements.txt        # cadquery (brings OpenCascade/OCP and VTK) + numpy
```

Tested with cadquery 2.6 (OCP 7.8.1), numpy 2.2 and Python 3.12 on Ubuntu (WSL). A fresh `pip install cadquery` should work.

## Run the test cases

```bash
python tools/make_tests.py                 # builds 9 test assemblies into tests_out/
python tools/run_tests.py --render         # runs them, prints PASS/FAIL, writes out/<case>/

python tools/make_stress.py                # 10 harder cases into tests_stress/
python tools/run_tests.py --tests tests_stress --out out_stress
```

Each case stores the expected answer next to its geometry, and `run_tests.py` compares the result against it. The expected answers were written before the code was run on them.

On a Linux machine without a display, rendering needs a virtual screen: `xvfb-run -a -s "-screen 0 1600x1200x24" python tools/run_tests.py --render`.

## Run on your own files

```bash
python -m fitfinder inspect lathe.step                               # parts, instances, positions
python -m fitfinder extract lathe.step --name "Key" --out key.step   # a part of the assembly -> its own file
python -m fitfinder faces key.step                                   # table of face IDs, types, sizes
python -m fitfinder faces key.step --at 3.9,0,62                     # which face passes through this point
python -m fitfinder match --part key.step --assembly lathe.step --faces 2,4,16,18 --out out/key
```

- Face IDs are 1-based and the same as FreeCAD's `FaceN`, so faces can also be picked in FreeCAD.
- Instead of `--faces`, `--green` uses the faces coloured green (RGB 0, 255, 0) in the part file.
- `--near "Chuck" --margin 150` searches only near the named parts; `--no-render` skips the pictures.
- `--max-clearance 3` raises the loosest gap still reported as a fit from 1 mm to 3 mm, for this run. The viewer has the same setting (**Largest clearance**).
- The first run on a big assembly is slower: the parsed file is cached for the next runs (see *Speed*).
- Exit code: `0` = fits found, `1` = none, `3` = input error.

**Browser viewer:**

```bash
python -m fitfinder serve --root ~/cad     # then open http://127.0.0.1:8765
```

Open an assembly, pick the part to mate in the model tree (or load a separate part file), click its faces, then press **Find fits**. Each fit is shown with your part placed in it. The viewer runs locally; nothing leaves your machine.

## Output

Everything goes into the `--out` folder:

| file | content |
|---|---|
| `summary.txt` | one block per part with all its positions, then near-misses and warnings |
| `matched_faces.csv` | one row per (selected face, matched face) pair: the list of matched face IDs |
| `matches.json` | everything: face pairs, overlap, fit, clearance, joint, free directions, part→world 4×4 transform |
| `overview.png`, `match_<n>.png` | matched faces orange, selected faces blue, your part placed in each fit green; other parts see-through. The repo keeps `overview.png` only. |

Each match in `summary.txt` ends with how it was found: `[search]` = found by the four steps; `[in-place]` = the part already sits there in the assembly; `[in-place + search]` = both agree.

**Near-misses** (at most one per part) are places with the right shape that fail a check, with the reason: `too small (-0.8 mm)`, `collides with X`, `mirror image only`, `faces do not really overlap`.

## Results

### Test assemblies

| case | situation | found | |
|---|---|---|---|
| A_chuck_key | key; 4 square sockets, a 45° decoy, an oversize socket | 4 exact, prismatic | PASS |
| B_keyed_shaft | keyed shaft; 2 hubs, a hub with no keyway, one with a wide keyway | 2 exact | PASS |
| C_vice | bench vice jaw on its rail, 0.1 mm gaps | 1, clearance 0.2 mm | PASS |
| D_pulley_motor | D-bore pulley; 2 D-shaft motors, 1 round-shaft motor | 2, clearance 0.05 mm | PASS |
| E_clutter | key among similar but wrong blocks | none (no false positives) | PASS |
| F_pin_plate | pin; a tilted plate with exact, +0.5 and −0.03 mm holes | 4 (exact ×2, clearance, press) | PASS |
| G_ball_socket | ball stud; spherical cups | 3, spherical | PASS |
| H_taper | taper shank; sleeves, one on its side, and a steeper decoy | 2 exact, revolute | PASS |
| I_cover_bolts | bolt already in the assembly | cover, 4 holes, revolute | PASS |
| S1_nested_chucks | 2 chucks in a tilted 3-level sub-assembly | 8 exact | PASS |
| S2_chamfered | chamfered key tip and socket mouths | 3 exact | PASS |
| S3_split_bore | bore stored as two half-faces | 1 match per bore | PASS |
| S4_mirror | chiral key; one socket is its mirror image | 2 (mirror rejected) | PASS |
| S5_tolerance_edges | holes at +0.015 / +0.6 / +0.98 / +1.04 / −0.04 / −0.07 mm | exact, clearance ×2, press | PASS |
| S6_long_bed | 1.5 m bed; pocket exactly the key length, and a longer one | 2 | PASS |
| S7_many_instances | 36 bushings, random orientations | 36 | PASS |
| S8_bspline | bushings with B-spline faces | 3 | PASS |
| S9_dovetail60 | 60° dovetail rails at compound angles, 45° decoy | 2 prismatic | PASS |
| S10_pinion_seat | pinion cylinder + bottom; all pocket floors in one plane | 4 (each bore with its own floor) | PASS |

Results per case: `out/<case>/` and `out_stress/<case>/`.

### Real assembly: WM290V lathe

A 22 MB CATIA export: 79 placed parts, 50 distinct shapes, about 10,800 faces. The motor alone has 2,831 faces, the chuck's scroll plate 1,929. The STEP file isn't in this repo; the results are, in `out/WM290V_<name>/`. Each part was taken out of the lathe with `extract` and then searched for in the **whole** lathe.

The first five rows are the assignment's four scenarios, as far as the lathe has them:

| scenario | selected | found | fit, joint | time |
|---|---|---|---|---|
| **Chuck + key** | `key.step` 2,4,16,18: the square drive | the 3 chuck pinions | clearance 0.4 mm, prismatic | 17 s |
| **Keyed shaft** | `motor.step` 984,992: the 24 mm drive shaft, + 2226,2228: the sides of its 8 mm key | nothing, correctly: no hub in the lathe has a keyway | — | 6 s |
| **Pulley on motor shaft** | `bush.step` 3,8: the outer cone of the 1210 taper bush | the small V-belt pulley it sits in | exact, revolute | 7 s |
| | `bush.step` 12,13: the bush's 20 mm bore | only the 20 mm lead screw. Not the motor: its shaft is 24 mm (see below) | exact, cylindrical | 27 s |
| **Vice jaw on its slide** (the lathe has no vice; a chuck jaw in its slot is the same kind of fit) | `jaw1.step` 23,33: the jaw's sides, + 24,26: its guide slot | its own slot in the chuck body. The other two slots are refused: jaws 2 and 3 and the scroll are in the way | exact, prismatic | 36 s |
| Dovetail slide | `t05.step` (`Test00-05`) 5,13: the two 45° faces, + 14: the flat between them, with `--max-clearance 3` | the slide on it, `Test00-06`. One 45° face touches; the other has 2.8 mm of room, as for a gib strip | clearance 2.82 mm, prismatic | 9 s |
| Taper bush 1610 | `hub1610.step` 5,6: outer cone | the big V-belt pulley it sits in | exact, revolute | 4 s |
| M8 bolt | `hex.step` 241,249: shank, + 57: head underside | the headstock housing: 2 through holes, head on either side = 4 positions | clearance 0.38 mm, revolute | 8 s |
| Retaining pin | `pin.step` 16,17: shank only | its 3 holes in the chuck body | clearance 0.22 mm, cylindrical | 20 s |

Times are for the search on a 2-core cloud machine. Reading the lathe takes another 15 s the first time and under a second after that (see *Speed*).

What these runs show beyond the matches:

- **Near-misses explain the rejects.** For the key: three square features 0.8–1.8 mm too small. For the bolt: the chuck's adapter plate holes are "too small by 1.35 mm", because they are tapped M8 holes drawn at core diameter (6.65 mm).
- **The tool finds mistakes in the CAD model.** Three fits are reported only by the in-place check, because the search refuses to put the part there: it overlaps a neighbour as modelled.
  - The 1210 bush still has its 20 mm *pilot* bore, but the motor shaft through it is 24 mm. The bush was never bored out in the model.
  - Each retaining pin overlaps the chuck body by 70 mm³.
  - The V-belt is modelled 345 mm³ into the small pulley's groove (seen when the pulley's cone is selected instead of the bush's).
- **The motor is a surface model.** Its 2,831 faces don't close into a solid, so it has no "inside". A collision with it is still caught where its skin cuts through the part; that is what rejects the bush on three 20 mm cylinders inside the motor. A part lying completely inside the motor's shell would not be caught. The summary carries a `SURFACE_MODEL` warning.

To repeat a run with your own copy of the file:

```bash
python -m fitfinder extract WM290V_RedrawStart.stp --name "Key" --out key.step
python -m fitfinder match --part key.step --assembly WM290V_RedrawStart.stp --faces 2,4,16,18 --out out/WM290V_key
```

The other parts: `--name "Jaw 1 Internal"`, `--name "PILOT BORE"` (the 1210 bush), `--name "1610x38mm"`, `--name "Hex M8x25"`, `--name "Retaining Pin"`, `--name "Test00-05"`, `--name "M90S B34(ok).1/=>[0:1:1:13]"` (the motor).

## Speed

The 19 test assemblies take 0.1–2 s each. On the lathe every search takes 4–36 s. Three things keep it there:

1. **Cheapest checks first.** Steps 1–3 are arithmetic. Only the few face groups that survive them reach step 4, which works on the real 3-D shapes.
2. **A fast collision check in front of the exact one** (`meshcheck.py`). OpenCascade's exact overlap calculation (a "boolean") takes 0.5–3 s against a big casting, and most candidate positions are wrong and collide. So first the moving part is filled with points that lie at least 0.25 mm inside it, and a line through each point counts its crossings with the other part's triangles. A point inside the other part, or the other part's surface right next to a point, means a sure collision. On the lathe's parts this point test agrees with OpenCascade's own point classifier on every sampled point and runs 15 to 4000× faster. When it finds nothing, the exact boolean still decides.
3. **Each position is checked once.** A slot wall split into pieces by a recess gives one face group per combination of pieces: 1,728 groups for the chuck jaw, but only 12 positions.

The fast check is also safer. On the lathe, the boolean returned an overlap of zero for some positions where the parts clearly overlap (a chuck jaw 4.6 cm³ deep in the scroll plate, a pin through the motor's bracket). The point test catches these.

Reading the lathe takes 15 s, so the parsed assembly is cached in `~/.cache/fitfinder` (or `$FITFINDER_CACHE`) and re-read in under a second while the STEP file is unchanged. `FITFINDER_NO_CACHE=1` turns this off.

## Layout

```
fitfinder/
  match.py        the whole method, step by step (run_match): start reading here
  faces.py        step 1: face records (type, normal/axis, radius, shaft or hole)
  fingerprint.py  step 2: the measurements between two faces that don't change when a part moves
  search.py       step 2: shortlist partner faces, build matching groups
  pose.py         step 3: solve the position; the free directions give the joint
  verify.py       step 4: overlap, collision, pushing the part in as far as it goes
  meshcheck.py    step 4: the fast collision check on triangle meshes
  fit.py          exact / clearance / interference
  inplace.py      if the part is already in the assembly: what it touches now
  loader.py       read STEP (parts, copies, placements), write STEP, cache
  output.py       summary, CSV, JSON, pictures
  cli.py          command line
  config.py       every tolerance, in one place
  web/            browser viewer: server.py + static page
tools/            make_tests.py, make_stress.py (test geometry), run_tests.py (PASS/FAIL)
out/, out_stress/ results
```
