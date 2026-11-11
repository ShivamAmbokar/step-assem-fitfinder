# step-assem-prj: auto-mating in a STEP assembly

Select a few faces on a part, and the tool finds every place in a STEP assembly where those faces fit: which part, the matching face IDs, exact or clearance fit, and how the part can still move.
NOTE: Writeup and Outputs files are in this repository.
## Install

You will need Python 3.12+

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Put your STEP files in one folder, for example `~/cad`.

## 1. Browser, with the part in its own file (i.e., separate assembly and part files)

```bash
python -m fitfinder serve --root ~/cad       # then open http://127.0.0.1:8765
```

1. In **Setup**, choose the assembly file, choose **Separate STEP file** and the part file, then press **Load**.
2. In the **Part** tab, click the faces to mate. They turn blue.
3. Press **Find fits**.
4. In **Results**, click a position to see your part placed there in green.

## 2. Browser, with the part inside the assembly (as in the case of the lathe and key)

```bash
python -m fitfinder serve --root ~/cad       # then open http://127.0.0.1:8765
```

1. In **Setup**, choose the assembly file, choose **Pick it in the assembly**, then press **Load**.
2. Select the part in the **Model tree** (or click it in the view), then press **Set as part to mate** (right pane).
3. Click its mating faces in the view. They turn blue.
4. Press **Find fits**, then click a position in **Results**.

Optional settings sit next to **Find fits**:

- **Largest clearance (mm):** the loosest gap still reported as a fit. The default is 1 mm.

## 3. Command line

```bash
python -m fitfinder inspect lathe.step                               # list the parts in the assembly
python -m fitfinder extract lathe.step --name "Key" --out key.step   # only if the part is inside the assembly
python -m fitfinder faces key.step                                   # find the face IDs
python -m fitfinder match --part key.step --assembly lathe.step --faces 2,4,16,18 --out out/key
```

The results go to `out/key/`. Start with `summary.txt`.

Options for `match`:

- `--green` instead of `--faces`: use the faces coloured green in the part file. (used for testing)
- `--max-clearance 3`: report fits with up to 3 mm of clearance (the default is 1 mm).
- `--near "Chuck"`: search only near the named part.
- `--no-render`: skip the pictures. On Linux without a display, put `xvfb-run -a` in front of the command to get them.

Face IDs start at 1 and are the same as FreeCAD's `FaceN`.

## Layout

```
README.md         this file
WRITEUP.docx      the write-up
OUTPUTS.docx      the results/outputs, as screenshots of the browser viewer
requirements.txt  Python packages

fitfinder/        the program
  match.py          the whole method, step by step: start reading here
  faces.py          step 1: describe each face (type, direction, size, shaft or hole)
  fingerprint.py    step 2: angles and distances between faces
  search.py         step 2: find groups of faces arranged like the selection
  pose.py           step 3: work out where the part goes
  verify.py         step 4: check overlap and collisions on the real shapes
  meshcheck.py      step 4: the fast collision check
  fit.py            exact / clearance / press fit
  inplace.py        if the part is already in the assembly: what it touches now
  loader.py         read and write STEP files
  output.py         summary, CSV, JSON and pictures
  cli.py            the command line
  config.py         every tolerance, in one place
  web/              the browser viewer

tools/            test assemblies: python tools/make_tests.py && python tools/run_tests.py
                  (the harder ones: tools/make_stress.py, then run_tests.py --tests tests_stress)
out/              results for the 9 test assemblies and the WM290V lathe runs
out_stress/       results for the 10 harder test cases
```
