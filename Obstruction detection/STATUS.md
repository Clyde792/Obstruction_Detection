# Harbour Two-Lane Obstruction Signaller — Status

A webcam watches two lanes. When a lane is blocked by an **obstruction** (not by
passing traffic), a command goes over USB serial to an ESP32-C3, which drives
red/green LEDs so drivers can see the blockage **before** they commit to the lane.

Deliberately **class-agnostic**: it never tries to identify *what* the object is.
A detector that only recognises a known list returns GREEN for the object it has
never seen — that fails in the dangerous direction.

---

## DONE

### Hardware
- **ESP32-C3 Super Mini** — header pins soldered, verified via the firmware's own
  pin self-test: all 8 GPIOs healthy, no solder bridges.
- **Serial protocol verified on real hardware** — all four states accepted,
  watchdog fires (`FAULT serial silent -> all red`), recovers (`link up`).
- Board enumerates as **COM8**, VID:PID `303A:1001` (Espressif native USB).
  Note: the C3's USB re-enumerates on every reset, so the COM port briefly
  disappears when the board reboots.

### Firmware — `firmware/src/lane_leds_esp32.ino`
Built with PlatformIO (installed inside `.venv`, not system-wide).

| Position | Lane | Red | Green |
|---|---|---|---|
| 1 | A | GPIO 0 | GPIO 1 |
| 2 | A | GPIO 3 | GPIO 4 |
| 3 | B | GPIO 5 | GPIO 6 |
| 4 | B | GPIO 7 | GPIO 10 |

Avoided: GPIO 2/8/9 (strapping), 18/19 (USB D-/D+), 20/21 (UART).
Those unused pins are the safe ones to practise soldering on.

**Serial protocol** — `'0'` both clear, `'1'` A blocked, `'2'` B blocked,
`'3'` both blocked. Resent every 250 ms as a watchdog keep-alive.
**Fail-safe** — no valid command for 2000 ms → all red.
**Diagnostics** — `'T'` pin self-test, `'A'`–`'H'` drive one pin, `'X'` all off,
`'?'` pin map. Diagnostic mode suspends the watchdog, auto-exits after 120 s.

### Software
| File | What it is |
|---|---|
| `lane_detect.py` | CLI detector (OpenCV windows) |
| `dashboard.py` + `dashboard_ui.html` | Web dashboard — live video, telemetry, operator controls, in-browser lane calibration |
| `led_test.py` | Hardware diagnostics over serial — works with **no LEDs attached** |
| `replay.py` | Record camera clips and replay them deterministically |
| `icons.json` | Inline SVG icons (Lucide, ISC licence) — **required by the dashboard** |

### Detection pipeline (in order)
1. **Photometric alignment** — normalises each frame's brightness/contrast to the
   baseline. Measured: without it a +40% exposure shift produced 44% false change;
   with it, 0.6%.
2. **MOG2 background subtraction** on intensity, with shadow rejection.
3. **Gradient channel** — a second MOG2 on the edge map, OR-ed in.
4. **Largest contiguous blob** per lane (not total changed pixels — noise is
   diffuse, objects are solid).
5. **Spatial stability** — the blob must hold position and size for N frames.
6. **Leaky integrator persistence** — credit accrues while occupied, drains while
   clear. Must exceed a threshold to latch RED, drain to release. This is what
   stops passing traffic triggering.
7. **YOLO traffic subtraction** (optional) — erases tracked vehicles/people;
   a track that stops moving for 20 s stops being excused.

### Tuned constants (`lane_detect.py`)
```
BLOCK_SECONDS = 8.0          RELEASE_SECONDS = 3.0      DECAY_RATE = 1.0
OCCUPY_BLOB_PX = 1200        OCCUPY_BLOB_FRAC = 0.02
REQUIRE_STABLE_FRAMES = 6    STABLE_CENTROID_TOL_PX = 15  STABLE_AREA_RATIO = 0.5
SHIFT_WARN = 0.35            SHIFT_ALARM = 0.30
MOG_LEARNING_RATE = 0.00005  MOG_CATCHUP_LEARNING_RATE = 0.0001
USE_GRADIENT_CHANNEL = True  GRAD_VAR_THRESHOLD = 40
```

### Bugs found and fixed (all verified with tests)
- **Stopwatch persistence** falsely triggered on intermittent noise → replaced with
  a leaky integrator requiring >50% duty cycle.
- **Blob threshold formula inverted** (`max` instead of `min`) — silently reinstated
  the blind spot for small distant objects.
- **Sudden-light deadlock** — both lanes latched, the model froze, and it could
  never self-correct. Fixed by exempting confirmed global disruption from the
  freeze, plus a catch-up learning rate.
- **Moderate-lighting deadlock** — lane thresholds trip on <1% of frame but the
  escape hatch needed 50%. Added an `all_occupied` signal, lowered `SHIFT_ALARM`
  to 0.30.
- **Catch-up rate too fast** (0.005) absorbed a real object in ~1.4 s, faster than
  persistence could flag it → slowed to 0.0001 (~37 s, a 4.6× margin).
- **Calibration canvas** sat at its default 300×150 px instead of filling the video,
  so clicks outside that corner did nothing.
- **Paths were CWD-relative** — running from another folder wrote `zones.json`
  somewhere the live run never looked.
- **`np.where` → `cv2.threshold`** — 48× faster; fps went 15 → 30.

---

## PENDING

### 1. Hardware blocker — LEDs not yet bought
Everything upstream and downstream is verified. This is the only thing stopping a
real end-to-end test.

**Order:** 10 × 5mm red diffused, 10 × 5mm green diffused (high-brightness),
plus a 1/4W resistor assortment including 47Ω / 68Ω / 100Ω / 220Ω.

- Start with **220Ω on red**, **100Ω on green**.
- Existing 820Ω resistors give only ~1.6 mA — they work but are dim.
- If the green LEDs are 3.0–3.4 V Vf, headroom on 3.3 V is only ~0.3 V; drop to
  68Ω or 47Ω. If still too dim, drive green from the 5 V pin via an NPN transistor.
- **Mount on breadboard — no soldering needed.** Only the C3's header pins needed
  solder, and that's done.

Once wired: `led_test.py --port COM8 --walk` lights each position in turn.

### 2. Validation clips not recorded
`replay.py` exists but there are **no clips**. The one clip that tuned
`REQUIRE_STABLE_FRAMES` was lost, so those values are **effectively unvalidated**.

Record these (baseline first, then don't touch the laptop):
```
lane_detect.py --baseline          # lanes empty, current lighting
replay.py record empty       --seconds 20 --note "control"
replay.py record textured-B  --seconds 20 --note "dark textured object in B"
replay.py record solid-A     --seconds 20 --note "smooth hard object in A"
replay.py record passthrough --seconds 20 --note "hand through, nothing left"
replay.py record light-change --seconds 20 --note "light toggled mid-clip"
```
Then `replay.py run <name>` and `replay.py run <name> --no-gradient` to compare.

### 3. Gradient channel only synthetically validated
Measured on synthetic frames:
- Dark **textured** object: intensity **0 px**, gradient **41,109 px** — MOG2's
  shadow classifier was *rejecting a real object*. Likely explains a live case
  where a visible object read CLEAR.
- Solid **untextured** object: gradient **0 px** (known blind spot — intensity
  covers it at 48,000 px).
- All three lighting scenarios: gradient **0 px** — no measured false-positive cost.

`GRAD_VAR_THRESHOLD = 40` has **never been tested on real footage.**

### 4. Environment / lighting
The room was too dim and unevenly lit for reliable detection. Under good lighting
this pipeline detected a screwdriver cleanly at 8.1 s with zero cross-lane noise.
**Better lighting will fix more than any further code change.**
Also: `--lock-exposure` exists but isn't being used — auto-exposure turns a local
lighting change into a global frame change.

### 5. Small pending items
- **`FAULT_BLINK = 0`** in firmware — should be `1`. A dead laptop currently looks
  *identical* to a commanded "both blocked". One-line change.
- **No firmware echo** — the dashboard shows *commanded* state, never *confirmed*.
  Nothing proves the LEDs match the decision.
- **Event log is in-memory only** (300 entries, lost on restart) — should be JSONL.
- **Dashboard icons never actually rendered** until `icons.json` was added to the
  repo — worth confirming visually on the next run.
- **Calibration canvas fix and modal layout fix** were applied but **never verified
  live**.

---

## SETTING UP ON A NEW MACHINE

```bat
git clone https://github.com/Clyde792/Obstruction_Detection.git
cd Obstruction_Detection\"Obstruction detection"
python -m venv .venv
.venv\Scripts\python.exe -m pip install ultralytics opencv-python pyserial platformio
```

`zones.json` and `baseline.png` are **not** in the repo — they're specific to
wherever the camera sits. Regenerate:
```bat
.venv\Scripts\python.exe lane_detect.py --calibrate
.venv\Scripts\python.exe lane_detect.py --baseline
```

`yolov8n.pt` downloads automatically on first run (~6.5 MB).

### Running
```bat
.venv\Scripts\python.exe dashboard.py --no-serial --fast     :: dashboard, no hardware
.venv\Scripts\python.exe dashboard.py --port COM8            :: with the ESP32
.venv\Scripts\python.exe lane_detect.py --no-serial --show-mask   :: CLI + mask view
.venv\Scripts\python.exe led_test.py --list                  :: find the board
.venv\Scripts\python.exe led_test.py --port COM8 --selftest  :: check the 8 GPIOs
```
Dashboard runs at http://127.0.0.1:8000

### Flashing firmware
```bat
cd firmware
..\.venv\Scripts\platformio.exe run -t upload --upload-port COM8
```
Upload can fail on the first attempt because the C3's USB re-enumerates — just retry.

---

## KNOWN LIMITATIONS

- **Low contrast** — an object the same brightness as its background is hard to see.
  The gradient channel helps for *textured* objects, not smooth ones.
- **Local/uneven lighting** — photometric alignment corrects *global* exposure only.
  A hard shadow across part of a lane can still read as an obstruction.
- **A lane already falsely BLOCKED cannot self-correct** — the freeze gate that
  protects a real obstruction also prevents recovery. Needs the dashboard's
  "Unstick" button, or a human.
- **Laptop-lid webcam** — adjusting the screen re-aims the camera and invalidates
  the baseline. A rigid mount or external USB camera would remove a whole class of
  failures.
- **Greyscale only** — colour is invisible to the pipeline; two different colours at
  the same brightness look identical.
