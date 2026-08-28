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
- Board enumerates with VID:PID `303A:1001` (Espressif native USB). It came up
  as **COM8** originally and as **COM3** on the current machine — the C3's USB
  re-enumerates on every reset, so the port both disappears briefly on reboot
  **and can change number**. Never hard-code it; run `led_test.py --list` first.

- **LEDs wired and verified end-to-end.** All 8 on breadboard, resistor soldered
  to each LED's long leg (anode) toward the GPIO, cathodes to a common GND rail.
  All four protocol states drive the correct pairs; watchdog fault and recovery
  both confirmed with LEDs attached.
- **220Ω on red *and* green.** Only 220Ω was available; it sits inside the
  150–330Ω range the firmware header recommends, and green was bright enough as
  wired. If a future green LED is dim, put a second 220Ω in parallel (=110Ω)
  rather than buying new values.

> **Gotcha: `--selftest` reports `STUCK LOW - shorted to GND?` for any pin that
> has an LED attached.** The test drives the pin against a weak internal pullup,
> which an LED + 220Ω to GND overwhelms. This is a false positive, not a fault —
> `--selftest` is only meaningful with **no LEDs connected**. Cost ~30 min of
> chasing a nonexistent solder bridge.

> **Gotcha: "all reds on when idle" is correct behaviour, not a wiring fault.**
> It is the 2000 ms watchdog fail-safe latching after `led_test.py` exits and
> serial goes quiet. Likewise, diagnostic mode (`--pin`/`--walk`) turns *every*
> other pin off, so reds going dark during a walk is expected.

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
**Fail-safe** — no valid command for 2000 ms → all red, **blinking at 400 ms**
(`FAULT_BLINK 1`, flashed and verified) so a dead laptop is visually distinct
from a commanded `'3'`. Greens stay off either way.
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

### Safety alerts (2026-08-26) — dashboard.py only, not lane_detect.py
Implements the two notification behaviours from the original design doc that
weren't built yet: the core "alert supervisory personnel" requirement, and the
"additional feature" alarm for a worker standing on a blocked walkway.

- **Supervisor alert.** Fires the moment a lane transitions to BLOCKED, stays
  open (banner + `since` timer) until the lane clears — a standing notification,
  not a one-off ping, matching "notify them of the situation to clear the
  obstruction." One chime on new onset, silent while it holds.
- **Person-on-blocked-lane alarm.** Requires `--no-yolo` to be **off** — there
  is no other signal in this pipeline that tells a person apart from an
  obstruction. Uses each YOLO person box's **bottom-centre** (feet), not its
  centroid — a standing person's box is tall, so the centroid sits over the
  torso, not where they're actually standing. Debounced both ways
  (`PERSON_ALARM_CONFIRM_S=0.4` / `RELEASE_S=0.3`) against single-frame YOLO
  jitter; drops immediately, no debounce, the instant the lane itself clears.
  Repeating tone while active, flashing banner.
- Both alarms are **software-only** — beeps via the browser's Web Audio API,
  not a physical siren. No buzzer has been bought; wiring one would need a
  free GPIO, and the C3 Super Mini's pins are already fully committed to the
  8 LEDs. Pending hardware decision, same shape as the LED purchase was.
- `alerts.jsonl` — durable, append-only, gitignored (site-specific runtime
  data, like `baseline.png`/`zones.json`). Survives a restart, unlike
  `self.events` (in-memory, 300-entry ring buffer, still lost on restart —
  see the small pending items below). One JSON object per line:
  `{ts, event, lane, text}`, events `supervisor_alert` / `resolved` /
  `person_alarm`.
- Verified: `/api/state` → `alerts.supervisor`/`alerts.person` populate
  correctly on a real BLOCKED transition; `alerts.jsonl` written correctly;
  both banners and the flash animation confirmed via direct DOM inspection
  (person-alarm path forced synthetically — no person detector was in frame
  during this test, so this specific path is UI-verified, not camera-verified).

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

### Validation clips — RECORDED AND SWEPT (2026-08-26)
Three clips at `clips/`, all sharing one camera position, zones and baseline:
`empty` (control), `object-A` (dark object sitting in lane A), `passthrough`
(hand sweeping through lane A, nothing left behind).

**The shipped constants pass.** At the defaults (`REQUIRE_STABLE_FRAMES = 6`,
photometric on, gradient on):

| clip | lane A | lane B | verdict |
|---|---|---|---|
| `empty` | 0% latched | 0% | correct |
| `passthrough` | 0% latched | 0% | correct — hand does not trigger |
| `object-A` | latched at **15.0 s** | 0% | correct |

`REQUIRE_STABLE_FRAMES` sweep (photometric on): 6 → clean, 4 → clean and
latches at 11.6 s, **2 → `passthrough` falsely latches lane B (7%, t=18.75s)**.
So 4 is the floor and 6 is the safe default. Do not go below 4.

### The gradient channel is doing ALL the work — real-footage proof
Re-run of `object-A` with the intensity channel alone:

| | blob min/med/max | latched |
|---|---|---|
| gradient **on** | 430 / **1035** / 1402 | 15.0 s |
| gradient **off** | 0 / **0** / 25 | never |

The object is **invisible to the intensity channel** — median blob 0 px. MOG2's
shadow classifier rejects it outright; the gradient channel carries the entire
detection. False-positive cost measured at zero (`empty` is 0 px either way,
`passthrough` latches under neither). This reproduces on real footage the exact
failure the synthetic tests predicted, and confirms `USE_GRADIENT_CHANNEL = True`
and `GRAD_VAR_THRESHOLD = 40` are load-bearing, not optional.

### What actually made this work: framing and object size, not constants
Two earlier clip sets failed completely and produced *misleading* tuning
conclusions. Both were data problems, not pipeline problems:

- **Camera moved between baseline and clips.** Messaging someone mid-session
  tilted the laptop lid. Clips read 29–35% frame-change against the baseline
  and empty lanes latched. `record_session.bat` now runs calibrate → baseline →
  all three clips in one uninterrupted pass to make this impossible.
- **Test object too small.** A ~2% -of-lane object produced a 495 px median blob
  against a 489 px threshold — flickering right on the bar, 0% surviving the
  stability gate. It also made photometric alignment *look* harmful (empty-lane
  median blob 1228 px). With a larger object and correct framing, the empty lane
  reads **0 px min/med/max with photometric on** — photometric alignment costs
  nothing and gives a slightly stronger signal. **Leave it on.**

Diagnostic worth keeping: compare clip frames to `baseline.png` directly. If raw
absdiff inside a lane is ~0 px but the pipeline reports a large blob, the fault
is upstream of detection — a stale baseline or a moved camera.

Healthy frame-change against a matched baseline is **0.2–1.5%**. Anything near
30% means the baseline does not match the clip; re-record, do not re-tune.

### Live end-to-end run on real hardware (2026-08-26)
Camera → detection → serial → LEDs, confirmed working as one chain for the first
time. Driven through the dashboard's `/api/state` and `/api/action`.

| stage | measured |
|---|---|
| both lanes clear | `cmd=0`, blob 0 px both lanes, frame-change **0.0%** |
| dark object in lane A | latched `cmd=1`, blob ~1500–1800 px vs need 366, held 40 s without absorption |
| object removed | released to `cmd=0`, blob 0 px, frame-change 0.0% |

The freeze gate holds a latched lane against absorption — 40 s with no decay.
`RELEASE_SECONDS = 3.0` is **still not stopwatch-verified**: the release completed
between polls, so only the settled state was captured, not the transition.

### Zone polygons must sit INSIDE the paper — this caused a false positive
Lane B latched RED with nothing in it. Cause was not the detector: only **2 of 4
corners** of each lane polygon were on the paper, the rest sat on bare dark wood.
Dark, textured surfaces are the noisiest thing in frame and MOG2 finds real
structure in them every frame.

Shrinking each polygon toward its centroid until all four corners landed on the
paper (lane B needed **f=0.50**) took the noise floor from 2.6–3.6% frame-change
to **0.0%**, blob 0 px in both lanes. Detect the paper by Otsu-thresholding the
baseline, keep the largest component, erode ~15 px for margin.

Note the threshold is `OCCUPY_BLOB_FRAC` × lane area, so shrinking a lane also
shrinks its threshold — lane B went from need=460 px to need=116 px, i.e. it
became the *more* sensitive lane. Redraw lanes larger by hand once framing allows.

### Measured blind spot: pale smooth object on pale background
A white tape roll on white paper, live:

| | tape roll (pale, smooth) | scissors (dark, textured) |
|---|---|---|
| blob vs need=366 | 450–950, swinging 2× | **1500–1800, steady** |
| spatial stability | flickering | **True in 54/57 samples** |
| credit | stalled ~5.0/8.0 | **pinned 8.0/8.0** |
| after 30 s | **absorbed to 0 px** | still held |
| outcome | never latched | latched, correct |

The gradient channel picked up only the tape's circular rim, not its interior —
exactly the "helps for textured objects, not smooth ones" limitation, now with
real numbers rather than synthetic frames.

**Absorption beats a stalled lane.** If credit never reaches the threshold the
lane never latches, so the freeze gate never engages, so the background model
keeps learning until the object *is* the background. Measured at ~30 s — thinner
than the 4.6× margin the catch-up-rate note above implies. A lane stuck at
partial credit is therefore not merely "slow to trigger", it is on a timer to
silently give up.

**Operational rule:** to reset the model with an object already in a lane, use
`reset_model` (restores the saved clean `baseline.png`), never `recapture_baseline`
— the latter would absorb the object into the reference permanently.

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

### 1. Environment / lighting
The room was too dim and unevenly lit for reliable detection. Under good lighting
this pipeline detected a screwdriver cleanly at 8.1 s with zero cross-lane noise.
**Better lighting will fix more than any further code change.**

Confirmed again 2026-08-26: in a dim room with the breadboard and lit LEDs inside
the frame, an *empty* lane read 26.7% fill while a real object read 33.6% — signal
and noise essentially indistinguishable. Fixing the light, moving the electronics
out of shot, and filling the frame with the paper dropped the empty lane to 0.0%.
**Keep the LEDs out of the camera's view** — they are lit by the detector's own
output, so leaving them in frame closes a feedback loop.
Also: `--lock-exposure` exists but isn't being used — auto-exposure turns a local
lighting change into a global frame change.

### 2. Small pending items
- **No firmware echo** — the dashboard shows *commanded* state, never *confirmed*.
  Nothing proves the LEDs match the decision.
- **Event log is in-memory only** (300 entries, lost on restart) — should be JSONL.
- **`FEED LOST` badge stays visible while the feed is working.** Confirmed live:
  `/stream.mjpg` decoding fine at 640x480, `img.complete` true, yet the badge is
  `display:inline; opacity:1`. It is not hidden on recovery. Cosmetic, but a
  permanent false warning on an operator display trains people to ignore warnings.
- **Calibration canvas fix and modal layout fix** were applied but **never verified
  live** — the dashboard was driven via the HTTP API this session, not by clicking.
- **Dashboard icons confirmed rendering** (21 SVGs in the DOM) — `icons.json` works.

---

## SETTING UP ON A NEW MACHINE

```bat
git clone https://github.com/Clyde792/Obstruction_Detection.git
cd Obstruction_Detection\"Obstruction detection"
python -m venv .venv
.venv\Scripts\python.exe -m pip install ultralytics opencv-python pyserial platformio
```

`zones.json`, `baseline.png` and `clips/` are **not** in the repo — they're
specific to wherever the camera sits. Regenerate everything in one uninterrupted
pass with `record_session.bat` (calibrate → baseline → 3 clips), or by hand:
```bat
.venv\Scripts\python.exe lane_detect.py --calibrate
.venv\Scripts\python.exe lane_detect.py --baseline
```

`yolov8n.pt` downloads automatically on first run (~6.5 MB).

**Verified working on Python 3.14 / OpenCV 5** (2026-08-25). cp314 wheels exist
for everything; pip resolves `opencv-python` to **5.0.0.93**, a major-version
jump from the OpenCV 4 this was written against. The APIs the pipeline uses
(`createBackgroundSubtractorMOG2`, `threshold`, `findContours`,
`connectedComponentsWithStats`, `Sobel`, `VideoCapture`) all still work — but
OpenCV 5 is otherwise untested here, so pin to `opencv-python<5` if odd
behaviour appears.

### Running
```bat
.venv\Scripts\python.exe dashboard.py --fast                 :: dashboard, no hardware (omit --port; there is no --no-serial flag)
.venv\Scripts\python.exe dashboard.py --port COM3             :: with the ESP32
.venv\Scripts\python.exe dashboard.py --port COM3 --bind 0.0.0.0   :: viewable from other devices on the LAN, see below
.venv\Scripts\python.exe lane_detect.py --no-serial --show-mask   :: CLI + mask view
.venv\Scripts\python.exe replay.py list                      :: recorded clips
.venv\Scripts\python.exe replay.py run object-A             :: replay one deterministically
.venv\Scripts\python.exe led_test.py --list                  :: find the board
.venv\Scripts\python.exe led_test.py --port COM3 --selftest  :: check the 8 GPIOs, NO LEDs attached
.venv\Scripts\python.exe led_test.py --port COM3             :: walk all 4 protocol states
.venv\Scripts\python.exe led_test.py --port COM3 --walk      :: light each pin in turn
```
Dashboard runs at http://127.0.0.1:8000

**Viewing from a second device (phone, another laptop) on the same Wi-Fi:**
run with `--bind 0.0.0.0`, then browse to `http://<laptop's LAN IP>:8000` from
the other device. Find the IP with `ipconfig` (look for IPv4 under the active
Wi-Fi adapter). The first connection may prompt an "Allow this app through
Windows Firewall" dialog — accept it, or the second device's connection just
hangs. `127.0.0.1` only ever works on the laptop itself; `0.0.0.0` means
"listen on every network interface," not a browsable address.

### Flashing firmware
```bat
cd firmware
..\.venv\Scripts\platformio.exe run -t upload --upload-port COM3
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
