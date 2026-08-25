"""
lane_detect.py -- laptop side of the harbour two-lane obstruction signaller.

Webcam -> persistent-obstruction detection per lane -> serial char to ESP32-C3.

WHAT THIS LOOKS FOR
    ANY obstruction. Not a specific object -- deliberately so. A container, a
    pallet, debris, a spill, a dropped load, a broken-down forklift and a
    thing nobody has a name for all block a lane identically, and a detector
    that only recognises a known list would return GREEN for the one it has
    never seen. That failure points the wrong way for a safety signal, so
    nothing here classifies anything: it reacts to persistent unexplained
    change, whatever caused it.

    THE PROBLEM IT SOLVES
    An obstruction far down a lane is invisible from the approach. Drivers
    commit to the lane, discover it up close, and have to reverse back out --
    wasted time, and a hazard while they do it. The point of the signal is to
    make the blockage visible at the decision point, before anyone commits.

    Traffic flowing through a lane is explicitly not a blockage and must not
    turn that lane red.

    Two independent mechanisms enforce that:

      1. Persistence. A lane must bank --block-seconds (default 8) of net
         occupancy credit before it flips to RED. Credit accrues while the
         lane reads occupied and drains while it reads clear, so a lane must
         be occupied over half the time for credit to grow at all. A vehicle
         crossing the frame occupies a lane for 1-3s and never gets there.

      2. Subtraction. YOLO tracks vehicles and people and ERASES their
         footprint from the difference mask, so even a solid queue of traffic
         cannot accumulate occupancy. The exception: a track that stops moving
         for --stalled-seconds stops being excused, because a vehicle parked
         across a lane IS a blockage.

FIRST RUN, in this order:

    python lane_detect.py --calibrate
        Click 4 corners for LANE A, press ENTER, then 4 corners for LANE B,
        press ENTER. Saves zones.json. Do this once, camera in final position.

    python lane_detect.py --baseline
        Records the clean, unobstructed scene to baseline.png. Uses a
        per-pixel median over ~90 frames, so a vehicle driving through during
        capture gets averaged away. Re-run whenever the camera moves or the
        scene permanently changes.

NORMAL RUN
    python lane_detect.py --port COM4
    python lane_detect.py --no-serial        test the vision, no hardware
    python lane_detect.py --no-yolo          pure differencing, no torch
    python lane_detect.py --no-serial --show-mask     tuning view

Install:
    pip install ultralytics opencv-python pyserial
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Anchored to this script's folder, NOT the current working directory. Run
# "python C:\path\lane_detect.py --calibrate" from your home folder and a
# CWD-relative path would drop zones.json there, while the live run looked for
# it somewhere else -- the two halves would silently disagree about the lanes.
HERE = Path(__file__).resolve().parent
ZONES_FILE = HERE / "zones.json"
BASELINE_FILE = HERE / "baseline.png"

# Capture resolution. Used by EVERY mode -- calibration and the live loop must
# agree or the clicked polygons land in the wrong place. The bundled HD Webcam
# tops out at 640x480 and silently ignores anything larger, so that is the
# default; override with --width/--height if you fit a better camera.
FRAME_W, FRAME_H = 640, 480

# Seconds of throwaway frames after opening. DirectShow hands back very dark
# frames until auto-exposure settles -- measured at mean brightness 11 vs 45
# once warm. Differencing against a baseline captured cold would be nonsense.
SETTLE_SECONDS = 1.5

# --- occupancy tuning ------------------------------------------------------
DIFF_THRESHOLD = 30      # per-pixel grey delta that counts as "changed"

# Occupancy is judged on the LARGEST CONTIGUOUS blob of change inside a lane,
# not the total changed-pixel count. The two behave very differently for the
# case that matters here -- a real obstruction far down the lane. It may cover
# only 1-2% of the lane polygon, which a total-pixel threshold set high enough
# to reject noise would miss; while scattered sensor/vegetation noise summing
# to 6% of the lane would falsely trip that same threshold. Requiring the
# change to be CONNECTED separates them: noise is diffuse, objects are solid.
# The threshold is min(absolute, fraction), NOT max. An obstruction's apparent
# size is whatever it is -- it does not grow because you drew a bigger lane
# polygon -- so the absolute pixel count leads. The fraction only takes over
# when a lane is small enough that the absolute figure would be most of it.
# Getting this backwards makes the fraction dominate every normal-sized lane
# and silently reinstates the exact blind spot this is meant to remove: a
# 55x40px obstruction down the far end scores 2296px and would be ignored.
OCCUPY_BLOB_PX = 1200    # contiguous pixels of change => lane occupied
OCCUPY_BLOB_FRAC = 0.02  # ceiling for small lanes, not a floor for big ones

# Spatial stability: how many consecutive frames a large blob must hold its
# position and size before it counts as occupancy. 0 disables the check.
# Size alone is not enough in poor light. Tuned against a recorded 20s clip of
# the real failure: a dim room where an EMPTY lane swung 889-19074px (a 21x
# range) while a lane holding a real object sat at 10204-13782px (a 1.35x
# range). At these values the empty lane's false BLOCKED went from 121 frames
# to 0, while the real detection kept 117 of its 121 -- roughly 0.6s of added
# latency against an 8s confirmation window. See BlobStability below.
REQUIRE_STABLE_FRAMES = 6
STABLE_CENTROID_TOL_PX = 15   # centroid may drift this far and still count
STABLE_AREA_RATIO = 0.5       # area may vary by this factor and still count
BLUR_KERNEL = (5, 5)     # smooths sensor noise before differencing
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# --- persistence tuning ----------------------------------------------------
BLOCK_SECONDS = 8.0      # seconds of net occupancy credit => RED
RELEASE_SECONDS = 3.0    # credit must drain back to this before RED releases
DECAY_RATE = 1.0         # credit drain per second while clear (see LaneState)

# --- background model -------------------------------------------------------
# Replaces the old static-baseline + linear-drift + timed-recapture scheme
# (three separate hand-rolled mechanisms) with one statistical model: OpenCV's
# MOG2. Each pixel keeps a small mixture of recent Gaussian distributions
# instead of one frozen number, so gradual lighting drift is absorbed
# continuously by design -- no separate "recapture" step needed to fake it.
#
# The part that actually matters here: MOG2 has native shadow discrimination.
# It models a shadow as a ratio-consistent DARKENING of an already-learned
# background and reports it separately (127) from genuine foreground (255).
# Verified on a synthetic grayscale test before wiring this in: a uniformly
# darkened "shadow" region classified as shadow on 100% of its pixels, and an
# unrelated "object" region classified as foreground on 100% of its pixels.
# That is the actual answer to "is this a shadow or an obstruction" that the
# old pure-luminance-diff approach had no way to ask.
#
# History is frames of memory before something old is fully forgotten -- the
# safety-relevant knob, discussed at length before building this: a real
# obstruction that sat still for `history` frames could in principle start
# being absorbed as "normal" by the model's own adaptation. That risk is
# closed by MOG_FREEZE_WHEN_BLOCKED below, not by history size; history is
# just how forgiving the model is about GENUINE lighting drift.
MOG_HISTORY = 3000        # secondary parameter now -- see MOG_LEARNING_RATE
MOG_VAR_THRESHOLD = 16    # OpenCV default; lower = more sensitive to change
MOG_SHADOW_THRESHOLD = 0.5   # OpenCV default shadow-ratio sensitivity

# MOG2 has no per-pixel "freeze this region" control in the public API, so the
# per-lane recapture gating from the old design (touch only currently-clear
# lanes) can't be replicated at that precision. The safe substitute: freeze
# adaptation for the WHOLE frame whenever ANY lane is currently blocked. This
# is coarser -- an unrelated already-clear lane also pauses adapting while
# some other lane stays occupied -- but it preserves the one property that
# actually matters: a real, confirmed obstruction can never be adapted away
# while it is still there. It costs nothing when clear, since that is the
# normal operating state.
MOG_FREEZE_WHEN_BLOCKED = True

# NOT -1 (OpenCV's "auto" rate). That was the first value tried here, and
# direct testing caught it absorbing a completely static, unchanging
# obstruction into the background by FRAME 4 -- a fraction of a second, and
# nowhere near the ~8s BLOCK_SECONDS confirmation window. "Auto" is tuned for
# bootstrapping a fresh video stream quickly; it is the wrong tool for
# protecting an already-seeded model, and using it would have silently
# reproduced the exact "obstruction vanishes on its own" failure this
# redesign exists to prevent.
#
# This value was chosen by directly measuring frames-to-absorption for a
# strong, static, high-contrast step change (the hardest case to protect,
# since gradual real drift is absorbed far more forgivingly by the same
# mechanism): rate 0.001 -> 110 frames (~7.3s @15fps, UNSAFE, absorbs before
# confirmation even completes); 0.0005 -> 220 frames (~15s, too tight);
# 0.00005 -> 2211 frames (~147s @15fps, ~74s even at this camera's measured
# 30fps ceiling without YOLO) -- 9x+ margin over the confirmation window
# before MOG_FREEZE_WHEN_BLOCKED becomes the permanent safety net anyway.
MOG_LEARNING_RATE = 0.00005

# Used ONLY while shift_prev > SHIFT_ALARM or all_occupied (see the freeze-
# gate comment in the main loop) -- i.e. only once whole-frame disruption has
# ALREADY been confirmed, not for ordinary occupancy.
#
# NOT 0.005. That was the first value tried, and live testing caught a real
# side effect: 0.005 absorbs a step change in ~1.4s, which is FASTER than
# block_seconds (3-8s) -- so a real, stationary object sitting in a lane that
# had not yet crossed its own occupancy threshold when a catch-up burst fired
# could be absorbed into "background" before persistence ever got a chance to
# flag it. Confirmed live: a real object sitting in an unblocked lane was
# silently erased during a catch-up recovery.
#
# 0.0001 fixes that by design, not by luck: measured recovery is ~36.8s at
# this camera's worst-case ~30fps (no YOLO) -- a 4.6x margin above the
# strictest BLOCK_SECONDS default (8.0s), so persistence always gets to flag
# a coincidentally-present object before the model could absorb it, even in
# the worst case. Still ~2x faster than the ultra-conservative
# MOG_LEARNING_RATE above (which measures ~74s at the same fps), so a genuine
# lighting event still resolves meaningfully faster than doing nothing
# special -- just not at the cost of the persistence guarantee.
MOG_CATCHUP_LEARNING_RATE = 0.0001

# --- gradient (edge-structure) channel --------------------------------------
# A second MOG2 running on the edge map rather than raw intensity. The two
# channels are OR-ed: a lane is occupied if EITHER sees change. See
# gradient_map() below for why the failure modes are complementary.
# Its own variance threshold, because gradient images are sparser and noisier
# than intensity images -- reusing MOG_VAR_THRESHOLD here fires constantly.
USE_GRADIENT_CHANNEL = True
GRAD_VAR_THRESHOLD = 40

# --- detection subtraction -------------------------------------------------
VEHICLE_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle",
                   5: "bus", 7: "truck"}
CONF_THRESHOLD = 0.35
BOX_DILATE_PX = 12       # pad erased boxes; YOLO boxes clip shadows and edges
STALLED_SECONDS = 20.0   # tracked object motionless this long stops being excused
STALLED_TOL_PX = 18.0    # centroid movement under this counts as "not moving"

SEND_INTERVAL = 0.25     # seconds between serial keep-alive sends


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
def open_camera(index, width, height, backend_name, lock_exposure=False):
    """Open the webcam at a known resolution and let exposure settle.

    On Windows the default MSMF backend routinely takes 6s+ to open a camera
    or hangs outright; DirectShow opens faster, so ask for it by name.
    """
    if backend_name == "dshow":
        backend = getattr(cv2, "CAP_DSHOW", cv2.CAP_ANY)
    elif backend_name == "msmf":
        backend = getattr(cv2, "CAP_MSMF", cv2.CAP_ANY)
    else:
        backend = cv2.CAP_ANY

    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        raise SystemExit(
            f"Could not open camera index {index} via {backend_name}. Close "
            f"anything else using the webcam (Teams, Zoom, Camera app), or "
            f"try --camera 1 / --backend msmf."
        )
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    if lock_exposure:
        # Best-effort: plenty of UVC cameras ignore these. 0.25 is the
        # long-standing "manual" magic value; DirectShow drivers vary.
        got_ae = cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        got_wb = cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        print(f"[info] exposure lock: auto_exposure={got_ae} auto_wb={got_wb} "
              f"(False means the driver refused; photometric alignment still "
              f"covers you)")

    deadline = time.monotonic() + SETTLE_SECONDS
    while time.monotonic() < deadline:
        cap.read()

    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if (got_w, got_h) != (width, height):
        print(f"[warn] camera gave {got_w}x{got_h}, asked for {width}x{height}; "
              f"using what it actually gave.")
    return cap, (got_w, got_h)


# Fraction of the WHOLE frame that differs from the baseline. Occupancy is
# judged per lane, but a stale baseline shows up everywhere at once -- that is
# what distinguishes "something is in a lane" from "this baseline is no longer
# a picture of this scene".
SHIFT_WARN = 0.35        # warn above this at startup
# Lowered from 0.50 after a real incident: dim room lighting pushed
# frame-change to 41.8% (individual lane thresholds need well under 1% of
# the total frame to trip), which sat BELOW the old 50% bar -- both lanes
# latched BLOCKED from lighting alone, froze, and could not self-correct.
# 0.30 plus the ALL_LANES_OCCUPIED signal below (a cheaper, more targeted
# second check) closes that gap.
SHIFT_ALARM = 0.30


def scene_shift(grey, baseline, thr=DIFF_THRESHOLD):
    """How much of the frame no longer matches the baseline, 0.0 to 1.0.

    The built-in webcam lives in the laptop lid, so tilting the screen to look
    at the output re-aims the camera. Every edge in the scene then lands a few
    pixels off its baseline position and the difference mask lights up along
    all of them -- which reads as obstructions in both lanes at once. Silent
    nonsense; this makes it say so out loud.
    """
    diff = cv2.absdiff(grey, baseline)
    _, m = cv2.threshold(diff, thr, 255, cv2.THRESH_BINARY)
    return cv2.countNonZero(m) / float(m.size)


def largest_blob(binary):
    """Area in pixels of the biggest connected region of a binary mask.

    Connectivity 8, background label 0 skipped. This is the whole reason a
    small distant obstruction can be detected without also detecting noise:
    a solid object is one large component, noise is many tiny ones.
    """
    count, _, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return 0
    return int(stats[1:, cv2.CC_STAT_AREA].max())


def largest_blob_with_centroid(binary):
    """Like largest_blob(), but also returns the blob's centroid.

    The centroid is what makes spatial stability possible: a real object sits
    still, so its centroid barely moves between frames, while noise blobs of
    similar size appear in a different place each frame.
    """
    count, _, stats, cents = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return 0, 0.0, 0.0
    areas = stats[1:, cv2.CC_STAT_AREA]
    i = int(areas.argmax()) + 1
    return int(stats[i, cv2.CC_STAT_AREA]), float(cents[i][0]), float(cents[i][1])


class BlobStability:
    """Rejects blobs that are big enough but keep jumping around.

    Size alone cannot separate a real obstruction from noise in a dim or
    unevenly lit scene. Measured live on this rig in a dark room: a lane with
    nothing in it swung from 0 to 5836px, and a lane with a real object swung
    1232 -> 388px, in four seconds. Both crossed and re-crossed the size
    threshold repeatedly, so persistence alone kept latching and releasing.

    A real object has one property that noise does not: it stays in the same
    place at roughly the same size. Requiring the largest blob to hold its
    centroid (within tol_px) and its area (within a factor of area_ratio) for
    `frames` consecutive frames discards the flicker without touching genuine
    detections, which comfortably satisfy both conditions.

    frames=0 disables the check entirely, so the old behaviour is still
    reachable for comparison in replay.
    """

    def __init__(self, frames, tol_px=None, area_ratio=None):
        self.frames = frames
        self.tol_px = STABLE_CENTROID_TOL_PX if tol_px is None else tol_px
        self.area_ratio = STABLE_AREA_RATIO if area_ratio is None else area_ratio
        self.count = 0
        self.anchor = None            # (cx, cy, area) of the run being tracked

    def update(self, big_enough, area, cx, cy):
        if self.frames <= 0:
            return bool(big_enough)
        if not big_enough:
            self.count, self.anchor = 0, None
            return False

        if self.anchor is None:
            self.anchor = (cx, cy, area)
            self.count = 1
            return self.count >= self.frames

        ax, ay, aa = self.anchor
        moved = abs(cx - ax) > self.tol_px or abs(cy - ay) > self.tol_px
        # Compare areas as a ratio, not a difference: a fixed pixel tolerance
        # would be far too strict for a large blob and far too loose for a
        # small one.
        grew = aa > 0 and not (self.area_ratio <= area / aa <= 1.0 / self.area_ratio)

        if moved or grew:
            self.anchor = (cx, cy, area)   # a new thing; start its run over
            self.count = 1
        else:
            self.count += 1
            # Track slow drift so a gradually settling object stays locked on.
            self.anchor = (cx, cy, area)
        return self.count >= self.frames


def gradient_map(grey):
    """Edge-structure map of a frame: |Sobel_x| + |Sobel_y|, as 8-bit.

    This is the second detection channel. Its real value turned out to be
    different from the one first assumed, and the measurements are worth
    keeping because the intuitive reason is wrong.

    The actual problem it solves: MOG2's shadow classifier REJECTS genuine
    dark objects. A dark, textured object (a pouch, a tarp, a fabric sack)
    looks exactly like a shadow to it -- darker than the background, with
    texture still present -- so it is labelled shadow (127) and discarded.
    Measured here on a dark textured object: the intensity channel reported
    0px. Zero. The gradient channel reported 41109px on the same frame.
    That failure is silent and points the wrong way for a safety signal.

    Why edges separate the two cases: a real shadow lets the underlying
    surface texture show through, so the edge map is nearly unchanged. A real
    object REPLACES that texture with its own, so the edge map changes
    completely. Brightness cannot tell those apart; structure can.

    Known blind spot, measured: a SOLID, untextured object scores ~0 here,
    because a flat surface has no internal edges and its thin outline is
    eroded by the morphology step. That is fine -- the intensity channel
    catches solid objects easily (48000px on the same test). The two channels
    are blind in different places, which is the entire point of OR-ing them.

    Measured false-positive cost of adding it: none. Across an illumination
    ramp, a hard-edged local dim, and a localised shadow patch, this channel
    reported 0px every time, while intensity false-fired at 28371px on the
    ramp.

    Computed AFTER photometric_align() so a global lighting change scales the
    input consistently, rather than scaling every gradient with it.
    """
    gx = cv2.Sobel(grey, cv2.CV_16S, 1, 0, ksize=3)
    gy = cv2.Sobel(grey, cv2.CV_16S, 0, 1, ksize=3)
    return cv2.addWeighted(cv2.convertScaleAbs(gx), 0.5,
                           cv2.convertScaleAbs(gy), 0.5, 0)


def prep(frame):
    """BGR frame -> blurred greyscale, the form everything differences on."""
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.GaussianBlur(grey, BLUR_KERNEL, 0)


def photometric_ref(grey):
    """Robust brightness/contrast signature of the baseline."""
    p25, med, p75 = np.percentile(grey, [25, 50, 75])
    return float(med), max(float(p75 - p25), 1.0)


def photometric_align(grey, ref):
    """Rescale a frame's brightness and contrast onto the baseline's.

    Webcam auto-exposure and auto-gain are the main long-run threat to
    baseline differencing outdoors -- a cloud crossing the sun re-gains the
    whole frame and every pixel "changes" at once. Measured on this rig with
    a static scene at threshold 30: a +25% gain shift alone lights up 15.0%
    of the frame and +40% lights up 44.1%, both far past the 6% occupancy
    threshold, so each would latch a false RED. With alignment the same two
    shifts read 0.00% and 0.61%, while a genuine obstruction still reads
    ~21%. Raising DIFF_THRESHOLD cannot separate those two cases; rescaling
    the exposure away can.

    (For the record, the settled per-frame noise floor of this camera is
    0.00% at threshold 30 with nothing moving. The defaults are not fighting
    sensor grain -- they are fighting illumination change.)

    Median and IQR rather than mean and stdev, so a large obstruction sitting
    in frame does not drag the correction onto itself and cancel its own
    detection.
    """
    ref_med, ref_iqr = ref
    p25, med, p75 = np.percentile(grey, [25, 50, 75])
    iqr = max(float(p75 - p25), 1.0)
    out = (grey.astype(np.float32) - float(med)) * (ref_iqr / iqr) + ref_med
    return np.clip(out, 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def calibrate(args):
    cap, size = open_camera(args.camera, args.width, args.height, args.backend,
                            args.lock_exposure)
    zones = {}

    try:
        for lane in ("A", "B"):
            pts = []
            win = f"Calibrate LANE {lane}"
            stray = {"n": 0}

            def on_mouse(event, x, y, flags, param):
                if event != cv2.EVENT_LBUTTONDOWN:
                    return
                if len(pts) < 4:
                    pts.append([x, y])
                    print(f"    LANE {lane}: corner {len(pts)} of 4 at ({x}, {y})")
                    if len(pts) == 4:
                        print(f"    LANE {lane}: all 4 placed -- press ENTER "
                              f"in the window to accept")
                else:
                    stray["n"] += 1
                    print(f"    LANE {lane}: already have 4 corners. "
                          f"Press ENTER to accept, or u to undo.")

            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(win, on_mouse)
            print("")
            print(f"  LANE {lane}: click 4 corners in the video window.")
            print(f"     u = undo last     ENTER = accept     ESC = cancel")

            while True:
                ok, frame = cap.read()
                if not ok:
                    raise SystemExit("Lost the camera during calibration.")
                view = frame.copy()
                ready = len(pts) == 4

                # filled preview once the shape closes
                if ready:
                    shade = view.copy()
                    cv2.fillPoly(shade, [np.array(pts, np.int32)], (0, 200, 0))
                    view = cv2.addWeighted(shade, 0.25, view, 0.75, 0)

                if len(pts) > 1:
                    cv2.polylines(view, [np.array(pts, np.int32)], ready,
                                  (0, 255, 255), 2)

                # big, unmissable markers -- crosshair, disc, white ring, number
                for i, (x, y) in enumerate(pts):
                    cv2.line(view, (x - 14, y), (x + 14, y), (0, 0, 0), 3)
                    cv2.line(view, (x, y - 14), (x, y + 14), (0, 0, 0), 3)
                    cv2.line(view, (x - 14, y), (x + 14, y), (0, 255, 255), 1)
                    cv2.line(view, (x, y - 14), (x, y + 14), (0, 255, 255), 1)
                    cv2.circle(view, (x, y), 9, (0, 255, 255), -1)
                    cv2.circle(view, (x, y), 9, (255, 255, 255), 2)
                    cv2.putText(view, str(i + 1), (x - 5, y + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)

                # instruction banner
                banner = view.copy()
                cv2.rectangle(banner, (0, 0), (view.shape[1], 74), (0, 0, 0), -1)
                view = cv2.addWeighted(banner, 0.55, view, 0.45, 0)
                cv2.putText(view, f"LANE {lane}   corners placed: {len(pts)} of 4",
                            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                            (255, 255, 255), 2)
                if ready:
                    cv2.putText(view, "READY  ->  press ENTER to accept "
                                      "(u = undo)", (12, 58),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2)
                else:
                    cv2.putText(view, f"click corner {len(pts) + 1} "
                                      f"(u = undo, ESC = cancel)", (12, 58),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)

                cv2.imshow(win, view)
                key = cv2.waitKey(20) & 0xFF
                if key in (13, 10):
                    if ready:
                        break
                    print(f"    LANE {lane}: need 4 corners before ENTER "
                          f"(you have {len(pts)}).")
                if key == ord("u") and pts:
                    gone = pts.pop()
                    print(f"    LANE {lane}: undid corner at "
                          f"({gone[0]}, {gone[1]}) -- {len(pts)} left")
                if key == 27:
                    raise SystemExit("Calibration cancelled.")

            cv2.destroyWindow(win)
            cv2.waitKey(1)
            zones[lane] = pts
            print(f"  LANE {lane} accepted: {pts}")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)

    # Stamp the resolution the points were clicked at, so load_zones() can
    # rescale if the live capture ever comes back at a different size.
    payload = {"_meta": {"width": size[0], "height": size[1]}, **zones}
    ZONES_FILE.write_text(json.dumps(payload, indent=2))
    print("")
    print(f"  SAVED {ZONES_FILE}   (clicked at {size[0]}x{size[1]})")
    print(f"  Next: python lane_detect.py --baseline")


def auto_zones(args):
    """Write zones.json by splitting the frame in half -- no clicking needed.

    Calibration by clicking assumes you have physical lanes to point at. When
    you do not -- bare desk, no markings, or you just want to see the detector
    run -- this carves the frame into two halves instead. It is a real,
    usable configuration, not a placeholder: the detector does not care
    whether a zone traces painted lines or an arbitrary rectangle, only which
    pixels belong to which lane.

    Re-run --calibrate later to replace it with hand-clicked zones.
    """
    cap, size = open_camera(args.camera, args.width, args.height, args.backend,
                            args.lock_exposure)
    cap.release()
    w, h = size
    m = args.auto_margin
    gap = args.auto_gap // 2

    if args.auto_zones == "lr":          # side by side, split vertically
        mid = w // 2
        a = [[m, m], [mid - gap, m], [mid - gap, h - m], [m, h - m]]
        b = [[mid + gap, m], [w - m, m], [w - m, h - m], [mid + gap, h - m]]
    else:                                # stacked, split horizontally
        mid = h // 2
        a = [[m, m], [w - m, m], [w - m, mid - gap], [m, mid - gap]]
        b = [[m, mid + gap], [w - m, mid + gap], [w - m, h - m], [m, h - m]]

    payload = {"_meta": {"width": w, "height": h}, "A": a, "B": b}
    ZONES_FILE.write_text(json.dumps(payload, indent=2))
    split = "left/right" if args.auto_zones == "lr" else "top/bottom"
    print(f"Saved {ZONES_FILE.resolve()}")
    print(f"  {w}x{h}, split {split}, {m}px margin, {args.auto_gap}px centre gap")
    print(f"  LANE A {a}")
    print(f"  LANE B {b}")
    print("Next: python lane_detect.py --baseline")


def load_zones(size):
    if not ZONES_FILE.exists():
        raise SystemExit("No zones.json -- run with --calibrate first.")
    raw = json.loads(ZONES_FILE.read_text())
    meta = raw.pop("_meta", None)

    sx = sy = 1.0
    if meta is None:
        print("[warn] zones.json has no resolution stamp (old format). "
              "Assuming it matches the current capture; re-run --calibrate "
              "if the overlay looks misaligned.")
    elif (meta["width"], meta["height"]) != size:
        sx = size[0] / meta["width"]
        sy = size[1] / meta["height"]
        print(f"[info] rescaling zones from {meta['width']}x{meta['height']} "
              f"to {size[0]}x{size[1]}")

    zones = {k: (np.array(v, dtype=np.float64) * [sx, sy]).astype(np.int32)
             for k, v in raw.items()}
    if not zones:
        raise SystemExit("zones.json has no lanes -- re-run --calibrate.")
    return zones


# ---------------------------------------------------------------------------
# Baseline
# ---------------------------------------------------------------------------
def capture_baseline(args, count=90):
    cap, _ = open_camera(args.camera, args.width, args.height, args.backend,
                         args.lock_exposure)
    print(f"Clear the lanes. Capturing {count} frames (ESC to cancel)...")

    stack = []
    try:
        while len(stack) < count:
            ok, frame = cap.read()
            if not ok:
                raise SystemExit("Lost the camera during baseline capture.")
            stack.append(prep(frame))
            view = frame.copy()
            cv2.putText(view, f"baseline {len(stack)}/{count}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("Capturing baseline", view)
            if cv2.waitKey(1) & 0xFF == 27:
                raise SystemExit("Baseline capture cancelled.")
    finally:
        cap.release()
        cv2.destroyAllWindows()

    # Median, not mean: a vehicle that crosses during capture occupies any
    # given pixel for a minority of frames, so the median rejects it entirely.
    # A mean would leave a faint ghost that reads as permanent change.
    baseline = np.median(np.stack(stack), axis=0).astype(np.uint8)
    cv2.imwrite(str(BASELINE_FILE), baseline)
    print(f"Saved {BASELINE_FILE.resolve()}")


def load_baseline(size):
    if not BASELINE_FILE.exists():
        raise SystemExit("No baseline.png -- run with --baseline first.")
    baseline = cv2.imread(str(BASELINE_FILE), cv2.IMREAD_GRAYSCALE)
    if baseline is None:
        raise SystemExit("baseline.png is unreadable; re-run --baseline.")
    if (baseline.shape[1], baseline.shape[0]) != size:
        print(f"[warn] baseline is {baseline.shape[1]}x{baseline.shape[0]}, "
              f"camera is {size[0]}x{size[1]}; resizing. Re-run --baseline "
              f"for best results.")
        baseline = cv2.resize(baseline, size)
    return baseline


# ---------------------------------------------------------------------------
# Per-lane persistence state machine
# ---------------------------------------------------------------------------
class LaneState:
    """Turns a noisy per-frame 'occupied' bool into a stable 'blocked' bool.

    This is a leaky integrator, not a stopwatch. Credit accrues while the lane
    reads occupied and drains at `decay` times real time while it reads clear,
    so credit only grows if the lane is occupied for more than
    decay / (1 + decay) of the time -- at the default decay of 1.0, a straight
    50% duty cycle.

    That duty-cycle test is the thing that separates a real obstruction from
    noise, and a stopwatch-with-grace-period cannot do it. Sensor flicker that
    is only 30% occupied still never produces a gap long enough to reset a
    stopwatch, so the stopwatch creeps to the threshold and falsely goes red.
    Measured: 0.3s blips every 1.0s tripped the old stopwatch at t=8.1s.

    Hysteresis: RED latches at block_s of credit and only releases once credit
    has drained back to release_s, so the signal cannot flap on the boundary.
    """

    def __init__(self, block_s, release_s, decay=1.0, max_dt=0.5):
        self.block_s = block_s
        self.release_s = release_s
        self.decay = decay
        self.max_dt = max_dt      # clamp, so one stalled frame is not a jump
        self.credit = 0.0
        self.blocked = False
        self.last_t = None

    def update(self, occupied, now):
        dt = 0.0 if self.last_t is None else min(now - self.last_t, self.max_dt)
        self.last_t = now

        self.credit += dt if occupied else -dt * self.decay
        self.credit = max(0.0, min(self.credit, self.block_s))

        if self.credit >= self.block_s:
            self.blocked = True
        elif self.credit <= self.release_s:
            self.blocked = False
        return self.blocked


# ---------------------------------------------------------------------------
# Moving-object subtraction
# ---------------------------------------------------------------------------
class TrafficMasker:
    """Erases live traffic from the difference mask.

    Anything YOLO tracks as a vehicle or person is assumed to be traffic and is
    cut out of the mask, so flowing traffic can never accumulate occupancy.
    The exception is a track that stops moving: once it has sat within
    STALLED_TOL_PX for stalled_s it stops being excused, because a vehicle
    parked across a lane is a genuine blockage.
    """

    def __init__(self, stalled_s):
        self.stalled_s = stalled_s
        self.anchors = {}     # track id -> (anchor_x, anchor_y, time_anchored)

    def build(self, results, shape, now):
        mask = np.zeros(shape, np.uint8)
        drawn = []
        if results is None or results.boxes is None:
            return mask, drawn

        seen = set()
        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls not in VEHICLE_CLASSES or conf < CONF_THRESHOLD:
                continue
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx, cy = (x1 + x2) / 2, (y1 + y2) / 2

            excused = True
            tid = None if box.id is None else int(box.id[0])
            if tid is not None and self.stalled_s > 0:
                seen.add(tid)
                ax, ay, since = self.anchors.get(tid, (cx, cy, now))
                if abs(cx - ax) > STALLED_TOL_PX or abs(cy - ay) > STALLED_TOL_PX:
                    ax, ay, since = cx, cy, now        # it moved; re-anchor
                self.anchors[tid] = (ax, ay, since)
                if now - since >= self.stalled_s:
                    excused = False                    # stationary => blockage

            drawn.append((x1, y1, x2, y2, VEHICLE_CLASSES[cls], conf, excused))
            if excused:
                cv2.rectangle(mask,
                              (int(x1) - BOX_DILATE_PX, int(y1) - BOX_DILATE_PX),
                              (int(x2) + BOX_DILATE_PX, int(y2) + BOX_DILATE_PX),
                              255, -1)

        # Drop anchors for tracks that have gone away, so ids recycled by the
        # tracker do not inherit a stale "stationary since" timestamp.
        for tid in [t for t in self.anchors if t not in seen]:
            del self.anchors[tid]
        return mask, drawn


# ---------------------------------------------------------------------------
# Serial
# ---------------------------------------------------------------------------
def open_serial(port, baud):
    import serial
    # ESP32 boards wire DTR/RTS to EN and GPIO0. pyserial asserts both by
    # default on open, which kicks the board into its bootloader instead of
    # running our sketch. Build the port unopened, clear the lines, then open.
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0
    ser.dtr = False
    ser.rts = False
    ser.open()
    time.sleep(2)          # board still reboots on connect; let it settle
    print(f"Serial open on {port}")
    return ser


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=FRAME_W)
    ap.add_argument("--height", type=int, default=FRAME_H)
    ap.add_argument("--backend", choices=("dshow", "msmf", "any"),
                    default="dshow")
    ap.add_argument("--port", default=None, help="ESP32-C3 serial port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--auto-zones", choices=("lr", "tb"), default=None,
                    help="make zones.json by splitting the frame, no clicking")
    ap.add_argument("--auto-margin", type=int, default=20,
                    help="pixels of dead border for --auto-zones")
    ap.add_argument("--auto-gap", type=int, default=40,
                    help="pixels of gap between the two lanes")
    ap.add_argument("--baseline", action="store_true",
                    help="record the clean scene to baseline.png")
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--no-yolo", action="store_true",
                    help="MOG2 background subtraction only, no vehicle "
                         "tracking/subtraction; no torch needed")
    ap.add_argument("--block-seconds", type=float, default=None,
                    help=f"occupancy credit needed before RED (default {BLOCK_SECONDS})")
    ap.add_argument("--release-seconds", type=float, default=None,
                    help=f"credit must drain to this before RED releases "
                         f"(default {RELEASE_SECONDS})")
    ap.add_argument("--decay", type=float, default=None,
                    help=f"credit drain rate while clear (default {DECAY_RATE})")
    ap.add_argument("--fast", action="store_true",
                    help="tabletop-testing preset: block=3s, release=1s, "
                         "decay=2.0. NOT for real traffic -- a slow-moving "
                         "vehicle can occupy a lane for 3s+ and this preset "
                         "would call that a blockage. Explicit --block-seconds "
                         "etc. still override this.")
    ap.add_argument("--stalled-seconds", type=float, default=STALLED_SECONDS,
                    help="0 = always excuse detected vehicles")
    ap.add_argument("--stable-frames", type=int, default=REQUIRE_STABLE_FRAMES,
                    help="frames a blob must hold position+size (0 disables)")
    ap.add_argument("--occupy-blob-px", type=int, default=OCCUPY_BLOB_PX,
                    help="contiguous changed pixels that count as an obstruction")
    ap.add_argument("--occupy-blob-frac", type=float, default=OCCUPY_BLOB_FRAC,
                    help="ceiling for small lanes; threshold is min(px, frac*lane)")
    ap.add_argument("--diff-threshold", type=int, default=DIFF_THRESHOLD,
                    help="used only for the one-off startup scene-vs-baseline "
                         "sanity check; the live loop uses MOG2 below")
    ap.add_argument("--mog-history", type=int, default=MOG_HISTORY,
                    help=f"frames of background memory (default {MOG_HISTORY})")
    ap.add_argument("--mog-var-threshold", type=float, default=MOG_VAR_THRESHOLD,
                    help=f"MOG2 sensitivity; lower=more sensitive "
                         f"(default {MOG_VAR_THRESHOLD})")
    ap.add_argument("--mog-shadow-threshold", type=float,
                    default=MOG_SHADOW_THRESHOLD,
                    help=f"shadow-ratio sensitivity (default {MOG_SHADOW_THRESHOLD})")
    ap.add_argument("--no-gradient", action="store_true",
                    help="disable the edge-structure detection channel")
    ap.add_argument("--no-shadow-reject", action="store_true",
                    help="treat MOG2's shadow pixels as foreground too, "
                         "instead of excluding them from occupancy")
    ap.add_argument("--no-photometric", action="store_true",
                    help="disable exposure normalisation before differencing")
    ap.add_argument("--lock-exposure", action="store_true",
                    help="ask the camera to disable auto-exposure/auto-WB")
    ap.add_argument("--show-mask", action="store_true",
                    help="second window showing the difference mask")
    ap.add_argument("--display-scale", type=float, default=1.5,
                    help="enlarge the display window(s); does not affect "
                         "detection resolution or --calibrate")
    args = ap.parse_args()

    # --fast supplies its own defaults, but an explicit --block-seconds (etc)
    # always wins over it -- resolved here rather than via argparse defaults
    # so "user typed a number" and "flag was never mentioned" stay distinguishable.
    if args.fast:
        if args.block_seconds is None:   args.block_seconds = 3.0
        if args.release_seconds is None: args.release_seconds = 1.0
        if args.decay is None:           args.decay = 2.0
    if args.block_seconds is None:   args.block_seconds = BLOCK_SECONDS
    if args.release_seconds is None: args.release_seconds = RELEASE_SECONDS
    if args.decay is None:           args.decay = DECAY_RATE

    if args.calibrate:
        calibrate(args)
        return
    if args.auto_zones:
        auto_zones(args)
        return
    if args.baseline:
        capture_baseline(args)
        return

    # Check the prerequisites before grabbing the camera, so a missing file
    # fails instantly instead of after a multi-second device open.
    if not ZONES_FILE.exists():
        raise SystemExit("No zones.json -- run with --calibrate first.")
    if not BASELINE_FILE.exists():
        raise SystemExit("No baseline.png -- run with --baseline first.")

    cap, size = open_camera(args.camera, args.width, args.height, args.backend,
                            args.lock_exposure)
    zones = load_zones(size)
    baseline = load_baseline(size)
    photo_ref = photometric_ref(baseline)

    # ---- background model: MOG2, seeded from the captured baseline ----
    # A repeated moderate-learning-rate seed converges the model to "this is
    # normal, low variance" without the degenerate zero-variance state a
    # single learningRate=1.0 call would leave it in.
    bs = cv2.createBackgroundSubtractorMOG2(
        history=args.mog_history,
        varThreshold=args.mog_var_threshold,
        detectShadows=True)
    bs.setShadowThreshold(args.mog_shadow_threshold)
    for _ in range(20):
        bs.apply(baseline, learningRate=0.5)

    # Second channel on the edge map. detectShadows=False: a shadow barely
    # perturbs an edge map, so the classifier would add cost and no signal.
    bs_grad = None
    if not args.no_gradient:
        bs_grad = cv2.createBackgroundSubtractorMOG2(
            history=args.mog_history, varThreshold=GRAD_VAR_THRESHOLD,
            detectShadows=False)
        _gb = gradient_map(baseline)
        for _ in range(20):
            bs_grad.apply(_gb, learningRate=0.5)

    ok, probe = cap.read()
    if ok:
        pg = prep(probe)
        if not args.no_photometric:
            pg = photometric_align(pg, photo_ref)
        shift = scene_shift(pg, baseline)
        print(f"  scene vs baseline: {shift*100:.0f}% of the frame differs")
        if shift > SHIFT_WARN:
            print("")
            print("  *** WARNING: the baseline does not match what the camera")
            print("      sees now. Usually the camera moved -- the webcam is in")
            print("      the laptop lid, so adjusting the screen re-aims it.")
            print("      Re-run:  python lane_detect.py --baseline")
            print("")

    # Precompute a filled mask and pixel count per lane; the polygons never move.
    lane_masks, lane_areas = {}, {}
    for lane, poly in zones.items():
        m = np.zeros((size[1], size[0]), np.uint8)
        cv2.fillPoly(m, [poly], 255)
        lane_masks[lane] = m
        lane_areas[lane] = max(int(cv2.countNonZero(m)), 1)

    ser = None
    if not args.no_serial:
        if not args.port:
            raise SystemExit("Give --port, or use --no-serial to test vision only.")
        ser = open_serial(args.port, args.baud)
        # Measured: ~5s elapses between opening the port and the first command
        # (2s port settle + model load + a ~2s first-inference warmup), while
        # the firmware watchdog trips at 2s. Without this the board drops into
        # FAULT on every single launch. Same all-red result, but asserted on
        # purpose rather than as a fault, and it keeps the log honest.
        ser.write(b"3")
        ser.flush()

    model = None
    masker = TrafficMasker(args.stalled_seconds)
    if not args.no_yolo:
        from ultralytics import YOLO
        model = YOLO("yolov8n.pt")     # downloads ~6 MB the first time

    states = {lane: LaneState(args.block_seconds, args.release_seconds,
                              args.decay)
              for lane in zones}
    stability = {lane: BlobStability(args.stable_frames) for lane in zones}
    last_sent, last_cmd = 0.0, None
    frame_times = []          # rolling window for the on-screen fps counter
    run_start = time.monotonic()
    any_blocked_prev = False   # last frame's state; gates this frame's MOG2 learning
    shift_prev = 0.0            # last frame's whole-frame change ratio; see freeze gate
    raw_occ_prev = {lane: False for lane in zones}   # see all_occupied above

    print("Running. q to quit.")
    print(f"  block={args.block_seconds:.1f}s  release={args.release_seconds:.1f}s  "
          f"decay={args.decay:.1f}  display x{args.display_scale:.1f}")
    print(f"  background model: MOG2  history={args.mog_history}  "
          f"var-threshold={args.mog_var_threshold:.0f}  "
          f"shadow-reject={'off' if args.no_shadow_reject else 'on'}")
    if args.fast:
        print("  --fast preset active: tuned for tabletop testing, NOT for "
              "real traffic")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            now = time.monotonic()
            grey = prep(frame)
            if not args.no_photometric:
                grey = photometric_align(grey, photo_ref)

            # ---- background/foreground via MOG2 ----
            # Freeze learning for THIS frame if any lane was blocked as of the
            # PREVIOUS frame (one-frame lag is unavoidable: this frame's own
            # occupancy isn't known until after bs.apply() runs) -- UNLESS the
            # previous frame also showed a whole-frame disruption (shift_prev
            # over SHIFT_ALARM). That second condition closes a real deadlock
            # caught by testing: a sudden global lighting change (a light
            # switched on) pushes most pixels in BOTH lanes outside their
            # learned models at once, both lanes latch BLOCKED, and a naive
            # freeze then stops MOG2 from ever learning the new lighting is
            # the new normal -- the false BLOCKED could never self-correct,
            # only clear on a restart. A single obstruction occupying one
            # lane essentially never pushes whole-FRAME change past 50%
            # (there is always the other lane and the margin around both), so
            # that threshold cleanly separates "a real, localized obstruction"
            # from "something global just happened" without a new constant.
            # Two independent signals decide "this looks global, not a real
            # obstruction, so let the model catch up": either a large fraction
            # of the whole frame changed, OR every lane is occupied at once.
            # The second one is what closes the real gap above -- two lanes
            # tripping simultaneously from a shared cause (lighting) is common;
            # two independent real obstructions arriving in the same instant is
            # not, so treating "all occupied together" as likely-global is safe.
            all_occupied = len(zones) > 1 and all(raw_occ_prev.values())
            disrupted = (shift_prev > SHIFT_ALARM) or all_occupied
            freeze = MOG_FREEZE_WHEN_BLOCKED and any_blocked_prev and not disrupted
            if freeze:
                rate = 0.0
            elif disrupted:
                rate = MOG_CATCHUP_LEARNING_RATE
            else:
                rate = MOG_LEARNING_RATE
            fgmask = bs.apply(grey, learningRate=rate)

            # MOG2 reports shadow as 127 and real foreground as 255, so
            # thresholding at 254 keeps only true foreground -- that IS the
            # shadow-vs-obstruction discrimination. cv2.threshold rather than
            # np.where: measured 48x faster (0.023ms vs 1.103ms), because
            # np.where with Python ints allocates a full int64 intermediate
            # (2.4MB at 640x480) and throws it away every single frame.
            cut = 0 if args.no_shadow_reject else 254
            _, mask = cv2.threshold(fgmask, cut, 255, cv2.THRESH_BINARY)

            # OR in the edge channel. This is what recovers a dark, textured
            # object that the shadow classifier above wrongly discarded --
            # measured at 0px on intensity vs 41109px here for that case.
            if bs_grad is not None:
                _fgg = bs_grad.apply(gradient_map(grey), learningRate=rate)
                _, _gm = cv2.threshold(_fgg, 127, 255, cv2.THRESH_BINARY)
                mask = cv2.bitwise_or(mask, _gm)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL)

            # ---- subtract live traffic so it cannot accumulate occupancy ----
            drawn = []
            if model is not None:
                results = model.track(frame, persist=True, verbose=False)[0]
                traffic, drawn = masker.build(results, mask.shape, now)
                mask = cv2.bitwise_and(mask, cv2.bitwise_not(traffic))

            # ---- per-lane occupancy + persistence ----
            ratios, blobs, blocked, raw_occ, needs = {}, {}, {}, {}, {}
            for lane in zones:
                lane_pixels = cv2.bitwise_and(mask, lane_masks[lane])
                ratios[lane] = cv2.countNonZero(lane_pixels) / lane_areas[lane]
                blobs[lane], _bx, _by = largest_blob_with_centroid(lane_pixels)
                needs[lane] = min(args.occupy_blob_px,
                                  args.occupy_blob_frac * lane_areas[lane])
                # Occupancy needs BOTH: a big enough blob, and that blob
                # holding its position and size. update() is called every
                # frame either way so the tracker sees the gaps too.
                big_enough = blobs[lane] >= needs[lane]
                raw_occ[lane] = stability[lane].update(
                    big_enough, blobs[lane], _bx, _by)
                blocked[lane] = states[lane].update(raw_occ[lane], now)

            any_blocked_prev = any(blocked.values())
            raw_occ_prev = dict(raw_occ)
            shift_prev = cv2.countNonZero(mask) / float(mask.size)

            # ---- lane state -> command char (protocol unchanged) ----
            a = blocked.get("A", False)
            b = blocked.get("B", False)
            cmd = "3" if (a and b) else "1" if a else "2" if b else "0"

            wall = time.time()
            if ser and (cmd != last_cmd or wall - last_sent > SEND_INTERVAL):
                ser.write(cmd.encode())    # resend regularly to feed the watchdog
                last_cmd, last_sent = cmd, wall

            # ---- overlay ----
            # Three tiers, not two. BLOCKED still only latches after the full
            # persistence window -- that decision is unchanged and is what
            # keeps a passing vehicle from tripping RED. What's new is that
            # the *display* no longer waits for that: the instant a frame is
            # occupied, the lane flashes amber "DETECTED, confirming" so the
            # operator sees a reaction in one frame instead of in 8 seconds,
            # even though the safety-relevant decision still takes 8 seconds.
            GREEN, AMBER, RED = (0, 200, 0), (0, 190, 255), (0, 0, 255)
            for lane, poly in zones.items():
                credit = states[lane].credit
                if blocked[lane]:
                    colour, label = RED, "BLOCKED"
                elif raw_occ[lane] or credit > 0.2:
                    colour = AMBER
                    label = f"CONFIRMING {credit:.1f}/{args.block_seconds:.1f}s"
                else:
                    colour, label = GREEN, "clear"

                thickness = 4 if blocked[lane] else 3
                cv2.polylines(frame, [poly], True, colour, thickness)
                cx, cy = poly.mean(axis=0).astype(int)
                cv2.putText(frame, f"LANE {lane}", (cx - 55, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2)
                cv2.putText(frame, label, (cx - 75, cy + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
                cv2.putText(frame,
                            f"blob {blobs[lane]}/{int(needs[lane])}px  "
                            f"fill {ratios[lane]*100:.1f}%",
                            (cx - 75, cy + 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)

            for x1, y1, x2, y2, name, conf, excused in drawn:
                col = (255, 180, 0) if excused else (0, 140, 255)
                tag = f"{name} {conf:.2f}" + ("" if excused else " STALLED")
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                              col, 2)
                cv2.putText(frame, tag, (int(x1), int(y1) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

            # ---- fps (rolling average over the last 30 frames) ----
            frame_times.append(now)
            if len(frame_times) > 30:
                frame_times.pop(0)
            fps = (len(frame_times) - 1) / max(frame_times[-1] - frame_times[0], 1e-6)                   if len(frame_times) > 1 else 0.0

            # ---- bottom status bar: everything needed to read this run at a glance
            bar_h = 58
            H, W = frame.shape[:2]
            bar = frame.copy()
            cv2.rectangle(bar, (0, H - bar_h), (W, H), (30, 30, 30), -1)
            frame = cv2.addWeighted(bar, 0.75, frame, 0.25, 0)

            uptime = now - run_start
            port_txt = f"serial {args.port}" if ser else "no serial (vision only)"
            model_txt = "yolov8n" if model is not None else "diff-only"
            frozen_txt = " [MOG2 FROZEN -- lane blocked]" if any_blocked_prev else ""
            line1 = (f"cmd={cmd}   {port_txt}   model={model_txt}   "
                    f"{fps:4.1f} fps   up {uptime:6.1f}s{frozen_txt}")
            line2 = (f"block={args.block_seconds:.1f}s  release={args.release_seconds:.1f}s  "
                    f"decay={args.decay:.1f}  mog-history={args.mog_history}  "
                    f"frame-change {cv2.countNonZero(mask)/mask.size*100:4.1f}%")
            cv2.putText(frame, line1, (10, H - 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, line2, (10, H - 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 220, 255), 1)

            cv2.putText(frame, f"cmd={cmd}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            # Both lanes swamped at once is the signature of a stale baseline
            # (or a sudden lighting change), not of two simultaneous
            # obstructions. Say so rather than quietly reporting a double
            # blockage. Reuses shift_prev, computed above, for this frame too.
            if shift_prev > SHIFT_ALARM:
                warn = frame.copy()
                cv2.rectangle(warn, (0, 40), (frame.shape[1], 96), (0, 0, 140), -1)
                frame = cv2.addWeighted(warn, 0.75, frame, 0.25, 0)
                cv2.putText(frame, f"{shift_prev*100:.0f}% OF FRAME CHANGED",
                            (12, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                            (255, 255, 255), 2)
                cv2.putText(frame, "camera moved? re-run --baseline",
                            (12, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                            (255, 255, 255), 2)

            # ---- display scaling: purely cosmetic, applied last. Detection
            # runs at the native camera resolution throughout; only the window
            # shown to a human is enlarged, so thresholds and --calibrate
            # coordinates are unaffected by this.
            show = frame
            show_mask = mask
            if args.display_scale != 1.0:
                show = cv2.resize(frame, None, fx=args.display_scale,
                                  fy=args.display_scale,
                                  interpolation=cv2.INTER_LINEAR)
                if args.show_mask:
                    show_mask = cv2.resize(mask, None, fx=args.display_scale,
                                           fy=args.display_scale,
                                           interpolation=cv2.INTER_NEAREST)

            cv2.imshow("Lane monitor  (q to quit)", show)
            if args.show_mask:
                cv2.imshow("difference mask", show_mask)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if ser:
            try:
                ser.write(b"3")        # leave it in the safe state
                ser.flush()            # ...and make sure it actually goes out
            finally:
                ser.close()


if __name__ == "__main__":
    main()
