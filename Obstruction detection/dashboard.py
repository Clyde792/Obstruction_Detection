"""
dashboard.py -- web dashboard for the harbour two-lane obstruction signaller.

Replaces the OpenCV imshow windows with a browser UI: live MJPEG video, live
per-lane telemetry, operator action buttons, in-browser lane calibration, and
an event log. Runs on Python's stdlib http.server -- no Flask, no new deps.

    python dashboard.py                     vision only, no hardware
    python dashboard.py --port COM8         drive the ESP32 too
    python dashboard.py --http-port 8080    serve somewhere else

Then open http://127.0.0.1:8000 .

The detection pipeline is the same one lane_detect.py runs, imported from it
rather than duplicated, so both entry points always agree about how detection
actually works. This file owns presentation, control, and the HTTP plumbing.
"""

import argparse
import collections
import json
import queue
import threading
import time
from pathlib import Path

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import lane_detect as ld

HERE = Path(__file__).resolve().parent
UI_FILE = HERE / "dashboard_ui.html"
ICONS_FILE = HERE / "icons.json"
SNAPSHOT_DIR = HERE / "snapshots"
ALERT_LOG = HERE / "alerts.jsonl"

# ---- safety alarm: person standing on a lane that is currently BLOCKED ----
# Requires --port... no, requires YOLO (--no-yolo disables it, since it has
# no other way to tell a person from an obstruction). Debounced both ways so
# a single dropped/jittery YOLO frame doesn't chatter the alarm.
PERSON_ALARM_CONFIRM_S = 0.4   # sustained overlap before the alarm latches
PERSON_ALARM_RELEASE_S = 0.3   # sustained clear before the alarm drops

# Streaming is decoupled from detection: the detector runs as fast as it can,
# the MJPEG stream samples the newest finished frame. A slow browser therefore
# drops frames instead of throttling the safety-relevant detection loop.
# OPTIMIZE and PROGRESSIVE are explicitly OFF. Measured on a real frame:
# quality 75 encodes in 1.05ms; OPTIMIZE=1 costs 3.10ms to save 4% of bytes,
# and PROGRESSIVE=1 costs 5.33ms AND is actively wrong for MJPEG because the
# browser cannot paint a progressive part until it is complete.
ENC_PARAMS = [cv2.IMWRITE_JPEG_QUALITY, 75,
              cv2.IMWRITE_JPEG_OPTIMIZE, 0,
              cv2.IMWRITE_JPEG_PROGRESSIVE, 0]

# OpenCV's thread pool buys nothing at 640x480 (morph measured 0.450ms at 1
# thread vs 0.462ms at 16) but does oversubscribe against torch's pool and
# cause scheduler thrash. Give the cores to YOLO instead.
cv2.setNumThreads(1)


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------
class Detector(threading.Thread):
    """Owns the camera and runs the CV pipeline in one background thread.

    Everything the UI needs is published through two guarded slots: the latest
    encoded JPEG (for the video stream) and the latest telemetry dict (for the
    JSON endpoint). Control actions arrive on a queue and are applied at the
    top of a frame, so no HTTP thread ever mutates pipeline state mid-frame.
    """

    daemon = True

    def __init__(self, args):
        super().__init__(name="detector")
        self.args = args
        self.stop_evt = threading.Event()
        self.actions = queue.Queue()

        self._lock = threading.Lock()          # guards _jpeg/_seq/_state
        self._frame_ready = threading.Condition(self._lock)
        self._jpeg = None
        self._seq = 0
        self._state = {"status": "starting"}

        self.events = collections.deque(maxlen=300)
        self._events_lock = threading.Lock()

        # Live-tunable parameters. Anything here can be changed from the UI
        # without a restart; anything not here is a restart-level decision.
        self.p = {
            "block_seconds": args.block_seconds,
            "release_seconds": args.release_seconds,
            "decay": args.decay,
            "blob_px": args.occupy_blob_px,
            "blob_frac": ld.OCCUPY_BLOB_FRAC,
            "var_threshold": ld.MOG_VAR_THRESHOLD,
            "shadow_threshold": ld.MOG_SHADOW_THRESHOLD,
            "shadow_reject": True,
            "gradient": ld.USE_GRADIENT_CHANNEL,
            "stable_frames": ld.REQUIRE_STABLE_FRAMES,
            "use_yolo": not args.no_yolo,
            "photometric": True,
            "show_mask": False,
        }
        self.override = {"A": "auto", "B": "auto"}
        # A green override asserts "this lane is safe" against the detector's
        # judgement -- the single most dangerous thing an operator can do here.
        # It therefore expires by itself; red overrides never do, because red
        # is the fail-safe direction and letting one lapse would be worse.
        self.override_until = {}
        self.GREEN_OVERRIDE_SECONDS = 300.0

        # Runtime handles, created inside run() on the detector thread.
        self.cap = None
        self.ser = None
        self.bs = None
        self.bs_grad = None
        self.model = None
        self.masker = None
        self.zones = {}
        self.lane_masks = {}
        self.lane_areas = {}
        self.states = {}
        self.baseline = None
        self.photo_ref = None
        self.size = (ld.FRAME_W, ld.FRAME_H)

        # Safety alerts -- see the block after the lane loop in _loop().
        self.supervisor_alert = {}     # lane -> {"since": monotonic, "text": str}
        self.person_since = {}         # lane -> monotonic overlap started, or None
        self.person_clear_since = {}   # lane -> monotonic overlap stopped, or None
        self.person_alarm_active = {}  # lane -> bool, latched after debounce

        # Baseline recapture runs across successive frames rather than blocking
        # the loop for ~6s, which would starve the serial watchdog keep-alive.
        self._baseline_frames = None
        self._relearn_frames = 0

    # -- event log ---------------------------------------------------------
    def log(self, text, level="info"):
        with self._events_lock:
            self.events.appendleft({"t": time.time(), "text": text, "level": level})

    def event_list(self):
        with self._events_lock:
            return list(self.events)[:80]

    # -- public accessors --------------------------------------------------
    def latest_jpeg(self, last_seq):
        """Block until a frame newer than last_seq exists, then return it."""
        with self._frame_ready:
            if not self._frame_ready.wait_for(
                    lambda: self._jpeg is not None and self._seq != last_seq,
                    timeout=5.0):
                return self._jpeg, self._seq
            return self._jpeg, self._seq

    def snapshot_state(self):
        with self._lock:
            return dict(self._state)

    def submit(self, action, params=None):
        self.actions.put((action, params or {}))

    # -- setup -------------------------------------------------------------
    def _seed_model(self):
        """(Re)create MOG2 and prime it from the stored baseline image."""
        self.bs = cv2.createBackgroundSubtractorMOG2(
            history=ld.MOG_HISTORY,
            varThreshold=self.p["var_threshold"],
            detectShadows=True)
        self.bs.setShadowThreshold(self.p["shadow_threshold"])
        for _ in range(20):
            self.bs.apply(self.baseline, learningRate=0.5)

        # Edge-structure channel; see gradient_map() in lane_detect.py.
        self.bs_grad = cv2.createBackgroundSubtractorMOG2(
            history=ld.MOG_HISTORY, varThreshold=ld.GRAD_VAR_THRESHOLD,
            detectShadows=False)
        gb = ld.gradient_map(self.baseline)
        for _ in range(20):
            self.bs_grad.apply(gb, learningRate=0.5)

    def _load_zones(self):
        self.zones = ld.load_zones(self.size)
        self.lane_masks, self.lane_areas = {}, {}
        for lane, poly in self.zones.items():
            m = np.zeros((self.size[1], self.size[0]), np.uint8)
            cv2.fillPoly(m, [poly], 255)
            self.lane_masks[lane] = m
            self.lane_areas[lane] = max(int(cv2.countNonZero(m)), 1)
        self.states = {
            lane: ld.LaneState(self.p["block_seconds"], self.p["release_seconds"],
                               self.p["decay"])
            for lane in self.zones}
        self.stability = {lane: ld.BlobStability(self.p["stable_frames"])
                          for lane in self.zones}
        for lane in self.zones:
            self.override.setdefault(lane, "auto")

    def _need(self, lane):
        return min(self.p["blob_px"], self.p["blob_frac"] * self.lane_areas[lane])

    # -- serial ------------------------------------------------------------
    def _serial_connect(self, port):
        self._serial_disconnect()
        try:
            self.ser = ld.open_serial(port, self.args.baud)
            self.ser.write(b"3")
            self.ser.flush()
            self.args.port = port
            self.log(f"serial connected on {port}", "ok")
        except Exception as exc:
            self.ser = None
            self.log(f"serial connect failed: {exc}", "error")

    def _serial_disconnect(self):
        if self.ser is not None:
            try:
                self.ser.write(b"3")
                self.ser.flush()
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.log("serial disconnected", "warn")

    # -- actions -----------------------------------------------------------
    def _handle_actions(self, frame, grey):
        now = time.monotonic()
        for lane, until in list(self.override_until.items()):
            if now >= until:
                self.override[lane] = "auto"
                self.override_until.pop(lane, None)
                self.log(f"LANE {lane} force-clear expired -> auto", "warn")

        while True:
            try:
                action, params = self.actions.get_nowait()
            except queue.Empty:
                return

            if action == "recapture_baseline":
                self._baseline_frames = []
                self.log("baseline recapture started -- keep lanes clear", "warn")

            elif action == "reset_model":
                self._seed_model()
                for st in self.states.values():
                    st.credit, st.blocked, st.last_t = 0.0, False, None
                self.log("background model reset from saved baseline", "ok")

            elif action == "relearn":
                self._relearn_frames = 30
                self.log("relearn burst -- absorbing current scene as normal", "warn")

            elif action == "clear_lane":
                lane = params.get("lane")
                if lane in self.states:
                    st = self.states[lane]
                    st.credit, st.blocked, st.last_t = 0.0, False, None
                    self._relearn_frames = 30
                    self.log(f"LANE {lane} force-cleared by operator", "warn")

            elif action == "override":
                lane, mode = params.get("lane"), params.get("mode", "auto")
                if lane in self.override and mode in ("auto", "red", "green"):
                    self.override[lane] = mode
                    if mode == "green":
                        self.override_until[lane] = (time.monotonic()
                                                     + self.GREEN_OVERRIDE_SECONDS)
                        self.log(f"LANE {lane} FORCE-CLEAR by operator -- "
                                 f"expires in "
                                 f"{int(self.GREEN_OVERRIDE_SECONDS//60)} min",
                                 "error")
                    else:
                        self.override_until.pop(lane, None)
                        self.log(f"LANE {lane} override -> {mode}",
                                 "warn" if mode != "auto" else "ok")

            elif action == "set_param":
                name, val = params.get("name"), params.get("value")
                if name in self.p:
                    try:
                        self.p[name] = type(self.p[name])(val)
                    except (TypeError, ValueError):
                        continue
                    if name == "stable_frames":
                        for tr in self.stability.values():
                            tr.frames = int(val)
                    elif name in ("block_seconds", "release_seconds", "decay"):
                        for st in self.states.values():
                            st.block_s = self.p["block_seconds"]
                            st.release_s = self.p["release_seconds"]
                            st.decay = self.p["decay"]
                    elif name == "var_threshold" and self.bs is not None:
                        self.bs.setVarThreshold(float(val))
                    elif name == "shadow_threshold" and self.bs is not None:
                        self.bs.setShadowThreshold(float(val))
                    self.log(f"{name} = {self.p[name]}")

            elif action == "toggle":
                name = params.get("name")
                if name in self.p and isinstance(self.p[name], bool):
                    self.p[name] = not self.p[name]
                    if name == "use_yolo" and self.p[name] and self.model is None:
                        self._load_yolo()
                    self.log(f"{name} -> {'on' if self.p[name] else 'off'}")

            elif action == "snapshot":
                self._save_snapshot(frame, "manual")

            elif action == "serial_connect":
                self._serial_connect(params.get("port"))

            elif action == "serial_disconnect":
                self._serial_disconnect()

            elif action == "calib_set":
                self._apply_calibration(params)

            elif action == "calib_auto":
                self._auto_zones(params.get("split", "lr"))

    def _load_yolo(self):
        try:
            from ultralytics import YOLO
            self.model = YOLO("yolov8n.pt")
            self.masker = ld.TrafficMasker(self.args.stalled_seconds)
            self.log("YOLO loaded", "ok")
        except Exception as exc:
            self.p["use_yolo"] = False
            self.log(f"YOLO failed to load: {exc}", "error")

    def _save_snapshot(self, frame, reason):
        SNAPSHOT_DIR.mkdir(exist_ok=True)
        name = time.strftime("%Y%m%d-%H%M%S") + f"-{reason}.jpg"
        cv2.imwrite(str(SNAPSHOT_DIR / name), frame)
        self.log(f"snapshot saved: {name}", "ok")

    def _append_alert_log(self, event, lane, text):
        """Durable trail of block/alarm/resolve events -- survives a restart,
        unlike self.events (in-memory, 300-entry ring buffer). One JSON object
        per line so it can be tailed or grepped without parsing the whole file."""
        rec = {"ts": time.time(), "event": event, "lane": lane, "text": text}
        try:
            with open(ALERT_LOG, "a", encoding="utf-8") as f:
                f.write(json.dumps(rec) + "\n")
        except OSError as exc:
            self.log(f"alert log write failed: {exc}", "warn")

    def _apply_calibration(self, params):
        """Lane polygons clicked in the browser, in normalised 0-1 coords."""
        lanes = params.get("lanes") or {}
        payload = {"_meta": {"width": self.size[0], "height": self.size[1]}}
        for lane, pts in lanes.items():
            if len(pts) != 4:
                self.log(f"LANE {lane}: need exactly 4 points", "error")
                return
            payload[lane] = [[int(round(x * self.size[0])),
                              int(round(y * self.size[1]))] for x, y in pts]
        ld.ZONES_FILE.write_text(json.dumps(payload, indent=2))
        self._load_zones()
        self.log(f"lanes recalibrated: {', '.join(sorted(lanes))}", "ok")

    def _auto_zones(self, split):
        w, h = self.size
        m, gap = 20, 20
        if split == "lr":
            mid = w // 2
            a = [[m, m], [mid - gap, m], [mid - gap, h - m], [m, h - m]]
            b = [[mid + gap, m], [w - m, m], [w - m, h - m], [mid + gap, h - m]]
        else:
            mid = h // 2
            a = [[m, m], [w - m, m], [w - m, mid - gap], [m, mid - gap]]
            b = [[m, mid + gap], [w - m, mid + gap], [w - m, h - m], [m, h - m]]
        ld.ZONES_FILE.write_text(json.dumps(
            {"_meta": {"width": w, "height": h}, "A": a, "B": b}, indent=2))
        self._load_zones()
        self.log(f"auto zones applied ({split})", "ok")

    # -- main loop ---------------------------------------------------------
    def run(self):
        try:
            self.cap, self.size = ld.open_camera(
                self.args.camera, self.args.width, self.args.height,
                self.args.backend, self.args.lock_exposure)
        except SystemExit as exc:
            self._publish_error(f"camera: {exc}")
            return

        if not ld.ZONES_FILE.exists():
            self._auto_zones("lr")
            self.log("no zones.json -- generated a left/right split", "warn")
        if not ld.BASELINE_FILE.exists():
            ok, f = self.cap.read()
            if ok:
                cv2.imwrite(str(ld.BASELINE_FILE), ld.prep(f))
                self.log("no baseline.png -- captured one from the live frame", "warn")

        self.baseline = ld.load_baseline(self.size)
        self.photo_ref = ld.photometric_ref(self.baseline)
        self._load_zones()
        self._seed_model()

        if self.p["use_yolo"]:
            self._load_yolo()
        if self.args.port:
            self._serial_connect(self.args.port)

        self.log("detector running", "ok")

        any_blocked_prev, shift_prev = False, 0.0
        raw_occ_prev = {lane: False for lane in self.zones}   # see all_occupied below
        last_cmd, last_sent = None, 0.0
        frame_times = collections.deque(maxlen=30)
        started = time.monotonic()
        prev_blocked = {lane: False for lane in self.zones}

        while not self.stop_evt.is_set():
            ok, frame = self.cap.read()
            if not ok:
                self.log("camera read failed", "error")
                time.sleep(0.3)
                continue
            now = time.monotonic()

            raw_grey = ld.prep(frame)
            grey = (ld.photometric_align(raw_grey, self.photo_ref)
                    if self.p["photometric"] else raw_grey)

            self._handle_actions(frame, grey)

            # ---- baseline recapture, spread across frames so the serial
            # watchdog keep-alive never starves (a blocking 6s capture would
            # exceed the firmware's 2s TIMEOUT_MS and trip a false all-red).
            if self._baseline_frames is not None:
                self._baseline_frames.append(grey.copy())
                if len(self._baseline_frames) >= 90:
                    self._commit_baseline()

            # ---- background model, same gating lane_detect.py uses ----
            # Two independent "this looks global, let the model catch up"
            # signals: a large whole-frame change, OR every lane occupied at
            # once. The second one closes a real gap found live: dim lighting
            # pushed frame-change to 41.8% -- under the old 50% bar -- while
            # tripping BOTH lanes' much more sensitive local thresholds, so
            # the model froze and stayed stuck reading a blockage that was
            # actually just the room going dark. Two lanes tripping together
            # from a shared cause is common; two real obstructions arriving in
            # the same instant is not, so this signal is safe to trust.
            all_occupied = len(self.zones) > 1 and all(raw_occ_prev.values())
            disrupted = (shift_prev > ld.SHIFT_ALARM) or all_occupied
            if self._relearn_frames > 0:
                rate = 0.05
                self._relearn_frames -= 1
            elif ld.MOG_FREEZE_WHEN_BLOCKED and any_blocked_prev and not disrupted:
                rate = 0.0
            elif disrupted:
                rate = ld.MOG_CATCHUP_LEARNING_RATE
            else:
                rate = ld.MOG_LEARNING_RATE
            frozen = (rate == 0.0)

            fg = self.bs.apply(grey, learningRate=rate)
            # threshold at 254 keeps MOG2's true foreground (255) and drops
            # its shadow class (127). cv2.threshold not np.where: 48x faster,
            # and avoids a 2.4MB throwaway int64 allocation per frame.
            cut = 254 if self.p["shadow_reject"] else 0
            _, mask = cv2.threshold(fg, cut, 255, cv2.THRESH_BINARY)

            # OR in the edge channel -- recovers dark textured objects that
            # the shadow classifier discards (measured 0px vs 41109px).
            if self.p["gradient"]:
                fgg = self.bs_grad.apply(ld.gradient_map(grey), learningRate=rate)
                _, gm = cv2.threshold(fgg, 127, 255, cv2.THRESH_BINARY)
                mask = cv2.bitwise_or(mask, gm)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ld.MORPH_KERNEL)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ld.MORPH_KERNEL)
            shadow_px = int((fg == 127).sum())

            drawn = []
            if self.p["use_yolo"] and self.model is not None:
                res = self.model.track(frame, persist=True, verbose=False)[0]
                traffic, drawn = self.masker.build(res, mask.shape, now)
                mask = cv2.bitwise_and(mask, cv2.bitwise_not(traffic))

            lanes_out, blocked = {}, {}
            for lane in self.zones:
                lp = cv2.bitwise_and(mask, self.lane_masks[lane])
                blob, bx, by = ld.largest_blob_with_centroid(lp)
                need = self._need(lane)
                # Occupancy needs BOTH a big enough blob AND that blob holding
                # its position and size -- size alone flickers badly in poor
                # light. See BlobStability in lane_detect.py.
                big_enough = blob >= need
                raw_occ = self.stability[lane].update(big_enough, blob, bx, by)
                auto_blocked = self.states[lane].update(raw_occ, now)

                mode = self.override.get(lane, "auto")
                final = (True if mode == "red"
                         else False if mode == "green" else auto_blocked)
                blocked[lane] = final
                lanes_out[lane] = {
                    "blob": int(blob),
                    "need": int(need),
                    "pct_of_need": round(blob / max(need, 1) * 100, 1),
                    "fill": round(cv2.countNonZero(lp) / self.lane_areas[lane] * 100, 1),
                    "credit": round(self.states[lane].credit, 2),
                    "block_seconds": self.p["block_seconds"],
                    "release_seconds": self.p["release_seconds"],
                    "occupied": bool(raw_occ),
                    "big_enough": bool(big_enough),
                    "stable": bool(raw_occ),
                    "blocked": bool(final),
                    "auto_blocked": bool(auto_blocked),
                    "override": mode,
                    "area": self.lane_areas[lane],
                }

                if final and not prev_blocked[lane]:
                    self.log(f"LANE {lane} BLOCKED (blob {blob}px vs {int(need)}px)",
                             "error")
                    # "Supervisor alert" per the design doc: stays open until
                    # the lane clears, not a one-off ping. _append_alert_log
                    # is the durable trail; self.log is the live UI feed.
                    self.supervisor_alert[lane] = {"since": now,
                        "text": f"Lane {lane} obstruction detected"}
                    self._append_alert_log("supervisor_alert", lane,
                        f"blob {blob}px vs need {int(need)}px")
                    if self.args.snapshot_on_block:
                        self._save_snapshot(frame, f"lane{lane}-blocked")
                elif prev_blocked[lane] and not final:
                    self.log(f"LANE {lane} cleared", "ok")
                    if self.supervisor_alert.pop(lane, None) is not None:
                        self._append_alert_log("resolved", lane,
                            "obstruction cleared")
                prev_blocked[lane] = final

            # ---- person-on-blocked-lane alarm --------------------------
            # PDF's "additional feature": alarm if a worker enters the lane
            # that's currently red. Uses the box's bottom-centre (feet), not
            # its centroid -- a standing person's box is tall, so the
            # centroid sits over the torso, not where they're actually
            # standing. Silently inert without YOLO (--no-yolo): there is no
            # other signal in this pipeline that distinguishes a person from
            # an obstruction.
            persons_in = set()
            for (x1, y1, x2, y2, cls, conf, excused) in drawn:
                if cls != "person":
                    continue
                fx, fy = int((x1 + x2) / 2), int(y2)
                for lane in self.zones:
                    if not blocked[lane]:
                        continue
                    m = self.lane_masks[lane]
                    if 0 <= fy < m.shape[0] and 0 <= fx < m.shape[1] and m[fy, fx]:
                        persons_in.add(lane)

            for lane in self.zones:
                if lane in persons_in:
                    self.person_clear_since[lane] = None
                    if self.person_since.get(lane) is None:
                        self.person_since[lane] = now
                    if (not self.person_alarm_active.get(lane)
                            and now - self.person_since[lane] >= PERSON_ALARM_CONFIRM_S):
                        self.person_alarm_active[lane] = True
                        self.log(f"ALARM: person on BLOCKED lane {lane}", "error")
                        self._append_alert_log("person_alarm", lane,
                            "person entered blocked lane")
                else:
                    self.person_since[lane] = None
                    if not self.person_alarm_active.get(lane):
                        continue
                    if not blocked[lane]:
                        # danger context is gone -- drop it now, no debounce
                        self.person_alarm_active[lane] = False
                        self.person_clear_since[lane] = None
                    elif self.person_clear_since.get(lane) is None:
                        self.person_clear_since[lane] = now
                    elif now - self.person_clear_since[lane] >= PERSON_ALARM_RELEASE_S:
                        self.person_alarm_active[lane] = False
                        self.person_clear_since[lane] = None
                        self.log(f"alarm cleared: lane {lane}", "ok")

            any_blocked_prev = any(blocked.values())
            raw_occ_prev = {lane: lanes_out[lane]["occupied"] for lane in self.zones}
            shift_prev = cv2.countNonZero(mask) / float(mask.size)

            # Geometric vs photometric discrimination, but ONLY once MOG2
            # itself reports a real disruption. Measuring against the stored
            # baseline.png unconditionally does not work any more: MOG2 adapts
            # continuously, so baseline.png legitimately drifts out of date and
            # a raw comparison against it reads ~77% on a perfectly healthy
            # system. Gating on the live foreground fraction means the question
            # is only asked when something is actually wrong -- and it also
            # skips two full-frame diffs per frame in the normal case.
            if shift_prev > ld.SHIFT_WARN:
                ref = self.bs.getBackgroundImage()
                if ref is not None and ref.shape == raw_grey.shape:
                    raw_shift = ld.scene_shift(raw_grey, ref)
                    aligned_shift = ld.scene_shift(
                        ld.photometric_align(raw_grey, ld.photometric_ref(ref)), ref)
                else:
                    raw_shift = aligned_shift = shift_prev
                # If aligning the exposure collapses the difference, the scene
                # merely got brighter or darker -- relearning fixes it. If it
                # does NOT, the change is geometric: the camera physically
                # moved, and re-baselining would silence the alarm while
                # leaving the lane polygons aimed at the wrong pixels.
                likely_moved = aligned_shift > ld.SHIFT_WARN and aligned_shift > raw_shift * 0.7
            else:
                raw_shift = aligned_shift = shift_prev
                likely_moved = False

            a, b = blocked.get("A", False), blocked.get("B", False)
            cmd = "3" if (a and b) else "1" if a else "2" if b else "0"
            wall = time.time()
            if self.ser is not None and (cmd != last_cmd
                                         or wall - last_sent > ld.SEND_INTERVAL):
                try:
                    self.ser.write(cmd.encode())
                    last_cmd, last_sent = cmd, wall
                except Exception as exc:
                    self.log(f"serial write failed: {exc}", "error")
                    self._serial_disconnect()

            frame_times.append(now)
            fps = ((len(frame_times) - 1)
                   / max(frame_times[-1] - frame_times[0], 1e-6)
                   ) if len(frame_times) > 1 else 0.0

            view = self._render(frame, mask, lanes_out, drawn)
            enc = cv2.imencode(".jpg", view, ENC_PARAMS)[1]

            state = {
                "status": "running",
                "lanes": lanes_out,
                "cmd": cmd,
                "cmd_text": {"0": "both clear", "1": "A blocked",
                             "2": "B blocked", "3": "both blocked"}[cmd],
                "fps": round(fps, 1),
                "uptime": round(now - started, 1),
                "frozen": bool(frozen),
                "disrupted": bool(disrupted),
                "frame_change": round(shift_prev * 100, 1),
                "raw_shift": round(raw_shift * 100, 1),
                "aligned_shift": round(aligned_shift * 100, 1),
                "likely_moved": bool(likely_moved),
                "shadow_px": shadow_px,
                "serial": self.args.port if self.ser is not None else None,
                "yolo": bool(self.p["use_yolo"] and self.model is not None),
                "params": dict(self.p),
                "overrides": dict(self.override),
                "override_expires": {k: round(max(v - now, 0), 1)
                                     for k, v in self.override_until.items()},
                "baseline_progress": (len(self._baseline_frames)
                                      if self._baseline_frames is not None else None),
                "alerts": {
                    "supervisor": [
                        {"lane": lane, "text": info["text"],
                         "since_s": round(now - info["since"], 1)}
                        for lane, info in sorted(self.supervisor_alert.items())
                    ],
                    "person": sorted(lane for lane in self.zones
                                     if self.person_alarm_active.get(lane)),
                },
                "size": list(self.size),
                "ts": wall,
            }

            with self._frame_ready:
                self._jpeg = enc.tobytes()
                self._seq += 1
                self._state = state
                self._frame_ready.notify_all()

        self._serial_disconnect()
        if self.cap is not None:
            self.cap.release()
        self.log("detector stopped", "warn")

    def _commit_baseline(self):
        """Finish a recapture: back up the old baseline, then swap it in.

        The old file is versioned rather than overwritten because the worst
        silent failure in this system is re-baselining while an obstruction is
        present -- the object becomes 'normal', the lane reads clear forever,
        and the freeze gate never engages because the lane never latches.
        Being able to roll back is the difference between a mistake and an
        unrecoverable one.
        """
        if ld.BASELINE_FILE.exists():
            backups = HERE / "baseline_history"
            backups.mkdir(exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            cv2.imwrite(str(backups / f"baseline-{stamp}.png"),
                        cv2.imread(str(ld.BASELINE_FILE), cv2.IMREAD_GRAYSCALE))

        self.baseline = np.median(
            np.stack(self._baseline_frames), axis=0).astype(np.uint8)
        cv2.imwrite(str(ld.BASELINE_FILE), self.baseline)
        self.photo_ref = ld.photometric_ref(self.baseline)
        self._seed_model()
        for st in self.states.values():
            st.credit, st.blocked, st.last_t = 0.0, False, None
        self._baseline_frames = None
        self.log("baseline recaptured, previous version archived", "ok")

    def _publish_error(self, msg):
        self.log(msg, "error")
        with self._lock:
            self._state = {"status": "error", "error": msg}

    # -- overlay -----------------------------------------------------------
    def _render(self, frame, mask, lanes_out, drawn):
        """Deliberately minimal. Numbers live in the HTML where they render
        as crisp text; burning them into the video makes both harder to read."""
        if self.p["show_mask"]:
            view = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        else:
            view = frame.copy()

        # BGR equivalents of the UI's CVD-safe palette, so the video overlay
        # and the HTML cards agree on what each state looks like.
        CLEAR, CONFIRM, BLOCKED = (143, 209, 0), (36, 165, 245), (58, 68, 240)
        for lane, poly in self.zones.items():
            info = lanes_out[lane]
            if info["blocked"]:
                col, tag = BLOCKED, "BLOCKED"
            elif info["occupied"] or info["credit"] > 0.2:
                col, tag = CONFIRM, "CONFIRMING"
            else:
                col, tag = CLEAR, "CLEAR"
            if info["override"] != "auto":
                tag += f" [{info['override'].upper()}]"

            cv2.polylines(view, [poly], True, col, 3, cv2.LINE_AA)
            x, y = int(poly[:, 0].min()), int(poly[:, 1].min())
            label = f"{lane}  {tag}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            y = max(y, th + 12)
            cv2.rectangle(view, (x, y - th - 12), (x + tw + 14, y), col, -1)
            cv2.putText(view, label, (x + 7, y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (12, 14, 20), 2, cv2.LINE_AA)

        for x1, y1, x2, y2, name, conf, excused in drawn:
            col = (200, 160, 60) if excused else (58, 68, 240)
            cv2.rectangle(view, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
            cv2.putText(view, name + ("" if excused else " STALLED"),
                        (int(x1), max(int(y1) - 6, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)

        if self._baseline_frames is not None:
            n = len(self._baseline_frames)
            cv2.rectangle(view, (0, 0), (view.shape[1], 34), (20, 20, 20), -1)
            cv2.putText(view, f"CAPTURING BASELINE  {n}/90  -- keep lanes clear",
                        (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (36, 165, 245), 2, cv2.LINE_AA)
        return view


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------
BOUNDARY = "frameboundary"


class Handler(BaseHTTPRequestHandler):
    """One instance per request. `detector` is injected as a class attribute
    by serve(); every handler thread shares that single detector."""

    detector = None
    protocol_version = "HTTP/1.1"
    # StreamRequestHandler.setup() turns this into a socket timeout. Without
    # it, once the kernel send buffer fills (~64KB on Windows -- only ~2 frames)
    # wfile.write() blocks forever on a stalled browser.
    timeout = 10
    disable_nagle_algorithm = True

    def log_message(self, fmt, *a):        # silence per-request console spam
        pass

    # -- helpers -----------------------------------------------------------
    def _send(self, code, body, ctype, extra=None):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj, code=200):
        self._send(code, json.dumps(obj), "application/json")

    # -- GET ---------------------------------------------------------------
    def do_GET(self):
        path = self.path.split("?", 1)[0]

        if path in ("/", "/index.html"):
            if not UI_FILE.exists():
                self._send(500, f"missing {UI_FILE.name}", "text/plain")
                return
            self._send(200, UI_FILE.read_text(encoding="utf-8"),
                       "text/html; charset=utf-8")

        elif path == "/icons.json":
            if ICONS_FILE.exists():
                self._send(200, ICONS_FILE.read_text(encoding="utf-8"),
                           "application/json")
            else:
                self._json({})

        elif path == "/api/state":
            st = self.detector.snapshot_state()
            st["events"] = self.detector.event_list()
            self._json(st)

        elif path == "/api/ports":
            try:
                from serial.tools import list_ports
                ports = [{"device": p.device,
                          "desc": p.description,
                          "espressif": (p.vid == 0x303A)}
                         for p in list_ports.comports()]
            except Exception:
                ports = []
            self._json({"ports": ports})

        elif path == "/snapshot.jpg":
            jpeg, _ = self.detector.latest_jpeg(-1)
            if jpeg is None:
                self._send(503, b"no frame yet", "text/plain")
            else:
                self._send(200, jpeg, "image/jpeg")

        elif path == "/stream.mjpg":
            self._stream()

        else:
            self._send(404, "not found", "text/plain")

    # -- MJPEG -------------------------------------------------------------
    def _stream(self):
        """multipart/x-mixed-replace, one JPEG per part.

        The detector encodes each frame exactly once; every connected client
        just re-sends those same bytes. A slow or stalled browser therefore
        costs one socket write, never a re-encode, and can never backpressure
        the detection loop -- it simply misses frames and picks up at whatever
        is current, because latest_jpeg() always returns the newest frame
        rather than a queued backlog.
        """
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()
        self.close_connection = True

        last_seq = -1
        try:
            while not self.detector.stop_evt.is_set():
                jpeg, last_seq = self.detector.latest_jpeg(last_seq)
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(
                    (f"--{BOUNDARY}\r\n"
                     f"Content-Type: image/jpeg\r\n"
                     f"Content-Length: {len(jpeg)}\r\n\r\n").encode("ascii"))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass                    # browser closed the tab; entirely normal

    # -- POST --------------------------------------------------------------
    def _origin_ok(self):
        """Reject cross-origin POSTs.

        These endpoints drive a physical traffic signal, and any web page the
        operator happens to have open can POST to 127.0.0.1 from their browser.
        Checking Origin/Host costs nothing and closes that door.
        """
        origin = self.headers.get("Origin")
        if origin:
            allowed = {f"http://{self.server.server_address[0]}:{self.server.server_address[1]}",
                       f"http://127.0.0.1:{self.server.server_address[1]}",
                       f"http://localhost:{self.server.server_address[1]}"}
            if origin not in allowed:
                return False
        host = (self.headers.get("Host") or "").split(":")[0]
        return host in ("127.0.0.1", "localhost", self.server.server_address[0])

    def do_POST(self):
        if not self._origin_ok():
            self._json({"ok": False, "error": "cross-origin request refused"}, 403)
            return
        if self.path.split("?", 1)[0] != "/api/action":
            self._send(404, "not found", "text/plain")
            return
        try:
            n = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(n) or b"{}")
        except Exception as exc:
            self._json({"ok": False, "error": f"bad request: {exc}"}, 400)
            return

        action = payload.get("action")
        params = payload.get("params", {})
        det = self.detector

        # ---- server-side guards. The UI also guards these, but a dangerous
        # action must not be one fetch() away from anyone who opens devtools.
        if action == "override" and params.get("mode") == "green":
            if not params.get("confirmed"):
                self._json({"ok": False,
                            "error": "force-clear requires confirmation"}, 403)
                return

        if action == "recapture_baseline" and not params.get("force"):
            st = det.snapshot_state()
            blocked = [k for k, v in (st.get("lanes") or {}).items()
                       if v.get("blocked")]
            if blocked:
                self._json({
                    "ok": False,
                    "needs_force": True,
                    "error": f"LANE {', '.join(blocked)} currently BLOCKED. "
                             f"Re-baselining now would absorb whatever is "
                             f"there into the background permanently."}, 409)
                return

        det.submit(action, params)
        self._json({"ok": True})


class Server(ThreadingHTTPServer):
    daemon_threads = True           # don't let open streams block shutdown
    # NOT True. On Windows SO_REUSEADDR means "steal the port", not the Unix
    # TIME_WAIT semantics -- a second instance would silently bind the same
    # port and the browser would receive frames from two cameras at random.
    allow_reuse_address = False

    def handle_error(self, request, client_address):
        pass                        # a closed tab is not an error worth a traceback


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=ld.FRAME_W)
    ap.add_argument("--height", type=int, default=ld.FRAME_H)
    ap.add_argument("--backend", choices=("dshow", "msmf", "any"), default="dshow")
    ap.add_argument("--port", default=None, help="ESP32-C3 serial port, e.g. COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--bind", default="127.0.0.1",
                    help="use 0.0.0.0 to reach it from another device")
    ap.add_argument("--no-yolo", action="store_true")
    ap.add_argument("--lock-exposure", action="store_true")
    ap.add_argument("--stalled-seconds", type=float, default=ld.STALLED_SECONDS)
    ap.add_argument("--occupy-blob-px", type=int, default=ld.OCCUPY_BLOB_PX)
    ap.add_argument("--block-seconds", type=float, default=None)
    ap.add_argument("--release-seconds", type=float, default=None)
    ap.add_argument("--decay", type=float, default=None)
    ap.add_argument("--fast", action="store_true",
                    help="tabletop preset: block=3s release=1s decay=2.0")
    ap.add_argument("--snapshot-on-block", action="store_true",
                    help="save a JPEG every time a lane latches BLOCKED")
    args = ap.parse_args()

    if args.fast:
        if args.block_seconds is None:   args.block_seconds = 3.0
        if args.release_seconds is None: args.release_seconds = 1.0
        if args.decay is None:           args.decay = 2.0
    if args.block_seconds is None:   args.block_seconds = ld.BLOCK_SECONDS
    if args.release_seconds is None: args.release_seconds = ld.RELEASE_SECONDS
    if args.decay is None:           args.decay = ld.DECAY_RATE

    det = Detector(args)
    det.start()

    Handler.detector = det
    try:
        httpd = Server((args.bind, args.http_port), Handler)
    except OSError as exc:
        det.stop_evt.set()
        raise SystemExit(
            f"Could not bind {args.bind}:{args.http_port} -- {exc}. "
            f"Another instance is probably already running. "
            f"Close it, or pass --http-port 8001.")

    url = f"http://{'127.0.0.1' if args.bind == '0.0.0.0' else args.bind}:{args.http_port}"
    print("=" * 62)
    print(f"  Lane monitor dashboard  ->  {url}")
    if args.bind == "0.0.0.0":
        print("  (bound to all interfaces -- reachable from other devices)")
    print(f"  block={args.block_seconds}s  release={args.release_seconds}s  "
          f"decay={args.decay}"
          + ("   [--fast preset]" if args.fast else ""))
    print("  Ctrl+C to stop")
    print("=" * 62)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nstopping...")
    finally:
        det.stop_evt.set()
        with det._frame_ready:
            det._frame_ready.notify_all()   # release any parked stream threads
        httpd.shutdown()
        httpd.server_close()
        det.join(timeout=4.0)
        print("stopped.")


if __name__ == "__main__":
    main()
