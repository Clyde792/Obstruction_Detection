"""
replay.py -- record real camera clips, then replay them through the detection
pipeline deterministically.

The problem this solves: every tuning change so far has been tested by putting
an object down and watching. That is not reproducible -- you cannot tell
whether a threshold change helped or the object simply moved a few pixels, and
you cannot check that fixing tonight's scene did not break last week's. A clip
replayed twice with the same parameters gives byte-identical results, so an
A/B comparison actually means something.

    python replay.py record dim-flicker --seconds 20
    python replay.py list
    python replay.py run dim-flicker
    python replay.py run dim-flicker --occupy-blob-px 2000
    python replay.py ab dim-flicker --occupy-blob-px 2000

Frames are stored as lossless PNG. Compressed video would slightly alter the
sensor noise, and sensor noise is precisely what we are trying to measure.
"""

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np

import lane_detect as ld

HERE = Path(__file__).resolve().parent
CLIPS = HERE / "clips"


# ---------------------------------------------------------------------------
def clip_dir(name):
    return CLIPS / name


def record(args):
    d = clip_dir(args.name)
    if d.exists():
        if not args.overwrite:
            raise SystemExit(f"clip '{args.name}' already exists -- pass "
                             f"--overwrite to replace it")
        shutil.rmtree(d)
    d.mkdir(parents=True)

    cap, size = ld.open_camera(args.camera, args.width, args.height,
                              args.backend, args.lock_exposure)
    print(f"recording '{args.name}' at {size[0]}x{size[1]} for "
          f"{args.seconds:.0f}s -- do not move the camera")

    frames, t0, last = 0, time.monotonic(), 0.0
    try:
        while time.monotonic() - t0 < args.seconds:
            ok, frame = cap.read()
            if not ok:
                print("camera read failed")
                break
            cv2.imwrite(str(d / f"f{frames:05d}.png"), frame)
            frames += 1
            el = time.monotonic() - t0
            if el - last >= 2.0:
                last = el
                print(f"  {el:4.1f}s  {frames} frames")
    finally:
        cap.release()

    meta = {"name": args.name, "frames": frames,
            "width": size[0], "height": size[1],
            "seconds": round(time.monotonic() - t0, 2),
            "fps": round(frames / max(time.monotonic() - t0, 1e-6), 2),
            "recorded": time.strftime("%Y-%m-%d %H:%M:%S"),
            "note": args.note or ""}
    (d / "meta.json").write_text(json.dumps(meta, indent=2))
    print(f"saved {frames} frames to {d}  ({meta['fps']} fps)")


def list_clips(args):
    if not CLIPS.exists() or not any(CLIPS.iterdir()):
        print("no clips recorded yet")
        return
    for d in sorted(CLIPS.iterdir()):
        mp = d / "meta.json"
        if not mp.exists():
            continue
        m = json.loads(mp.read_text())
        note = f"  -- {m['note']}" if m.get("note") else ""
        print(f"  {m['name']:<24} {m['frames']:>4} frames  "
              f"{m['seconds']:>5.1f}s @ {m['fps']:>4.1f}fps  "
              f"{m['recorded']}{note}")


def load_clip(name):
    d = clip_dir(name)
    if not d.exists():
        raise SystemExit(f"no clip named '{name}' -- try: python replay.py list")
    meta = json.loads((d / "meta.json").read_text())
    files = sorted(d.glob("f*.png"))
    if not files:
        raise SystemExit(f"clip '{name}' has no frames")
    return meta, files


# ---------------------------------------------------------------------------
def run_pipeline(name, overrides, quiet=False):
    """Replay a clip through the real detection pipeline.

    Deliberately mirrors lane_detect.py's main loop rather than importing it,
    because that loop is welded to a live camera and a GUI. Every decision
    value comes from lane_detect (thresholds, LaneState, largest_blob,
    photometric_align), so behaviour tracks the real system.
    """
    meta, files = load_clip(name)
    size = (meta["width"], meta["height"])

    p = {
        "block_seconds": ld.BLOCK_SECONDS,
        "release_seconds": ld.RELEASE_SECONDS,
        "decay": ld.DECAY_RATE,
        "occupy_blob_px": ld.OCCUPY_BLOB_PX,
        "occupy_blob_frac": ld.OCCUPY_BLOB_FRAC,
        "var_threshold": ld.MOG_VAR_THRESHOLD,
        "shadow_threshold": ld.MOG_SHADOW_THRESHOLD,
        "shadow_reject": True,
        "photometric": True,
        "stability": ld.REQUIRE_STABLE_FRAMES,
        "gradient": ld.USE_GRADIENT_CHANNEL,
        "grad_var": ld.GRAD_VAR_THRESHOLD,
    }
    p.update({k: v for k, v in overrides.items() if v is not None})

    zones = ld.load_zones(size)
    baseline = ld.load_baseline(size)
    photo_ref = ld.photometric_ref(baseline)

    lane_masks, lane_areas = {}, {}
    for lane, poly in zones.items():
        m = np.zeros((size[1], size[0]), np.uint8)
        cv2.fillPoly(m, [poly], 255)
        lane_masks[lane] = m
        lane_areas[lane] = max(int(cv2.countNonZero(m)), 1)

    bs = cv2.createBackgroundSubtractorMOG2(
        history=ld.MOG_HISTORY, varThreshold=p["var_threshold"],
        detectShadows=True)
    bs.setShadowThreshold(p["shadow_threshold"])
    for _ in range(20):
        bs.apply(baseline, learningRate=0.5)

    bs_g = None
    if p["gradient"]:
        # detectShadows=False: shadows barely alter an edge map in the first
        # place, so the classifier would only add cost and noise here.
        bs_g = cv2.createBackgroundSubtractorMOG2(
            history=ld.MOG_HISTORY, varThreshold=p["grad_var"],
            detectShadows=False)
        gb = ld.gradient_map(baseline)
        for _ in range(20):
            bs_g.apply(gb, learningRate=0.5)

    states = {l: ld.LaneState(p["block_seconds"], p["release_seconds"], p["decay"])
              for l in zones}
    trackers = {l: ld.BlobStability(p["stability"]) for l in zones}

    dt = 1.0 / max(meta["fps"], 1e-6)
    any_blocked_prev, shift_prev = False, 0.0
    raw_occ_prev = {l: False for l in zones}
    rows, transitions = [], []
    prev_blocked = {l: False for l in zones}

    for i, fp in enumerate(files):
        frame = cv2.imread(str(fp))
        t = i * dt
        raw_grey = ld.prep(frame)
        grey = (ld.photometric_align(raw_grey, photo_ref)
                if p["photometric"] else raw_grey)

        all_occupied = len(zones) > 1 and all(raw_occ_prev.values())
        disrupted = (shift_prev > ld.SHIFT_ALARM) or all_occupied
        if ld.MOG_FREEZE_WHEN_BLOCKED and any_blocked_prev and not disrupted:
            rate = 0.0
        elif disrupted:
            rate = ld.MOG_CATCHUP_LEARNING_RATE
        else:
            rate = ld.MOG_LEARNING_RATE

        fg = bs.apply(grey, learningRate=rate)
        cut = 254 if p["shadow_reject"] else 0
        _, mask = cv2.threshold(fg, cut, 255, cv2.THRESH_BINARY)

        if bs_g is not None:
            fgg = bs_g.apply(ld.gradient_map(grey), learningRate=rate)
            _, gmask = cv2.threshold(fgg, 127, 255, cv2.THRESH_BINARY)
            # OR, not AND: the whole point is that the channels fail in
            # different places, so requiring both would inherit both blind
            # spots instead of covering them.
            mask = cv2.bitwise_or(mask, gmask)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ld.MORPH_KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, ld.MORPH_KERNEL)

        row = {"t": round(t, 3), "frame": i}
        raw_occ = {}
        for lane in zones:
            lp = cv2.bitwise_and(mask, lane_masks[lane])
            blob, cx, cy = ld.largest_blob_with_centroid(lp)
            need = min(p["occupy_blob_px"], p["occupy_blob_frac"] * lane_areas[lane])
            big_enough = blob >= need
            stable = trackers[lane].update(big_enough, blob, cx, cy)
            raw_occ[lane] = big_enough and stable
            blocked = states[lane].update(raw_occ[lane], t)
            if blocked and not prev_blocked[lane]:
                transitions.append((round(t, 2), lane, "BLOCKED", blob))
            elif prev_blocked[lane] and not blocked:
                transitions.append((round(t, 2), lane, "cleared", blob))
            prev_blocked[lane] = blocked
            row[f"{lane}_blob"] = blob
            row[f"{lane}_need"] = int(need)
            row[f"{lane}_bigenough"] = big_enough
            row[f"{lane}_stable"] = stable
            row[f"{lane}_credit"] = round(states[lane].credit, 2)
            row[f"{lane}_blocked"] = blocked

        any_blocked_prev = any(prev_blocked.values())
        raw_occ_prev = raw_occ
        shift_prev = cv2.countNonZero(mask) / float(mask.size)
        row["frame_change"] = round(shift_prev * 100, 2)
        rows.append(row)

    return meta, zones, rows, transitions, p


def summarise(meta, zones, rows, transitions, p, label=""):
    n = len(rows)
    print(f"\n{'=' * 66}")
    print(f"  {label or meta['name']}   {n} frames @ {meta['fps']}fps "
          f"({meta['seconds']}s)")
    print(f"  block={p['block_seconds']}s release={p['release_seconds']}s "
          f"blob_px={p['occupy_blob_px']} stability={p['stability']} "
          f"gradient={'on' if p['gradient'] else 'off'}")
    print(f"{'=' * 66}")
    for lane in sorted(zones):
        blobs = np.array([r[f"{lane}_blob"] for r in rows])
        big = sum(r[f"{lane}_bigenough"] for r in rows)
        stab = sum(r[f"{lane}_stable"] for r in rows)
        blk = sum(r[f"{lane}_blocked"] for r in rows)
        need = rows[0][f"{lane}_need"]
        print(f"  LANE {lane}: need={need}px  blob min={blobs.min()} "
              f"med={int(np.median(blobs))} max={blobs.max()}")
        print(f"           frames over size threshold : {big}/{n}"
              f"   ({big/n*100:.0f}%)")
        print(f"           ...that also passed stability: {stab}/{n}"
              f"   ({stab/n*100:.0f}%)")
        print(f"           frames latched BLOCKED      : {blk}/{n}"
              f"   ({blk/n*100:.0f}%)")
    fc = np.array([r["frame_change"] for r in rows])
    print(f"  frame change: med {np.median(fc):.1f}%  max {fc.max():.1f}%")
    print(f"  transitions ({len(transitions)}):")
    for t, lane, what, blob in transitions[:16]:
        print(f"     t={t:6.2f}s  LANE {lane} {what}  (blob {blob}px)")
    if len(transitions) > 16:
        print(f"     ... and {len(transitions)-16} more")
    if not transitions:
        print("     (none -- state never changed)")


def cmd_run(args):
    ov = {"block_seconds": args.block_seconds,
          "release_seconds": args.release_seconds,
          "occupy_blob_px": args.occupy_blob_px,
          "stability": args.stability,
          "gradient": False if args.no_gradient else None,
          "shadow_reject": (not args.no_shadow_reject) if args.no_shadow_reject else None,
          "photometric": (not args.no_photometric) if args.no_photometric else None}
    meta, zones, rows, tr, p = run_pipeline(args.name, ov)
    summarise(meta, zones, rows, tr, p)
    if args.csv:
        out = HERE / f"replay-{args.name}.csv"
        keys = list(rows[0].keys())
        out.write_text(",".join(keys) + "\n" +
                       "\n".join(",".join(str(r[k]) for k in keys) for r in rows))
        print(f"\n  wrote {out}")


def cmd_ab(args):
    """Same clip, two parameter sets -- the whole point of the harness."""
    base = {"block_seconds": None, "release_seconds": None,
            "occupy_blob_px": None, "stability": None,
            "shadow_reject": None, "photometric": None}
    meta, zones, rows_a, tr_a, p_a = run_pipeline(args.name, dict(base))
    summarise(meta, zones, rows_a, tr_a, p_a, label=f"{args.name}  [A: defaults]")

    ov = dict(base)
    ov.update({k: v for k, v in
               {"block_seconds": args.block_seconds,
                "release_seconds": args.release_seconds,
                "occupy_blob_px": args.occupy_blob_px,
                "stability": args.stability}.items() if v is not None})
    meta, zones, rows_b, tr_b, p_b = run_pipeline(args.name, ov)
    summarise(meta, zones, rows_b, tr_b, p_b, label=f"{args.name}  [B: modified]")

    print(f"\n{'=' * 66}")
    print("  A vs B")
    print(f"{'=' * 66}")
    for lane in sorted(zones):
        ba = sum(r[f"{lane}_blocked"] for r in rows_a)
        bb = sum(r[f"{lane}_blocked"] for r in rows_b)
        print(f"  LANE {lane} blocked frames: {ba} -> {bb}"
              f"   ({bb-ba:+d})")
    print(f"  state transitions: {len(tr_a)} -> {len(tr_b)}"
          f"   ({len(tr_b)-len(tr_a):+d})   <-- fewer usually means less flicker")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("record", help="capture a clip from the camera")
    r.add_argument("name")
    r.add_argument("--seconds", type=float, default=20.0)
    r.add_argument("--camera", type=int, default=0)
    r.add_argument("--width", type=int, default=ld.FRAME_W)
    r.add_argument("--height", type=int, default=ld.FRAME_H)
    r.add_argument("--backend", choices=("dshow", "msmf", "any"), default="dshow")
    r.add_argument("--lock-exposure", action="store_true")
    r.add_argument("--overwrite", action="store_true")
    r.add_argument("--note", default="")
    r.set_defaults(func=record)

    sub.add_parser("list", help="list recorded clips").set_defaults(func=list_clips)

    g = sub.add_parser("run", help="replay a clip through the pipeline")
    g.add_argument("name")
    g.add_argument("--block-seconds", type=float)
    g.add_argument("--release-seconds", type=float)
    g.add_argument("--occupy-blob-px", type=int)
    g.add_argument("--stability", type=int,
                   help="frames a blob must persist in place (0 disables)")
    g.add_argument("--no-gradient", action="store_true",
                   help="disable the edge-structure channel")
    g.add_argument("--no-shadow-reject", action="store_true")
    g.add_argument("--no-photometric", action="store_true")
    g.add_argument("--csv", action="store_true")
    g.set_defaults(func=cmd_run)

    b = sub.add_parser("ab", help="compare defaults against modified params")
    b.add_argument("name")
    b.add_argument("--block-seconds", type=float)
    b.add_argument("--release-seconds", type=float)
    b.add_argument("--occupy-blob-px", type=int)
    b.add_argument("--stability", type=int)
    b.set_defaults(func=cmd_ab)

    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
