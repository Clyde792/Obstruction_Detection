"""
led_test.py -- exercise the ESP32-C3 link without the camera, and without LEDs.

The firmware talks back over USB serial ("ready", "link up", "FAULT ..."), so
the whole protocol and the watchdog can be proven before a single LED exists.
Wire the LEDs later and re-run to check each position physically.

    python led_test.py --list                  which COM ports exist
    python led_test.py --port COM8             automatic sequence (default)
    python led_test.py --port COM8 --selftest  test the 8 GPIO pins
    python led_test.py --port COM8 --walk      light each pin in turn
    python led_test.py --port COM8 --pin C     hold one pin HIGH
    python led_test.py --port COM8 --interactive   drive it by hand

--selftest needs NO LEDs and NO multimeter. The C3 tests its own pins using
internal pullups and pulldowns: it reports any pin stuck to GND or 3V3, and
any pair of pins joined by a solder bridge. Run it after soldering headers.

--walk and --pin drive one pin at a time, for checking LEDs once wired (or
for probing pads with a multimeter).

Automatic sequence walks 0 -> 1 -> 2 -> 3, then deliberately goes silent to
prove the fail-safe fires, then resumes.
"""

import argparse
import sys
import time

import serial
from serial.tools import list_ports

PROTOCOL = {
    "0": "both lanes GREEN",
    "1": "lane A RED, lane B GREEN",
    "2": "lane A GREEN, lane B RED",
    "3": "both lanes RED",
}


def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports at all.")
        return
    print(f"{len(ports)} serial port(s):")
    for p in ports:
        # The C3 Super Mini uses the ESP32's native USB, so it enumerates as a
        # plain USB CDC device (VID 0x303A is Espressif). Bluetooth virtual
        # ports show up here too and are NOT your board.
        vid = f"{p.vid:04X}" if p.vid is not None else "----"
        pid = f"{p.pid:04X}" if p.pid is not None else "----"
        flag = ""
        if p.vid == 0x303A:
            flag = "   <-- Espressif, this is almost certainly your C3"
        elif "bluetooth" in (p.description or "").lower():
            flag = "   (Bluetooth virtual port, not your board)"
        print(f"  {p.device:<8} VID:PID={vid}:{pid}  {p.description}{flag}")


def open_board(port, baud):
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
    return ser


# Partial line left over from the last read. USB CDC hands us whatever
# happened to be in the buffer, which routinely splits a message mid-word --
# printing each read as if it were whole turns "GPIO 10  pos4 GRN OK" into two
# broken lines. Hold the tail back until its newline actually arrives.
_rx_tail = ""


def drain(ser, prefix="  board: "):
    """Print complete lines the firmware has sent since last time."""
    global _rx_tail
    try:
        data = ser.read(4096)
    except serial.SerialException as exc:
        print(f"  [serial error] {exc}")
        return
    if not data:
        return
    _rx_tail += data.decode("utf-8", "replace")
    parts = _rx_tail.split("\n")
    _rx_tail = parts.pop()          # keep the incomplete tail for next time
    for line in parts:
        if line.strip():
            print(prefix + line.strip())


def drain_tail(prefix="  board: "):
    """Flush any trailing line that never got its newline."""
    global _rx_tail
    if _rx_tail.strip():
        print(prefix + _rx_tail.strip())
    _rx_tail = ""


def send(ser, char):
    ser.write(char.encode())
    ser.flush()
    print(f"-> sent '{char}'  ({PROTOCOL[char]})")


def hold(ser, char, seconds, keepalive=0.25):
    """Send `char` repeatedly for `seconds`, the way lane_detect.py does."""
    end = time.monotonic() + seconds
    send(ser, char)
    last = time.monotonic()
    while time.monotonic() < end:
        if time.monotonic() - last >= keepalive:
            ser.write(char.encode())
            last = time.monotonic()
        drain(ser)
        time.sleep(0.02)


def auto(ser, dwell):
    print("\n--- startup: the firmware runs its LED sweep, then holds all-red ---")
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end:
        drain(ser)
        time.sleep(0.05)

    print(f"\n--- walking the protocol, {dwell:.0f}s each ---")
    for char in ("0", "1", "2", "3"):
        hold(ser, char, dwell)

    print("\n--- back to all-green, then going SILENT to test the watchdog ---")
    hold(ser, "0", dwell)
    print("(no data for 4s; firmware should report FAULT within ~2s)")
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end:
        drain(ser)
        time.sleep(0.05)

    print("\n--- resuming; firmware should report 'link up' ---")
    hold(ser, "0", 2.0)
    print("\nDone. Leaving the board in the safe state.")
    send(ser, "3")


def interactive(ser):
    print("\nType 0/1/2/3 then Enter to send. 'w' = go silent 4s (watchdog).")
    print("'q' = quit. Anything the board says is printed as it arrives.\n")
    while True:
        drain(ser)
        try:
            line = input("cmd> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line == "q":
            break
        if line == "w":
            print("silent for 4s...")
            t_end = time.monotonic() + 4.0
            while time.monotonic() < t_end:
                drain(ser)
                time.sleep(0.05)
            continue
        if line in PROTOCOL:
            send(ser, line)
            collect(ser, 0.6)
        elif line.lower() in list("abcdefghtx?"):
            ser.write(line.encode())
            ser.flush()
            collect(ser, 6.0 if line.lower() == "t" else 1.0, "  ")
        elif line:
            print(f"  '{line}' not recognised; use 0-3, a-h, t, x, ?, w or q.")
    send(ser, "3")
    print("Left the board in the safe state.")


def collect(ser, seconds, prefix="  board: "):
    """Print everything the board says for `seconds`."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        drain(ser, prefix)
        time.sleep(0.03)
    drain_tail(prefix)


def selftest(ser):
    print("\nRunning pin self-test (no LEDs or multimeter needed)...\n")
    ser.write(b"T")
    ser.flush()
    collect(ser, 6.0, "  ")
    print("\nIf every pin says OK and no BRIDGE lines appeared, your")
    print("solder joints are electrically sound.")


def walk(ser, dwell):
    print(f"\nWalking all 8 pins, {dwell:.1f}s each.")
    print("With LEDs wired, exactly one should light at a time.\n")
    ser.write(b"?")
    ser.flush()
    collect(ser, 1.0, "  ")
    for ch in "ABCDEFGH":
        ser.write(ch.encode())
        ser.flush()
        collect(ser, dwell, "  ")
    ser.write(b"X")
    ser.flush()
    collect(ser, 0.5, "  ")


def hold_pin(ser, letter, seconds):
    letter = letter.upper()
    if letter not in "ABCDEFGH":
        raise SystemExit(f"--pin must be one of A-H, got {letter!r}")
    print(f"\nHolding pin {letter} HIGH for {seconds:.0f}s "
          f"(probe it or watch its LED)...\n")
    ser.write(letter.encode())
    ser.flush()
    collect(ser, seconds, "  ")
    ser.write(b"X")
    ser.flush()
    collect(ser, 0.5, "  ")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    ap.add_argument("--interactive", action="store_true")
    ap.add_argument("--selftest", action="store_true",
                    help="check the 8 GPIO pins for stuck pins and bridges")
    ap.add_argument("--walk", action="store_true",
                    help="drive each of the 8 pins HIGH in turn")
    ap.add_argument("--pin", default=None, metavar="A-H",
                    help="hold one pin HIGH so you can probe or watch it")
    ap.add_argument("--hold", type=float, default=5.0,
                    help="seconds for --pin (default 5)")
    ap.add_argument("--dwell", type=float, default=2.0,
                    help="seconds to hold each state in the automatic sequence")
    args = ap.parse_args()

    if args.list or not args.port:
        list_serial_ports()
        if not args.port:
            print("\nGive --port COMx to run the test.")
        return

    try:
        ser = open_board(args.port, args.baud)
    except serial.SerialException as exc:
        raise SystemExit(f"Could not open {args.port}: {exc}")

    print(f"Opened {args.port} at {args.baud}.")
    # The board reboots when the port opens regardless of DTR/RTS; give it a
    # moment or the startup banner is missed.
    time.sleep(2.0)

    try:
        if args.selftest:
            selftest(ser)
        elif args.walk:
            walk(ser, args.dwell)
        elif args.pin:
            hold_pin(ser, args.pin, args.hold)
        elif args.interactive:
            interactive(ser)
        else:
            auto(ser, args.dwell)
    finally:
        try:
            ser.write(b"3")
            ser.flush()
        finally:
            ser.close()


if __name__ == "__main__":
    main()
