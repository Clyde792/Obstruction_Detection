/*
 * lane_leds_esp32.ino
 * ESP32-C3 Super Mini firmware for the harbour two-lane obstruction signaller.
 *
 * 4 indicator positions, each with its own red + green LED (8 LEDs total).
 * Positions 1-2 belong to LANE A, positions 3-4 belong to LANE B.
 *
 * Serial protocol (single char, newline optional):
 *   '0'  both lanes GREEN
 *   '1'  lane A RED, lane B GREEN     (lane A blocked)
 *   '2'  lane A GREEN, lane B RED     (lane B blocked)
 *   '3'  both lanes RED               (both blocked / fault / idle)
 *
 * Diagnostics (see DIAGNOSTIC MODE below):
 *   'T'  run the pin self-test        (needs no LEDs and no multimeter)
 *   'A'-'H'  drive one pin HIGH alone (walk the LEDs once wired)
 *   'X'  all outputs off
 *   '?'  print the pin map
 *
 * Fail-safe: no valid command for TIMEOUT_MS -> everything RED, greens off.
 *
 * ---------------------------------------------------------------------------
 * BOARD SETTINGS
 *   Built with PlatformIO: see ../platformio.ini. The two build flags there
 *   (ARDUINO_USB_MODE=1, ARDUINO_USB_CDC_ON_BOOT=1) are what put Serial on
 *   the native USB. In the Arduino IDE the equivalent is board
 *   "ESP32C3 Dev Module" with "USB CDC On Boot: Enabled".
 *
 * ---------------------------------------------------------------------------
 * ESP32-C3 PIN NOTES -- the C3 is NOT the classic ESP32. It only has
 * GPIO 0-10 and 18-21.
 *
 *   Avoided here:  2, 8, 9    strapping pins (boot mode / bootlog)
 *                  18, 19     native USB D- / D+
 *                  20, 21     UART0 RX / TX
 *                  8          also the onboard LED on most Super Minis
 *
 *   GPIO 4,5,6,7 double as the JTAG pins (MTMS/MTDI/MTCK/MTDO). Harmless as
 *   plain outputs; only a problem if you later want hardware JTAG debugging.
 *
 *   Because 2, 8, 9, 20 and 21 are unused, they are the right pins to
 *   practise your first solder joints on -- a bad joint there breaks nothing.
 *
 * ---------------------------------------------------------------------------
 * WIRING: one resistor per LED, LED cathode (short leg / flat side) to GND.
 *   GPIO --[ R ]--|>|-- GND
 *
 * RESISTOR VALUE: at 3.3V an 820R gives only about (3.3 - 2.0) / 820 =
 * ~1.6 mA. That lights a modern high-brightness LED but looks dim. 150R-330R
 * (roughly 4-9 mA) suits 3.3V better. You need 8 resistors, one per LED.
 *
 * ---------------------------------------------------------------------------
 * DIAGNOSTIC MODE
 * 'T' runs a self-test that needs no external hardware at all. It uses the
 * C3's own internal pullups and pulldowns:
 *
 *   Stuck-pin check -- with an internal pullup a healthy floating pin reads
 *   HIGH; with an internal pulldown it reads LOW. A pin that will not follow
 *   is shorted to GND or to 3V3.
 *
 *   Bridge check -- drive one pin HIGH as an output while every other pin is
 *   an input with its pulldown on. Any other pin that reads HIGH is joined to
 *   the driven one by a solder bridge. This is the defect that actually
 *   happens when soldering 2.54mm headers, and it is invisible to the eye
 *   under the plastic.
 *
 * The watchdog is suspended while in diagnostic mode, or it would drag the
 * pins back to all-red mid-measurement. Diagnostic mode auto-exits to the
 * safe state after DIAG_TIMEOUT_MS, and any of '0'-'3' leaves it immediately.
 */

// Position:              1   2   3   4
const int RED_PINS[]   = { 0,  3,  5,  7};
const int GREEN_PINS[] = { 1,  4,  6, 10};

// Which lane each position belongs to (0 = lane A, 1 = lane B)
const int LANE_OF[]    = { 0,  0,  1,  1};

const int NUM_POS = 4;

// Every driven pin, in 'A'..'H' order, for the diagnostics.
const int ALL_PINS[]   = { 0,  1,  3,  4,  5,  6,  7, 10};
const char *PIN_LABEL[] = {"pos1 RED", "pos1 GRN", "pos2 RED", "pos2 GRN",
                           "pos3 RED", "pos3 GRN", "pos4 RED", "pos4 GRN"};
const int NUM_PINS = 8;

const unsigned long TIMEOUT_MS = 2000;
const unsigned long DIAG_TIMEOUT_MS = 120000;

// Set to 1 to make the fail-safe blink red instead of holding solid red, so a
// dead serial link is visually distinct from a commanded '3'. Greens stay off
// either way, so it is fail-safe in both modes.
#define FAULT_BLINK 1
const unsigned long BLINK_MS = 400;

unsigned long lastMsg   = 0;
bool inFault            = true;   // start faulted: laptop has not spoken yet
bool blinkPhase         = false;
unsigned long lastBlink = 0;

bool diagMode           = false;
unsigned long diagUntil = 0;

// ---------------------------------------------------------------------------
// Normal operation
// ---------------------------------------------------------------------------

// laneRed[0] = lane A shows red?   laneRed[1] = lane B shows red?
void setLanes(bool laneRedA, bool laneRedB) {
  bool laneRed[2] = {laneRedA, laneRedB};
  for (int i = 0; i < NUM_POS; i++) {
    bool isRed = laneRed[LANE_OF[i]];
    digitalWrite(RED_PINS[i],   isRed ? HIGH : LOW);
    digitalWrite(GREEN_PINS[i], isRed ? LOW  : HIGH);
  }
}

void allOff() {
  for (int i = 0; i < NUM_POS; i++) {
    digitalWrite(RED_PINS[i], LOW);
    digitalWrite(GREEN_PINS[i], LOW);
  }
}

// All reds on, all greens off -- the fail-safe state.
void allRed(bool redsOn) {
  for (int i = 0; i < NUM_POS; i++) {
    digitalWrite(RED_PINS[i], redsOn ? HIGH : LOW);
    digitalWrite(GREEN_PINS[i], LOW);
  }
}

void allOutputs() {
  for (int i = 0; i < NUM_PINS; i++) {
    pinMode(ALL_PINS[i], OUTPUT);
    digitalWrite(ALL_PINS[i], LOW);
  }
}

void enterDiag() {
  diagMode = true;
  diagUntil = millis() + DIAG_TIMEOUT_MS;
}

void leaveDiag() {
  if (!diagMode) return;
  diagMode = false;
  allOutputs();
  allRed(true);
  Serial.println("diag: exit -> safe state (all red)");
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------
void printPinMap() {
  Serial.println("--- pin map ---");
  for (int i = 0; i < NUM_PINS; i++) {
    Serial.printf("  %c = GPIO %-2d  %s\n", 'A' + i, ALL_PINS[i], PIN_LABEL[i]);
  }
}

void drivePin(int idx) {
  enterDiag();
  allOutputs();
  digitalWrite(ALL_PINS[idx], HIGH);
  Serial.printf("diag: GPIO %d HIGH (%s), all others LOW\n",
                ALL_PINS[idx], PIN_LABEL[idx]);
}

void selfTest() {
  enterDiag();
  int problems = 0;

  Serial.println("=== pin self-test ===");
  Serial.println("-- stuck-pin check (internal pullup then pulldown) --");
  for (int i = 0; i < NUM_PINS; i++) {
    int p = ALL_PINS[i];
    pinMode(p, INPUT_PULLUP);
    delay(4);
    int hi = digitalRead(p);
    pinMode(p, INPUT_PULLDOWN);
    delay(4);
    int lo = digitalRead(p);

    if (hi == HIGH && lo == LOW) {
      Serial.printf("  GPIO %-2d %-9s OK\n", p, PIN_LABEL[i]);
    } else if (hi == LOW) {
      Serial.printf("  GPIO %-2d %-9s STUCK LOW  - shorted to GND?\n",
                    p, PIN_LABEL[i]);
      problems++;
    } else {
      Serial.printf("  GPIO %-2d %-9s STUCK HIGH - shorted to 3V3?\n",
                    p, PIN_LABEL[i]);
      problems++;
    }
  }

  Serial.println("-- bridge check (drive one pin, read the rest) --");
  for (int i = 0; i < NUM_PINS; i++) {
    for (int j = 0; j < NUM_PINS; j++) {
      if (j != i) pinMode(ALL_PINS[j], INPUT_PULLDOWN);
    }
    pinMode(ALL_PINS[i], OUTPUT);
    digitalWrite(ALL_PINS[i], HIGH);
    delay(6);

    for (int j = 0; j < NUM_PINS; j++) {
      if (j == i) continue;
      if (digitalRead(ALL_PINS[j]) == HIGH) {
        Serial.printf("  BRIDGE: GPIO %d (%s) <-> GPIO %d (%s)\n",
                      ALL_PINS[i], PIN_LABEL[i], ALL_PINS[j], PIN_LABEL[j]);
        problems++;
      }
    }
    digitalWrite(ALL_PINS[i], LOW);
    pinMode(ALL_PINS[i], INPUT_PULLDOWN);
  }

  if (problems == 0) {
    Serial.println("RESULT: all 8 pins healthy, no bridges found.");
  } else {
    Serial.printf("RESULT: %d problem(s) found -- see above.\n", problems);
  }
  Serial.println("=== self-test done ===");

  allOutputs();
  allRed(true);
}

// ---------------------------------------------------------------------------
void setup() {
  allOutputs();
  allRed(true);                  // safe state from the very first instruction

  Serial.begin(115200);
  delay(300);

  // Startup sweep: walks each position red then green so you can immediately
  // spot a dead LED, a backwards LED, or a swapped pair.
  for (int i = 0; i < NUM_POS; i++) {
    allOff();
    digitalWrite(RED_PINS[i], HIGH);
    delay(180);
    digitalWrite(RED_PINS[i], LOW);
    digitalWrite(GREEN_PINS[i], HIGH);
    delay(180);
  }
  allOff();
  delay(200);

  allRed(true);                  // hold safe until the laptop actually talks
  inFault = true;
  lastMsg = millis();
  lastBlink = millis();
  Serial.println("ready");
}

void loop() {
  // ---- drain the input buffer; only recognised chars feed the watchdog ----
  while (Serial.available()) {
    char c = Serial.read();
    bool valid = true;

    if (c >= 'a' && c <= 'z') c -= 32;      // accept lower case

    switch (c) {
      case '0': leaveDiag(); setLanes(false, false); break;
      case '1': leaveDiag(); setLanes(true,  false); break;
      case '2': leaveDiag(); setLanes(false, true ); break;
      case '3': leaveDiag(); setLanes(true,  true ); break;
      case 'T': selfTest();          valid = false; break;
      case 'X': enterDiag(); allOutputs();
                Serial.println("diag: all outputs off"); valid = false; break;
      case '?': printPinMap();       valid = false; break;
      default:
        if (c >= 'A' && c < 'A' + NUM_PINS) {
          drivePin(c - 'A');
        }
        valid = false;                      // ignore \n, \r, noise
        break;
    }
    if (valid) {
      lastMsg = millis();
      if (inFault) {
        inFault = false;
        Serial.println("link up");
      }
    }
  }

  // ---- diagnostic mode suspends the watchdog, but not forever ----
  if (diagMode) {
    if ((long)(millis() - diagUntil) >= 0) {
      Serial.println("diag: timed out");
      leaveDiag();
      lastMsg = millis();
    }
    return;
  }

  // ---- watchdog: unsigned subtraction, so millis() rollover is safe ----
  if (millis() - lastMsg > TIMEOUT_MS) {
    if (!inFault) {
      inFault = true;
      Serial.println("FAULT serial silent -> all red");
      allRed(true);
      blinkPhase = true;
      lastBlink = millis();
    }
#if FAULT_BLINK
    if (millis() - lastBlink >= BLINK_MS) {
      lastBlink = millis();
      blinkPhase = !blinkPhase;
      allRed(blinkPhase);
    }
#endif
  }
}
