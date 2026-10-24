/*****************************************************************
  MESHMERIZE v9 - ESP32-S3          (loops + end box in the middle, LQR actual run)

  v9 = v8 dry run (PID line following + map + Dijkstra shortest path, unchanged)
     + ACTUAL RUN WITH LQR: no stopping at turns. The bot slows to vTurnMM, rolls, then drives a
       tight arc whose end is measured with the encoders (yaw), and hands back to the LQR on the new
       line. Straights: LQR on [line error, heading error] with an observer + wheel speed PI.
     RUN button (GPIO 7)        -> LQR actual run
     MAIN button, 3rd press     -> old PID actual run (fallback)
     Serial 'l' prints the LQR gain table.  MEASURE the constants in the LQR section first!

  What changed from v7
    The right-hand rule only works when the maze has no loops. The new arena has
    loops (the two squares at the bottom left, the cells on the right), so the
    dry run now builds a MAP instead:
      - every node (junction / corner / dead end) becomes a map node
      - its position is dead-reckoned from the encoders + the 90 deg turns
      - arriving at a node that is already on the map through a NEW line = a loop
        was closed -> the two are merged (see matchTolMM)
      - the bot explores using LEFT-HAND RULE (Left > Straight > Right); when a node
        is finished it drives over the known map via Dijkstra to the nearest node that
        still has an unexplored arm
      - dead ends reverse straight back to the junction with encoder lock (no stalling)
      - when all frontiers are explored (or end box found): Dijkstra over the map =
        absolute shortest path (distance + turnPenaltyMM per turn), converted back into
        the move/distance list for the actual run.
    The actual run is UNCHANGED.

  Dry run : explores until the end box is found, then builds the shortest path
  Run     : replays the shortest path, full speed on straights, slows only before turns
  Brakes  : both IN pins HIGH on each DRV8833 channel, then waits until the encoders
            confirm the wheels have stopped

  BUTTON GPIO 5 (main):
    press 1 -> calibrate  (slide the bot sideways over the line for 5 s)
    press 2 -> dry run
    press 3 -> actual run, OLD PID version (fallback)
    long press (>1.5 s)     -> forget everything (path too)
    hold at power-on        -> reload the saved path from flash
  BUTTON GPIO 7 (run):
    short press -> ACTUAL RUN, LQR version (needs a calibration + a recorded path)
    long press  -> re-calibrate only, the path is kept
  Serial monitor 115200 (robot idle):  m = motor/encoder test
                                       d = distance test (push 500 mm by hand)
                                       b = test straight reverse (200 mm)
                                       p = print the recorded path
                                       g = print the map built by the last dry run
                                       j = run Dijkstra shortest path on current map
                                       l = print the LQR gain table
                                       s = live sensor values (calibrate first)
*****************************************************************/
#include <Arduino.h>
#include <Preferences.h>

// defined first so the Arduino IDE's auto-prototypes can see it
struct Scan { bool left, right, straight, end, early; long stopOdo; };

// ==========================================
// 🎛️ TUNING DASHBOARD
// ==========================================
bool isWhiteLine = true;

// PID.  Kp/Kd = dry run + approach to turns.  KpRun/KdRun = fast straights
float Kp = 6.2,    Kd = 24.5;
float KpRun = 6.2, KdRun = 24.5;
const unsigned long LOOP_US = 5500;      // fixed PID period

// speeds (0-255)
int baseSpeed     = 75;     // DRY RUN speed
int runSpeed      = 150;    // ACTUAL RUN top speed on straights
int approachSpeed = 90;     // actual run: launch speed after a turn (ramps up to runSpeed from here)
int inchSpeed     = 85;     // creeping onto a junction
int turnSpeed     = 75;     // 90 deg pivot speed
int uTurnSpeed    = 70;     // 180 deg spin at dead ends (v7 tuning)
int crawlSpeed    = 60;     // last few ticks of the creep: slow, so the stop is exact (not below minPWM)
int crawlTicks    = 15;
int alignSpeed    = 80;     // tiny corrective pulses after a pivot
int alignPulseMs  = 10;
int alignTol      = 8;      // |error| (weights units) accepted as centred
int minPWM        = 55;     // lowest PWM that reliably starts a wheel on the floor ('m' test)

// geometry - MEASURE (v7 proven numbers)
float TICKS_PER_MM    = 1.5;   // 'd' test: push 500 mm, divide ticks by 500
int   ticksInchForward = 70;   // creep from "bar seen" to axle-over-junction-centre (v7 value)
int   brakeLeadTicks = 40;     // brake this many ticks EARLY at junctions (v7 value)
int   cornerTrimTicks = 8;     // dry run: when nothing continues straight (a corner), stop the creep this many ticks
                               // earlier so the turn starts sooner. 0 = off. Keep <= ~12: the pivot still has to
                               // happen close to the junction centre or it will not catch the new arm.

// ---- MAP & EXPLORATION ----
float matchTolMM    = 110;     // arriving through a NEW line at a node that is already on the map: same node if it is
                               // within this distance (dead-reckoning error). Loop closure tolerance.
float turnPenaltyMM = 200;     // shortest path: every turn counts like this many extra mm (turns cost time)
bool  useLeftHandRule  = true; // true = LEFT-HAND RULE (Left > Straight > Right)
                               // false = RIGHT-HAND RULE (Right > Straight > Left)

// ---- ACTUAL RUN: arrive at every turn exactly like the dry run did ----
int   turnEntrySpeed = 75;     // speed when the junction arrives (keep = baseSpeed; lower = even less slide)
float brakeZoneMM = 250;       // start slowing this far before the junction ...
float slowZoneMM  = 40;        // ... and hold turnEntrySpeed for the last this-far
// ---- ACTUAL RUN TURN, DEFAULT: brake the moment the junction is detected, then crawl to the centre ----
bool  brakeAtNode  = true;     // false = old rolling creep (uses runLeadTicks)
int   runStopLead  = 5;        // ticks the bot coasts after the final crawl-brake. Passes the centre -> raise (8, 12 ...). Stops short -> lower.
int   runLeadTicks = 30;       // only for brakeAtNode = false: stop the rolling creep this many ticks early
// ---- ACTUAL RUN FAST TURN: turn the moment the junction is seen, no stop, no brake ----
bool  arcTurn        = false;
int   arcRollTicks   = 15;     // roll straight this many ticks before arcing (more = turn later / wider)
int   arcOuterSpeed  = 90;     // outer wheel
int   arcInnerSpeed  = 0;      // inner wheel: 0 = locked (tightest), >0 = wider arc (not below minPWM), <0 = spins tighter
int   arcMinMs       = 90;     // arc at least this long before looking for the new line
int   arcTimeoutMs   = 900;    // no line caught by then = give up (red LED)
int   arcExitTicks   = 60;     // how far along the new leg the bot already is when the arc ends (distance bookkeeping only)
float accelZoneMM = 100;       // actual run: ramp up over this distance after a turn
float windowMM    = 50;        // node accepted when within this of its recorded distance
float startLockMM = 60, turnLockMM = 40, passLockMM = 60;
int   endMinLit   = 6;         // END BOX = at least this many of the 8 sensors white (tolerates a weak sensor)
float endMMDry    = 40;        // ... for this far in the dry run (must be > the 30 mm junction bars)
float endMMRun    = 15;        // ... for this far in the actual run (final leg has no bars)
int   minSpin90   = 120;       // pivot: spin blind this long before looking for the new line (ms)
int   minSpin180  = 340;       // same for dead-end U-turn (tuned to turn all the way around)
int   lostMs      = 100;       // line gone this long = dead end
int   turnTimeoutMs = 2500;

const unsigned long DRY_TIMEOUT_MS = 240000;   // v8: exploring a maze with loops can take longer than a tree
const unsigned long RUN_TIMEOUT_MS = 150000;
const int PWM_FREQ = 20000;
const int MIN_SPAN = 400;          // calibration: each sensor must see white AND dark by this much

// ==========================================
// 🔌 PINS & GLOBALS
// ==========================================
const int BTN_PIN = 5;
const int RUN_BTN_PIN = 7;         // actual-run button
const int LED_PIN = 47;            // red end-zone LED

const int irPins[8] = {2, 3, 4, 6, 9, 10, 11, 12};
int sensorMin[8], sensorMax[8], thresholds[8];
int weights[8] = {-40, -30, -20, -10, 10, 20, 30, 40};

const int IN1 = 13; const int IN2 = 16;   // left  (IN1 forward, IN2 reverse)
const int IN3 = 17; const int IN4 = 15;   // right (IN4 forward, IN3 reverse)

const int LEFT_ENC_A = 39; const int LEFT_ENC_B = 40;
const int RIGHT_ENC_A = 41; const int RIGHT_ENC_B = 42;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
int leftDir = 1, rightDir = 1;     // auto-detected in calibration (forward = positive)

bool seesLine[8];
bool anyLit, allLit, nodeL, nodeR;
int  litCount = 0;
int  lastError = 0;
unsigned long lostSince = 0;

bool calibrated = false, dryDone = false;

#define MM(x) ((long)((x) * TICKS_PER_MM))

void rgb(uint8_t r, uint8_t g, uint8_t b) { neopixelWrite(RGB_BUILTIN, r, g, b); }
// blue = calibrating | green = dry run | yellow = at a junction | magenta = U-turn
// cyan/white = actual run | red = something timed out / failed

// ==========================================
// ⚙️ ENCODERS
// ==========================================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_A) == digitalRead(LEFT_ENC_B)) leftTicks++; else leftTicks--;
}
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_A) == digitalRead(RIGHT_ENC_B)) rightTicks++; else rightTicks--;
}
// forward progress of the bot (average of both wheels, forward = +)
long odo() { return ((long)leftTicks * leftDir + (long)rightTicks * rightDir) / 2; }

// ==========================================
// 🤖 MOTORS  (slow decay, hard brake)
// ==========================================
// Forward : IN(fwd) HIGH, IN(rev) = inverted PWM.  The PWM off-time is BOTH pins
// HIGH = DRV8833 brake, so the motor is never left coasting.
void driveMotor(int fwdPin, int revPin, int speed) {
  if (speed == 0) { ledcWrite(fwdPin, 255); ledcWrite(revPin, 255); return; }
  int mag = abs(speed);
  if (mag > 255) mag = 255;
  if (mag < minPWM) mag = minPWM;
  if (speed > 0) { ledcWrite(fwdPin, 255); ledcWrite(revPin, 255 - mag); }
  else           { ledcWrite(revPin, 255); ledcWrite(fwdPin, 255 - mag); }
}
void setMotors(int l, int r) { driveMotor(IN1, IN2, l); driveMotor(IN4, IN3, r); }

// IN1 = IN2 = HIGH and IN3 = IN4 = HIGH
void brakeMotors() {
  ledcWrite(IN1, 255); ledcWrite(IN2, 255);
  ledcWrite(IN3, 255); ledcWrite(IN4, 255);
}

// hard brake, then WAIT until the encoders say the wheels really stopped
void stopAndSettle(int maxMs = 300) {
  brakeMotors();
  unsigned long t0 = millis(), tQuiet = millis();
  long l = leftTicks, r = rightTicks;
  while (millis() - t0 < (unsigned long)maxMs) {
    delay(1);
    if (leftTicks != l || rightTicks != r) { l = leftTicks; r = rightTicks; tQuiet = millis(); }
    else if (millis() - tQuiet >= 15) break;
  }
}

// ==========================================
// 👁️ SENSORS
// ==========================================
void readSensors() {
  anyLit = false; allLit = true; litCount = 0;
  for (int i = 0; i < 8; i++) {
    int val = analogRead(irPins[i]);
    seesLine[i] = isWhiteLine ? (val < thresholds[i]) : (val > thresholds[i]);
    if (seesLine[i]) { anyLit = true; litCount++; } else allLit = false;
  }
  nodeL = seesLine[0] && seesLine[1];
  nodeR = seesLine[6] && seesLine[7];
}

// ---- END BOX: a wide white area under the array for a few mm ----
long endFrom = -1;
int  endMiss = 0;
void endReset() { endFrom = -1; endMiss = 0; }
bool endCheck(float needMM) {
  if (litCount >= endMinLit) {
    endMiss = 0;
    if (endFrom < 0) endFrom = odo();
  } else if (endFrom >= 0 && ++endMiss > 4) {
    endFrom = -1; endMiss = 0;
  }
  return endFrom >= 0 && (odo() - endFrom) >= MM(needMM);
}

// ==========================================
// 🧠 PID LINE FOLLOW
// ==========================================
void pidStep(int speed, bool centerOnly, float kp, float kd) {
  static unsigned long tLoop = 0;
  while (micros() - tLoop < LOOP_US) { }
  tLoop = micros();

  long ws = 0; int n = 0;
  int a = centerOnly ? 2 : 0, b = centerOnly ? 6 : 8;
  for (int i = a; i < b; i++) if (seesLine[i]) { ws += weights[i]; n++; }
  if (n == 0 && centerOnly) {                       // centre blind: fall back to all sensors
    for (int i = 0; i < 8; i++) if (seesLine[i]) { ws += weights[i]; n++; }
  }
  int error = n ? (int)(ws / n) : lastError;
  int P = error;
  int D = error - lastError;
  lastError = error;
  int correction = (int)(kp * P + kd * D);
  setMotors(speed + correction, speed - correction);
}

// ==========================================
// 🔄 TURNS (MESHMERIZE v7 Proven Turning Logic)
// ==========================================
// Spin until the CURRENT line has left the two centre sensors, then until the
// NEXT line touches them. For a 90 deg turn that is the new arm, for a dead
// end it is the line behind. Hard brake, wait for standstill, then nudge.
void alignOnLine() {
  for (int k = 0; k < 10; k++) {
    readSensors();
    long ws = 0; int n = 0;
    for (int i = 0; i < 8; i++) if (seesLine[i]) { ws += weights[i]; n++; }
    if (n == 0) return;
    int err = (int)(ws / n);
    if (abs(err) <= alignTol) return;
    int dir = err > 0 ? +1 : -1;                    // line on the right -> rotate right
    setMotors(dir * alignSpeed, -dir * alignSpeed);
    delay(alignPulseMs);
    stopAndSettle(150);
  }
}

bool pivot(int dir, int speed, int minSpinMs) {     // dir: -1 = left, +1 = right
  unsigned long t0 = millis();
  bool ok = false;
  setMotors(dir * speed, -dir * speed);
  while (millis() - t0 < (unsigned long)turnTimeoutMs) {
    readSensors();
    if (millis() - t0 >= (unsigned long)minSpinMs) {
      if (seesLine[3] || seesLine[4]) { ok = true; break; }   // catch the line's EDGE after leaving the old line
    }
    delay(2);
  }
  stopAndSettle();
  if (ok) alignOnLine(); else rgb(80, 0, 0);
  return ok;
}

// v8/v9: pivot that COUNTS lines. Used for a 180 at a junction that has side arms:
bool pivotCount(int dir, int speed, int minSpinMs, int need) {
  unsigned long t0 = millis();
  bool ok = false, dark = false;
  int seen = 0;
  setMotors(dir * speed, -dir * speed);
  while (millis() - t0 < (unsigned long)turnTimeoutMs) {
    readSensors();
    if (millis() - t0 >= (unsigned long)minSpinMs) {
      bool c = seesLine[3] || seesLine[4];
      if (!c) dark = true;
      else if (dark) {
        dark = false;
        if (++seen >= need) { ok = true; break; }
      }
    }
    delay(2);
  }
  stopAndSettle();
  if (ok) alignOnLine(); else rgb(80, 0, 0);
  return ok;
}

bool doMove(char m) {
  int uDir = useLeftHandRule ? -1 : +1;
  switch (m) {
    case 'L': return pivot(-1, turnSpeed,  minSpin90);
    case 'R': return pivot(+1, turnSpeed,  minSpin90);
    case 'U': return pivot(uDir, uTurnSpeed, minSpin180);
    default:  return true;                          // 'S'
  }
}

// 180 at a node. arms = arms of that node (absolute), hd = heading the bot is facing.
bool uTurnAt(uint8_t arms, int hd) {
  bool hasS = (arms >> hd) & 1;
  bool hasR = (arms >> ((hd + 1) & 3)) & 1;
  bool hasL = (arms >> ((hd + 3) & 3)) & 1;
  int defaultDir = useLeftHandRule ? -1 : +1;
  if (!hasS && !hasR && !hasL) return pivot(defaultDir, uTurnSpeed, minSpin180);
  int dir  = (hasR && !hasL) ? -1 : +1;
  int need = 1 + ((dir > 0) ? (hasR ? 1 : 0) : (hasL ? 1 : 0));
  return pivotCount(dir, uTurnSpeed, minSpin90, need);
}

// ==========================================
// 📍 NODE EVENTS (what the actual run replays)
// ==========================================
const int MAX_EV = 80;
char evMove[MAX_EV];       // 'L' 'R' 'S'  (S = drive straight past a node)
long evDist[MAX_EV];       // ticks from the previous node to this one
int  evN = 0;
long endDist = 0;          // ticks from the last node to the end box

void record(char m, long dist) {
  if (evN < MAX_EV) { evMove[evN] = m; evDist[evN] = dist; evN++; }
}

void printPath() {
  Serial.print("FINAL: ");
  for (int i = 0; i < evN; i++) Serial.print(evMove[i]);
  Serial.println("   (what the actual run will do, node by node)");
  Serial.println("PATH (move, ticks from previous node):");
  for (int i = 0; i < evN; i++) Serial.printf("  %2d: %c  %ld  (%.0f mm)\n", i, evMove[i], evDist[i], evDist[i] / TICKS_PER_MM);
  Serial.printf("  end box after %ld ticks (%.0f mm)\n", endDist, endDist / TICKS_PER_MM);
}

void savePath() {
  Preferences p;
  p.begin("maze", false);
  p.putUChar("n", (uint8_t)evN);
  p.putBytes("mv", evMove, evN);
  p.putBytes("ds", evDist, evN * sizeof(long));
  p.putLong("end", endDist);
  p.end();
}
bool loadPath() {
  Preferences p;
  p.begin("maze", true);
  int n = p.getUChar("n", 0);
  if (n > 0 && n <= MAX_EV) {
    p.getBytes("mv", evMove, n);
    p.getBytes("ds", evDist, n * sizeof(long));
    endDist = p.getLong("end", 0);
  } else n = 0;
  p.end();
  evN = n;
  return n > 0;
}

// ==========================================
// 🗺️ MAP  (loops -> remember WHERE every node is)
// ==========================================
const int MAX_NODES = 48;
const long INF = 2000000000L;
const int DX[4] = {0, 1, 0, -1};
const int DY[4] = {1, 0, -1, 0};

struct MapNode {
  long    x, y;        // ticks (dead reckoned)
  uint8_t arms;        // bit h set = a line leaves this node towards heading h
  int8_t  nbr[4];      // node at the other end of arm h (-1 = never driven)
  long    len[4];      // ticks, measured the way the actual run needs them (detection to detection)
};
MapNode mapN[MAX_NODES];
int  mapCount = 0;
int  endNode = -1, endArm = 0;     // the end box hangs off this node, through this arm
long endLen = 0;
bool endBoxFound = false;

// the move needed to go from facing 'hd' to leaving through absolute arm 'arm'
char relMove(int hd, int arm) {
  switch ((arm - hd) & 3) {
    case 0:  return 'S';
    case 1:  return 'R';
    case 2:  return 'U';
    default: return 'L';
  }
}

const char* armsStr(uint8_t arms) {
  static char b[5];
  const char* n = "NESW";
  for (int i = 0; i < 4; i++) b[i] = ((arms >> i) & 1) ? n[i] : '-';
  b[4] = 0;
  return b;
}

int newNode(long x, long y) {
  if (mapCount >= MAX_NODES) { Serial.println("!! map full (MAX_NODES)"); return -1; }
  MapNode &n = mapN[mapCount];
  n.x = x; n.y = y; n.arms = 0;
  for (int a = 0; a < 4; a++) { n.nbr[a] = -1; n.len[a] = 0; }
  return mapCount++;
}

void linkEdge(int a, int armA, int b, int armB, long len) {
  mapN[a].nbr[armA] = b; mapN[a].len[armA] = len;
  mapN[b].nbr[armB] = a; mapN[b].len[armB] = len;
}

int matchNode(long x, long y, int arm, long *errTicks) {
  int best = -1;
  long bestD = MM(matchTolMM);
  for (int i = 1; i < mapCount; i++) {                // 0 = the start spot, never a loop target
    if (mapN[i].nbr[arm] >= 0) continue;
    long d = labs(mapN[i].x - x) + labs(mapN[i].y - y);
    if (!((mapN[i].arms >> arm) & 1)) d *= 2;
    if (d < bestD) { bestD = d; best = i; }
  }
  if (errTicks) *errTicks = bestD;
  return best;
}

int arrive(int from, int hd, long dist, uint8_t seen, bool allowMatch) {
  int back = (hd + 2) & 3;
  int id = mapN[from].nbr[hd];
  if (id >= 0) {                                      // driving a known line: nothing new to learn
    long expect = mapN[from].len[hd];
    if (labs(dist - expect) > MM(matchTolMM))
      Serial.printf("   (known line %d->%d: measured %.0f mm, stored %.0f mm)\n", from, id, dist / TICKS_PER_MM, expect / TICKS_PER_MM);
    mapN[id].arms |= seen | (1 << back);
    return id;
  }
  long d = dist + (from != 0 ? ticksInchForward : 0); // centre to centre
  long x = mapN[from].x + (long)DX[hd] * d;
  long y = mapN[from].y + (long)DY[hd] * d;
  long err = 0;
  id = allowMatch ? matchNode(x, y, back, &err) : -1;
  if (id >= 0) Serial.printf("   ★ [MAP LOOP CLOSED] Node %d connected to Node %d (loop closure error %.0f mm)\n", from, id, err / TICKS_PER_MM);
  else id = newNode(x, y);
  if (id < 0) return -1;
  mapN[id].arms |= seen | (1 << back);
  linkEdge(from, hd, id, back, dist);
  return id;
}

bool hasOpenArm(int i) {
  for (int a = 0; a < 4; a++) if (((mapN[i].arms >> a) & 1) && mapN[i].nbr[a] < 0) return true;
  return false;
}

int nearestFrontierArm(int from) {
  long d[MAX_NODES]; int8_t first[MAX_NODES]; bool done[MAX_NODES];
  for (int i = 0; i < mapCount; i++) { d[i] = INF; first[i] = -1; done[i] = false; }
  d[from] = 0;
  for (;;) {
    int u = -1;
    for (int i = 0; i < mapCount; i++) if (!done[i] && d[i] < INF && (u < 0 || d[i] < d[u])) u = i;
    if (u < 0) return -1;
    done[u] = true;
    if (u != from && hasOpenArm(u)) {
      Serial.printf("   [DIJKSTRA FRONTIER] Node %d all arms explored -> routing to nearest open Node %d (dist %.0f mm) via arm %c\n",
                    from, u, d[u] / TICKS_PER_MM, "NESW"[first[u]]);
      return first[u];
    }
    for (int a = 0; a < 4; a++) {
      int v = mapN[u].nbr[a];
      if (v < 0) continue;
      long nd = d[u] + mapN[u].len[a];
      if (nd < d[v]) { d[v] = nd; first[v] = (u == from) ? a : first[u]; }
    }
  }
}

// Exploration priority at junctions:
// Left-Hand Rule (LHR):  Left (3) > Straight (0) > Right (1)
// Right-Hand Rule (RHR): Right (1) > Straight (0) > Left (3)
int chooseArm(int id, int hd) {
  const int orderLHR[3] = {3, 0, 1}; // Left > Straight > Right
  const int orderRHR[3] = {1, 0, 3}; // Right > Straight > Left
  const int *order = useLeftHandRule ? orderLHR : orderRHR;

  for (int k = 0; k < 3; k++) {
    int a = (hd + order[k]) & 3;
    if (((mapN[id].arms >> a) & 1) && mapN[id].nbr[a] < 0) return a;
  }
  return nearestFrontierArm(id);
}

bool buildPath(bool verbose = true) {
  if (endNode < 0) {
    if (verbose) Serial.println("!! [DIJKSTRA] Cannot build path: End box has not been found yet!");
    return false;
  }
  const int NS = MAX_NODES * 4;
  long dist[NS]; int16_t prev[NS]; bool done[NS];
  int ns = mapCount * 4;
  for (int s = 0; s < ns; s++) { dist[s] = INF; prev[s] = -1; done[s] = false; }
  long turnCost = MM(turnPenaltyMM);
  dist[0] = 0;                                        // node 0, heading 0
  for (;;) {
    int u = -1;
    for (int s = 0; s < ns; s++) if (!done[s] && dist[s] < INF && (u < 0 || dist[s] < dist[u])) u = s;
    if (u < 0) break;
    done[u] = true;
    int n = u >> 2, h = u & 3;
    for (int a = 0; a < 4; a++) {
      if (a == ((h + 2) & 3)) continue;               // never turn round
      int v = mapN[n].nbr[a];
      if (v < 0) continue;
      long nd = dist[u] + mapN[n].len[a] + (a != h ? turnCost : 0);
      int sv = v * 4 + a;
      if (nd < dist[sv]) { dist[sv] = nd; prev[sv] = u; }
    }
  }
  int best = -1; long bestCost = INF;
  for (int h = 0; h < 4; h++) {
    int s = endNode * 4 + h;
    if (dist[s] >= INF) continue;
    long c = dist[s] + (endArm != h ? turnCost : 0);
    if (c < bestCost) { bestCost = c; best = s; }
  }
  if (best < 0) {
    if (verbose) Serial.println("!! [DIJKSTRA] No path found from Start to End box in map!");
    return false;
  }

  int chain[NS]; int m = 0;                           // chain[0] = end node ... chain[m-1] = start
  for (int s = best; s >= 0 && m < NS; s = prev[s]) chain[m++] = s;
  if (m - 1 > MAX_EV) {
    if (verbose) Serial.println("!! [DIJKSTRA] Path exceeds MAX_EV!");
    return false;
  }
  evN = 0;
  for (int i = m - 2; i >= 0; i--) {
    int h        = chain[i] & 3;                      // heading when arriving at this node
    int prevNode = chain[i + 1] >> 2;
    int out      = (i > 0) ? (chain[i - 1] & 3) : endArm;
    record(relMove(h, out), mapN[prevNode].len[h]);
  }
  endDist = endLen;

  if (verbose) {
    Serial.println("\n========================================================");
    Serial.println("  🧭 DIJKSTRA SHORTEST PATH CALCULATION");
    Serial.println("========================================================");
    Serial.printf("  Total Nodes in Map   : %d\n", mapCount);
    Serial.printf("  Start Node           : 0 (Heading N)\n");
    Serial.printf("  End Target Junction  : Node %d (Face Arm %c into End Box)\n", endNode, "NESW"[endArm]);
    Serial.printf("  Turn Penalty Weight  : %.0f mm per 90-deg turn\n", turnPenaltyMM);
    Serial.printf("  Total Path Segments  : %d nodes traversed\n", m);
    Serial.println("  --------------------------------------------------------");
    Serial.println("  STEP-BY-STEP DIJKSTRA OPTIMAL PATH:");
    float totalRoadMM = 0;
    int turnCount = 0;
    for (int i = m - 2; i >= 0; i--) {
      int currNode = chain[i] >> 2;
      int prevNode = chain[i + 1] >> 2;
      int hIn      = chain[i] & 3;
      int hOut     = (i > 0) ? (chain[i - 1] & 3) : endArm;
      char mv      = relMove(hIn, hOut);
      float segMM  = mapN[prevNode].len[hIn] / TICKS_PER_MM;
      totalRoadMM += segMM;
      if (mv != 'S') turnCount++;
      Serial.printf("   [%d] Node %2d -> %2d | Leg: %5.0f mm | Turn at Node %2d: %c (Head %c)\n",
                    (m - 2 - i) + 1, prevNode, currNode, segMM, currNode, mv, "NESW"[hOut]);
    }
    totalRoadMM += endLen / TICKS_PER_MM;
    Serial.printf("   [*] Node %2d -> END | Final Leg: %5.0f mm straight into End Box\n",
                  endNode, endLen / TICKS_PER_MM);
    Serial.println("  --------------------------------------------------------");
    Serial.printf("  Total Track Distance : %.0f mm\n", totalRoadMM);
    Serial.printf("  Total Real Turns     : %d\n", turnCount);
    Serial.printf("  Dijkstra Min Score   : %.0f (distance + penalties)\n", totalRoadMM + (turnCount * turnPenaltyMM));
    Serial.println("========================================================\n");
  }
  return true;
}

void printMap() {
  Serial.println("MAP (node: position mm, arms [N = start heading, E = right], arm>neighbour):");
  for (int i = 0; i < mapCount; i++) {
    Serial.printf("  %2d (%5.0f,%5.0f) %s ", i, mapN[i].x / TICKS_PER_MM, mapN[i].y / TICKS_PER_MM, armsStr(mapN[i].arms));
    for (int a = 0; a < 4; a++) {
      if (!((mapN[i].arms >> a) & 1)) continue;
      if (mapN[i].nbr[a] >= 0) Serial.printf(" %c>%d(%.0f)", "NESW"[a], mapN[i].nbr[a], mapN[i].len[a] / TICKS_PER_MM);
      else                     Serial.printf(" %c>?", "NESW"[a]);
    }
    Serial.println();
  }
  if (endNode >= 0) Serial.printf("  end box: node %d, arm %c, %.0f mm\n", endNode, "NESW"[endArm], endLen / TICKS_PER_MM);
}

// ==========================================
// 🚦 JUNCTION HANDLING
// ==========================================
long lockRef = 0, lockLen = 0;
void startLock(float mm) { lockRef = odo(); lockLen = MM(mm); }

Scan creepScan() {
  Scan s = {false, false, false, false, false, 0};
  long ref = odo();
  long target = ticksInchForward - brakeLeadTicks;
  long cornerTarget = target - cornerTrimTicks;
  unsigned long t0 = millis();
  setMotors(inchSpeed, inchSpeed);
  while (millis() - t0 < 3000) {
    readSensors();
    long d = odo() - ref;
    if (nodeL) s.left = true;
    if (nodeR) s.right = true;
    if (endCheck(endMMDry)) { s.end = true; break; }
    bool wide = litCount >= endMinLit;
    if (cornerTrimTicks > 0 && !wide && d >= cornerTarget && !seesLine[3] && !seesLine[4]) { s.early = true; break; }
    if (d >= target && !wide) break;
    if (d > ticksInchForward + MM(endMMDry + 80)) break;
    int sp = (!wide && d >= cornerTarget - crawlTicks) ? crawlSpeed : inchSpeed;
    setMotors(sp, sp);
  }
  stopAndSettle();
  s.stopOdo = odo();
  if (!s.end) { readSensors(); s.straight = seesLine[3] || seesLine[4]; }
  return s;
}

void creepOnly(int lead) {
  long ref = odo();
  unsigned long t0 = millis();
  long target = ticksInchForward - lead;
  while (millis() - t0 < 2000) {
    long d = odo() - ref;
    if (d >= target) break;
    int sp = (d >= target - crawlTicks) ? crawlSpeed : inchSpeed;
    setMotors(sp, sp);
  }
  stopAndSettle();
}

void creepTo(long targetOdo) {
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    long left = targetOdo - odo();
    if (left <= 0) break;
    int sp = (left <= crawlTicks) ? crawlSpeed : inchSpeed;
    setMotors(sp, sp);
  }
  stopAndSettle();
}

// Drive backward straight with closed-loop encoder steering back to targetOdo (used at dead ends and end box exploration)
void reverseTo(long targetOdo, int revSpd = 80) {
  long startL = (long)leftTicks * leftDir;
  long startR = (long)rightTicks * rightDir;
  unsigned long t0 = millis();

  while (odo() > targetOdo && millis() - t0 < 3500) {
    long curL = (long)leftTicks * leftDir;
    long curR = (long)rightTicks * rightDir;
    long traveledL = labs(curL - startL);
    long traveledR = labs(curR - startR);

    // Keep both wheels moving backward symmetrically
    long diff = traveledL - traveledR;
    int steer = constrain((int)(diff * 2), -25, 25);

    int pwmL = revSpd - steer;
    int pwmR = revSpd + steer;
    setMotors(-pwmL, -pwmR);
    delay(2);
  }
  brakeMotors();
  stopAndSettle(80);
  alignOnLine();
}

bool arcTurnMove(char mv) {
  int dir = (mv == 'R') ? +1 : -1;
  long ref = odo();
  unsigned long t0 = millis();
  while (odo() - ref < arcRollTicks && millis() - t0 < 1000) setMotors(turnEntrySpeed, turnEntrySpeed);
  t0 = millis();
  if (dir > 0) setMotors(arcOuterSpeed, arcInnerSpeed); else setMotors(arcInnerSpeed, arcOuterSpeed);
  bool wasDark = false, ok = false;
  while (millis() - t0 < (unsigned long)arcTimeoutMs) {
    readSensors();
    if (!seesLine[3] && !seesLine[4]) wasDark = true;
    else if (wasDark && millis() - t0 >= (unsigned long)arcMinMs) { ok = true; break; }
  }
  if (!ok) { stopAndSettle(); rgb(80, 0, 0); return false; }
  long ws = 0; int n = 0;
  for (int i = 0; i < 8; i++) if (seesLine[i]) { ws += weights[i]; n++; }
  lastError = n ? (int)(ws / n) : 0;
  return true;
}

void endReached() {
  brakeMotors();
  digitalWrite(LED_PIN, HIGH);        // required red LED
  rgb(255, 0, 0);
}
void failed() { brakeMotors(); rgb(80, 0, 0); }

// ==========================================
// 🗺️ DRY RUN - EXPLORE THE MAP, THEN SHORTEST PATH
// ==========================================
bool finishDry(int from, int hd, long lenTicks) {
  endNode = from; endArm = hd; endLen = lenTicks;
  endReached();
  Serial.printf("\n========================================================\n");
  Serial.printf("  🎯 [DRY RUN COMPLETE] White End Box at Node %d, Arm %c (%.0f mm)!\n",
                endNode, "NESW"[endArm], endLen / TICKS_PER_MM);
  Serial.printf("========================================================\n");
  printMap();
  if (!buildPath(true)) { Serial.println("!! [DIJKSTRA ERROR] No path found from start to end box!"); failed(); return false; }
  savePath();
  printPath();
  return true;
}

bool dryRun() {
  mapCount = 0; evN = 0; endDist = 0; endNode = -1;
  digitalWrite(LED_PIN, LOW);
  rgb(0, 40, 0);
  lastError = 0; lostSince = 0;
  int cur = newNode(0, 0);             // node 0 = the start spot, one arm: straight ahead
  mapN[cur].arms = 1;
  int hd = 0;                          // absolute heading
  long segRef = odo();                 // position of the previous node (axle over its centre)
  startLock(startLockMM);
  endReset();
  unsigned long t0 = millis();

  while (millis() - t0 < DRY_TIMEOUT_MS) {
    readSensors();
    bool locked = (odo() - lockRef) < lockLen;

    // 1. WHITE END BOX (wide white area) -> Finished!
    if (endCheck(endMMDry)) return finishDry(cur, hd, endFrom - segRef);

    // 2. JUNCTION DETECTION (nodeL || nodeR)
    if (!locked && (nodeL || nodeR)) {
      rgb(80, 60, 0);
      long det  = odo();
      long dist = det - segRef;
      Scan s = creepScan();
      if (s.end) return finishDry(cur, hd, (endFrom >= 0 ? endFrom : det) - segRef);

      uint8_t seen = 0;                                        // arms scanned here, as absolute headings
      if (s.left)     seen |= 1 << ((hd + 3) & 3);
      if (s.right)    seen |= 1 << ((hd + 1) & 3);
      if (s.straight) seen |= 1 << hd;
      int id = arrive(cur, hd, dist, seen, true);
      if (id < 0) { failed(); return false; }
      cur = id;

      // Left-Hand Rule priority: Left > Straight > Right
      int a = chooseArm(cur, hd);
      if (a < 0) { Serial.println("!! whole map explored, no end box found"); printMap(); failed(); return false; }
      char m = relMove(hd, a);
      Serial.printf("node %d (%.0f,%.0f) arms %s : %c\n", cur, mapN[cur].x / TICKS_PER_MM, mapN[cur].y / TICKS_PER_MM, armsStr(mapN[cur].arms), m);
      bool ok = (m == 'U') ? uTurnAt(mapN[cur].arms, hd) : doMove(m);
      if (!ok) { failed(); return false; }
      hd = a;
      segRef = s.early ? s.stopOdo : det + ticksInchForward;
      lastError = 0; lostSince = 0;
      endReset();
      startLock(turnLockMM);
      rgb(0, 40, 0);
      continue;
    }

    // 3. BLACK BACKGROUND DEAD END (!anyLit) -> 180 deg turn logic (no reverse)
    if (!anyLit) {
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > (unsigned long)lostMs) {       // dead end
        rgb(80, 0, 80); // Magenta indicator for dead end (NOT RED!)
        long det = odo();
        stopAndSettle();
        int id = arrive(cur, hd, det - segRef, 0, false);
        if (id < 0) { failed(); return false; }
        cur = id;

        int a = chooseArm(cur, hd);                             // only the way back is open here
        if (a < 0) { Serial.println("!! dead end and nothing left to explore"); printMap(); failed(); return false; }
        char m = relMove(hd, a);
        Serial.printf("node %d (%.0f,%.0f) dead end : %c\n", cur, mapN[cur].x / TICKS_PER_MM, mapN[cur].y / TICKS_PER_MM, m);
        bool ok = (m == 'U') ? uTurnAt(mapN[cur].arms, hd) : doMove(m);
        if (!ok) { failed(); return false; }
        hd = a;
        segRef = odo();
        lastError = 0; lostSince = 0;
        endReset();
        startLock(turnLockMM);
        rgb(0, 40, 0);
        continue;
      }
    } else lostSince = 0;

    pidStep(baseSpeed, false, Kp, Kd);
  }

  failed();
  return false;
}

// ==========================================
// 🏁 ACTUAL RUN - replay with a speed profile
// ==========================================
bool actualRun() {
  digitalWrite(LED_PIN, LOW);
  rgb(0, 60, 80);
  lastError = 0; lostSince = 0;
  int k = 0;                           // index of the next node on the path (node COUNT)
  long ref = odo();                    // where the current leg started
  endReset();
  startLock(startLockMM);
  unsigned long t0 = millis();

  while (millis() - t0 < RUN_TIMEOUT_MS) {
    readSensors();
    long traveled = odo() - ref;
    bool locked = (odo() - lockRef) < lockLen;
    bool finalLeg = (k >= evN);
    char mv = finalLeg ? 'S' : evMove[k];
    long D = finalLeg ? endDist : evDist[k];
    long remaining = D - traveled;
    bool turnAhead = !finalLeg && mv != 'S';
    bool inBrakeZone = turnAhead && remaining < MM(brakeZoneMM);

    // ---- speed profile ----
    int spd = runSpeed;
    if (turnAhead) {
      long zone = MM(brakeZoneMM), slow = MM(slowZoneMM);
      if (zone <= slow) zone = slow + 1;
      if (remaining <= slow) spd = turnEntrySpeed;
      else if (remaining < zone) spd = turnEntrySpeed + (long)(runSpeed - turnEntrySpeed) * (remaining - slow) / (zone - slow);
    }
    if (traveled < MM(accelZoneMM)) {
      long t = traveled < 0 ? 0 : traveled;
      int ramp = approachSpeed + (long)(runSpeed - approachSpeed) * t / MM(accelZoneMM);
      if (ramp < spd) spd = ramp;
    }

    if (finalLeg && endCheck(endMMRun)) { endReached(); return true; }

    if (!finalLeg && !locked && (nodeL || nodeR) && traveled >= D - MM(windowMM)) {
      long det = odo();
      if (mv == 'S') {
        startLock(passLockMM);
      } else {
        rgb(80, 60, 0);
        bool ok;
        bool fast = arcTurn && mv != 'U';
        if (fast) ok = arcTurnMove(mv);
        else if (brakeAtNode) {
          stopAndSettle();
          creepTo(det + ticksInchForward - runStopLead);
          ok = doMove(mv);
        }
        else { creepOnly(runLeadTicks); ok = doMove(mv); }
        if (!ok) { failed(); return false; }
        if (!fast) lastError = 0;
        startLock(turnLockMM);
        rgb(0, 60, 80);
        ref = fast ? (odo() - arcExitTicks) : (det + ticksInchForward);
        k++;
        lostSince = 0;
        continue;
      }
      ref = det + ticksInchForward;
      k++;
      lostSince = 0;
      continue;
    }

    if (!finalLeg && mv == 'S' && traveled > D + MM(windowMM)) {
      ref += D + ticksInchForward;
      k++;
      continue;
    }

    if (!anyLit) {
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > 400) { failed(); return false; }
    } else lostSince = 0;

    if (inBrakeZone) pidStep(spd, false, Kp, Kd);
    else             pidStep(spd, true, KpRun, KdRun);
  }
  failed();
  return false;
}

// ==========================================
// 🧮 LQR ACTUAL RUN (v9)
// ==========================================
float TRACK_MM        = 90;     // wheel contact patch to wheel contact patch
float SENSOR_FWD_MM   = 45;     // axle -> centre of the sensor array (forward)
float SENSOR_PITCH_MM = 10;     // between neighbouring sensors
float kS = 45, kV = 0.17;
float wheelKp = 0.30, wheelKi = 2.0;
float vMaxMM  = 1000;           // straights
float vTurnMM = 450;            // entry to a turn and speed along the arc
float vEndMM  = 700;            // when the end box arrives
float aAccMM  = 3000, aDecMM = 3500;
float lqPreMM = 20;
float vMinMM  = 250;
float lqrQe   = 1.0 / (6.0 * 6.0);
float lqrQpsi = 1.0 / (0.5 * 0.5);
float lqrR    = 1.0 / (3.2 * 3.2);
float obsWn   = 45;
float omegaMax = 30;

float arcRadiusMM = 60;
float yawExitDeg  = 75;
int   lqArcTimeoutMs = 900;
float lqDrMM = 8;
float lqWindowMM = 25;

const int LQ_NV = 8;
const float lqSpeeds[LQ_NV] = {150, 300, 500, 700, 900, 1100, 1400, 1800};
float lqK1[LQ_NV], lqK2[LQ_NV];
bool lqDesigned = false;
bool lqVerbose = false;

void lqrDesign() {
  double dt = LOOP_US * 1e-6, L = SENSOR_FWD_MM;
  for (int s = 0; s < LQ_NV; s++) {
    double v = lqSpeeds[s];
    double a12 = -v * dt;
    double b1 = -(L * dt + v * dt * dt / 2), b2 = dt;
    double p11 = lqrQe, p12 = 0, p22 = lqrQpsi, k1 = 0, k2 = 0;
    for (int it = 0; it < 3000; it++) {
      double pb1 = p11 * b1 + p12 * b2, pb2 = p12 * b1 + p22 * b2;
      double den = lqrR + b1 * pb1 + b2 * pb2;
      double g1 = pb1, g2 = a12 * pb1 + pb2;
      k1 = g1 / den; k2 = g2 / den;
      double n11 = p11, n12 = p11 * a12 + p12, n22 = a12 * a12 * p11 + 2 * a12 * p12 + p22;
      p11 = n11 - g1 * g1 / den + lqrQe;
      p12 = n12 - g1 * g2 / den;
      p22 = n22 - g2 * g2 / den + lqrQpsi;
    }
    lqK1[s] = (float)k1; lqK2[s] = (float)k2;
  }
  lqDesigned = true;
}
void lqrGains(float v, float &k1, float &k2) {
  if (v <= lqSpeeds[0]) { k1 = lqK1[0]; k2 = lqK2[0]; return; }
  for (int i = 1; i < LQ_NV; i++) if (v <= lqSpeeds[i]) {
    float t = (v - lqSpeeds[i - 1]) / (lqSpeeds[i] - lqSpeeds[i - 1]);
    k1 = lqK1[i - 1] + t * (lqK1[i] - lqK1[i - 1]);
    k2 = lqK2[i - 1] + t * (lqK2[i] - lqK2[i - 1]);
    return;
  }
  k1 = lqK1[LQ_NV - 1]; k2 = lqK2[LQ_NV - 1];
}
void printLqr() {
  if (!lqDesigned) lqrDesign();
  Serial.println("LQR gains  w = -(k1*e[mm] + k2*psi[rad])");
  for (int i = 0; i < LQ_NV; i++) Serial.printf("  v %5.0f mm/s : k1 %8.4f  k2 %8.3f\n", lqSpeeds[i], lqK1[i], lqK2[i]);
}

float lqE = 0, lqPsi = 0;
float wvL = 0, wvR = 0;
float wiL = 0, wiR = 0;
long  lqLastL = 0, lqLastR = 0;
unsigned long lqLastUs = 0;
float lqXlat = 0;

void lqReset() {
  lqE = lqPsi = 0; wvL = wvR = 0; wiL = wiR = 0;
  lqLastL = (long)leftTicks * leftDir; lqLastR = (long)rightTicks * rightDir;
  lqLastUs = micros();
}
float lqTick() {
  while (micros() - lqLastUs < LOOP_US) { }
  unsigned long now = micros();
  float dt = (now - lqLastUs) * 1e-6f;
  lqLastUs = now;
  long l = (long)leftTicks * leftDir, r = (long)rightTicks * rightDir;
  float rawL = (l - lqLastL) / dt / TICKS_PER_MM, rawR = (r - lqLastR) / dt / TICKS_PER_MM;
  lqLastL = l; lqLastR = r;
  wvL += 0.5f * (rawL - wvL);
  wvR += 0.5f * (rawR - wvR);
  return dt;
}
float lqSense() { float dt = lqTick(); readSensors(); return dt; }

bool lineErrorMM(bool centerOnly, float &e) {
  long ws = 0; int n = 0;
  int a = centerOnly ? 2 : 0, b = centerOnly ? 6 : 8;
  for (int i = a; i < b; i++) if (seesLine[i]) { ws += weights[i]; n++; }
  if (n == 0 && centerOnly) for (int i = 0; i < 8; i++) if (seesLine[i]) { ws += weights[i]; n++; }
  if (n == 0) return false;
  e = (float)ws / n * (SENSOR_PITCH_MM / 10.0f);
  return true;
}

int wheelPwm(float vt, float vm, float &I, float dt) {
  float ff = (fabsf(vt) < 15) ? 0 : (vt > 0 ? kS + kV * vt : -(kS + kV * -vt));
  float err = vt - vm;
  float u = ff + wheelKp * err + I;
  if (fabsf(u) < 255) { I += wheelKi * err * dt; if (I > 80) I = 80; if (I < -80) I = -80; }
  if (u > 255) u = 255;
  if (u < -255) u = -255;
  if (fabsf(u) < 6) u = 0;
  return (int)u;
}
void lqDrive(float vtL, float vtR, float dt) {
  setMotors(wheelPwm(vtL, wvL, wiL, dt), wheelPwm(vtR, wvR, wiR, dt));
}

void lqControl(float vCmd, bool centerOnly, float dt) {
  float v = 0.5f * (wvL + wvR); if (v < 150) v = 150;
  float w = (wvL - wvR) / TRACK_MM;
  lqE   += (-v * lqPsi - SENSOR_FWD_MM * w) * dt;
  lqPsi += w * dt;
  float em;
  if (lineErrorMM(centerOnly, em)) {
    float innov = em - lqE;
    lqE   += 1.4f * obsWn * dt * innov;
    lqPsi -= (obsWn * obsWn / v) * dt * innov;
  }
  float k1, k2; lqrGains(v, k1, k2);
  float u = -(k1 * lqE + k2 * lqPsi);
  if (u > omegaMax) u = omegaMax;
  if (u < -omegaMax) u = -omegaMax;
  float vtL = vCmd + u * TRACK_MM / 2, vtR = vCmd - u * TRACK_MM / 2;
  float top = vtL > vtR ? vtL : vtR, cap = vMaxMM * 1.15f;
  if (top > cap) { vtL -= top - cap; vtR -= top - cap; }
  lqDrive(vtL, vtR, dt);
}

float arcKRho = 0.35;
float arcKTh  = 12.0;
bool lqTurn(char mv) {
  int dir = (mv == 'R') ? +1 : -1;
  float d0 = ticksInchForward / TICKS_PER_MM;
  float r = arcRadiusMM; if (r > d0) r = d0;
  float cy = d0 - r;
  float px = -lqE - SENSOR_FWD_MM * lqPsi, py = 0, th = lqPsi;
  long lp = (long)leftTicks * leftDir, rp = (long)rightTicks * rightDir;
  float yawExit = yawExitDeg * 0.0174533f;
  bool inArc = false, ok = false;
  float xm = 0, thm = 0, phi = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)lqArcTimeoutMs) {
    float dt = lqSense();
    long l = (long)leftTicks * leftDir, rr = (long)rightTicks * rightDir;
    float dl = (l - lp) / TICKS_PER_MM, dr = (rr - rp) / TICKS_PER_MM; lp = l; rp = rr;
    float ds = 0.5f * (dl + dr), dth = (dl - dr) / TRACK_MM;
    px += ds * sinf(th + 0.5f * dth); py += ds * cosf(th + 0.5f * dth); th += dth;
    xm = dir * px; thm = dir * th;
    if (!inArc && py >= cy) inArc = true;
    if (!inArc) { lqControl(vTurnMM, true, dt); continue; }
    float rx = xm - r, ry = py - cy;
    float rho = sqrtf(rx * rx + ry * ry);
    phi = atan2f(ry, -rx);
    if ((phi >= yawExit && litCount <= 4 && (seesLine[2] || seesLine[3] || seesLine[4] || seesLine[5])) || phi >= 1.50f) { ok = true; break; }
    float wm = vTurnMM / r + arcKRho * (rho - r) - arcKTh * (thm - phi);
    if (wm > omegaMax) wm = omegaMax;
    if (wm < -omegaMax) wm = -omegaMax;
    float w = dir * wm;
    lqDrive(vTurnMM + w * TRACK_MM / 2, vTurnMM - w * TRACK_MM / 2, dt);
  }
  if (!ok) { stopAndSettle(); rgb(80, 0, 0); return false; }
  lqXlat = xm;
  float em;
  lqE = lineErrorMM(false, em) ? em : 0;
  lqPsi = dir * (thm - 1.5708f);
  return true;
}

bool actualRunLQR() {
  if (!lqDesigned) lqrDesign();
  digitalWrite(LED_PIN, LOW);
  rgb(0, 60, 80);
  lostSince = 0;
  int k = 0;
  long ref = odo();
  endReset();
  startLock(startLockMM);
  lqReset();
  float vCmd = vMinMM;
  bool armed = true;
  int  clean = 0;
  unsigned long t0 = millis();

  while (millis() - t0 < RUN_TIMEOUT_MS) {
    float dt = lqSense();
    if (!armed) {
      bool cleanLine = !nodeL && !nodeR && litCount <= 4 && (seesLine[3] || seesLine[4]);
      clean = cleanLine ? clean + 1 : 0;
      if (clean >= 3) armed = true;
    }
    long traveled = odo() - ref;
    bool locked = (odo() - lockRef) < lockLen;
    bool finalLeg = (k >= evN);
    char mv = finalLeg ? 'S' : evMove[k];
    long D = finalLeg ? endDist : evDist[k];
    float remMM = (D - traveled) / TICKS_PER_MM;
    bool turnAhead = !finalLeg && mv != 'S';

    float vProf = vMaxMM;
    if (turnAhead)     vProf = fminf(vMaxMM, sqrtf(vTurnMM * vTurnMM + 2 * aDecMM * fmaxf(0, remMM - lqPreMM)));
    else if (finalLeg) vProf = fminf(vMaxMM, sqrtf(vEndMM * vEndMM + 2 * aDecMM * fmaxf(0, remMM)));
    float dv = vProf - vCmd;
    if (dv > aAccMM * dt) dv = aAccMM * dt;
    if (dv < -aDecMM * dt) dv = -aDecMM * dt;
    vCmd += dv;

    if (finalLeg && endCheck(endMMRun)) { endReached(); return true; }

    bool barSeen = armed && (nodeL || nodeR) && traveled >= D - MM(lqWindowMM);
    bool deadReck = (mv != 'S') && anyLit && traveled >= D + MM(lqDrMM);
    if (!finalLeg && !locked && (barSeen || deadReck)) {
      long det = odo();
      if (lqVerbose) Serial.printf("node %d %c  expected %ld ticks, travelled %ld, v %.0f\n", k, mv, D, traveled, vCmd);
      if (mv == 'S') {
        startLock(passLockMM);
        ref = det + ticksInchForward;
        k++; lostSince = 0;
        continue;
      }
      rgb(80, 60, 0);
      if (!lqTurn(mv)) { failed(); return false; }
      ref = odo() - MM(lqXlat);
      lockLen = 0; armed = false; clean = 0;
      vCmd = vTurnMM;
      k++; lostSince = 0;
      rgb(0, 60, 80);
      continue;
    }

    if (!finalLeg && mv == 'S' && traveled > D + MM(windowMM)) { ref += D + ticksInchForward; k++; continue; }

    if (!anyLit) {
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > 400) { failed(); return false; }
    } else lostSince = 0;

    lqControl(vCmd, true, dt);
  }
  failed();
  return false;
}

// ==========================================
// 🛠️ CALIBRATION (slide the bot over the line!)
// ==========================================
void detectEncoderDirs() {
  long l0 = leftTicks, r0 = rightTicks;
  setMotors(100, 100);
  delay(150);
  stopAndSettle();
  long dl = leftTicks - l0, dr = rightTicks - r0;
  if (labs(dl) > 8) leftDir  = dl > 0 ? 1 : -1;
  if (labs(dr) > 8) rightDir = dr > 0 ? 1 : -1;
  Serial.printf("encoder dirs: left %d (%ld)  right %d (%ld)\n", leftDir, dl, rightDir, dr);
}

bool calibrateSensors() {
  rgb(0, 0, 255);
  for (int i = 0; i < 8; i++) { sensorMin[i] = 4095; sensorMax[i] = 0; }
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    for (int i = 0; i < 8; i++) {
      int val = analogRead(irPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    delay(1);
  }
  bool ok = true;
  for (int i = 0; i < 8; i++) {
    thresholds[i] = (sensorMin[i] + sensorMax[i]) / 2;
    Serial.printf("sensor %d: min %4d max %4d thr %4d %s\n", i, sensorMin[i], sensorMax[i], thresholds[i],
                  (sensorMax[i] - sensorMin[i] < MIN_SPAN) ? "<-- NEVER SAW BOTH COLOURS" : "");
    if (sensorMax[i] - sensorMin[i] < MIN_SPAN) ok = false;
  }
  if (!ok) {
    for (int k = 0; k < 4; k++) { rgb(80, 0, 0); delay(150); rgb(0, 0, 0); delay(150); }
    return false;
  }
  detectEncoderDirs();
  return true;
}

// ==========================================
// 🧪 SERIAL TOOLS
// ==========================================
void motorTest() {
  int saved = minPWM; minPWM = 0;
  Serial.println("pwm   dL    dR    (both should be positive; if dR is negative the bot auto-corrects it)");
  for (int p = 20; p <= 120; p += 4) {
    long l0 = leftTicks, r0 = rightTicks;
    setMotors(p, p);
    delay(250);
    Serial.printf("%3d  %4ld  %4ld\n", p, (long)(leftTicks - l0), (long)(rightTicks - r0));
  }
  brakeMotors();
  minPWM = saved;
  Serial.println("minPWM = first pwm where BOTH wheels show ticks (+ ~8 margin)");
}
void sensorTest() {
  Serial.println("raw values, * = lit (calibrate first). Put the bot on the line / inside the END box. 10 s:");
  unsigned long t0 = millis();
  while (millis() - t0 < 10000) {
    int lit = 0;
    for (int i = 0; i < 8; i++) {
      int v = analogRead(irPins[i]);
      bool l = isWhiteLine ? (v < thresholds[i]) : (v > thresholds[i]);
      if (l) lit++;
      Serial.printf("%4d%c ", v, l ? '*' : ' ');
    }
    Serial.printf("| lit %d\n", lit);
    delay(200);
  }
}
void distTest() {
  Serial.println("push the bot 500 mm along the floor; TICKS_PER_MM = odo / 500");
  long o0 = odo();
  for (int i = 0; i < 50; i++) { Serial.printf("odo %ld\n", odo() - o0); delay(200); }
}

// ==========================================
// 🚀 SETUP / LOOP
// ==========================================
void showIdle() {
  if (!calibrated)    rgb(40, 30, 0);      // yellow : needs calibration
  else if (!dryDone)  rgb(0, 30, 30);      // cyan   : ready for dry run
  else                rgb(30, 0, 40);      // purple : ready for actual run
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);

  const int mp[4] = {IN1, IN2, IN3, IN4};
  for (int i = 0; i < 4; i++) ledcAttach(mp[i], PWM_FREQ, 8);
  brakeMotors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP); pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP); pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(RUN_BTN_PIN, INPUT_PULLUP);
  for (int i = 0; i < 8; i++) { pinMode(irPins[i], INPUT); sensorMin[i] = 0; sensorMax[i] = 4095; thresholds[i] = 2048; }

  if (digitalRead(BTN_PIN) == LOW) {
    delay(1200);
    if (digitalRead(BTN_PIN) == LOW && loadPath()) {
      dryDone = true;
      printPath();
      rgb(0, 80, 0); delay(400);
    }
    while (digitalRead(BTN_PIN) == LOW) delay(10);
  }
  delay(500);
  showIdle();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'm') motorTest();
    else if (c == 'd') distTest();
    else if (c == 'b') { Serial.println("Testing straight reverse for 200 mm..."); reverseTo(odo() - MM(200)); Serial.println("Done."); }
    else if (c == 'p') printPath();
    else if (c == 'g') printMap();
    else if (c == 'j') {
      Serial.println("\n[SERIAL] Re-calculating Dijkstra Shortest Path on current Map:");
      if (buildPath(true)) printPath();
    }
    else if (c == 'l') printLqr();
    else if (c == 'v') { lqVerbose = !lqVerbose; Serial.println(lqVerbose ? "lqr log on" : "lqr log off"); }
    else if (c == 's') sensorTest();
  }

  // ---- RUN button (GPIO 7) ----
  if (digitalRead(RUN_BTN_PIN) == LOW) {
    unsigned long t = millis();
    while (digitalRead(RUN_BTN_PIN) == LOW) delay(5);
    unsigned long held = millis() - t;
    if (held >= 30) {
      if (held > 1500) { calibrated = calibrateSensors(); showIdle(); }
      else if (!calibrated || evN == 0) {
        for (int k = 0; k < 3; k++) { rgb(80, 0, 0); delay(150); rgb(0, 0, 0); delay(150); }
        showIdle();
      } else { delay(1000); actualRunLQR(); }
    }
    return;
  }

  if (digitalRead(BTN_PIN) == LOW) {
    unsigned long t = millis();
    while (digitalRead(BTN_PIN) == LOW) delay(5);
    unsigned long held = millis() - t;
    if (held < 30) return;
    if (held > 1500) {
      calibrated = false; dryDone = false; evN = 0;
      digitalWrite(LED_PIN, LOW);
      showIdle();
      return;
    }
    delay(1000);
    if (!calibrated)    calibrated = calibrateSensors();
    else if (!dryDone)  dryDone = dryRun();
    else                actualRun();
    if (!(calibrated && dryDone)) showIdle();
  }
}
