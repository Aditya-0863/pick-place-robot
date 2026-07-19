"""
============================================================
2WD Robot Vision Controller  —  Python / OpenCV
============================================================

What this file does:
  1. Captures video from an overhead webcam
  2. Detects robot pose (blue=front, green=back), red object,
     yellow destination — all with preprocessing + smoothing
  3. Runs a state machine that commands the ESP32 over UDP

UDP command protocol (must match esp32_robot.ino):
  "M <left> <right>"  → motor speeds, each -255..255
  "B <angle>"         → base servo angle 0-180
  "G <angle>"         → grip servo angle 0 (open) to 90 (closed)
  "STOP"              → both motors off

State machine:
  FIND_OBJECT  → spin slowly, looking for red blob
  ORIENT_OBJ   → rotate to face object (heading-based)
  APPROACH_OBJ → drive toward object (with heading correction)
  GRAB         → arm sequence: lower, close gripper, raise
  FIND_DEST    → spin slowly, looking for yellow blob
  ORIENT_DST   → rotate to face destination
  APPROACH_DST → drive toward destination
  RELEASE      → arm sequence: lower, open gripper, raise
  IDLE         → done; stop motors, arm to idle

Tuning guide:
  Run hsv_tuner.py first to get correct HSV ranges for your
  lighting, then update the COLORS dict below.
  Adjust STOP_DISTANCE_PX if robot over/undershoots targets.
============================================================
"""

import cv2
import numpy as np
import socket
import time
import math
from collections import deque

# ===========================================================
#  CONFIGURATION  — edit these before first run
# ===========================================================

ESP32_IP   = "10.121.31.180"   # set to your ESP32 IPS
ESP32_PORT = 4210
CAM_INDEX  = 0                 # try 0 if 1 does not work

FRAME_W = 640
FRAME_H = 480

# HSV colour ranges — tune with hsv_tuner.py
COLORS = {
    "blue":   {"lo": (100, 120,  80), "hi": (130, 255, 255)},  # robot front
    "green":  {"lo": ( 45, 100,  80), "hi": ( 85, 255, 255)},  # robot back
    "red_lo": {"lo": (  0, 120,  80), "hi": ( 10, 255, 255)},  # object (hue wraps)
    "red_hi": {"lo": (165, 120,  80), "hi": (180, 255, 255)},  # object (hue wraps)
    "yellow": {"lo": ( 18,  80, 100), "hi": ( 35, 255, 255)},  # destination
}

# Navigation
ANGLE_THRESHOLD   = 6    # degrees: below this the robot is considered "aligned"
APPROACH_SPEED    = 90    # max forward PWM 0-255  (was 65)
MIN_ROT_SPEED     = 75 # minimum PWM to overcome static friction when rotating
MAX_ROT_SPEED     = 100   # cap rotation speed for smooth turns  (was 80)
STOP_DISTANCE_PX  = 220 # pixel distance to target to consider "arrived"
KP_ANGLE          = 1.8   # proportional gain: angle error -> rotation speed
ARM_OFFSET_PX = 0
MIN_FWD_SPEED= 75

# Search behaviour
SEARCH_ROT_SPEED  = 50    # slow PWM used when spinning to find a target
SEARCH_FLIP_SEC   = 8.0   # reverse spin direction after this many seconds

# Temporal smoothing (rolling average to kill jitter)
SMOOTH_WINDOW     = 2     # frames to average positions over
STALE_LIMIT       = 8     # frames before a missing detection clears its history

# Safety
MISSING_STOP_LIMIT = 3    # consecutive frames without robot -> safety STOP

# Image preprocessing
BLUR_K            = 5     # Gaussian kernel size (must be odd)
CLAHE_CLIP        = 2.0   # CLAHE clip limit
CLAHE_TILE        = (8, 8)# CLAHE tile grid

# Heartbeat — resend last command to prevent ESP32 from stalling
HEARTBEAT_SEC     = 0.2

# Arm pose: (base_angle, grip_angle)
ARM_IDLE  = (50,  105)   # raised, gripper open
ARM_DOWN  = (0,   105)   # lowered, gripper open
ARM_CARRY = (65,   20)   # raised, gripper closed

# Grab/release sequence timing (seconds after state entry)
GRAB_T = {
    "send_lower":  0.00,  # lower arm + open gripper
    "send_close":  1.50,  # close gripper
    "send_raise":  3.40,  # raise arm
    "done":        3.80,  # leave state
}
RELEASE_T = {
    "send_lower":  0.00,
    "send_open":   1.20,
    "send_raise":  1.50,
    "done":        2.50,
}

# ===========================================================
#  UDP
# ===========================================================

_sock           = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
_last_cmd       = ""
_last_send_time = 0.0


def send_command(cmd: str):
    """Send UDP command with heartbeat repeat to prevent ESP32 stall."""
    global _last_cmd, _last_send_time
    now = time.time()
    if cmd != _last_cmd or (now - _last_send_time) >= HEARTBEAT_SEC:
        try:
            _sock.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
        except OSError as e:
            print(f"UDP error: {e}")
        _last_cmd       = cmd
        _last_send_time = now
        print(f"  TX: {cmd}")


def motors(left: int, right: int):
    left  = max(-255, min(255, int(left)))
    right = max(-255, min(255, int(right)))
    send_command(f"M {left} {right}")


def arm_cmd(base: int, grip: int):
    """Send both servo angles with a short pause between them."""
    send_command(f"B {base}")
    time.sleep(0.04)
    send_command(f"G {grip}")


def stop():
    send_command("STOP")


# ===========================================================
#  IMAGE PREPROCESSING
# ===========================================================

_clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP, tileGridSize=CLAHE_TILE)


def preprocess(frame: np.ndarray) -> np.ndarray:
    """
    Gaussian blur (noise suppression) followed by CLAHE on the L* channel
    of CIE Lab so only luminance is equalised — hue and saturation are
    untouched, keeping HSV thresholding reliable under uneven lighting.
    """
    blurred  = cv2.GaussianBlur(frame, (BLUR_K, BLUR_K), 0)
    lab      = cv2.cvtColor(blurred, cv2.COLOR_BGR2Lab)
    l, a, b  = cv2.split(lab)
    lab_eq   = cv2.merge([_clahe.apply(l), a, b])
    return cv2.cvtColor(lab_eq, cv2.COLOR_Lab2BGR)


# ===========================================================
#  COLOUR DETECTION
# ===========================================================

_morph_k = np.ones((5, 5), np.uint8)


def _range_mask(hsv: np.ndarray, key: str) -> np.ndarray:
    c = COLORS[key]
    return cv2.inRange(hsv, np.array(c["lo"]), np.array(c["hi"]))


def detect_blob(hsv: np.ndarray, color: str, min_area: int = 400):
    """
    Find largest blob of the given colour.
    color: "blue" | "green" | "red" | "yellow"
    Returns (cx, cy) or None.
    """
    if color == "red":
        mask = cv2.bitwise_or(_range_mask(hsv, "red_lo"), _range_mask(hsv, "red_hi"))
    else:
        mask = _range_mask(hsv, color)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  _morph_k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _morph_k)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None

    best = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(best) < min_area:
        return None

    M = cv2.moments(best)
    if M["m00"] == 0:
        return None

    return (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))


# ===========================================================
#  TEMPORAL SMOOTHING
# ===========================================================

_hist    = {k: deque(maxlen=SMOOTH_WINDOW) for k in ("front", "back", "obj", "dst")}
_missing = {k: 0 for k in ("front", "back", "obj", "dst")}


def smooth(key: str, pos):
    """Rolling-average position smoother. Returns averaged (cx, cy) or None."""
    if pos is not None:
        _hist[key].append(pos)
        _missing[key] = 0
    else:
        _missing[key] += 1
        if _missing[key] > STALE_LIMIT:
            _hist[key].clear()

    if not _hist[key]:
        return None

    return (int(np.mean([p[0] for p in _hist[key]])),
            int(np.mean([p[1] for p in _hist[key]])))


# ===========================================================
#  GEOMETRY
# ===========================================================

def heading(front, back) -> float:
    """Robot heading in degrees. 0=right, 90=up (screen Y is flipped)."""
    return math.degrees(math.atan2(-(front[1] - back[1]),
                                     front[0] - back[0]))


def target_angle(robot_center, target) -> float:
    dx = target[0] - robot_center[0]
    dy = target[1] - robot_center[1]
    return math.degrees(math.atan2(-dy, dx))


def midpoint(a, b):
    return ((a[0] + b[0]) // 2, (a[1] + b[1]) // 2)


def dist(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def wrap180(d) -> float:
    while d >  180: d -= 360
    while d < -180: d += 360
    return d


def nav_command(front, back, target):
    rc    = midpoint(front, back)
    h     = heading(front, back)
    h_rad = math.radians(h)

    # Navigate from arm position, not robot center
    arm_pos = (rc[0] + ARM_OFFSET_PX * math.sin(h_rad),
               rc[1] + ARM_OFFSET_PX * math.cos(h_rad))

    ta   = target_angle(arm_pos, target)
    diff = wrap180(ta - h)
    d    = dist(arm_pos, target)

    if d <= STOP_DISTANCE_PX:
        return 0, 0, "stop"

    if abs(diff) > ANGLE_THRESHOLD:
        spd = int(np.clip(abs(diff) * KP_ANGLE, MIN_ROT_SPEED, MAX_ROT_SPEED))
        if diff > 0:
            return  spd, -spd, "rotate"
        else:
            return -spd,  spd, "rotate"

    fwd     = int(np.clip(d * 0.8, MIN_FWD_SPEED, APPROACH_SPEED))

    correct = int(np.clip(diff * 1.2, -40, 40))
    return fwd - correct, fwd + correct, "forward"


# ===========================================================
#  SAFETY GUARD
# ===========================================================

class VisibilityGuard:
    """Force STOP if robot markers vanish for N consecutive frames."""
    def __init__(self, limit: int):
        self._limit   = limit
        self._missing = 0

    def update(self, visible: bool):
        self._missing = 0 if visible else self._missing + 1

    @property
    def safe(self) -> bool:
        return self._missing <= self._limit

    @property
    def count(self) -> int:
        return self._missing


# ===========================================================
#  SEARCH SPINNER
# ===========================================================

class SearchSpinner:
    """
    Slowly rotates the robot to scan for a target.
    Reverses direction every SEARCH_FLIP_SEC seconds to guarantee
    a full 360-degree sweep from any starting orientation.
    Uses differential rotation (both wheels) for consistent spin rate.
    """
    def __init__(self):
        self._start = None
        self._dir   = 1   # +1 = one direction, -1 = other

    def reset(self):
        self._start = None

    def spin(self):
        now = time.time()
        if self._start is None:
            self._start = now
        if now - self._start >= SEARCH_FLIP_SEC:
            self._dir   = -self._dir
            self._start = now
        spd = int(SEARCH_ROT_SPEED * self._dir)
        motors(-spd, spd)


# ===========================================================
#  DISPLAY HELPERS
# ===========================================================

def draw_robot(vis, front, back):
    if front:
        cv2.circle(vis, front, 8, (255, 80, 0), -1)
        cv2.putText(vis, "F", (front[0] + 6, front[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 80, 0), 2)
    if back:
        cv2.circle(vis, back, 8, (0, 200, 0), -1)
        cv2.putText(vis, "B", (back[0] + 6, back[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 2)
    if front and back:
        rc    = midpoint(front, back)
        h     = heading(front, back)
        h_rad = math.radians(h)

        # Perpendicular right vector in screen coords
        rx = math.sin(h_rad)
        ry = math.cos(h_rad)

        # Offset arrow origin to where the arm actually is
        origin = (int(rc[0] + ARM_OFFSET_PX * rx),
                  int(rc[1] + ARM_OFFSET_PX * ry))
        tip    = (int(origin[0] + 55 * math.cos(h_rad)),
                  int(origin[1] - 55 * math.sin(h_rad)))

        cv2.arrowedLine(vis, origin, tip, (0, 255, 255), 2, tipLength=0.3)
        cv2.circle(vis, rc, 4, (0, 255, 255), -1)  # centre dot stays at true midpoint


def draw_target(vis, pos, colour, label):
    if pos:
        cv2.circle(vis, pos, 10, colour, 2)
        cv2.drawMarker(vis, pos, colour, cv2.MARKER_CROSS, 16, 2)
        cv2.putText(vis, label, (pos[0] + 8, pos[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)


def draw_hud(vis, state_name, extra=""):
    cv2.putText(vis, f"STATE: {state_name}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    if extra:
        cv2.putText(vis, extra, (10, 56),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)


# ===========================================================
#  STATE MACHINE CONSTANTS
# ===========================================================

(FIND_OBJECT, ORIENT_OBJ, APPROACH_OBJ,
 GRAB,
 FIND_DEST, ORIENT_DST, APPROACH_DST,
 RELEASE, IDLE) = range(9)

STATE_NAMES = {
    FIND_OBJECT:  "FIND OBJECT",
    ORIENT_OBJ:   "ORIENT TO OBJECT",
    APPROACH_OBJ: "APPROACH OBJECT",
    GRAB:         "GRAB",
    FIND_DEST:    "FIND DESTINATION",
    ORIENT_DST:   "ORIENT TO DEST",
    APPROACH_DST: "APPROACH DEST",
    RELEASE:      "RELEASE",
    IDLE:         "IDLE - DONE",
}

# Tracks which timed events inside GRAB/RELEASE have been sent
_arm_sent = {}


# ===========================================================
#  MAIN
# ===========================================================

def main():
    # Camera init with fallback
    cap = cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        fallback = 0 if CAM_INDEX != 0 else 1
        print(f"Camera {CAM_INDEX} failed, trying {fallback}…")
        cap = cv2.VideoCapture(fallback)
        if not cap.isOpened():
            raise RuntimeError("Cannot open any camera")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Initialise arm
    arm_cmd(*ARM_IDLE)
    time.sleep(1.0)

    state   = FIND_OBJECT
    state_t = time.time()

    guard    = VisibilityGuard(MISSING_STOP_LIMIT)
    spin_obj = SearchSpinner()
    spin_dst = SearchSpinner()

    print("Controller running — press 'q' to quit")

    def enter(s):
        nonlocal state, state_t
        _arm_sent.clear()
        state, state_t = s, time.time()
        print(f"\n>>> {STATE_NAMES[s]}")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        now = time.time()
        vis = frame.copy()

        # Preprocessing + HSV conversion (once per frame)
        proc = preprocess(frame)
        hsv  = cv2.cvtColor(proc, cv2.COLOR_BGR2HSV)

        # Raw detections
        raw_front = detect_blob(hsv, "blue",   min_area=300)
        raw_back  = detect_blob(hsv, "green",  min_area=300)
        raw_obj   = detect_blob(hsv, "red",    min_area=500)
        raw_dst   = detect_blob(hsv, "yellow", min_area=500)

        # Smoothed positions
        front = smooth("front", raw_front)
        back  = smooth("back",  raw_back)
        obj   = smooth("obj",   raw_obj)
        dst   = smooth("dst",   raw_dst)

        robot_ok = front is not None and back is not None
        guard.update(robot_ok)

        # Draw detections
        draw_robot(vis, front, back)
        draw_target(vis, obj, (0, 0, 255),   "OBJECT")
        draw_target(vis, dst, (0, 215, 255), "DEST")

        extra = ""

        # Safety override — stop everything if robot invisible
        if not guard.safe:
            stop()
            extra = f"!! SAFETY STOP — robot missing {guard.count} frames !!"
            draw_hud(vis, STATE_NAMES[state], extra)
            cv2.imshow("Robot Vision", vis)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        elapsed = now - state_t

        # ===================================================
        #  STATE MACHINE
        # ===================================================

        # ── FIND_OBJECT ─────────────────────────────────────
        if state == FIND_OBJECT:
            if robot_ok and obj is not None:
                spin_obj.reset()
                stop()
                enter(ORIENT_OBJ)
            else:
                spin_obj.spin()
                extra = ("Object detected, waiting for robot markers…"
                         if obj and not robot_ok else "Scanning for object…")

        # ── ORIENT_OBJ ──────────────────────────────────────
        elif state == ORIENT_OBJ:
            if not robot_ok or obj is None:
                stop()
                enter(FIND_OBJECT)
            else:
                l, r, action = nav_command(front, back, obj)
                if action == "stop":
                    stop()
                    enter(GRAB)
                elif action == "forward":
                    stop()
                    enter(APPROACH_OBJ)
                else:
                    motors(l, r)
                    rc = midpoint(front, back)
                    h  = heading(front, back)
                    ta = target_angle(rc, obj)
                    extra = f"Angle error: {wrap180(ta - h):.1f} deg"

        # ── APPROACH_OBJ ─────────────────────────────────────
        elif state == APPROACH_OBJ:
            if not robot_ok or obj is None:
                stop()
                enter(FIND_OBJECT)
            else:
                l, r, action = nav_command(front, back, obj)
                rc   = midpoint(front, back)
                d_px = dist(rc, obj)
                extra = f"Distance to object: {int(d_px)} px"
                if action == "stop":
                    stop()
                    enter(ORIENT_OBJ)
                elif action == "rotate":
                    enter(ORIENT_OBJ)
                else:
                    motors(l, r)

        # ── GRAB ─────────────────────────────────────────────
        elif state == GRAB:
            if "lower" not in _arm_sent and elapsed >= GRAB_T["send_lower"]:
                arm_cmd(ARM_DOWN[0], ARM_DOWN[1])
                _arm_sent["lower"] = True
                extra = "Lowering arm…"
            elif "close" not in _arm_sent and elapsed >= GRAB_T["send_close"]:
                send_command(f"G {ARM_CARRY[1]}")
                _arm_sent["close"] = True
                extra = "Closing gripper…"
            elif "raise" not in _arm_sent and elapsed >= GRAB_T["send_raise"]:
                send_command(f"B {ARM_CARRY[0]}")
                _arm_sent["raise"] = True
                extra = "Lifting object…"
            elif elapsed >= GRAB_T["done"]:
                spin_dst.reset()
                enter(FIND_DEST)
            else:
                extra = "Grab sequence…"

        # ── FIND_DEST ─────────────────────────────────────────
        elif state == FIND_DEST:
            if robot_ok and dst is not None:
                spin_dst.reset()
                stop()
                enter(ORIENT_DST)
            else:
                spin_dst.spin()
                extra = ("Destination visible, waiting for robot markers…"
                         if dst and not robot_ok else "Scanning for destination…")

        # ── ORIENT_DST ────────────────────────────────────────
        elif state == ORIENT_DST:
            if not robot_ok or dst is None:
                stop()
                enter(FIND_DEST)
            else:
                l, r, action = nav_command(front, back, dst)
                if action == "stop":
                    stop()
                    enter(RELEASE)
                elif action == "forward":
                    stop()
                    enter(APPROACH_DST)
                else:
                    motors(l, r)
                    rc   = midpoint(front, back)
                    h    = heading(front, back)
                    ta   = target_angle(rc, dst)
                    extra = f"Angle error: {wrap180(ta - h):.1f} deg"

        # ── APPROACH_DST ──────────────────────────────────────
        elif state == APPROACH_DST:
            if not robot_ok or dst is None:
                stop()
                enter(FIND_DEST)
            else:
                l, r, action = nav_command(front, back, dst)
                rc   = midpoint(front, back)
                d_px = dist(rc, dst)
                extra = f"Distance to dest: {int(d_px)} px"
                if action == "stop":
                    stop()
                    enter(ORIENT_DST)
                elif action == "rotate":
                    enter(ORIENT_DST)
                else:
                    motors(l, r)

        # ── RELEASE ───────────────────────────────────────────
        elif state == RELEASE:
            if "lower" not in _arm_sent and elapsed >= RELEASE_T["send_lower"]:
                arm_cmd(ARM_DOWN[0], ARM_CARRY[1])
                _arm_sent["lower"] = True
                extra = "Lowering arm…"
            elif "open" not in _arm_sent and elapsed >= RELEASE_T["send_open"]:
                send_command(f"G {ARM_IDLE[1]}")
                _arm_sent["open"] = True
                extra = "Opening gripper…"
            elif "raise" not in _arm_sent and elapsed >= RELEASE_T["send_raise"]:
                send_command(f"B {ARM_IDLE[0]}")
                _arm_sent["raise"] = True
                extra = "Raising arm…"
            elif elapsed >= RELEASE_T["done"]:
                enter(IDLE)
            else:
                extra = "Release sequence…"

        # ── IDLE ──────────────────────────────────────────────
        elif state == IDLE:
            stop()
            arm_cmd(*ARM_IDLE)
            extra = "Task complete."

        # HUD + display
        draw_hud(vis, STATE_NAMES[state], extra)
        cv2.imshow("Robot Vision", vis)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    stop()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
