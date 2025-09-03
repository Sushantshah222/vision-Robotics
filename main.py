# main.py — Two small, beautiful pressable buttons (index fingertip "poke" to click)
# pip install opencv-python mediapipe pyserial

import cv2
import mediapipe as mp
import serial
import time
from collections import deque

# ======================= CONFIG =======================
SERIAL_PORT = "/dev/tty.usbserial-1130"  # <-- your Mac port
BAUD = 115200
DRAW_LANDMARKS = True

# Button sizes/placement (relative to frame)
BTN_W_RATIO = 0.18     # width ~18% of frame width
BTN_H_RATIO = 0.22     # height ~22% of frame height
BTN_MARGIN_X = 0.08    # left/right margin
BTN_CENTER_Y = 0.58    # vertical placement (0 top .. 1 bottom)

# Press detection
HISTORY = 6            # frames to keep for motion baseline
PRESS_Z_DELTA = 0.1  # how much fingertip Z must move toward camera (prev_z - curr_z)
PRESS_Y_PIX = 30       # fallback: downward motion in pixels to still count as "tap" default =18
COOLDOWN_FRAMES = 10   # frames to ignore after a successful press

TIP_RADIUS = 8
WINDOW = "Hand LED Control (Press Buttons)"
# =======================================================

# ---- Serial setup ----
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0.1)
    time.sleep(2)  # let Arduino reset
    print("Serial connected:", SERIAL_PORT)
except Exception as e:
    print("Serial not connected:", e)

def send_state(state):  # state: "ON" or "OFF"
    if not ser:
        return
    try:
        ser.write(b'1' if state == "ON" else b'0')
    except Exception as e:
        print("Serial write failed:", e)

# ---- MediaPipe Hands ----
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

def get_index_tip(hand, w, h):
    lm = hand.landmark
    x = int(lm[8].x * w)
    y = int(lm[8].y * h)
    z = lm[8].z  # normalized depth relative to wrist; more negative ~ closer to camera
    return x, y, z

def in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return (rx <= x <= rx+rw) and (ry <= y <= ry+rh)

def draw_pretty_button(frame, rect, label, active=False, pressed=False, base_color=(40,40,40), on_color=(40,180,80), off_color=(220,60,60)):
    x, y, w, h = rect
    # Make a soft filled rectangle using overlay for nicer visuals
    overlay = frame.copy()
    color = on_color if label == "ON" else off_color
    border = (255, 255, 255)

    # Idle tint
    fill = tuple(int(c*0.25) for c in color)
    cv2.rectangle(overlay, (x, y), (x+w, y+h), fill, thickness=-1)

    # Hover glow
    if active:
        glow = tuple(int(min(255, c*0.5)) for c in color)
        cv2.rectangle(overlay, (x-3, y-3), (x+w+3, y+h+3), glow, thickness=-1)

    # Press flash
    if pressed:
        flash = (255, 255, 255)
        cv2.rectangle(overlay, (x, y), (x+w, y+h), flash, thickness=-1)

    # Blend
    alpha = 0.35 if not pressed else 0.55
    cv2.addWeighted(overlay, alpha, frame, 1-alpha, 0, frame)

    # Border
    cv2.rectangle(frame, (x, y), (x+w, y+h), color, thickness=2)

    # Label
    font_scale = 0.9
    thick = 2
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thick)
    cv2.putText(frame, label, (x + (w - tw)//2, y + (h + th)//2 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, border, thick)

# ---- Camera ----
cv2.namedWindow(WINDOW)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Camera not found")

last_led = "OFF"          # current LED state
cooldown = 0              # cooldown counter after a press
pressed_flash_frames = 0  # short visual flash on press
pressed_button = None     # 'ON' or 'OFF' for flash

# fingertip motion history
z_hist = deque(maxlen=HISTORY)
y_hist = deque(maxlen=HISTORY)

with mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    model_complexity=1,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
) as hands:
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]

        # Compute button rects
        btn_w = int(BTN_W_RATIO * w)
        btn_h = int(BTN_H_RATIO * h)
        left_x = int(BTN_MARGIN_X * w)
        right_x = w - left_x - btn_w
        center_y = int(BTN_CENTER_Y * h)
        top_y = center_y - btn_h // 2

        left_btn  = (left_x,  top_y, btn_w, btn_h)   # ON
        right_btn = (right_x, top_y, btn_w, btn_h)   # OFF

        # Process hand
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)

        hover = None
        pressed_now = False

        if res.multi_hand_landmarks:
            hand = res.multi_hand_landmarks[0]
            if DRAW_LANDMARKS:
                mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            fx, fy, fz = get_index_tip(hand, w, h)

            # Show fingertip
            cv2.circle(frame, (fx, fy), TIP_RADIUS, (0, 255, 255), -1)

            # Update histories
            z_hist.append(fz)
            y_hist.append(fy)

            # Which button are we over?
            if in_rect(fx, fy, left_btn):
                hover = "ON"
            elif in_rect(fx, fy, right_btn):
                hover = "OFF"

            # Press detection (only when hovering and not in cooldown)
            if hover and cooldown == 0 and len(z_hist) >= 2:
                # Depth "poke": previous average z vs current z (more negative = toward camera)
                prev_z = sum(list(z_hist)[:-1]) / max(1, (len(z_hist)-1))
                dz = prev_z - fz  # positive if moving toward camera
                # Downward tap fallback (screen y grows downward)
                prev_y = sum(list(y_hist)[:-1]) / max(1, (len(y_hist)-1))
                dy = fy - prev_y  # positive if moving down

                if dz >= PRESS_Z_DELTA or dy >= PRESS_Y_PIX:
                    # Register a press
                    if hover == "ON" and last_led != "ON":
                        last_led = "ON"
                        send_state(last_led)
                        pressed_now = True
                        pressed_button = "ON"
                        cooldown = COOLDOWN_FRAMES
                        print("LED -> ON (press)")
                    elif hover == "OFF" and last_led != "OFF":
                        last_led = "OFF"
                        send_state(last_led)
                        pressed_now = True
                        pressed_button = "OFF"
                        cooldown = COOLDOWN_FRAMES
                        print("LED -> OFF (press)")

        # Decay cooldown
        if cooldown > 0:
            cooldown -= 1

        # Press flash frames (for nicer feedback)
        if pressed_now:
            pressed_flash_frames = 5
        if pressed_flash_frames > 0:
            pressed_flash_frames -= 1
        active_flash_on  = pressed_flash_frames > 0 and pressed_button == "ON"
        active_flash_off = pressed_flash_frames > 0 and pressed_button == "OFF"

        # Draw buttons (hover and flash states)
        draw_pretty_button(frame, left_btn,  "ON",
                           active=(hover=="ON"), pressed=active_flash_on)
        draw_pretty_button(frame, right_btn, "OFF",
                           active=(hover=="OFF"), pressed=active_flash_off)

        # HUD: current LED state
        cv2.putText(frame, f"LED: {last_led}", (10, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 220, 0) if last_led=="ON" else (0, 0, 220), 2)

        cv2.imshow(WINDOW, frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC
            break

cap.release()
cv2.destroyAllWindows()
