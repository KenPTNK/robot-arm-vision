import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

# ================================ SERIAL ================================
try:
    arduino = serial.Serial(port='COM7', baudrate=115200, timeout=1)
    time.sleep(2)
    print("[INFO] Đã kết nối Arduino qua COM7.")
except:
    arduino = None
    print("[WARNING] Không kết nối được Arduino.")

# ================================ CAMERA PARAMS ================================
K = np.array([
    [5.92799421e+03, 0.0, 3.11721406e+02],
    [0.0, 6.21744584e+03, 2.17903216e+02],
    [0.0, 0.0, 1.0]
], dtype=np.float64)

dist = np.array([
    3.06455841e+00, 4.51029715e+02,
    -6.92128701e-03, -1.11233319e-02,
    1.63859437e+00
], dtype=np.float64)

R = np.array([
    [0.0100562, 0.99994838, 0.00145431],
    [-0.94670014, 0.0090524, 0.32198897],
    [0.32195919, -0.00461478, 0.9467423]
], dtype=np.float64)

t = np.array([
    [-83.68073246],
    [61.72491908],
    [2726.23776139]
], dtype=np.float64)

# ================================ WORLD → ROBOT ================================
theta_x = np.deg2rad(0)
theta_y = np.deg2rad(180)
theta_z = np.deg2rad(45)

Rx = np.array([
    [1, 0, 0],
    [0, np.cos(theta_x), -np.sin(theta_x)],
    [0, np.sin(theta_x), np.cos(theta_x)]
], dtype=np.float64)

Ry = np.array([
    [np.cos(theta_y), 0, np.sin(theta_y)],
    [0, 1, 0],
    [-np.sin(theta_y), 0, np.cos(theta_y)]
], dtype=np.float64)

Rz = np.array([
    [np.cos(theta_z), -np.sin(theta_z), 0],
    [np.sin(theta_z), np.cos(theta_z), 0],
    [0, 0, 1]
], dtype=np.float64)

R_wr = Rz @ Ry

t_wr = np.array([
    [270],
    [210],
    [0]
], dtype=np.float64)

# ================================ CALIBRATION ================================
# Each row: [screen_X, screen_Y, real_X, real_Y]
#   screen = camera-computed coords (what was sent as command)
#   real   = where the robot physically ended up
#
# The robot undershoots — when commanded T it arrives at real(T).
# The correction inverts this: given target T, we solve for the command C
# such that real(C) = T.  A 2-D affine model (Real = M @ Screen + b) is
# fitted here, and apply_calibration returns M_inv @ (target - b).
#
# To improve accuracy: add more rows (spread across the workspace) and re-run.
CALIB_DATA = np.array([
    [229.0,  60.1,  224.5,  41.9],
    [137.5,  41.8,  124.7,  39.6],
    [236.5, -27.8,  233.3, -47.7],
    [209.5,   1.1,  203.3, -14.6],
    [211.8,  82.2,  198.0,  82.1],
    [211.8,  81.6,  199.3,  82.1],
], dtype=np.float64)

# Fit 2-D affine: real = M @ screen + b
# Uses both screen_X and screen_Y to predict each output (handles X↔Y coupling).
_A = np.column_stack([CALIB_DATA[:, 0], CALIB_DATA[:, 1], np.ones(len(CALIB_DATA))])
_px, _, _, _ = np.linalg.lstsq(_A, CALIB_DATA[:, 2], rcond=None)  # coeffs for real_X
_py, _, _, _ = np.linalg.lstsq(_A, CALIB_DATA[:, 3], rcond=None)  # coeffs for real_Y

_M = np.array([[_px[0], _px[1]], [_py[0], _py[1]]])   # 2x2 rotation/scale part
_b = np.array([_px[2], _py[2]])                         # translation part
_M_inv = np.linalg.inv(_M)
OFFSET_Z = 0.0

print(f"[CALIB] Affine M =\n{_M}")
print(f"[CALIB] Offset b = {_b}")

# Verify fit residuals on training data
for i, row in enumerate(CALIB_DATA):
    pred = _M @ row[:2] + _b
    err = row[2:4] - pred
    print(f"[CALIB] pt{i+1}: pred=({pred[0]:.1f},{pred[1]:.1f})  real=({row[2]:.1f},{row[3]:.1f})  err=({err[0]:.1f},{err[1]:.1f})")

# ================================ YOLO ================================
model = YOLO("best.pt")

# ================================ CAMERA ================================
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("[ERROR] Không mở được camera.")
    exit()

# ================================ FUNCTIONS ================================
def image_to_world(u, v, K, dist, R, t):
    pixel = np.array([[[u, v]]], dtype=np.float64)
    undistorted = cv2.undistortPoints(pixel, K, dist, P=None)[0][0]

    cam_ray = np.array([[undistorted[0]], [undistorted[1]], [1.0]])

    R_inv = R.T
    C = -R_inv @ t
    d = R_inv @ cam_ray

    n = np.array([[0, 0, 1]])
    s = np.dot(n, -C) / np.dot(n, d)

    Pw = C + s * d
    return Pw.flatten()

def world_to_robot(Pw):
    Pw = Pw.reshape(3, 1)
    Pr = R_wr.T @ (Pw - t_wr)
    return Pr.flatten()

def apply_calibration(Pr):
    # Inverse affine: given target T, find command C such that real(C) = T
    # real = M @ cmd + b  →  cmd = M_inv @ (T - b)
    cmd_xy = _M_inv @ (np.array([Pr[0], Pr[1]]) - _b)
    return np.array([cmd_xy[0], cmd_xy[1], Pr[2] + OFFSET_Z])

def send_position(Pr):
    corrected = apply_calibration(Pr)
    msg = f"I({corrected[0]:.1f} {corrected[1]:.1f} {corrected[2]:.1f})"
    print(f"[SEND] {msg}  (raw: X={Pr[0]:.1f} Y={Pr[1]:.1f} Z={Pr[2]:.1f})")
    arduino.write((msg + "\n").encode())

# ================= STATE =================
auto_mode = False
robot_busy = False
robot_start_time = 0
ROBOT_TIMEOUT = 20
pending_target = None
ready_to_send = False

stable_count = 0
last_Pr = None
STABLE_THRESHOLD = 8
POSITION_TOLERANCE = 3

cal_mode = False   # calibration display mode (press 'e' to toggle)

# ================================ MAIN LOOP ================================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    annotated_frame = results[0].plot()

    # ===== đọc phản hồi Arduino (NON-BLOCK) =====
    if arduino and arduino.in_waiting > 0:
        line = arduino.readline().decode().strip()
        if line:
            print("[ARDUINO]", line)
            if "Done" in line:
                robot_busy = False

                pending_target = None
                ready_to_send = False
                stable_count = 0
                last_Pr = None

                print("[INFO] Reset tracking")

    # ===== TIMEOUT ROBOT =====
    if robot_busy and (time.time() - robot_start_time > ROBOT_TIMEOUT):
        print("[WARNING] Robot timeout -> reset")
        robot_busy = False

    # ================= AUTO =================
    if auto_mode and not robot_busy:

        if len(results[0].boxes.xyxy) > 0:

            min_dist = float('inf')
            best = None

            for box in results[0].boxes.xyxy:
                x1, y1, x2, y2 = box

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                Pw = image_to_world(cx, cy, K, dist, R, t)
                Pr = world_to_robot(Pw)

                d = np.linalg.norm(Pr)

                if d < min_dist:
                    min_dist = d
                    best = (Pr, cx, cy)

            if best is not None:
                Pr, cx, cy = best

                if last_Pr is not None:
                    if np.linalg.norm(Pr - last_Pr) < POSITION_TOLERANCE:
                        stable_count += 1
                    else:
                        stable_count = 0
                else:
                    stable_count = 0

                last_Pr = Pr

                if stable_count >= STABLE_THRESHOLD and pending_target is None:
                    pending_target = (Pr, cx, cy)
                    ready_to_send = False
                    stable_count = 0
                    last_Pr = None

        # ===== HIỂN THỊ =====
        if pending_target is not None:
            Pr, cx, cy = pending_target

            cv2.circle(annotated_frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(annotated_frame,
                        f"{Pr[0]:.1f},{Pr[1]:.1f},{Pr[2]:.1f}",
                        (cx+10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            if not ready_to_send:
                ready_to_send = True
            else:
                time.sleep(0.3)

                if arduino:
                    send_position(Pr)
                    robot_busy = True
                    robot_start_time = time.time()

                pending_target = None
                ready_to_send = False

    # ===== CALIBRATION MODE OVERLAY =====
    if cal_mode:
        # Show raw coords of the nearest detected object in real time
        cal_Pr = None
        if len(results[0].boxes.xyxy) > 0:
            min_dist = float('inf')
            for box in results[0].boxes.xyxy:
                x1, y1, x2, y2 = box
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                Pw = image_to_world(cx, cy, K, dist, R, t)
                Pr = world_to_robot(Pw)
                d = np.linalg.norm(Pr)
                if d < min_dist:
                    min_dist = d
                    cal_Pr = Pr

        overlay = annotated_frame.copy()
        cv2.rectangle(overlay, (0, 0), (annotated_frame.shape[1], 110), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, annotated_frame, 0.45, 0, annotated_frame)

        cv2.putText(annotated_frame, "=== CALIBRATION MODE (press E to exit) ===",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if cal_Pr is not None:
            cv2.putText(annotated_frame,
                        f"Raw robot coords:  X={cal_Pr[0]:.1f}  Y={cal_Pr[1]:.1f}  Z={cal_Pr[2]:.1f}",
                        (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            corrected = apply_calibration(cal_Pr)
            cv2.putText(annotated_frame,
                        f"After offset:      X={corrected[0]:.1f}  Y={corrected[1]:.1f}  Z={corrected[2]:.1f}",
                        (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 200, 0), 2)
        else:
            cv2.putText(annotated_frame, "No object detected",
                        (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 100, 255), 2)
    else:
        # Always show active offset in corner so user knows what correction is applied
        offset_label = f"CALIB 2D-affine  pts={len(CALIB_DATA)}"
        cv2.putText(annotated_frame, offset_label,
                    (10, annotated_frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 1)

    # ===== SHOW =====
    status = "BUSY" if robot_busy else "READY"
    mode_label = "AUTO" if auto_mode else "IDLE"
    cv2.putText(annotated_frame, f"{mode_label} | {status}",
                (annotated_frame.shape[1] - 160, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("YOLO + Robot", annotated_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):
        auto_mode = True
        print("[AUTO START]")

    if key == ord('h'):
        print("[REQUEST HOME]")
        if arduino:
            msg = "H()"
            print(f"[SEND] {msg}")
            arduino.write((msg + "\n").encode())
        auto_mode = False

    if key == ord('e'):
        cal_mode = not cal_mode
        print(f"[CAL MODE] {'ON' if cal_mode else 'OFF'}")

    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
