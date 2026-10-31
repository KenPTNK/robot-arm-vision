import cv2
import numpy as np
from ultralytics import YOLO
import serial
import time

# ================================ SERIAL ================================
try:
    arduino = serial.Serial(port='COM7', baudrate=115200, timeout=1)
    time.sleep(2)
    print("[INFO] Connected to Arduino on COM7.")
except:
    arduino = None
    print("[WARNING] Could not connect to Arduino.")

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

# ================================ YOLO ================================
model = YOLO("best.pt")

# ================================ CAMERA ================================
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("[ERROR] Could not open camera.")
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

def send_position(Pr):
    msg = f"I({Pr[0]:.1f} {Pr[1]:.1f} {Pr[2]:.1f})"
    print("[SEND]", msg)
    arduino.write((msg + "\n").encode())

# ================= STATE =================
auto_mode = False
robot_busy = False
robot_start_time = 0
ROBOT_TIMEOUT = 20  # seconds
pending_target = None
ready_to_send = False

stable_count = 0
last_Pr = None
STABLE_THRESHOLD = 8
POSITION_TOLERANCE = 3

# ================================ MAIN LOOP ================================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    annotated_frame = results[0].plot()

    # ===== read Arduino response (non-blocking) =====
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

        # ===== DISPLAY =====
        if pending_target is not None:
            Pr, cx, cy = pending_target

            cv2.circle(annotated_frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(annotated_frame,
                        f"{Pr[0]:.1f},{Pr[1]:.1f},{Pr[2]:.1f}",
                        (cx+10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

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

    # ===== SHOW =====
    status = "BUSY" if robot_busy else "READY"
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
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()