import cv2
import numpy as np
from ultralytics import YOLO

# ================================ CAMERA PARAMS ================================
import numpy as np

# Intrinsic matrix (K)
K = np.array([
    [2.53079925e+03, 0.00000000e+00, 3.78291184e+02],
    [0.00000000e+00, 2.53976610e+03, 1.71554213e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float64)

# Distortion coefficients (k1, k2, p1, p2, k3)
dist = np.array([
    2.31410258e+00, -1.51786785e+02, -1.98479426e-02,  2.51739569e-02,  3.98742691e+03
], dtype=np.float64)

# Rotation matrix (R)
R = np.array([[-0.99924161, -0.00478305, -0.03864356], [ 0.00845754, -0.99539438, -0.09549087], [-0.03800885, -0.09574528,  0.99467993]], dtype=np.float64)

# Translation vector (t)
t = np.array([[38.8638989], [77.99665902], [1019.14555637]], dtype=np.float64)

# ================================ WORLD → ROBOT ================================
theta_z = np.deg2rad(45)
theta_x = np.deg2rad(180)

Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0], [np.sin(theta_z),  np.cos(theta_z), 0], [0, 0, 1]], dtype=np.float64)

Rx = np.array([[1, 0, 0], [0, np.cos(theta_x), -np.sin(theta_x)], [0, np.sin(theta_x),  np.cos(theta_x)]], dtype=np.float64)

R_wr = Rx @ Rz
t_wr = np.array([[210], [120], [0]], dtype=np.float64)
# -227 155
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
    Pr = R_wr.T @ (Pw + t_wr)
    return Pr.flatten()

# ================================ MAIN LOOP ================================
while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    annotated_frame = results[0].plot()

    if len(results[0].boxes.xyxy) > 0:
        box = results[0].boxes.xyxy[0]
        x1, y1, x2, y2 = box
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

        # ===== COORDINATES =====
        Xw, Yw, Zw = image_to_world(cx, cy, K, dist, R, t)
        Pw = np.array([Xw, Yw, Zw])

        Pr = world_to_robot(Pw)

        # ===== DISPLAY =====
        cv2.circle(annotated_frame, (cx, cy), 5, (0, 0, 255), -1)

        text1 = f"Pixel: ({cx},{cy})"
        text2 = f"World: ({Xw:.1f},{Yw:.1f},{Zw:.1f})"
        text3 = f"Robot: ({Pr[0]:.1f},{Pr[1]:.1f},{Pr[2]:.1f})"

        cv2.putText(annotated_frame, text1, (cx+10, cy-40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)

        cv2.putText(annotated_frame, text2, (cx+10, cy-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        cv2.putText(annotated_frame, text3, (cx+10, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

    cv2.imshow("YOLO + Coordinates", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()