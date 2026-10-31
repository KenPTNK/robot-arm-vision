# Robot Arm Vision

A 3-DOF robot arm that picks and places objects using a camera. The camera looks at the workspace, a YOLO model detects the object, Python converts image coordinates to robot coordinates and sends a command over Serial to an Arduino, which drives three stepper motors.

```
Camera ──► YOLO (best.pt) ──► pixel (u, v) ──► world coords ──► robot coords ──► Serial ──► Arduino ──► steppers
```

## How it works

**1. Pixel to table point** (`image_to_world` in `Final/Cam_detect_fixed.py`). The pixel is undistorted
with the calibrated intrinsics $K$ and distortion coefficients, giving a normalised ray $\mathbf{d}_c$.
With the camera pose $(R, \mathbf{t})$ from calibration, the camera centre in world coordinates is
$\mathbf{C} = -R^\top\mathbf{t}$ and the ray direction is $\mathbf{d} = R^\top\mathbf{d}_c$. The object lies
on the table plane $z = 0$, so

$$
\mathbf{P}_w = \mathbf{C} + s\,\mathbf{d},\qquad s = -\frac{C_z}{d_z}.
$$

**2. World to robot frame** (`world_to_robot`). A fixed rigid transform:
$\mathbf{P}_r = R_{wr}^\top(\mathbf{P}_w - \mathbf{t}_{wr})$, with $R_{wr} = R_z(45^\circ)\,R_y(180^\circ)$.

**3. Correcting the arm's error** (`apply_calibration`). Measured pairs of commanded and reached
positions are fitted with a 2-D affine model $\mathbf{p}_{\text{real}} = M\,\mathbf{p}_{\text{cmd}} + \mathbf{b}$
by least squares. To reach a target $T$, the script sends $M^{-1}(T - \mathbf{b})$.

**4. Inverse kinematics on the Arduino** (`inverse_kinematics1` in
`Final/RobotAIcontrol_New_new/RobotAIcontrol_New_new.ino`). Base angle $\theta_1 = \operatorname{atan2}(P_y, P_x)$;
the shoulder and elbow form a two-link planar arm ($L_2 = L_3 = 140$ mm) solved with the law of cosines:

$$
D = \frac{r_w^2 + s^2 - L_2^2 - L_3^2}{2L_2L_3},\quad
\theta_3 = \operatorname{atan2}\!\left(-\sqrt{1-D^2},\, D\right),\quad
\theta_2 = \operatorname{atan2}(s, r_w) - \operatorname{atan2}(L_3\sin\theta_3,\ L_2 + L_3\cos\theta_3),
$$

where $r_w$ is the horizontal reach minus the 54 mm end-effector offset and $s = P_z - L_1$.
Targets with $|D| > 1$ are out of reach and rejected. The angles are converted to step pulses
for the three stepper motors.

## Repository layout

| Path | Description |
|---|---|
| `Final/Cam_detect_fixed.py` | Main program: detection, coordinate transforms, sends commands to the Arduino |
| `Final/RobotAIcontrol_New_new/` | Latest Arduino sketch: command parsing, kinematics, pick-and-place |
| `Code_3DOF_Robot_Control/` | Basic 3-DOF Arduino control sketch |
| `calibrate.py`, `new_calibrate.py` | Camera calibration with a chessboard |
| `calib_images/`, `calib_images3/` | Chessboard images used for calibration |
| `camera_calibration.txt` | Calibration output (K matrix, distortion coefficients) |
| `b1.py`, `robot_dataset.py`, `yolo_dataset.py` | Capture webcam images for the dataset |
| `merge_img.py` | Merge dataset image folders |
| `roboflow.py` | Upload images to Roboflow |
| `Cam_detect.py`, `result.py` | Detection test scripts |
| `best.pt` | Trained YOLO weights |
| `CAD_3DRobot_Arm/` | SolidWorks parts and assembly (`Assem1.SLDASM`) |

## Setup

Requires Python 3.11 and the Arduino IDE.

```bash
pip install -r requirements.txt
```

Upload `Final/RobotAIcontrol_New_new/RobotAIcontrol_New_new.ino` to the Arduino (baud rate 115200).

## Run

```bash
cd Final
python Cam_detect_fixed.py
```

By default the script opens camera `1` and port `COM7`; change them in the code if your setup differs.

| Key | Action |
|---|---|
| `q` | Start automatic pick-and-place |
| `h` | Send the robot home |
| `e` | Toggle calibration mode |
| `Esc` | Quit |

### Serial protocol

| Command | Meaning |
|---|---|
| `I(x y z)` | Pick the object at robot coordinates `(x, y, z)` in mm |
| `H()` | Go home |

The Arduino replies `Done` when the move is finished.

## Camera calibration

```bash
python calibrate.py        # uses images in calib_images/
python new_calibrate.py    # uses images in calib_images3/
```

Copy the printed `K` matrix and distortion coefficients `dist` into the `CAMERA PARAMS` section of `Final/Cam_detect_fixed.py`.

## Roboflow

`roboflow.py` reads the API key from an environment variable, never from the code:

```bash
export ROBOFLOW_API_KEY=...        # PowerShell: $env:ROBOFLOW_API_KEY="..."
python roboflow.py
```

## Not in this repository

Datasets (`*.zip`, `funda_images/`, `hardcore_images/`) and older weights (`last.pt`, `old_remove_best.pt`) are too large to include. `result.py` expects `old_remove_best.pt`; switch it to `best.pt` if you do not have that file.
