# Robot Arm Vision

A 3-DOF robot arm that picks and places objects using a camera. The camera looks at the workspace, a YOLO model detects the object, Python converts image coordinates to robot coordinates and sends a command over Serial to an Arduino, which drives three stepper motors.

```
Camera ──► YOLO (best.pt) ──► pixel (u, v) ──► world coords ──► robot coords ──► Serial ──► Arduino ──► steppers
```

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
