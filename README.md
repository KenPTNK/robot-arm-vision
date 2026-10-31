# Robot Arm Vision

Cánh tay robot 3 bậc tự do (3DOF) gắp – thả vật thể bằng camera. Camera chụp khu vực làm việc, mô hình YOLO nhận diện vật, Python đổi tọa độ ảnh sang tọa độ robot rồi gửi lệnh qua Serial cho Arduino điều khiển 3 động cơ bước.

```
Camera ──► YOLO (best.pt) ──► pixel (u, v) ──► tọa độ thế giới ──► tọa độ robot ──► Serial ──► Arduino ──► động cơ bước
```

## Cấu trúc thư mục

| Đường dẫn | Nội dung |
|---|---|
| `Final/Cam_detect_fixed.py` | Chương trình chính: nhận diện, chuyển tọa độ, gửi lệnh cho Arduino |
| `Final/RobotAIcontrol_New_new/` | Sketch Arduino mới nhất: nhận lệnh, động học, gắp – thả |
| `Code_3DOF_Robot_Control/` | Sketch Arduino điều khiển 3DOF cơ bản |
| `calibrate.py`, `new_calibrate.py` | Hiệu chỉnh camera bằng bàn cờ |
| `calib_images/`, `calib_images3/` | Ảnh bàn cờ dùng để hiệu chỉnh |
| `camera_calibration.txt` | Kết quả hiệu chỉnh (ma trận K, hệ số méo) |
| `b1.py`, `robot_dataset.py`, `yolo_dataset.py` | Chụp ảnh từ webcam để làm dataset |
| `merge_img.py` | Gộp các thư mục ảnh dataset |
| `roboflow.py` | Upload ảnh lên Roboflow |
| `Cam_detect.py`, `result.py` | Script thử nghiệm nhận diện |
| `best.pt` | Trọng số YOLO đã huấn luyện |
| `CAD_3DRobot_Arm/` | Bản vẽ SolidWorks (chi tiết + `Assem1.SLDASM`) |

## Cài đặt

Cần Python 3.11 và Arduino IDE.

```bash
pip install -r requirements.txt
```

Nạp `Final/RobotAIcontrol_New_new/RobotAIcontrol_New_new.ino` vào Arduino (baud 115200).

## Chạy

```bash
cd Final
python Cam_detect_fixed.py
```

Mặc định script mở camera số `1` và cổng `COM7`; sửa trong code nếu máy bạn khác.

| Phím | Tác dụng |
|---|---|
| `q` | Bật chế độ tự động gắp |
| `h` | Đưa robot về vị trí home |
| `e` | Bật/tắt chế độ hiệu chỉnh |
| `Esc` | Thoát |

### Giao thức Serial

| Lệnh | Ý nghĩa |
|---|---|
| `I(x y z)` | Gắp vật tại tọa độ robot `(x, y, z)` (mm) |
| `H()` | Về home |

Arduino trả về `Done` khi làm xong.

## Hiệu chỉnh camera

```bash
python calibrate.py        # ảnh trong calib_images/
python new_calibrate.py    # ảnh trong calib_images3/
```

Chép ma trận `K` và hệ số méo `dist` in ra vào phần `CAMERA PARAMS` của `Final/Cam_detect_fixed.py`.

## Roboflow

`roboflow.py` đọc API key từ biến môi trường, không ghi trong code:

```bash
export ROBOFLOW_API_KEY=...        # PowerShell: $env:ROBOFLOW_API_KEY="..."
python roboflow.py
```

## Không có trong repo

Dataset (`*.zip`, `funda_images/`, `hardcore_images/`) và các trọng số cũ (`last.pt`, `old_remove_best.pt`) quá nặng nên không đưa lên. `result.py` cần `old_remove_best.pt`; đổi sang `best.pt` nếu không có file đó.
