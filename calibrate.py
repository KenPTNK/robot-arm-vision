import cv2
import numpy as np
import glob
import os

criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

chessboard_size = (6,6)
square_size = 30.0
# tạo bảng 6x6 ô vuông để hiệu chỉnh, mỗi ô vuông có kích thước 20mm (square_size)

objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)

# Tạo thành 1 cái bảng 63 giá trị, mỗi giá trị là bộ (x,y,z) của 1 điểm trên bảng

objp[:, :2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2)

# np.mgrid[a:b,c:d] = một bảng 2D với các giá trị x từ a đến b-1 và y từ c đến d-1
# [[a, c], [a, c+1], ..., [a, d-1]]
# [[a+1, c], [a+1, c+1], ..., [a+1, d-1]]
# ...
# [[b-1, c], [b-1, c+1], ..., [b-1, d-1]]

# x_matrix là:
# [[a, a, ..., a],
# [a+1, a+1, ..., a+1]
# ...
# [b-1, b-1, ..., b-1]]
# y_matrix là:
# [[c, c+1, ..., d-1],
# [c, c+1, ..., d-1],
# ...
# [c, c+1, ..., d-1]]

# .T là transpose, coi ảnh, đổi hàng thành cột, cột thành hàng
#  Cái giá trị của np.mgrid[0:chessboard_size[0],0:chessboard_size[1]] là (x_matrix, y_matrix) có shape (2, 6, 6) tức 2 layer matrix 6x6
# Ta cần giá trị thuộc (x,y) nên dùng transpose để đổi shape (2,6,6) -> (6,6,2) 
# Khi đó ta sẽ có 6 layer matrix 6x2

# reshape(-1,2) sẽ đổi shape (6,6,2) -> (36,2) tức 36 điểm có tọa độ (x,y)
# vì reshape(-1,2) tức là reshape thành 2 cột, số hàng tự động tính toán dựa trên tổng số phần tử

objp *= square_size

# Nhân tọa độ (x,y) với kích thước ô vuông để có tọa độ thực tế của các điểm trên bảng hiệu chỉnh
# result:
# [[0, 0, 0],
# [30, 0, 0],
# [60, 0, 0],
# ...
# [150, 150, 0]]

objpoints = []
imgpoints = []

images = glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calib_images', '*.png'))

if len(images) == 0:
    print("No images found")
    exit()

image_size = None

for fname in images:
    img = cv2.imread(fname)
    
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if image_size is None:
        image_size = gray.shape[::-1]

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if not ret:
        continue

    objpoints.append(objp.copy())
    corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    imgpoints.append(corners2)

if len(objpoints) == 0:
    print("No valid chessboard detections")
    exit()

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, image_size, None, None)

print("\n--- Calibration Results ---")
print("Intrinsic matrix: \n", mtx)
print("Distortion coefficients: \n", dist.ravel())

with open("camera_calibration.txt", "w", encoding="utf-8") as f:
    
    f.write("Kết quả hiệu chỉnh:\n")
    f.write("Ma trận nội (Intrinsic matrix):\n")
    f.write(str(mtx) + "\n\n")
    f.write("Hệ số méo (Distortion coefficients):\n")
    f.write(str(dist.ravel()) + "\n\n")
    
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        R, _ = cv2.Rodrigues(rvec)
        extrinsic = np.hstack((R, tvec))
        
        f.write(f"Ảnh {i+1}:\n")
        f.write("R:\n" + str(R) + "\n")
        f.write("t:\n" + str(tvec) + "\n")
        f.write("[R|t]:\n" + str(extrinsic) + "\n\n")
        
        # Ma trận ngoại của camera (camera pose so với thế giói)
        R_inv = R.T
        t_inv = -R_inv @ tvec
        camera_pose = np.hstack((R_inv, t_inv))
        f.write(f"Camera pose (ngoại của camera): \n" + str(camera_pose) + "\n")
        
print("Calibration results saved to camera_calibration.txt")


