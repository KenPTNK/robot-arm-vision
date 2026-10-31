import cv2
import numpy as np
import glob
import os

criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

chessboard_size = (6,6)
square_size = 30.0
# 6x6 calibration board, each square has size square_size (mm)

objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)

# Build the board points: each row is the (x, y, z) of one point on the board

objp[:, :2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2)

# np.mgrid[a:b,c:d] = a 2D grid with x values from a to b-1 and y values from c to d-1
# [[a, c], [a, c+1], ..., [a, d-1]]
# [[a+1, c], [a+1, c+1], ..., [a+1, d-1]]
# ...
# [[b-1, c], [b-1, c+1], ..., [b-1, d-1]]

# x_matrix is:
# [[a, a, ..., a],
# [a+1, a+1, ..., a+1]
# ...
# [b-1, b-1, ..., b-1]]
# y_matrix is:
# [[c, c+1, ..., d-1],
# [c, c+1, ..., d-1],
# ...
# [c, c+1, ..., d-1]]

# .T is the transpose: rows become columns and columns become rows
#  np.mgrid[0:chessboard_size[0],0:chessboard_size[1]] returns (x_matrix, y_matrix) with shape (2, 6, 6), i.e. two 6x6 matrices
# We need (x, y) pairs, so transpose the shape (2,6,6) -> (6,6,2)
# which gives six 6x2 matrices

# reshape(-1,2) turns shape (6,6,2) into (36,2), i.e. 36 points with (x, y) coordinates
# reshape(-1,2) means 2 columns; the row count is inferred from the total number of elements

objp *= square_size

# Multiply (x, y) by the square size to get real-world coordinates of the board points
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
    
    f.write("Calibration results:\n")
    f.write("Intrinsic matrix:\n")
    f.write(str(mtx) + "\n\n")
    f.write("Distortion coefficients:\n")
    f.write(str(dist.ravel()) + "\n\n")
    
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        R, _ = cv2.Rodrigues(rvec)
        extrinsic = np.hstack((R, tvec))
        
        f.write(f"Image {i+1}:\n")
        f.write("R:\n" + str(R) + "\n")
        f.write("t:\n" + str(tvec) + "\n")
        f.write("[R|t]:\n" + str(extrinsic) + "\n\n")
        
        # Camera extrinsics (camera pose relative to the world)
        R_inv = R.T
        t_inv = -R_inv @ tvec
        camera_pose = np.hstack((R_inv, t_inv))
        f.write(f"Camera pose (inverse extrinsic): \n" + str(camera_pose) + "\n")
        
print("Calibration results saved to camera_calibration.txt")


