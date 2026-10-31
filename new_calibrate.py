import cv2
import numpy as np
import glob

# Termination criteria for cornerSubPix
criteria = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

# Chessboard settings
chessboard_size = (9, 7)   # 9x7 INNER corners (10x8 squares)
square_size = 20           # mm

# Prepare object points (3D real-world coordinates)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)

# Create grid of (x, y)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Scale by square size (convert to real-world mm)
objp *= square_size

# Arrays to store points
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image

# Load images
images = glob.glob('calib_images3/*.png')
print("Loaded images:", len(images))

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("Processing:", fname)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)

        # Refine corners
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw corners
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(200)

cv2.destroyAllWindows()

# Check before calibration
if len(objpoints) == 0:
    print("ERROR: No chessboard corners detected!")
    exit()

# Camera calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Output results
print("\n--- Calibration Results ---")
print("Intrinsic matrix:\n", mtx)
print("Distortion coefficients:\n", dist.ravel())

# Save results
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

        # Camera pose (inverse extrinsic)
        R_inv = R.T
        t_inv = -R_inv @ tvec
        camera_pose = np.hstack((R_inv, t_inv))

        f.write("Camera pose:\n" + str(camera_pose) + "\n\n")

print("Calibration results saved to camera_calibration.txt")