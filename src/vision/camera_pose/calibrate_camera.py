"""
Project: Lab Powder Manipulation
File: calibrate_camera.py
Author: Lang Yun

Description:
    This script calibrates a camera using images of a checkerboard pattern.
    It computes the camera's intrinsic parameters and distortion coefficients, and saves them to a file.
 
Required: 
    - OpenCV (cv2)
    - NumPy (np)    
"""


import cv2
import numpy as np
import glob
import os


# =========================
# 1. Parameters
# =========================

# NUmber of corner points on the Chessboard
# 10x7 chessboard => 9x6 inner corners
CHECKERBOARD = (8, 5)

# Every square's size in meters, e.g., 25 mm = 0.025 m
SQUARE_SIZE = 0.025

IMAGE_DIR = "images"
OUTPUT_FILE = "camera_calibration.npz"


# =========================
# 2. Define 3D points of the checkerboard corners in the world coordinate system
# =========================

# Use 'objp' to save the 3D coordinates of the checkerboard corners in the world coordinate system. 
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)

# Assume Z=0
objp[:, :2] = np.mgrid[
    0:CHECKERBOARD[0],
    0:CHECKERBOARD[1]
].T.reshape(-1, 2)

# Multiply by square size to get real-world coordinates in meters
objp *= SQUARE_SIZE


# =========================
# 3. Use to store 3D points and 2D points from all images
# =========================

# World coordinates of 3D points
objpoints = []

# Image coordinates of 2D points
imgpoints = []


# =========================
# 4. Read all calibration images and detect checkerboard corners
# =========================

image_paths = glob.glob(os.path.join(IMAGE_DIR, "*.png"))

if len(image_paths) == 0:
    print("Error: No calibration images found.")
    exit()

print(f"Found {len(image_paths)} images.")

image_size = None

for image_path in image_paths:
    img = cv2.imread(image_path)

    if img is None:
        print(f"Warning: Cannot read {image_path}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # OpenCV camera calibration function expects sequence in 'width, height'
    image_size = gray.shape[::-1]  

    # Detect checkerboard inner corner points
    ret, corners = cv2.findChessboardCorners(
        gray,
        CHECKERBOARD,
        None
    )

    if ret:
        print(f"Detected corners: {image_path}")

        objpoints.append(objp)

        # Refine corner locations to subpixel accuracy
        # Maximum number of iterations = 30 or epsilon = 0.001
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001
        )

        corners_subpix = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria
        )

        imgpoints.append(corners_subpix)

        # Draw and display the corners
        cv2.drawChessboardCorners(
            img,
            CHECKERBOARD,
            corners_subpix,
            ret
        )

        cv2.imshow("Detected Corners", img)
        cv2.waitKey(300)

    else:
        print(f"Failed to detect corners: {image_path}")

cv2.destroyAllWindows()


# =========================
# 5. Start camera calibration
# =========================

if len(objpoints) < 10:
    print("Warning: Too few valid images. Recommend at least 15-20 valid images.")

print(f"Valid calibration images: {len(objpoints)}")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    image_size,
    None,
    None
)


# =========================
# 6. Print calibration results
# =========================

print("\nCalibration finished.")
print(f"RMS reprojection error: {ret}")

print("\nCamera matrix:")
print(camera_matrix)

print("\nDistortion coefficients:")
print(dist_coeffs)

fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]

print("\nIntrinsic parameters:")
print(f"fx = {fx}")
print(f"fy = {fy}")
print(f"cx = {cx}")
print(f"cy = {cy}")


# =========================
# 7. Save calibration results to a file
# =========================

np.savez(
    OUTPUT_FILE,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    image_size=image_size,
    rms=ret
)

print(f"\nSaved calibration to: {OUTPUT_FILE}")