"""
Project: Lab Powder Manipulation
File: capture_calibration_images.py
Author: Lang Yun

Description:
 This script captures images from a camera for calibration purposes.
 It saves the captured images in a specified directory for later use in camera calibration.
 
Required: 
 - OpenCV (cv2)
 - NumPy (np)
"""



import cv2
import os

CAMERA_ID = 0   
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
TARGET_FPS = 150

# Create a new directory for saving captured images
SAVE_DIR = "images"
os.makedirs(SAVE_DIR, exist_ok=True)


# This function initializes the camera with the specified parameters.
cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)

fourcc = cv2.VideoWriter_fourcc(*"MJPG")
cap.set(cv2.CAP_PROP_FOURCC, fourcc)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Error: Cannot open camera.")
    exit()

print("Camera opened.")
print("Actual width :", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print("Actual height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Actual fps   :", cap.get(cv2.CAP_PROP_FPS))
print("Actual fourcc:", int(cap.get(cv2.CAP_PROP_FOURCC)))

print("Press SPACE to save image.")
print("Press q to quit.")

# Count the number of saved images
count = 0

while True:
    ret, frame = cap.read() # Read each frame from the camera

    if not ret:
        print("Error: Cannot read frame.")
        break

    display = frame.copy()

    # Display the number of saved images on the frame
    cv2.putText(
        display,
        f"Saved images: {count}",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 0),
        2
    )

    # Display the frame in a window
    cv2.imshow("Capture Calibration Images", display)

    # Wait for a key press for 1 ms
    key = cv2.waitKey(1)
    # Accept SPACE to save the current frame as an image, or 'q' to quit the program
    if key == ord(" "):
        filename = os.path.join(SAVE_DIR, f"calib_{count:02d}.png")
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")
        count += 1

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()