import cv2
import os
import time
from supercamera import Camera

# Create a new directory for saving captured images
SAVE_DIR = "images"
os.makedirs(SAVE_DIR, exist_ok=True)

print("Press SPACE to save image.")
print("Press q to quit.")

count = 0
prev_time = time.time()

with Camera() as cam:
    print("Camera opened via supercamera.")
    
    # Grab an initial frame just to extract the hardware resolution
    ret, initial_frame = cam.read()
    if ret:
        height, width, _ = initial_frame.shape
        print(f"Actual width : {width}")
        print(f"Actual height: {height}")
        print("Actual fourcc: N/A (Proprietary JPEG stream via libusb)")
        print("Streaming...")
    
    while True:
        ret, frame = cam.read() 

        if not ret:
            continue
            
        # Calculate the real-time FPS
        current_time = time.time()
        fps = 1.0 / (current_time - prev_time) if (current_time - prev_time) > 0 else 0
        prev_time = current_time

        display = frame.copy()

        # Display the live FPS on the feed
        cv2.putText(
            display,
            f"FPS: {int(fps)}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        # Display the number of saved images
        cv2.putText(
            display,
            f"Saved images: {count}",
            (20, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imshow("Capture Calibration Images", display)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(" "):
            filename = os.path.join(SAVE_DIR, f"calib_{count:02d}.png")
            cv2.imwrite(filename, frame) 
            print(f"Saved: {filename}")
            count += 1

        elif key == ord("q"):
            break

cv2.destroyAllWindows()