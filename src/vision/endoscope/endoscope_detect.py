import cv2
from supercamera import Camera

# The 'with' statement handles opening and closing the camera safely
with Camera() as cam:
    while True:
        # Pull the raw USB data directly into a BGR numpy array
        ret, frame = cam.read()
        
        # If a frame drops or is corrupted, skip to the next one
        if not ret:
            continue
            
        # Tell OpenCV to generate a window and display the current frame
        cv2.imshow("UseePlus Endoscope Feed", frame)
        
        # Keep the window open and wait for the 'q' key to be pressed to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Destroy the GUI window when the loop breaks
cv2.destroyAllWindows()