import os

if 'GTK_PATH' in os.environ:
    del os.environ['GTK_PATH']
if 'LD_LIBRARY_PATH' in os.environ:
    del os.environ['LD_LIBRARY_PATH']

import h5py
import cv2

# Open the file in read mode
with h5py.File('camera_data_ros.h5', 'r') as f:
    dataset = f['live_feed']
    
    print(f"Total frames saved: {dataset.shape[0]}")
    
    # Loop through the dataset one frame at a time
    for i in range(dataset.shape[0]):
        frame = dataset[i] 
        
        # Display the frame in a window
        cv2.imshow('H5 Video Playback', frame)
        
        # Wait 30 milliseconds before the next frame (~30 FPS)
        # Also listens for the 'q' key to quit early
        if cv2.waitKey(30) & 0xFF == ord('q'):
            print("Playback stopped by user.")
            break

# Close the window when done
cv2.destroyAllWindows()
