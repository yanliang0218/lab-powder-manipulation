## Project part: Camera pose
---
# Description:
- Get Camera pose by Apriltag detector. Editable tagsize and Tag family (You can find tag images for the pre-generated layouts at:https://github.com/AprilRobotics/apriltag-imgs. I recommend using the tagStandard41h12 layout.)
- After executing these codes, camera pose with timestamp will be stored in 'vision_dataset/current_time/vision_log.csv' file. Calebration chessboard images will be stored in the folder 'image' and all images captured by camera will be stored in folder 'vision_dataset/current_time/image_raw' or 'vision_dataset/current_time/image_annotated'
---
# How to use
1. Run the file 'capture_calibration_images.py' to capture at least 30 images of chessboard.
2. Run the file 'calibrate_camera.py' to calibrate camera.
3. Run the file 'collect_vision_dataset.py'

