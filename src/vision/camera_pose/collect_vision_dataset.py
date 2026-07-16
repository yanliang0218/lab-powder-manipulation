import cv2
import csv
import os
import time
import numpy as np
from pupil_apriltags import Detector


# =========================
# 1. Parameters
# =========================
TAG_FAMILY = "tag25h9" #AprilTag ID
TAG_SIZE = 0.09
CAMERA_ID = 0

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
TARGET_FPS = 150

# Build a new folder for each dataset collection session, using timestamp as the folder name
session_name = time.strftime("%Y%m%d_%H%M%S")
DATASET_DIR = os.path.join("vision_dataset", session_name)

RAW_DIR = os.path.join(DATASET_DIR, "images_raw")
ANNOTATED_DIR = os.path.join(DATASET_DIR, "images_annotated")
CSV_PATH = os.path.join(DATASET_DIR, "vision_log.csv")

os.makedirs(RAW_DIR, exist_ok=True)
os.makedirs(ANNOTATED_DIR, exist_ok=True)


# =========================
# 2. Load camera calibration
# =========================

calib = np.load("camera_calibration.npz")
camera_matrix = calib["camera_matrix"].astype(np.float64)
dist_coeffs = calib["dist_coeffs"].astype(np.float64)

print("Loaded camera_matrix:")
print(camera_matrix)
print("Loaded dist_coeffs:")
print(dist_coeffs)


# =========================
# 3. undistortion setup
# =========================
# getOptimalNewCameraMatrix + initUndistortRectifyMap + remap + ROI crop

image_size = (IMAGE_WIDTH, IMAGE_HEIGHT)

# Get the optimal new camera matrix for undistortion
new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
    camera_matrix,
    dist_coeffs,
    image_size,
    alpha=0,          # alpha=0: Cut black borders；alpha=1: maintain all pixels (may have black borders)
    newImgSize=image_size
)

# Define 2 mapping arrays for remap function
map1, map2 = cv2.initUndistortRectifyMap(
    camera_matrix,
    dist_coeffs,
    None,
    new_camera_matrix,
    image_size,
    cv2.CV_16SC2
)

roi_x, roi_y, roi_w, roi_h = roi

# notice:
# We will cut the undistorted image to the ROI, so the final image size will be (roi_w, roi_h) 
fx = new_camera_matrix[0, 0]
fy = new_camera_matrix[1, 1]
cx = new_camera_matrix[0, 2] - roi_x  # roi_w
cy = new_camera_matrix[1, 2] - roi_y  # roi_h

print("New camera matrix:")
print(new_camera_matrix)
print(f"ROI = {roi}")
print(f"Pose camera params = fx={fx}, fy={fy}, cx={cx}, cy={cy}")


# =========================
# 4. Transform functions
# =========================

def invert_transform(R, t):
    R_inv = R.T
    t_inv = -R_inv @ t
    return R_inv, t_inv


def rotation_matrix_to_euler_zyx(R):
    """
    Convention:
    R = Rz(yaw) * Ry(pitch) * Rx(roll)

    ZYX Eular angles rotation matrix
    a = yaw, b = pitch, c = roll
    R = [cosc*cosb, cosc*sinb*sina - sinc*cosa, cosc*sinb*cosa + sinc*sina;
         sinc*cosb, sinc*sinb*sina + cosc*cosa, sinc*sinb*cosa - cosc*sina;
         -sinb,     cosb*sina,                  cosb*cosa]
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)  # |cos(pitch)|
    singular = sy < 1e-6 # In ZYX sequence, when pitch ~= ±90°, roll and yaw are not uniquely defined

    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0

    return roll, pitch, yaw


# =========================
# 5. AprilTag detector
# =========================

detector = Detector(
    families=TAG_FAMILY,
    nthreads=1,
    quad_decimate=1.0,
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0
)


# =========================
# 6. Camera setup
# =========================

cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)

# 关键：强制使用 MJPG，否则可能默认 YUYV，帧率会被限制到 30fps
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
cap.set(cv2.CAP_PROP_FOURCC, fourcc)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

# 尽量减少 buffer，降低滞后
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

if not cap.isOpened():
    print("Error: Cannot open camera.")
    exit()

print("Camera opened.")
print("Press q to quit.")
print("Actual camera width :", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print("Actual camera height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print("Actual camera fps   :", cap.get(cv2.CAP_PROP_FPS))
print("Actual fourcc       :", int(cap.get(cv2.CAP_PROP_FOURCC)))


# =========================
# 7. CSV setup
# =========================
# 每检测到一个 tag，写一行
# 如果这一帧没有检测到 tag，也写一行（tag_id = -1）

csv_file = open(CSV_PATH, mode="w", newline="")
csv_writer = csv.writer(csv_file)

csv_writer.writerow([
    "frame_id",
    "raw_image",
    "annotated_image",
    "elapsed_time_ns",
    "tag_detected",
    "num_tags",
    "tag_id",
    "camera_x_m",
    "camera_y_m",
    "camera_z_m",
    "camera_roll_deg",
    "camera_pitch_deg",
    "camera_yaw_deg"
])


# =========================
# 8. Main loop
# =========================

frame_id = 0
program_start_ns = time.time_ns()


print(f"Program start time ns: {program_start_ns}")

try:
    while True:
        ret, frame = cap.read()
        timestamp_ns = time.time_ns()

        elapsed_time_ns = timestamp_ns - program_start_ns

        if not ret:
            print("Error: Cannot read frame.")
            break

        frame_id += 1

        raw_filename = f"frame_{frame_id:06d}.png"
        annotated_filename = f"frame_{frame_id:06d}.png"

        raw_path = os.path.join(RAW_DIR, raw_filename)
        annotated_path = os.path.join(ANNOTATED_DIR, annotated_filename)

        # -------------------------
        # A. Save raw image
        # -------------------------
        # raw 保留原始图像，不去畸变
        cv2.imwrite(raw_path, frame)

        # -------------------------
        # B. Undistort (final version)
        # -------------------------
        undistorted = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)

        # ROI crop
        undistorted = undistorted[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]

        annotated = undistorted.copy()
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        # -------------------------
        # C. Detect AprilTag
        # -------------------------
        results = detector.detect(  
            # Alternatively, you can use OpenCv's Pnp solver with SOLVEPNP_IPPE_SQUARE
            gray,
            estimate_tag_pose=True,
            camera_params=[fx, fy, cx, cy],
            tag_size=TAG_SIZE
        )

        tag_detected = len(results) > 0
        num_tags = len(results)

        tag_records = []

        for tag in results:
            tag_id = tag.tag_id

            R_camera_tag = tag.pose_R
            t_camera_tag = tag.pose_t

            R_tag_camera, t_tag_camera = invert_transform(
                R_camera_tag,
                t_camera_tag
            )

            camera_x = float(t_tag_camera[0, 0])
            camera_y = float(t_tag_camera[1, 0])
            camera_z = float(t_tag_camera[2, 0])

            roll, pitch, yaw = rotation_matrix_to_euler_zyx(R_tag_camera)

            roll_deg = float(np.degrees(roll))
            pitch_deg = float(np.degrees(pitch))
            yaw_deg = float(np.degrees(yaw))

            tag_records.append({
                "tag_id": tag_id,
                "camera_x": camera_x,
                "camera_y": camera_y,
                "camera_z": camera_z,
                "roll_deg": roll_deg,
                "pitch_deg": pitch_deg,
                "yaw_deg": yaw_deg
            })

            # Draw tag outline and center
            corners = tag.corners.astype(int)
            for i in range(4):
                pt1 = tuple(corners[i])
                pt2 = tuple(corners[(i + 1) % 4])
                cv2.line(annotated, pt1, pt2, (0, 255, 0), 2)

            center = tuple(tag.center.astype(int))
            cv2.circle(annotated, center, 5, (0, 255, 0), -1)

        # -------------------------
        # D. Draw only timestamp + pose
        # -------------------------
        # Green color for text
        green = (0, 255, 0)

        overlay_lines = [f"t_ns: {elapsed_time_ns}"]

        if len(tag_records) > 0:
            for record in tag_records:
                overlay_lines.append(
                    f"ID {record['tag_id']}: "
                    f"x={record['camera_x']:.3f}, "
                    f"y={record['camera_y']:.3f}, "
                    f"z={record['camera_z']:.3f}"
                )
                overlay_lines.append(
                    f"ID {record['tag_id']}: "
                    f"roll={record['roll_deg']:.2f}, "
                    f"pitch={record['pitch_deg']:.2f}, "
                    f"yaw={record['yaw_deg']:.2f}"
                )
        else:
            overlay_lines.append("No tag detected")

        y0 = 30
        for i, line in enumerate(overlay_lines):
            cv2.putText(
                annotated,
                line,
                (20, y0 + i * 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                green,
                2
            )

        # -------------------------
        # E. Save annotated image
        # -------------------------
        cv2.imwrite(annotated_path, annotated)

        # -------------------------
        # F. Write CSV
        # -------------------------
        if len(tag_records) > 0:
            for record in tag_records:
                csv_writer.writerow([
                    frame_id,
                    raw_path,
                    annotated_path,
                    elapsed_time_ns,
                    True,
                    num_tags,
                    record["tag_id"],
                    record["camera_x"],
                    record["camera_y"],
                    record["camera_z"],
                    record["roll_deg"],
                    record["pitch_deg"],
                    record["yaw_deg"]
                ])
        else:
            csv_writer.writerow([
                frame_id,
                raw_path,
                annotated_path,
                elapsed_time_ns,
                False,
                0,
                -1,
                np.nan,
                np.nan,
                np.nan,
                np.nan,
                np.nan,
                np.nan
            ])

        csv_file.flush()

        # -------------------------
        # G. Show image
        # -------------------------
        cv2.imshow("Vision Dataset Collection", annotated)

        key = cv2.waitKey(1)
        if key == ord("q"):
            break

finally:
    csv_file.close()
    cap.release()
    cv2.destroyAllWindows()

    print(f"Dataset saved to: {DATASET_DIR}")
    print(f"CSV log saved to: {CSV_PATH}")