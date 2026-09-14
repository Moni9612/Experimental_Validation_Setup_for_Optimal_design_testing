import cv2
import numpy as np
from google.colab.patches import cv2_imshow

pose1_path = '/content/sample_data/pose1.png'
pose2_path = '/content/sample_data/pose2.png'
known_movement_mm = 13.0 # Provided by the user

# Dictionary to store centroids for calibration images
calibration_centroids = {}

for img_path in [pose1_path, pose2_path]:
    print(f"\nProcessing calibration image: {img_path}")
    img_cal = cv2.imread(img_path)

    if img_cal is None:
        print(f"Error: Could not load image from {img_path}")
        continue

    hsv_img_cal = cv2.cvtColor(img_cal, cv2.COLOR_BGR2HSV)

    # Reuse the red color bounds defined earlier
    mask_red1_cal = cv2.inRange(hsv_img_cal, lower_red1, upper_red1)
    mask_red2_cal = cv2.inRange(hsv_img_cal, lower_red2, upper_red2)
    red_mask_cal = cv2.add(mask_red1_cal, mask_red2_cal)

    contours_cal, _ = cv2.findContours(red_mask_cal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours_cal:
        largest_contour_cal = max(contours_cal, key=cv2.contourArea)
        M_cal = cv2.moments(largest_contour_cal)

        if M_cal["m00"] != 0:
            cX_cal = int(M_cal["m10"] / M_cal["m00"])
            cY_cal = int(M_cal["m01"] / M_cal["m00"])
            calibration_centroids[img_path] = (cX_cal, cY_cal)

            img_with_marker_cal = img_cal.copy()
            cv2.circle(img_with_marker_cal, (cX_cal, cY_cal), 10, (0, 0, 255), -1)
            cv2.putText(img_with_marker_cal, "Red Marker", (cX_cal - 20, cY_cal - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2_imshow(img_with_marker_cal)
            print(f"Red marker detected at: ({cX_cal}, {cY_cal})")
        else:
            print("No significant red marker found in calibration image.")
    else:
        print("No red contours found in calibration image.")

# Calculate pixel movement and PIXEL_TO_MM_RATIO
if pose1_path in calibration_centroids and pose2_path in calibration_centroids:
    x1_pose, _ = calibration_centroids[pose1_path]
    x2_pose, _ = calibration_centroids[pose2_path]

    pixel_movement = abs(x2_pose - x1_pose)

    if pixel_movement > 0:
        calibrated_PIXEL_TO_MM_RATIO = known_movement_mm / pixel_movement
        print(f"\nHorizontal pixel movement between pose1.png and pose2.png: {pixel_movement} pixels")
        print(f"Given real-world movement: {known_movement_mm} mm")
        print(f"Calibrated PIXEL_TO_MM_RATIO: {calibrated_PIXEL_TO_MM_RATIO:.4f} mm/pixel")
    else:
        print("Pixel movement is zero, cannot calculate PIXEL_TO_MM_RATIO.")
else:
    print("Could not find centroids for both pose1.png and pose2.png for calibration.")
