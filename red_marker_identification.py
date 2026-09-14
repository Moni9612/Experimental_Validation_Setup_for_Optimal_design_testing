import cv2
import numpy as np
from google.colab.patches import cv2_imshow # For displaying OpenCV images in Colab

# Define the lower and upper bounds for red color in HSV
# Red color wraps around 0 and 180 in OpenCV's HSV hue range (0-179)
lower_red1 = np.array([0, 100, 100])
upper_red1 = np.array([10, 255, 255])

lower_red2 = np.array([160, 100, 100])
upper_red2 = np.array([179, 255, 255])

image_paths = [
    '/content/sample_data/camera1_image1.png',
    '/content/sample_data/camera1_image2.png',
    '/content/sample_data/camera2_image1.png',
    '/content/sample_data/camera2_image2.png'
]

# Dictionary to store marker centroids for each image
marker_centroids = {}

for img_path in image_paths:
    print(f"\nProcessing image: {img_path}")
    # Load the image using OpenCV
    image_cv_new = cv2.imread(img_path)

    if image_cv_new is None:
        print(f"Error: Could not load image from {img_path}")
        continue

    # Convert the image from BGR to HSV color space
    hsv_image_new = cv2.cvtColor(image_cv_new, cv2.COLOR_BGR2HSV)

    # Create masks for both red ranges (lower_red1, upper_red1, lower_red2, upper_red2 are assumed to be defined)
    mask_red1_new = cv2.inRange(hsv_image_new, lower_red1, upper_red1)
    mask_red2_new = cv2.inRange(hsv_image_new, lower_red2, upper_red2)

    # Combine the two red masks
    red_mask_new = cv2.add(mask_red1_new, mask_red2_new)

    # Find contours in the red mask
    contours_new, _ = cv2.findContours(red_mask_new, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours_new:
        largest_contour_new = max(contours_new, key=cv2.contourArea)
        M_new = cv2.moments(largest_contour_new)

        if M_new["m00"] != 0:
            cX_new = int(M_new["m10"] / M_new["m00"])
            cY_new = int(M_new["m01"] / M_new["m00"])

            # Store the centroid
            marker_centroids[img_path] = (cX_new, cY_new)

            image_with_marker_new = image_cv_new.copy()
            cv2.circle(image_with_marker_new, (cX_new, cY_new), 10, (0, 0, 255), -1) # Red circle
            cv2.putText(image_with_marker_new, "Red Marker", (cX_new - 20, cY_new - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            print("Image with Detected Red Marker:")
            cv2_imshow(image_with_marker_new)
            print(f"Red marker detected at: ({cX_new}, {cY_new})")
        else:
            print("No significant red marker found.")
    else:
        print("No red contours found.")

print("\nAll marker centroids:", marker_centroids)
