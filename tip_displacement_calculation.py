PIXEL_TO_MM_RATIO = 0.25 # Updated to the calibrated value

# User-defined X: Horizontal movement of red marker from camera1_image1 to camera1_image2
# User-defined Y: Vertical movement of red marker from camera1_image1 to camera1_image2
img1_cam1_path = '/content/sample_data/camera1_image1.png'
img2_cam1_path = '/content/sample_data/camera1_image2.png'

if img1_cam1_path in marker_centroids and img2_cam1_path in marker_centroids:
    x1_cam1, y1_cam1 = marker_centroids[img1_cam1_path]
    x2_cam1, y2_cam1 = marker_centroids[img2_cam1_path]

    # Calculate X movement (horizontal for camera1)
    X_movement_pixels = x2_cam1 - x1_cam1
    X_movement_mm = X_movement_pixels * PIXEL_TO_MM_RATIO
    print(f"X (Horizontal movement) from {img1_cam1_path} to {img2_cam1_path}: {X_movement_pixels} pixels ({X_movement_mm:.2f} mm)")

    # Calculate Y movement (vertical for camera1)
    Y_movement_pixels = y2_cam1 - y1_cam1
    Y_movement_mm = Y_movement_pixels * PIXEL_TO_MM_RATIO
    print(f"Y (Vertical movement) from {img1_cam1_path} to {img2_cam1_path}: {Y_movement_pixels} pixels ({Y_movement_mm:.2f} mm)")
else:
    print(f"Could not find centroids for {img1_cam1_path} or {img2_cam1_path} to calculate X and Y movements.")

# User-defined Z: Horizontal movement of red marker from camera2_image1 to camera2_image2
img1_cam2_path = '/content/sample_data/camera2_image1.png'
img2_cam2_path = '/content/sample_data/camera2_image2.png'

if img1_cam2_path in marker_centroids and img2_cam2_path in marker_centroids:
    x1_cam2, _ = marker_centroids[img1_cam2_path]
    x2_cam2, _ = marker_centroids[img2_cam2_path]

    # Calculate Z movement (horizontal for camera2)
    Z_movement_pixels = x2_cam2 - x1_cam2
    Z_movement_mm = Z_movement_pixels * PIXEL_TO_MM_RATIO

    print(f"\nZ (Horizontal movement) from {img1_cam2_path} to {img2_cam2_path}: {Z_movement_pixels} pixels ({Z_movement_mm:.2f} mm)")
else:
    print(f"Could not find centroids for {img1_cam2_path} or {img2_cam2_path} to calculate Z movement.")