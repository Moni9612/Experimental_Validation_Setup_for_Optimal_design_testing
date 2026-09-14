import os
import time

import cv2
import numpy as np
import pyrealsense2 as rs


# ============================================================
# SAVE LOCATION
# ============================================================

save_folder = "/home/moniesha/Camera_measurements/Rank2/O_0_ecoflex/P0_6"
os.makedirs(save_folder, exist_ok=True)


# ============================================================
# HELPERS
# ============================================================

def get_camera_devices(max_cameras=2):
    ctx = rs.context()
    devices = ctx.query_devices()
    return list(devices[:max_cameras])


# ============================================================
# INITIALIZE REALSENSE DEVICES
# ============================================================

camera_devices = get_camera_devices(max_cameras=2)

if len(camera_devices) < 2:
    raise RuntimeError(
        f"Expected 2 RealSense cameras, but found {len(camera_devices)}. "
        "Please connect both cameras and run the script again."
    )

cameras = []
for camera_index, device in enumerate(camera_devices, start=1):
    serial = device.get_info(rs.camera_info.serial_number)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    pipeline.start(config)
    time.sleep(2)

    cameras.append({
        "id": camera_index,
        "serial": serial,
        "pipeline": pipeline,
    })

    print(f"Camera {camera_index} connected. Serial: {serial}")

print("\nCamera 1 and Camera 2 are assigned in the order they were detected.")
print(f"Images will be saved to: {save_folder}")
print("Controls:")
print("  Press '1' -> Capture both cameras for shot 1")
print("  Press '2' -> Capture both cameras for shot 2")
print("  Press 'q' -> Quit")


# ============================================================
# CAPTURE STATUS
# ============================================================

shot1_taken = False
shot2_taken = False


# ============================================================
# LIVE DISPLAY
# ============================================================

try:
    while True:
        display_frames = []

        for cam in cameras:
            frames = cam["pipeline"].wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            display_image = color_image.copy()

            if not shot1_taken and not shot2_taken:
                status = "Press 1 to capture shot 1"
            elif shot1_taken and not shot2_taken:
                status = "Shot 1 saved - Press 2 to capture shot 2"
            else:
                status = "Shot 2 saved - Press q to quit"

            cv2.putText(
                display_image,
                f"Camera {cam['id']} - {status}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            display_frames.append((cam["id"], display_image))

        for camera_id, display_image in display_frames:
            cv2.imshow(f"Camera {camera_id}", display_image)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('1'):
            for cam in cameras:
                frames = cam["pipeline"].wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                save_path = os.path.join(
                    save_folder,
                    f"camera{cam['id']}_image1.png"
                )
                cv2.imwrite(save_path, color_image)
                print(f"Saved: {save_path}")

            shot1_taken = True
            print("\nShot 1 captured from both cameras.")

        elif key == ord('2'):
            for cam in cameras:
                frames = cam["pipeline"].wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_image = np.asanyarray(color_frame.get_data())
                save_path = os.path.join(
                    save_folder,
                    f"camera{cam['id']}_image2.png"
                )
                cv2.imwrite(save_path, color_image)
                print(f"Saved: {save_path}")

            shot2_taken = True
            print("\nShot 2 captured from both cameras.")

        elif key == ord('q'):
            break

finally:
    for cam in cameras:
        cam["pipeline"].stop()

    cv2.destroyAllWindows()
    print("\nBoth camera pipelines stopped.")

    if shot1_taken and shot2_taken:
        print("All four images were saved successfully.")
    elif shot1_taken:
        print("Shot 1 was captured; Shot 2 has not been captured yet.")
    elif shot2_taken:
        print("Shot 2 was captured; Shot 1 has not been captured yet.")
    else:
        print("No images were captured.")
