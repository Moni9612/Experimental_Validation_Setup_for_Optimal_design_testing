# Experimental Validation Setup for Optimal Design Testing

This repository contains the experimental setup and image-processing workflow used to validate the deformation behaviour of optimized Fin-Ray finger designs under controlled contact conditions.

The experimental validation combines:

- Dynamixel-based finger actuation
- An Ecoflex 00-50 deformable half-proxy with a rigid core
- Two approximately orthogonal camera views
- Red-marker-based fingertip tracking
- Pixel-to-millimetre camera calibration
- Three-dimensional fingertip displacement measurement

The overall experimental setup and validation procedure are illustrated below.

<p align="center">
  <img src="Experimental_setup_for_validation.jpeg" alt="Experimental setup for validation" width="95%">
</p>

## Experimental Displacement Measurement

The three-dimensional fingertip displacement of the Fin-Ray finger was experimentally quantified using a calibrated multi-view marker-tracking approach. A red marker attached to the fingertip was tracked between the initial and deformed configurations from two approximately orthogonal camera views.

The imaging system was first calibrated using a known 13-mm displacement to determine a pixel-to-millimetre conversion factor. For marker localization, each image was converted from BGR to HSV colour space, and the red marker was isolated using dual-range HSV colour thresholding to account for the hue wrap-around of red. The resulting binary masks were combined, followed by contour extraction and largest-contour selection to identify the marker region. Its centre was then determined using spatial image moments and centroid estimation.

The differences between the initial and final centroid coordinates provided the pixel-level marker displacement, which was converted to physical displacement using the calibrated scale. Horizontal and vertical displacements from the first camera view provided two in-plane displacement components, while horizontal displacement from the approximately orthogonal second view captured the remaining out-of-plane component. Finally, these measurements were mapped to the corresponding $\Delta x$, $\Delta y$, and $\Delta z$ COMSOL coordinate directions according to the known camera orientations, enabling direct experimental validation of the simulated fingertip deformation.

---

# Experimental Procedure

## 1. Prepare the Experimental Workspace

Clone or place the following components in the same working location:

```text
dynamixel_motor
DynamixelSDK
camera.py
```

The `dynamixel_motor` package is used to control the Dynamixel actuator through ROS 2, while `camera.py` simultaneously accesses the two approximately orthogonal cameras and captures the image pairs required for fingertip displacement measurement.

---

## 2. Start the Two-Camera Image Acquisition

Run:

```bash
python3 camera.py
```

The program opens the two camera views simultaneously.

Before actuating the Fin-Ray finger, press:

```text
1
```

to capture the **initial configuration** from both cameras.

This produces the initial images corresponding to the two approximately orthogonal views.

Keep the camera program running while proceeding with the Dynamixel actuation.

---

## 3. Start the Dynamixel Motor Node

Open a separate terminal window and run:

```bash
ros2 run dynamixel_motor dynamixel_position_node
```

This starts the Dynamixel motor control subscriber node and waits for a command to be published to the motor-control topic.

Keep this terminal running.

---

## 4. Actuate the Fin-Ray Finger

Open another terminal window and publish the forward command:

```bash
ros2 topic pub --once /motor_control std_msgs/msg/String "{data: forward}"
```

The Dynamixel actuator moves the gripper forward, causing the Fin-Ray finger to deform against the **Ecoflex 00-50 half-proxy with rigid core**.

---

## 5. Capture the Deformed Configuration

After the finger has contacted and deformed against the half-proxy, return to the previously opened camera window.

Press:

```text
2
```

to capture the **post-contact/deformed configuration** from both cameras.

Then press:

```text
q
```

to exit the camera program.

The image-acquisition stage therefore provides four images:

```text
camera1_image1.png    # Camera 1 - initial configuration
camera1_image2.png    # Camera 1 - deformed configuration

camera2_image1.png    # Camera 2 - initial configuration
camera2_image2.png    # Camera 2 - deformed configuration
```

These four images are subsequently used for marker detection and three-dimensional fingertip displacement calculation.

---

# Image Processing and Displacement Calculation

The image-processing stage is performed in a separate **Google Colab** environment using the three Python scripts provided in this repository:

```text
Cell 1 → pixel_to_mm_ratio_calculation.py
Cell 2 → red_marker_identification.py
Cell 3 → tip_displacement_calculation.py
```

The first cell performs a **one-time camera calibration**, while Cells 2 and 3 are repeated for each experimental test position.

---

## 6. Pixel-to-Millimetre Calibration

Copy the contents of [`pixel_to_mm_ratio_calculation.py`](pixel_to_mm_ratio_calculation.py) into **Cell 1** of the Google Colab notebook.

Before executing the code, a calibration image pair must be obtained.

Using the Dynamixel motor control setup, move the fingertip through a **known displacement along the x-axis**, from calibration position A to position B.

For the calibration used in this experiment, the known displacement is:

```text
13.0 mm
```

Use `camera.py` to capture the fingertip at positions A and B and provide the two calibration images to the Colab environment as:

```text
pose1.png
pose2.png
```

Then execute:

```python
# Cell 1
# Use the contents of:
# pixel_to_mm_ratio_calculation.py
```

The code detects the red fingertip marker in both calibration images and calculates its horizontal pixel displacement.

The pixel-to-millimetre conversion factor is calculated as:

**PIXEL_TO_MM_RATIO = Known displacement (mm) / Measured displacement (pixels)**

The resulting value is expressed in:

```text
mm/pixel
```

and is subsequently used to convert image-space marker displacement into physical fingertip displacement.

> **Important:** `pixel_to_mm_ratio_calculation.py` only needs to be executed **once** for the calibrated camera setup. The resulting `PIXEL_TO_MM_RATIO` should then be used for the subsequent displacement calculations.

---

## 7. Red-Marker Identification

Copy the contents of [`red_marker_identification.py`](red_marker_identification.py) into **Cell 2**.

Upload the four images obtained during the experimental test:

```text
camera1_image1.png
camera1_image2.png
camera2_image1.png
camera2_image2.png
```

Then execute:

```python
# Cell 2
# Use the contents of:
# red_marker_identification.py
```

The code identifies the red fingertip marker in each of the four images.

The marker-detection procedure consists of:

- Loading each captured image
- Converting the image from BGR to HSV colour space
- Applying two HSV threshold ranges for red
- Combining the two red binary masks
- Extracting contours from the resulting mask
- Selecting the largest detected red contour
- Calculating spatial image moments
- Estimating the centroid of the red marker
- Storing the marker coordinates for subsequent displacement calculation

Two HSV ranges are used because red wraps around the beginning and end of the OpenCV HSV hue range.

The detected marker position is also displayed on each image, allowing the marker localization to be visually verified before proceeding with the displacement calculation.

---

## 8. Three-Dimensional Fingertip Displacement Calculation

Copy the contents of [`tip_displacement_calculation.py`](tip_displacement_calculation.py) into **Cell 3**.

Before executing the code, update:

```python
PIXEL_TO_MM_RATIO = 0.25
```

with the actual calibrated value obtained from **Cell 1**.

Then execute:

```python
# Cell 3
# Use the contents of:
# tip_displacement_calculation.py
```

The script uses the marker centroids identified in Cell 2 to calculate the fingertip displacement between the initial and deformed configurations.

### Camera 1

The horizontal marker displacement between:

```text
camera1_image1.png
```

and:

```text
camera1_image2.png
```

is used to determine the displacement corresponding to:

$$\Delta x$$

The vertical marker displacement from the same image pair is used to determine:

$$\Delta y$$

### Camera 2

The horizontal marker displacement between:

```text
camera2_image1.png
```

and:

```text
camera2_image2.png
```

is used to determine the remaining approximately orthogonal displacement component:

$$\Delta z$$

The pixel displacements are converted to millimetres using:

**Δd (mm) = Δd (pixels) × PIXEL_TO_MM_RATIO**

The resulting measurements are therefore mapped as:

```text
Camera 1 horizontal displacement → Δx
Camera 1 vertical displacement   → Δy
Camera 2 horizontal displacement → Δz
```

according to the known camera orientations relative to the COMSOL coordinate system.

The final output provides the experimentally measured:

```text
Δx (mm)
Δy (mm)
Δz (mm)
```

for the tested Fin-Ray finger configuration.

---

# Repeating the Measurement Across Test Positions

The experimental displacement measurement must be performed at the **P1, P2, and P3 positions** for every Fin-Ray finger design being evaluated, following the test arrangement shown in `Experimental_setup_for_validation.jpeg`.

The camera calibration performed using:

```text
pixel_to_mm_ratio_calculation.py
```

is required only once for the calibrated camera configuration.

For each finger and each test position:

```text
P1
P2
P3
```

repeat the following procedure:

1. Position the experimental setup at the required test position.
2. Run `camera.py`.
3. Press `1` to capture the two initial images.
4. Start the Dynamixel motor node if it is not already running.
5. Publish the forward motor command.
6. Allow the Fin-Ray finger to contact and deform against the Ecoflex 00-50 half-proxy.
7. Return to the camera window and press `2` to capture the two deformed-state images.
8. Press `q` to exit the camera program.
9. Upload the four captured images to Google Colab.
10. Run `red_marker_identification.py`.
11. Run `tip_displacement_calculation.py`.
12. Record the resulting $\Delta x$, $\Delta y$, and $\Delta z$ fingertip displacements.

Therefore, **Codes 2 and 3 must be repeated for P1, P2, and P3 for every finger design tested**, while Code 1 is used only for the initial camera calibration.

---

# Workflow Summary

<p align="center">
  <img src="workflow.jpeg" alt="workflow" width="95%">
</p>

The experimentally measured three-dimensional fingertip displacements can subsequently be compared with the corresponding simulated responses for validation of the optimized Fin-Ray finger designs.
