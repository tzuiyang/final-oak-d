# Slide 1 Notes: Perception Pipeline

**One-minute script**

Our estimated state is the person's 3-D position relative to the OAK-D camera:

```text
P_obj = (x, y, z) meters
```

We are not estimating the robot's global pose. We only need the target's relative position.

The OAK-D gives us two measurements. The RGB image goes into YOLOv8n to find the class and bounding box. The mono left and right cameras give stereo depth. DepthAI fuses these into:

```text
(class, x, y, z)
```

For control, we convert that state into distance and bearing:

```text
d_hat = sqrt(x^2 + y^2 + z^2)
bearing = atan2(x, z)
```

This is similar to a measurement update in class because raw sensors become a state estimate. But it is not a Kalman filter: there is no prediction step, covariance, or map. Each frame gives a new depth-fused detection. The only smoothing is a small action decay during brief detection dropouts.

**Key points**

- State: target/person position in camera frame.
- Inputs: RGB + stereo mono pair.
- Method: YOLOv8n + stereo depth fusion.
- Target class: `person`.
- Confidence threshold: `0.7`.
- Depth cleanup: `5x5` median filter.
- Filter class: per-frame spatial detection, not Kalman/EKF/SLAM.
