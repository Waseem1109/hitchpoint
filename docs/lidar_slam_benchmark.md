# Lidar SLAM / LIO benchmark

Eight open-source lidar odometry and SLAM methods, run one after the other on the **same recorded drive** of the
Hitchpoint articulated truck through a container yard (Gazebo, ROS 2 Humble, about 11 minutes). Every method gets
the same input and is scored against the Gazebo ground truth by the same evaluation node.

![All eight methods on the same replay](../media/hitchpoint_all_methods_readme.gif)

Green: Gazebo ground truth. Red: the method alone. Yellow: the method fused with wheel odometry and IMU in an EKF.
Sharper video (15x speed): [hitchpoint_all_methods_15x.mp4](../media/hitchpoint_all_methods_15x.mp4)

## Setup

- **Sensors:** six lidars on the truck (roof, two side, two low corner, one on the trailer rear) merged into one
  cloud in the roof lidar frame, an IMU, wheel odometry from the Ackermann controller, and RTK GNSS.
- **Self-filtering:** the tractor and trailer outline is cut out of every cloud; the trailer box is rotated with the
  live hitch (fifth-wheel) angle, so the truck never sees itself.
- **Replay:** one rosbag played into each method with simulation time taken from the sensor stamps, so every method
  sees the identical drive.
- **Scoring:** each estimate is aligned to the truth over its first seconds only, then left alone, so drift shows.
  ATE, maximum error, final error and relative error per distance are logged per method.
- **Fusion:** each method is also fed into a `robot_localization` EKF together with wheel odometry and IMU, and a
  second EKF adds GNSS (with a 60 s outage) and lidar map localization.

## Results (error at the end of the run)

| Method | Method alone | + wheels + IMU |
|---|---|---|
| LIO-SAM | 0.14 m | 0.19 m |
| Point-LIO | 0.58 m | 0.14 m |
| FAST-LIO2 | 0.61 m | 0.08 m |
| GLIM | 0.67 m | 0.16 m |
| Cartographer 3D | 0.82 m | 0.38 m |
| Cartographer 3D + wheel odometry | 1.31 m | 0.27 m |
| RKO-LIO | 2.96 m | 0.10 m |
| KISS-ICP (lidar only) | 22 m | 0.35 m |

With lidar map localization added on top, the fused estimate stays at a few centimetres, also during the GNSS outage.

## What went wrong, and what fixed it

**Methods "sticking" to the ground.** Gazebo's ground is perfectly flat and noise free, so a spinning lidar sees
exactly the same rings on it in every scan, wherever the truck is. Point-to-point ICP (KISS-ICP, RKO-LIO) and
Cartographer's scan matcher matched those rings to themselves and under-estimated the motion: KISS-ICP stood almost
still while the truck drove on (230 m error), Cartographer lost about 6 m the moment the truck started. Point-to-plane
methods (FAST-LIO2, Point-LIO) get no pull along the road from a plane and were not affected.
Fix: a second cloud without the ground (estimated per scan) for those three methods. Cartographer went from 6.9 m to
0.8 m, KISS-ICP from 230 m to 22 m. I reproduced the effect offline with KISS-ICP on a synthetic yard before changing
the pipeline (113 m with the ground, 0.5 m without).

**Point-LIO falling behind.** Point-LIO runs one filter update per lidar point; on the six-lidar cloud it published
at 0.14 Hz and fell 18 s behind real time, so its estimate never moved. Keeping every 12th point and replaying at
half speed brought it to 0.6 m.

**Fusion as a safety net.** Every method, including the two that drift, stays below 0.4 m once it is fused with wheel
odometry and IMU. A lidar method on its own is not enough for an articulated truck; inside a filter with the vehicle's
own sensors it is robust.

## Tools built for this

- `cloud_merge`: six lidars to one cloud, truck and trailer removed with the live hitch angle
- `live_viz`: RViz view of the truth, the method and the fused estimate, with live error text
- automatic recording: two virtual screens, two RViz views (top and isometric) and ffmpeg per method, unattended
- offline replay of the lidar input through KISS-ICP for tuning without ROS
