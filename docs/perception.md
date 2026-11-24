# Lidar perception: detection, tracking and prediction (work in progress)

Detection, tracking and short-term prediction of the moving traffic around the truck: terminal trucks, reach
stackers, forklifts and people on foot in the container yard. Written in C++ (ROS 2 Humble), with no GPU, and
scored every scan against the ground truth of the simulated traffic. This page shows where it stands now,
including what does not work yet.

## Pipeline

1. **Six lidars, one cloud:** the same idea as commercial robotaxi rigs (one long-range roof lidar plus near-field
   blind-spot lidars), plus a lidar on the trailer. The tractor and trailer are cut out of every scan using the
   live hitch angle.
2. **Detection:** ground removal, a 0.3–9 m height band (a reach stacker holds its container up to 8.7 m), voxel
   downsampling, Euclidean clustering on the ground plane, and an oriented box per cluster.
3. **Road area from the lane map:** detections off the Lanelet2 lanes (+1.5 m) are dropped, the same idea as
   Autoware's lanelet filter. Container rows next to the road never become tracks.
4. **Free-space evidence:** a grid remembers where the lidar recently saw empty space. A vehicle that drives into
   that space is moving. A container whose visible part slides along as the truck passes is not.
5. **Tracking:** an EKF per object with a physics model (constant turn rate and velocity, CTRV), Hungarian
   association, a velocity from the first two detections, and an anchor point for long vehicles seen only in part.
6. **Prediction, 0–4 s, per object:** constant velocity, the CTRV arc, lane following on the Lanelet2 map, and an
   **adaptive** choice. Each track keeps checking its own 1 s predictions and uses the model that has been right
   lately. That needs no ground truth, so it runs live.

## Results so far

**Real drive:** about 10 minutes through the changed yard with 13 scripted vehicles, replayed into the tracker.

| | |
|---|---|
| Processing time | **8–11 ms per scan** on a laptop CPU (max 55 ms) |
| Recall / precision (moving objects within 50 m, on the road) | 0.66 / 0.46 |
| Recall per kind | forklift **0.96**, worker 0.68, ext. truck 0.49, stacker 0.41, truck 0.33 |
| Prediction error at 1 / 2 / 3 s, adaptive (best of the four) | 1.80 / 5.55 / 6.83 m |
| … constant velocity | 1.86 / 5.78 / 7.47 m |
| … CTRV (physics) | 2.14 / 6.38 / 8.35 m |

Forklifts and people work. Long, fast trucks and too many false tracks don't yet. The current work is a
scan-by-scan recording of this drive, to find where each false track and each missed truck comes from.

**Controlled tests (synthetic, unit tests):**

| | |
|---|---|
| Vehicle on a 30 m circle, 3 s prediction: constant velocity → CTRV | 5.32 m → **0.35 m** |
| Synthetic yard, speed-only tracking → with road area | precision 0.01 → **1.00** (35,732 → 0 false tracks) |

## Problems found on the way

- **Point clouds lost between processes.** Six lidar clouds of about 0.7 MB each at 10 Hz: with ROS 2's default
  Fast DDS settings, messages over the 0.5 MB shared-memory limit go over UDP in fragments, and one lost fragment
  drops the whole cloud. Measured: 18 of 100 merged clouds arrived. With larger shared-memory segments: 99 of 100.
- **Python was the bottleneck.** The Python lidar merge managed about 1.5 clouds per second against 10 Hz lidars.
  Rewritten in C++, it keeps up at full rate. This also means the earlier six-lidar SLAM benchmark ran at about
  1.5 Hz lidar. The comparison between methods stays fair (same input for all), and a full-rate rerun is planned.
- **Ground truth must come from the model, not a guess.** Box sizes taken from the world generator, instead of
  estimates, moved reach-stacker recall from 0 to 0.41. Their raised container (6.1–8.7 m) was above the
  detector's height band.
- **PCL clustering split vehicles by height:** far lidar rings are more than the clustering distance apart
  vertically. Fixed by clustering on the ground plane.

## Next

- Find the sources of the false tracks and the missed trucks in the scan recording, then tune.
- A fresh full-rate recording of the drive.
- A learned detector (PointPillars, trained in the cloud on auto-labelled simulator drives, TensorRT on the laptop
  GPU), compared with this classical pipeline on the same ground truth.
