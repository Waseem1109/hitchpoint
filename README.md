# Hitchpoint

**An autonomous articulated truck (cabless tractor + 13.6 m semi-trailer) that drives a container port on its own
lane map, stops for people and obstacles ahead and behind, and reverses the trailer into a warehouse dock door to
within a few centimetres: built in ROS 2 and Gazebo, every module measured against ground truth.**

**Skills shown:** lidar map localization and sensor fusion (EKF), SLAM / LIO evaluation with a ground-truth scorer, articulated-vehicle control and docking, safety stop, ROS 2 / Gazebo simulation.

**ROS 2 Humble · Gazebo Harmonic · ros2_control · robot_localization · Python / NumPy · SolidWorks**

![Hitchpoint driving autonomously through the container port (Gazebo + RViz, sped up)](docs/Video_project.gif)

*Autonomous driving through the port: routing on the lane map, path tracking and the front safety corridor (Gazebo + RViz, sped up).* ▶ **[Full video](docs/Video_project.mp4)**

![Hitchpoint reversing a 13.6 m trailer into a warehouse dock door, fully autonomous (Gazebo, sped up)](docs/docking.gif)

*Autonomous reversing into a warehouse dock door: localization, lidar hitch-angle estimation and the reversing
controller running together in Gazebo (sped up ~30×).*


## Headline results (Gazebo, scored on ground truth)

| | |
|---|---|
| **Forward driving** | **724 m** route at up to **30 km/h**: **3.2 cm RMSE** from the lane centre (max 10.7 cm), stopped **0.0 cm** off the line at the goal |
| **Front safety stop** | Person on the road ahead: braked from 22 km/h and stopped **3.25 m** before them (target 3.0 m); drove on by itself ~2 s after they left |
| **Rear safety stop** | Low pallet (0.5 m) behind the trailer: stopped **1.02 m** from it (target 1.0 m); no false stops at the dock |
| **Road map** | 52 junctions, 182 lanes (8.3 km); all **450 / 450** junction turns checked clear for the full truck + trailer swept path |
| **Docking** | After **193 m in reverse**, the trailer stops **3.0 cm** off the door centre line, **0.34 m** from the bumper, **0.07°** off square |
| **Reversing** | 90° reversing turns end within **2.6 cm** and **0.43°**; a 2 m start offset is closed within ~15 m |
| **Hitch-angle estimation** | **0.22° RMSE** (max 1.4°) during hard reversing, from tractor-mounted lidar, no sensor on the trailer |
| **Localization** | **0.15 m RMSE** over 883 m (wheel odometry alone: 271 m) |
| **Without GNSS** | Roof lidar matched to a map of the port: **2.7 cm RMSE** (ideal map) over the full 2.6 km mission, docking included, with **no GNSS after the first 60 s** (wheels + IMU alone: 1.4 m) |
| **SLAM benchmark** | 8 open-source lidar SLAM / LIO methods replayed on the same drive (FAST-LIO2, Point-LIO, LIO-SAM, GLIM, RKO-LIO, KISS-ICP, Cartographer 3D ± wheel odometry): the map localization stays at **5 cm** (ideal map) without GNSS, about **10× better** than the best map-free odometry (it has a prior map, they do not); tight lidar-IMU methods drift ~0.1 %, ICP-based ones lose track on a 290 m straight road |

## What I built

![Hitchpoint vehicle model](docs/vehicle.png)

| Module | Approach |
|---|---|
| Vehicle model | URDF/xacro from my SolidWorks CAD (custom STEP → COLLADA converter that restores colours), Ackermann steering via `ros2_control`, articulated trailer, 6 lidars, 7 cameras, IMU, dual GNSS |
| Simulation world | Generated 840 × 500 m container port: ships, cranes, 25 yard blocks, road grid, warehouse with 8 dock doors; geometry merged into tiles for real-time sensor rendering |
| Localization | Dual EKF: wheel odometry + IMU in `odom`, + RTK GNSS in `map` |
| Lidar map localization | 2.5D signed-distance map of the port (4 height slabs, 0.25 m); Gauss-Newton scan matching with robust weights and a covariance from the fit, fused into the global EKF; wide search to relocalize; live map update (log-odds with 3D ray clearing) for a yard that changes |
| SLAM benchmark | Record once in Gazebo, replay into every method; each scored alone, + IMU, + wheels + IMU and + lidar map; a sanity check drops odometry that disagrees with wheels and IMU |
| Hitch-angle estimation | 1-DOF fit of the trailer outline to lidar points + single-track kinematic model, fused in an EKF that applies late lidar scans at their own timestamp and replays newer inputs |
| Reversing control | Cascade: pure pursuit on the trailer axle → exact steady-state curvature-to-hitch-angle map → feedback-linearised hitch-angle loop, with jackknife protection |
| Docking | Plans a line-arc-line reversing path to a door's **world** position; map → world alignment from GNSS |
| Road map + routing | Lane graph generated from a road layout file; A\* with a straight-approach constraint so the trailer arrives aligned; speed profile from lane limits and lateral acceleration; swept-path check of every turn against the world; Lanelet2 export |
| Forward control | Path-curvature feed-forward with a 0.3 s preview + kinematic lateral feedback whose preview grows with speed; speed follows the planned profile, then a position loop with stop anticipation for the final metres |
| Front safety | Roof lidar, two corner lidars and a front radar → ground plane fitted per scan → obstacles inside the **swept corridor of tractor + trailer** (hitch angle integrated) 60 m ahead → braking curve with reaction time; stops on a silent sensor |
| Rear safety | Rear lidar (tilted 30° down) + radar watch the reversing corridor; obstacles are forgotten only on free-space evidence, not just because they are no longer seen |
| Mission | Sequencer: drive to a crane, dock or parking bay, reverse into docks and bays, dwell, continue; waits for GNSS alignment before planning |
| Evaluation | Every module has an evaluator against Gazebo ground truth (CSV, summary, plots) and offline unit tests |

## Driving the port

![Road map with an example mission](docs/road_map.png)

The road map is generated from a layout file of the terminal roads: one lane per direction, turn connectors at
every junction, places (STS cranes, dock doors, parking bays) attached to their lanes. Every connector is checked
against the world geometry with the swept path of the tractor **and** the 13.6 m trailer, hitch angle integrated.

| Check | Result |
|---|---|
| Junction turns, swept path clear of the world (≥ 0.5 m) | **450 / 450** (tightest 1.5 m) |
| STS cranes / dock doors serviceable, reachable from the gate and back | **5 / 5**, **8 / 8** |
| Lanelet2 export: routing-graph errors / route match on 16 test routes | **0** / within **4 cm** |
| Example mission gate → crane 3 → dock 2 → bay n16 → gate | 2.49 km, ~8.8 min of driving |

## Forward driving and front safety stop

![Forward driving with a person on the road: front safety stop and restart](docs/front_safety_stop.png)

*Gazebo, gate → crane 3. A person is placed on the road 60 m ahead of the truck and removed after the stop.*

| Gate → crane 3 (724 m, three 90° turns) | Result (ground truth) |
|---|---|
| Speed | up to **8.3 m/s (30 km/h)**, following the planned profile |
| Cross-track error | **3.2 cm RMSE**, max 10.7 cm (at turn entry / exit) |
| At the goal | **0.0 cm** lateral, 7.7 cm short, 0.01° |
| Max hitch angle | 30° |
| Person on the road | speed limit drops from ~40 m away; braked from 6.2 m/s at ~1.4 m/s²; stopped **3.25 m** before the person (target 3.0 m) |
| Restart | ~2 s after the person left (1.5 s clear + cell expiry), no operator action |

The front corridor is not a straight box: it is the area the tractor and trailer **will sweep** along the route,
so in a turn the trailer's inside corner is protected and a person standing beside a straight road is not.

## Safety stop behind the trailer

![Rear safety stop: low pallet behind the trailer, released by itself after removal](docs/rear_safety_pallet.png)

| Test (Gazebo, reversing at 2 m/s) | Result |
|---|---|
| Low pallet (1.2 × 0.8 × 0.5 m) on the path | stopped **1.02 m** from it (estimated 0.98 m); released by itself 2.7 s after removal |
| Person on the path | detected 14.9 m away, stopped **1.03 m** from them |
| Person 2.3 m beside the path | no stop, no slow-down |
| Dock door 1, no obstacle (bumper and platform behind the goal) | **no false stop**; docked 3.4 cm off the centre line |

## Docking

![Docking into door 0 after 193 m in reverse](docs/dock_door0.png)

| At the dock (trailer rear, ground truth) | Door 0 (193 m approach) | Door 1 (58 m approach) |
|---|---|---|
| lateral offset from the door centre line | **+3.0 cm** | −1.7 cm |
| gap to the bumper (target 0.30 m) | **0.336 m** | 0.335 m |
| angle | **−0.07°** | −0.01° |

On the 137 m straight part of the approach the trailer stayed within ±3.5 cm of the path.

## Reversing

A trailer reversing on its own is unstable: the hitch angle grows until it jackknifes. The controller steers the
tractor to hold a target hitch angle, and chooses that angle so the trailer follows the path.

![90 degree reversing turn](docs/reverse_left.png)

| Run | Cross-track RMSE | Final lateral | Final heading | Max hitch |
|---|---|---|---|---|
| straight, starting 2 m off the line | 0.12 m once converged | +12.9 cm | −0.18° | 25° |
| 90° left, R = 20 m | 0.153 m | −1.5 cm | −0.22° | 25° |
| 90° right, R = 20 m | 0.160 m | −2.6 cm | +0.43° | 25° |

## Hitch-angle estimation

Real trailers carry no angle sensor (they are swapped between tractors), so the angle is estimated from the tractor.

![Hitch angle vs ground truth](docs/hitch_angle.png)

| Source | RMSE | Max | Availability |
|---|---|---|---|
| lidar fit | 0.245° | 0.85° | 99.8 % |
| kinematic model alone | 14.9° | 81.7° | 100 % |
| **fused EKF** | **0.223°** | **1.45°** | 100 % |

The model on its own runs away in reverse; the lidar keeps the fused estimate on track, and the fusion beats the
lidar alone.

## Localization

![Localization vs ground truth](docs/localization.png)

| Source | RMSE | Final error | Drift |
|---|---|---|---|
| wheel odometry | 271 m | 434 m | 49.2 % |
| EKF wheel + IMU | 11.2 m | 18.3 m | 2.08 % |
| **EKF + RTK GNSS** | **0.152 m** | **0.198 m** | 0.02 % |

## Localization without GNSS

Under the ship-to-shore cranes and between the container stacks, GNSS degrades or drops out. Wheel odometry with
the IMU then drifts. The truck matches its roof lidar to a map of the port instead, and that match goes into the
same EKF as the GNSS.

![Gate to crane 3 with GNSS lost under the stacks and cranes (shaded): lidar map localization vs wheels + IMU](docs/lidar_no_gnss.png)

*Gate → crane 3 (724 m), GNSS cut between the container stacks and under the cranes (shaded). Wheel odometry
drifts 4.4 m; the EKF with the lidar map stays within a few centimetres (lines on the zero line).*

| Full mission, no GNSS after 60 s (2.6 km, 645 s, docking included) | RMSE | Max | Final |
|---|---|---|---|
| **EKF + lidar map localization** | **2.7 cm** | 24 cm (at forward/reverse changes) | **7 cm** |
| lidar map localization alone | 2.8 cm | 13 cm | |
| EKF wheels + IMU (no lidar) | | 1.41 m | |

## SLAM benchmark: which odometry survives a port without GNSS?

The same recorded drive (gate → crane 3, 724 m, GNSS cut after 60 s) replayed into 8 open-source lidar SLAM / LIO
methods and into Hitchpoint's own localization. Each method is scored alone, fused with the IMU, with wheels +
IMU, and with wheels + IMU + the lidar map.

![Worst error after GNSS is lost, per method and fusion](docs/slam_benchmark.png)

- **A prior map beats map-free odometry by about 10×** (the map is ideal, see the note under the table): 5 cm worst error with any odometry, against 0.42 m for the best odometry-only
  combination (Cartographer with wheel odometry, fused with wheels + IMU).

| Method | Drift per 100 m | Worst error without GNSS, alone |
|---|---|---|
| FAST-LIO2 | **0.08 %** | 1.6 m |
| Point-LIO (lighter settings, 0.3× replay) | 0.09 % | 2.7 m |
| LIO-SAM | 0.11 % | 201 m (heading creeps while parked) |
| GLIM (CPU) | 0.34 % | 2.2 m |
| Cartographer 3D + wheel odometry | 0.47 % | 7.9 m |
| RKO-LIO | 12 % | 103 m |
| KISS-ICP | 12 % | 111 m |
| Cartographer 3D (lidar + IMU) | 42 % | 299 m |
| **Hitchpoint lidar map localization** | 0.04 % | **5 cm** |

*All numbers in this table: Gazebo, gate → crane 3 (724 m), worst error after GNSS is lost. The lidar map is ray-cast from the world geometry, so it is an ideal map (1 cm lidar noise), and the 8 methods have no map: this compares map-based localization with map-free odometry. Point-LIO ran at 0.3× replay with lighter settings, so its cost is not comparable. Each method got settings for this lidar (32 beams, 10 Hz) and this port, chosen once and not tuned against the results; a few were changed only to make them run (Point-LIO, LIO-SAM). One run per scenario.*

- **Tightly coupled lidar + IMU holds on a straight road; ICP-based scan matching slides.** On the 290 m gate
  road every scan looks like the last one. Cartographer 3D (lidar + IMU) estimated 3 m of motion while the
  truck drove 290 m; KISS-ICP and RKO-LIO (built on KISS-ICP) drift 12 %. FAST-LIO2, Point-LIO, LIO-SAM and
  GLIM stay at 0.1–0.3 %. Cartographer with the wheel odometry as its motion prior: 7.9 m instead of 299 m.
- **The worst errors of the good methods come from the start:** a few degrees of heading held while parked
  become metres 700 m later (LIO-SAM's heading creeps ~15° while standing still).
- **Cost differs a lot:** RKO-LIO needs 3.6 ms per scan; Point-LIO fell behind at full replay speed and needed
  lighter settings and a 0.3× replay on a laptop CPU.
- **One lost odometry source broke the whole stack** (103 m) until I added a check: a method's motion is only
  fused when it agrees with the wheel speed and the IMU yaw rate. With the check: 6.5 cm.

### When the yard changes

Containers move all day, so the map is never the yard. In an offline ray-cast test with 60 % of the stack
slots changed, the stale map matched confidently to the wrong containers (1.7 m). A live map, updated by an
earlier drive with RTK GNSS (log-odds per cell, 3D ray clearing), brought it back to 7 cm.

![Lidar map localization when the container yard changes](docs/changing_yard.png)

The same drive in the Gazebo world with 60 % of the stacks changed and 13 moving vehicles (oncoming convoy,
crossing trucks, reach stacker, forklifts, workers crossing the road): the survey map costs one 5 s event of
2.7 m; a live map brings it to 0.35 m; the true map of that yard gives 3.6 cm. The moving traffic itself did not
hurt the map localization.

![Worst error after GNSS is lost in a changed yard with moving traffic](docs/yard60_traffic_high.png)


## Problems found and solved

- **16 cm lateral drift that grew with distance.** Arrival error was 0.16 m at 460 m from the start, too regular to be
  noise. Cause: robot_localization's `map` frame is in UTM scale (k = 0.99966 here), so map metres are not world
  metres. Converting with the UTM scale factor → **0.0 cm** lateral at the goal, tracking RMSE 11.5 → 3.2 cm.
- **A blind zone behind the trailer.** Ray-casting the sensors against the trailer mesh showed a level rear lidar
  cannot see a pallet close behind the bumper, and neither can the radar: a "forget what you no longer see" rule
  would reverse into it. Fixed twice: obstacles are forgotten only on free-space evidence, and the lidar is now
  tilted 30° down (angle chosen from the same ray-cast).
- **Stopping 1.9 m instead of 3 m before an obstacle** at full speed in the offline tests: the braking curve had no
  reaction time. Added the reaction-time term → 2.9 m offline, 3.25 m in Gazebo.

- **Hitch estimate spikes (3.6°) during hard reversing.** Traced from the logged errors to wheel spin: odometry
  speed read high and was re-applied after every late lidar scan. Reproduced offline, fixed by taking speed from the
  GNSS-corrected EKF → max 1.4°.
- **GNSS noise in degrees.** Gazebo applies NavSat horizontal noise in degrees, not metres as the SDF spec says, so
  "2 cm" was ~2 km. Confirmed for Harmonic on [gazebosim/sdformat#1572](https://github.com/gazebosim/sdformat/issues/1572).
- **EKF trusting every GNSS fix completely.** The ROS–Gazebo bridge publishes fixes with zero covariance; added
  a node that sets the real covariance.
- **robot_localization `use_local_cartesian`** (Humble 3.5.4) mixes local and UTM coordinates (fixes landed
  ~5,800 km away); found in the source, documented, avoided.
- **Wheel-odometry heading fails with a trailer** (drive-axle slip in turns): the hitch model uses the IMU.
- **An integral term** for the 0.2 m arc offset was tried and dropped: it made the final docking accuracy worse.

- **The benchmark replay measured itself first.** `ros2 bag record` stores wall-clock receive times, so the
  replay clock is now taken from the sensor stamps. Without the EKF's TF, `navsat_transform` silently kept the
  GNSS antenna 3.4 m off the truck. The scorer aligned odometry on its first pose, which the EKF publishes
  before the IMU heading arrives: 371 m of fake error from a 0.48 m result.
- **Wall-clock lag in the lidar scans** (rendering load) put the scan-matching prior 4 m off at 8 m/s; the
  localizer now takes the EKF pose at the scan's own time.

## Roadmap

- [x] Vehicle model, port world, sensors
- [x] Localization, hitch-angle estimation
- [x] Reversing control and docking, rear safety stop
- [x] Road map, routing, Lanelet2 export
- [x] Forward driving, front safety stop
- [x] Full mission in Gazebo (crane → dock → parking bay → gate)
- [x] Lidar-map localization through GNSS outages, live map update
- [x] SLAM / LIO benchmark, static port: 8 methods (FAST-LIO2, Point-LIO, LIO-SAM, GLIM, RKO-LIO, KISS-ICP,
  Cartographer 3D ± wheel odometry)
- [x] SLAM benchmark in a changed yard with moving traffic (60 % of the stacks, 13 vehicles)
- [ ] Same drive with a live map from an earlier RTK drive (`premapped`)
- [ ] Randomized scenario runs, fault injection, CI

## Source code

The source code is in a private repository. **Recruiters and hiring teams: read access on request**:
Waseem Sajjad 

---
© 2026 Waseem Sajjad. All rights reserved. No license is granted to use, copy, modify or distribute this work.
