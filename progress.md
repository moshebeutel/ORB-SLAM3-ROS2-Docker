# Development Progress – ORB-SLAM3 ROS 2 Integration

## ✅ Completed

- [x] Dockerized ROS 2 Humble workspace
- [x] ORB-SLAM3 built and wrapped in ROS 2 node (`mono`)
- [x] ORB-SLAM3 tested successfully with EuRoC bag (mono-inertial)
- [x] Custom video + IMU data converted into ROS 2 bag
  - [x] Image messages with proper grayscale conversion
  - [x] IMU messages timestamped and ordered
  - [x] Clock messages published at 100Hz
- [x] TF buffer errors debugged (partial success)
- [x] IMU delay introduced to ensure initial frame has IMU context

## ⚠️ In Progress

- [ ] ORB-SLAM3 fails to initialize reliably with custom data
  - [ ] Sometimes starts and tracks, but crashes with `SIGSEGV`
  - [ ] GDB debug incomplete due to hanging
- [ ] Debugging `imuMeasurements` buffer (often contains only 1–2 samples)
- [ ] Investigate crash inside `TrackMonocular()` post-initialization
- [ ] Consistent TF tree behavior under `/robot_0` namespace

## 🚫 Not Done Yet

- [ ] Use actual calibration
- [ ] Validate map quality, compare to EuRoC performance
- [ ] Optimize ROS 2 bag creation for size and speed
- [ ] Runtime loop closure and trajectory export
