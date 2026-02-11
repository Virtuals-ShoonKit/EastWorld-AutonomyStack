# EastWorld Autonomy Stack

Autonomy software for [EastWorld](https://github.com/Virtuals-ShoonKit) autonomous agents. Provides LiDAR-inertial odometry and flight-controller integration so an agent can localise itself and feed pose/velocity to PX4 for GPS-denied flight.

Built on ROS 2 Humble with a Livox Mid-360, **SPARK-FAST-LIO** ([fork](https://github.com/Virtuals-ShoonKit/spark-fast-lio)), and Pixhawk (PX4 v1.16). The LIO node publishes odometry directly to `/fmu/in/vehicle_visual_odometry` (`px4_msgs/VehicleOdometry`) for EKF2 fusion over uXRCE-DDS.

## Hardware

- **Livox Mid-360** LiDAR (onboard IMU)
- **Pixhawk** (PX4 v1.16)
- Companion (e.g. Jetson, Jetpack 6.2)

## Workspace layout

- **`src/`** — ROS2 packages: **spark_fast_lio** (in `src/FAST_LIO_ROS2`), livox_ros_driver2, px4_msgs, px4_ros_com, **eastworld_bringup** (configs and launch).
- **`third_party/`** — Livox-SDK2; build and install separately.

## Dependencies

```bash
sudo apt update
sudo apt install -y libpcl-dev ros-humble-pcl-ros ros-humble-pcl-conversions
```

## Clone and build

```bash
git clone --recurse-submodules https://github.com/Virtuals-Shoonkit/EastWorld-AutonomyStack.git
cd EastWorld-AutonomyStack
git submodule update --init --recursive
```

If you already had the repo cloned, run `git submodule sync` then `git submodule update --init --recursive` to update submodules (e.g. LIO fork).

**1. Install Livox-SDK2** (required by livox_ros_driver2):

```bash
cd third_party/Livox-SDK2 && mkdir -p build && cd build && cmake .. && make -j && sudo make install
cd ../../..
```

**2. Prepare livox_ros_driver2:**

```bash
cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
```

**3. Build workspace:**

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DROS_EDITION=ROS2 -DHUMBLE_ROS=humble -DCMAKE_CXX_STANDARD=17
source install/setup.bash
```

## Run

1. Start **Micro XRCE-DDS agent** on the companion (e.g. `MicroXRCEAgent udp4 -p 8888`).
2. Launch Mid-360 driver + SPARK-FAST-LIO:

   ```bash
   source install/setup.bash
   ros2 launch eastworld_bringup autonomy_launch.py config_file:=mid360.yaml rviz:=false
   ```

   Optional: `config_file:=mid360.yaml`, `rviz:=true`, `use_sim_time:=false`.

## Configs

- **`eastworld_bringup/config/MID360_config.json`** — Livox driver (host IP, lidar IP). Set your machine and Mid-360 IPs; host and lidar must be on the same subnet.
- **`eastworld_bringup/config/mid360.yaml`** — SPARK-FAST-LIO (topics, extrinsics). For Mid-360 onboard IMU use factory extrinsics and `extrinsic_est_en: false` when known. Adapt from [spark-fast-lio config examples](https://github.com/MIT-SPARK/spark-fast-lio#-how-to-run-spark-fast-lio2-using-your-own-ros2-bag) if needed.

## References

- [Virtuals-ShoonKit/spark-fast-lio](https://github.com/Virtuals-ShoonKit/spark-fast-lio) (LIO fork with PX4 odometry output)
- [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide.html)
- [Seeed MID360 wiki](https://wiki.seeedstudio.com/mid360/)
