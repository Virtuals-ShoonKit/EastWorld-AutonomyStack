## Build

```bash
# Install CycloneDDS RMW and Foxglove Bridge
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-foxglove-bridge ros-humble-foxglove-msgs

# Build Livox driver
./src/livox_ros_driver2/build.sh humble

# Build bringup
colcon build --base-paths src/eastworld_bringup --symlink-install
```

## Run

```bash
source install/setup.bash
ros2 launch eastworld_bringup bringup.launch.py
```

## Launch on boot (systemd)

```bash
# Install and enable the systemd service
sudo cp src/eastworld_bringup/systemd/eastworld-autonomy.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable eastworld-autonomy.service

# Start now (or reboot)
sudo systemctl start eastworld-autonomy.service

# Check status / logs
systemctl status eastworld-autonomy.service
journalctl -u eastworld-autonomy.service -f
```

## Verify odometry pitch correction

```bash
ros2 topic echo /mavros/odometry/out --once | python3 -c "
import sys, math, yaml
for doc in yaml.safe_load_all(sys.stdin):
    if doc and isinstance(doc, dict) and 'pose' in doc:
        o = doc['pose']['pose']['orientation']
        sinp = 2.0 * (o['w'] * o['y'] - o['z'] * o['x'])
        pitch_deg = math.degrees(math.asin(max(-1, min(1, sinp))))
        sinr = 2.0 * (o['w'] * o['x'] + o['y'] * o['z'])
        cosr = 1.0 - 2.0 * (o['x']**2 + o['y']**2)
        roll_deg = math.degrees(math.atan2(sinr, cosr))
        print(f'Roll: {roll_deg:+.2f} deg   Pitch: {pitch_deg:+.2f} deg')
        break
"
```


ros2 service call /mavros/cmd/command mavros_msgs/srv/CommandLong "{command: 187, param1: -1.0, param7: 0.0}"


