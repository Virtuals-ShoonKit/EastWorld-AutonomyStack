```
./src/livox_ros_driver2/build.sh humble
colcon build --base-paths src/eastworld_bringup --symlink-install
```


```
source install/setup.bash
ros2 launch eastworld_bringup bringup.launch.py
```