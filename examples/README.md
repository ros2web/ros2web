# ros2web Example

## ros2web_example_cpp
```bash
sudo apt install nlohmann-json3-dev

cd ~/ros2_ws/src/ros2web/examples/ros2web_example_cpp
rm COLCON_IGNORE

cd ~/ros2_ws
colcon build --packages-select ros2web_example_cpp
. ./install/local_setup.bash
ros2 run ros2web_example_cpp api
```
