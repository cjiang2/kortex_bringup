# kortex_bringup
Launch files and global scripts to communicate with Kinova Gen3

## Note
Make sure [ros_kortex](https://github.com/Kinovarobotics/ros_kortex) and ros_kortex_vision](https://github.com/Kinovarobotics/ros_kortex_vision) are installed. 

Clone this repo under "catkin_ws/src", and build the package under your catkin workspace.

"kinova_gen3.py" is a global Python class to issue control commands from rospy to gen3. Codebase is from the [official example scripts of ros_kortex](https://github.com/Kinovarobotics/ros_kortex/blob/noetic-devel/kortex_examples/src/full_arm/example_full_arm_movement.py).
