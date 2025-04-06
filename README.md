# Kimera Semantics Instance

This project is an extension of Kimera Semantic (for the original code, see this [repo](https://github.com/MIT-SPARK/Kimera-Semantics)).

## Docker
To run our code, we provide a docker container with ROS Noetic installed.
To run the code with ROS using the container we provide a script with four commands:
* `start`: Starts the ROS Docker environment by executing the specified Docker Compose file.
* `stop`: Stops and removes the Docker containers defined in the Docker Compose file.
* `restart`: Stops, rebuilds, and restarts the Docker containers.
* `build`: Builds the Docker containers defined in the Docker Compose file.

To use the script just run `./use_container.sh <command>`

The first time run the following command inside docker:
* `cd ~/ros_ws/src`
* `wstool init ~/ros_ws/src ~/ros_ws/src/ros-instance-mapping/ros-instance-mapping.rosinstall`
* `wstool update`
* `cd ..`
* `catkin config --cmake-args -DCMAKE_CXX_STANDARD=14 -DCMAKE_BUILD_TYPE=Release`
* `catkin build pcl_catkin`
* `catkin build consistent_gsm`
* `echo "source /root/code/ros_ws/devel/setup.bash" >> ~/.bashrc`
* `source ~/.bashrc`
