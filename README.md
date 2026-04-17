A. sim environment setting
1. create account for docker
2.
```bash
docker pull happygaj/my_drone_env:v1
```

B. Mount git cloned local folder with the docker environ. (Use below command) 
terminal 1
```bash
xhost +local:docker
docker run -it --rm --name ros_humble_container \
   --gpus all \
   --network=host \
   -e DISPLAY=$DISPLAY \
   -e NVIDIA_DRIVER_CAPABILITIES=all \
   -v /tmp/.X11-unix:/tmp/.X11-unix \
   -v ~/.Xauthority:/root/.Xauthority:rw \
   -v ~/ros2_humble/src/106a_final_project:/ros2_ws/106a_final_project \
   -v ~/.gazebo/models:/root/.gazebo/models \
   my_drone_env:v1
```
```bash
cd /ros2_ws/106a_final_project/drone_simulation
source /ros2_ws/106a_final_project/drone_simulation/install/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/home/edg/.gazebo/models
ros2 launch tello_gazebo simple_launch.py
```

terminal 2
```bash
docker exec -it ros_humble_container bash
source /ros2_ws/106a_final_project/drone_simulation/install/setup.bash
ros2 launch my_drone_vision vision_control.launch.py
```

C. some other commands
end Gazebo
```bash
pkill -9 gzserver
pkill -9 gzclient
```

source 
```bash
source /opt/ros/humble/setup.bash
```

