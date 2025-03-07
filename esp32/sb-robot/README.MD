To get all the types of interfaces avaliable in ros:
```bash
ros2 interface list
```

To run micro ros agent:
```bash
source install/local_setup.bash
#ros2 run micro_ros_setup build_agent.sh
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

To run rqt GUI:
```bash
rqt
```