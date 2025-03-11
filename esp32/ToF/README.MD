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


Initial param set
```text
Parameter Kp_pitch updated to 33.800000
Parameter Ki_pitch updated to 40.800000
Parameter Kd_pitch updated to 0.400000
Parameter Max_pitch updated to 255.000000
Parameter Min_pitch updated to -255.000000
Parameter Period_pitch updated to 7
Parameter Direction_pitch updated to false
Parameter Auto_pitch updated to true
Parameter Kp_vel updated to 25.950000
Parameter Ki_vel updated to 14.930000
Parameter Kd_vel updated to 1.330000
Parameter Max_vel updated to 4.000000
Parameter Min_vel updated to -4.000000
Parameter Period_vel updated to 210
Parameter Direction_vel updated to false
Parameter Auto_vel updated to true
Parameter Kp_yaw updated to 7.800000
Parameter Ki_yaw updated to 8.300000
Parameter Kd_yaw updated to 0.000000
Parameter Max_yaw updated to 200.000000
Parameter Min_yaw updated to -200.000000
Parameter Period_yaw updated to 187
Parameter Direction_yaw updated to false
Parameter Auto_yaw updated to true
Parameter LW_threshold updated to 0
Parameter RW_threshold updated to 0
Parameter TicKs_p_r updated to 292
```

