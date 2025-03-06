#ifndef MICROROS_H
#define MICROROS_H
#include <utils.h>
#include <std_msgs/msg/int32.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>
#include <stdio.h>
#include <string.h>
#include <geometry_msgs/msg/twist.h>
#define ROS_DOMAIN_ID 77

class controllerNode {
    public:
        
    controllerNode(String& agent_port, String& agentIP, String& ssid, String& password, String& twist_topic){

    }

    void connect_to_wifi();
    void setup();
    void spinNode();
       
    
     
        
    private:

        
        String _twist_topic;
        uint16_t _agent_port;
        String _agentIP;
        String _ssid = "Beeline_2G_F13F37";
        String _password = "1122334455667788";

        rcl_publisher_t _publisher;
        rcl_subscription_t _subscriber_twist;
        //std_msgs__msg__Int32 msgIn, msgOut;
        

        rclc_executor_t _executor;
        rclc_support_t _support;
        rcl_allocator_t _allocator;
        rcl_init_options_t _init_options;
        rcl_node_t _node;
        rcl_timer_t _timer;

        geometry_msgs__msg__Twist _twst_msg;
        int _handle_count;

        
        
    
    };



#endif // MICROROS_H