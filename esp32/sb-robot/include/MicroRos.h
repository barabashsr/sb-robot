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
#include <rcl/timer.h>
#include <geometry_msgs/msg/twist.h>
//#define ROS_DOMAIN_ID 77

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

class controllerNode {
    public:
        
    controllerNode  (
                    uint ros_domain_id, 
                    uint16_t& agent_port, 
                    String& agentIP, 
                    String& ssid, 
                    String& password, 
                    String& twist_topic,
                    double& targetVel,
                    double& targetYawRate,
                    controllerState& contrState,
                    pidParams& pitchParams,
                    pidParams& velParams,
                    pidParams& yawParams
                    );

    void connect_to_wifi();
    void setup();
    void spinNode();
    void handle_twist_message(const void * msgin);
    double& _targetVel;
    double& _targetYawRate;

    
   
       
    
     
        
    private:

        
        uint8_t _ros_domain_id;
        String& _agentIP;
        uint16_t& _agent_port;
        String& _ssid;
        String& _password;

        String& _twist_topic;

        
        
        controllerState& _controllerState;
        pidParams& _pitchParams;
        pidParams& _velParams;
        pidParams& _yawParams;
        

        
        rcl_subscription_t _subscriber_twist;
        
        

        rclc_executor_t _executor;
        rclc_support_t _support;
        rcl_allocator_t _allocator;
        rcl_init_options_t _init_options;
        rcl_node_t _node;
        rcl_timer_t _timer;

        geometry_msgs__msg__Twist _twst_msg;
        const int _handle_count = 4;

        static std_msgs__msg__Int32 _msgOut;
        static rcl_publisher_t _publisher;

        static void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
        static void subscription_callback_twist(const void * msgin, void * context);
        


        


        
        
    
    };



#endif // MICROROS_H