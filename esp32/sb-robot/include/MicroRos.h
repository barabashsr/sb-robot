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
#include <sensor_msgs/msg/joint_state.h>
#include <rclc_parameter/rclc_parameter.h>
#include "BalanceController.h"
#include "ParameterRegistry.h"
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
//#include </tf2_msgs/tf2_msgs/msg/tf_message.h>
//#include "tf2_msgs/msg/tf_message.h"

#include <micro_ros_utilities/type_utilities.h>


// #define ROS_DOMAIN_ID 77

#define JOINT_COUNT 3

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            return;                                                                      \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

class controllerNode
{
public:
    controllerNode(
        ParameterRegistry &parameterRegistry,
        BalanceController &balance_controller,
        uint ros_domain_id,
        uint16_t &agent_port,
        String &agentIP,
        String &ssid,
        String &password,
        String &twist_topic,
        double &targetVel,
        double &targetYawRate,
        controllerState &contrState,
        pidParams &pitchParams,
        pidParams &velParams,
        pidParams &yawParams);

    void connect_to_wifi();
    void setup();
    void spinNode();
    void handle_twist_message(const void *msgin);
    double &_targetVel;
    double &_targetYawRate;
    static controllerNode *getInstance()
    {
        return instance;
    };

    
    controllerState &_controllerState;

private:
    transform _tf;
    ParameterRegistry &_parameterRegistry;
    static controllerNode *instance;

    BalanceController &_balance_controller;
    uint8_t _ros_domain_id;
    String &_agentIP;
    uint16_t &_agent_port;
    String &_ssid;
    String &_password;

    String &_twist_topic;

    pidParams &_pitchParams;
    pidParams &_velParams;
    pidParams &_yawParams;

    rcl_subscription_t _subscriber_twist; // twist subscriber object

    static rclc_parameter_server_t _param_server; // Parameter server object
    static rcl_publisher_t _publisher;            // publisher object
    static rcl_publisher_t _tf_publisher;
    static tf2_msgs__msg__TFMessage _tf_msg;

    //static geometry_msgs__msg__TransformStamped _tf_msg;

    static rcl_publisher_t _joint_state_publisher;
    static sensor_msgs__msg__JointState _joint_state_msg;

    rclc_executor_t _executor;
    rclc_support_t _support;
    rcl_allocator_t _allocator;
    rcl_init_options_t _init_options;
    static rclc_parameter_options_t _param_options;
    static rcl_node_t _node;
    rcl_timer_t _timer;

    micro_ros_utilities_memory_conf_t _memory_conf = {0};


    const char * tf_topic_name = "tf";
    const char * base_link_name = "base_link";
   

    //const unsigned int _tf_timer_time = 20;
    const unsigned int timer_timeout = 500;
    const unsigned int timer_timeout_joint_state = 20;

    rcl_timer_t _timer_tf;
    const unsigned int timer_timeout_tf = 50; // 20Hz TF updates
    const char * odom_frame_name = "odom";


    rcl_timer_t _timer_joint_state;

    geometry_msgs__msg__Twist _twst_msg;
    const int _handle_count = 8;

    static geometry_msgs__msg__Twist _msgOut;

    const char *chassis_joint_name = "chassis_joint"; // name of the pivoting joint
    const char *lw_joint = "LW_joint"; // name of the left wheel joint
    const char *rw_joint = "RW_joint"; // name of the left wheel joint


    // float angleY = 0.0; // This will be updated with your IMU data

    static void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
    static void timer_callback_joint_state(rcl_timer_t *timer, int64_t last_call_time);
    static void timer_callback_tf(rcl_timer_t *timer, int64_t last_call_time);

    static void subscription_callback_twist(const void *msgin, void *context);
    static bool on_parameter_changed(const Parameter *old_param, const Parameter *new_param, void *context);
    // static void timer_callback_wrapper(rcl_timer_t * timer, int64_t last_call_time);
    // void controllerNode::timer_callback_impl(rcl_timer_t * timer, int64_t last_call_time);
    // controllerNode getInstance(){return this};
    void joint_state_message_init();

    void set_parameters();
    //void initTfMessage(geometry_msgs__msg__TransformStamped* msg);
    void initTfMessage();

};

#endif // MICROROS