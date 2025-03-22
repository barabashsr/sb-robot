#include "MicroRos.h"

controllerNode::controllerNode(
    ToFSensor &tof_sensor,
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
    pidParams &yawParams)
    : _tof_sensor(tof_sensor), _parameterRegistry(parameterRegistry), _balance_controller(balance_controller), _ros_domain_id(ros_domain_id), _agentIP(agentIP), _ssid(ssid), _password(password), _agent_port(agent_port),
      _twist_topic(twist_topic), _targetVel(targetVel), _targetYawRate(targetYawRate),
      _controllerState(contrState), _pitchParams(pitchParams), _velParams(velParams), _yawParams(yawParams)
{

    instance = this;

    //_msgOut.data = 0;
};

rcl_node_t controllerNode::_node;
geometry_msgs__msg__Twist controllerNode::_msgOut;
rcl_publisher_t controllerNode::_publisher;
rcl_publisher_t controllerNode::_joint_state_publisher;
sensor_msgs__msg__JointState controllerNode::_joint_state_msg;
//geometry_msgs__msg__TransformStamped controllerNode::_tf_msg;
tf2_msgs__msg__TFMessage controllerNode::_tf_msg;

//point cloud publisher
rcl_publisher_t controllerNode::_point_cloud_publisher;  
sensor_msgs__msg__PointCloud2 controllerNode::_point_cloud_msg;


rcl_publisher_t controllerNode::_tf_publisher;

rclc_parameter_server_t controllerNode::_param_server;
rclc_parameter_options_t controllerNode::_param_options;
controllerNode *controllerNode::instance = nullptr; // Definition

void controllerNode::connect_to_wifi()
{
    char ssidR[_ssid.length() + 1];
    char passwordR[_password.length() + 1];

    strcpy(ssidR, _ssid.c_str());
    strcpy(passwordR, _password.c_str());

    IPAddress agent_ip;
    if (!agent_ip.fromString(_agentIP))
    {
        Serial.println("Invalid IP address format");
        return;
    }

    set_microros_wifi_transports(ssidR, passwordR, agent_ip, _agent_port);
    
}

// Static wrapper function
void controllerNode::timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // Assuming you have a way to get the instance of controllerNode
    // For example, if you have a global or static instance:
    controllerNode *node = controllerNode::getInstance();
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        _msgOut.linear.x = node->_controllerState.currentVel;
        _msgOut.linear.y = node->_controllerState.targetVel;
        _msgOut.angular.y = node->_controllerState.currentPitch;
        _msgOut.angular.y = node->_controllerState.targetPitch;

        RCSOFTCHECK(rcl_publish(&_publisher, &_msgOut, NULL));
    }
    // controllerNode* node = controllerNode::getInstance(); // You need to implement getInstance()
    // node->timer_callback_impl(timer, last_call_time);
}

void controllerNode::timer_callback_joint_state(rcl_timer_t *timer, int64_t last_call_time) {
    controllerNode *node = controllerNode::getInstance();

    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Update timestamp
        rmw_uros_sync_session(10); // Sync time with agent (timeout 10ms)
        int64_t time_ns = rmw_uros_epoch_nanos();
        _joint_state_msg.header.stamp.sec = time_ns / 1000000000;
        _joint_state_msg.header.stamp.nanosec = time_ns % 1000000000;
        
        // Update chassis joint position (convert degrees to radians)
        _joint_state_msg.position.data[0] = node->_controllerState.currentPitch * (3.14159265359 / 180.0);
        
        // Update wheel joint positions (in radians)
        _joint_state_msg.position.data[1] = node->_controllerState.positionB;
        _joint_state_msg.position.data[2] = node->_controllerState.positionA;
        
        // Update wheel velocities if available
        _joint_state_msg.velocity.data[1] = node->_controllerState.speedB;
        _joint_state_msg.velocity.data[2] = node->_controllerState.speedA;
        
        RCSOFTCHECK(rcl_publish(&_joint_state_publisher, &_joint_state_msg, NULL));
    }
}


void controllerNode::joint_state_message_init() {
    // Initialize joint state message
    _joint_state_msg.header.frame_id.data = (char *)"";
    _joint_state_msg.header.frame_id.size = 0;
    _joint_state_msg.header.frame_id.capacity = 0;

    // Allocate memory for joint names array
    _joint_state_msg.name.data = (rosidl_runtime_c__String *)malloc(JOINT_COUNT * sizeof(rosidl_runtime_c__String));
    _joint_state_msg.name.size = JOINT_COUNT;
    _joint_state_msg.name.capacity = JOINT_COUNT;

    // Set joint names
    const char* joint_names[JOINT_COUNT] = {chassis_joint_name, lw_joint, rw_joint};
    
    for (int i = 0; i < JOINT_COUNT; i++) {
        _joint_state_msg.name.data[i].data = (char *)malloc(strlen(joint_names[i]) + 1);
        strcpy(_joint_state_msg.name.data[i].data, joint_names[i]);
        _joint_state_msg.name.data[i].size = strlen(joint_names[i]);
        _joint_state_msg.name.data[i].capacity = strlen(joint_names[i]) + 1;
    }

    // Allocate memory for position, velocity, and effort arrays
    _joint_state_msg.position.data = (double *)malloc(JOINT_COUNT * sizeof(double));
    _joint_state_msg.position.size = JOINT_COUNT;
    _joint_state_msg.position.capacity = JOINT_COUNT;

    _joint_state_msg.velocity.data = (double *)malloc(JOINT_COUNT * sizeof(double));
    _joint_state_msg.velocity.size = JOINT_COUNT;
    _joint_state_msg.velocity.capacity = JOINT_COUNT;

    _joint_state_msg.effort.data = (double *)malloc(JOINT_COUNT * sizeof(double));
    _joint_state_msg.effort.size = JOINT_COUNT;
    _joint_state_msg.effort.capacity = JOINT_COUNT;

    // Initialize values
    for (int i = 0; i < JOINT_COUNT; i++) {
        _joint_state_msg.position.data[i] = 0.0;
        _joint_state_msg.velocity.data[i] = 0.0;
        _joint_state_msg.effort.data[i] = 0.0;
    }
}


// Non-static member function to handle the timer callback
// void controllerNode::timer_callback_impl(rcl_timer_t * timer, int64_t last_call_time) {
//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL) {
//         RCSOFTCHECK(rcl_publish(&_publisher, &_msgOut, NULL));
//         _msgOut.linear.x = _controllerState.currentVel;
//         _msgOut.linear.x = _controllerState.currentPitch;

//     }
// }

// void controllerNode::timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
//     //controllerNode* node = static_cast<controllerNode*>(timer);

//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL) {
//       RCSOFTCHECK(rcl_publish(&_publisher, &_msgOut, NULL));
//       _msgOut.linear.x = _controllerState;
//       //Serial.println(msg.data);
//     }
// }

void controllerNode::subscription_callback_twist(const void *msgin, void *context)
{
    controllerNode *node = static_cast<controllerNode *>(context);
    // Use node to access non-static members
    // Process the message
    if (node)
    {
        node->handle_twist_message(msgin);
    }
}

void controllerNode::handle_twist_message(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    _targetVel = msg->linear.x;
    _targetYawRate = msg->angular.z;
    Serial.printf("vel_x: %f, yawRate: %f\n", _targetVel, _targetYawRate);
}

void controllerNode::setup()
{

    connect_to_wifi();

    _allocator = rcl_get_default_allocator();
    _init_options = rcl_get_zero_initialized_init_options();
    //_msgOut.data = 0;
    // Initialize init options
    RCCHECK(rcl_init_options_init(&_init_options, _allocator));
    Serial.println("Options init");
    // Set ROS_DOMAIN_ID
    RCCHECK(rcl_init_options_set_domain_id(&_init_options, _ros_domain_id));
    Serial.println("ID set init");
    // Initialize support with options
    RCCHECK(rclc_support_init(&_support, 0, NULL, &_allocator));
    // RCCHECK(rclc_support_init_with_options(&_support, 0, NULL, &_init_options, &_allocator));
    Serial.println("Support init");

    // create node
    RCCHECK(rclc_node_init_default(&_node, "mr_sb_robot_node", "", &_support));
    Serial.println("Node init");
    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &_publisher,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "sb_robot_output"));

    Serial.println("Publisher init");

    // create subscriber
    RCCHECK(rclc_subscription_init_default(
        &_subscriber_twist,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "twist_topic"));
    
    // Create point cloud publisher
    RCCHECK(rclc_publisher_init_default(
        &_point_cloud_publisher,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
        "point_cloud"));

    // Create marker array publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &_marker_array_publisher,
    //     &_node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(visualization_msgs, msg, MarkerArray),
    //     "marker_array"));
    
    RCCHECK(rclc_publisher_init_default(
        &_joint_state_publisher,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "joint_states"));

   RCCHECK(rclc_publisher_init_default(
        &_tf_publisher,
        &_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
        tf_topic_name));

    // create timer,
    Serial.println("Subscr init");

    RCCHECK(rclc_timer_init_default(
        &_timer,
        &_support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));

    RCCHECK(rclc_timer_init_default(
        &_timer_joint_state,
        &_support,
        RCL_MS_TO_NS(timer_timeout_joint_state),
        timer_callback_joint_state));

    Serial.println("Timer init");

    // create param server

    // Initialize parameter server options
    _param_options = {
        .notify_changed_over_dds = true,
        .max_params = 40,
        .allow_undeclared_parameters = true,
        .low_mem_mode = false};

    // Initialize parameter server with configured options
    RCCHECK(rclc_parameter_server_init_with_option(
        &_param_server,
        &_node,
        &_param_options));

    // rclc_parameter_server_init_default(&_param_server, &_node);
    Serial.println("Param server init");

    // create executor

    RCCHECK(rclc_executor_init(
        &_executor,
        &_support.context, _handle_count + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES,
        &_allocator));

    // delay(1000);
    RCCHECK(rclc_executor_add_parameter_server_with_context(
        &_executor,
        &_param_server,
        on_parameter_changed, // on_parameter_changed
        this));



 
    

    // Initialize TF timer
    RCCHECK(rclc_timer_init_default(
        &_timer_tf,
        &_support,
        RCL_MS_TO_NS(timer_timeout_tf),
        timer_callback_tf));

    RCCHECK(rclc_executor_add_timer(&_executor, &_timer));
    RCCHECK(rclc_executor_add_timer(&_executor, &_timer_joint_state));
    RCCHECK(rclc_executor_add_subscription_with_context(
        &_executor,
        &_subscriber_twist,
        &_twst_msg,
        &subscription_callback_twist,
        this,
        ON_NEW_DATA));
    
    RCCHECK(rclc_executor_add_timer(&_executor, &_timer_tf));
    

    // RCCHECK(micro_ros_utilities_create_message_memory(
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
    //     &_tf_msg,
    //     _memory_conf));

    set_parameters();
    joint_state_message_init();
    initTfMessage();
    //_msgOut.data = 0;
    // rclc_executor_set_context(&_executor, this);
}

void controllerNode::set_parameters()
{
    const double pid_gain_from = 0.0;
    const double pid_gain_to = 70.0;
    const double pid_gain_step = 0.01;

    const unsigned int period_from = 1;
    const unsigned int period_to = 1000;
    const unsigned int period_step = 1;

    // Load parameters from registry to local variables first
    bool success = _parameterRegistry.loadAllParameters();
    if (!success)
    {
        Serial.println("WARNING: Failed to load some parameters from EEPROM");
    }

    // Initialize parameters in the registry if they don't exist

    // Pitch PID parameters
    _parameterRegistry.addDoubleParameter("Kp_pitch", _pitchParams.Kp, "Pitch PID Kp", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Ki_pitch", _pitchParams.Ki, "Pitch PID Ki", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Kd_pitch", _pitchParams.Kd, "Pitch PID Kd", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Max_pitch", _pitchParams.max, "Pitch PID max output", "", 0, 255, 1);
    _parameterRegistry.addDoubleParameter("Min_pitch", _pitchParams.min, "Pitch PID min output", "", -255, 0, 1);
    _parameterRegistry.addIntParameter("Period_pitch", _pitchParams.period, "Pitch PID period", "ms", period_from, period_to, period_step);
    _parameterRegistry.addBoolParameter("Direction_pitch", _pitchParams.direct, "Pitch PID Direction", "");
    _parameterRegistry.addBoolParameter("Auto_pitch", _pitchParams.modeAuto, "Pitch PID auto", "");

    // Vel PID parameters
    _parameterRegistry.addDoubleParameter("Kp_vel", _velParams.Kp, "Vel PID Kp", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Ki_vel", _velParams.Ki, "Vel PID Ki", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Kd_vel", _velParams.Kd, "Vel PID Kd", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Max_vel", _velParams.max, "Vel PID max output", "", -4.0, 4.0, 0.01);
    _parameterRegistry.addDoubleParameter("Min_vel", _velParams.min, "Vel PID min output", "", -4.0, 4.0, 0.01);
    _parameterRegistry.addIntParameter("Period_vel", _velParams.period, "Vel PID period", "ms", period_from, period_to, period_step);
    _parameterRegistry.addBoolParameter("Direction_vel", _velParams.direct, "Vel PID Direction", "");
    _parameterRegistry.addBoolParameter("Auto_vel", _velParams.modeAuto, "Vel PID auto", "");

    // Yaw rate PID parameters
    _parameterRegistry.addDoubleParameter("Kp_yaw", _yawParams.Kp, "Yaw rate PID Kp", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Ki_yaw", _yawParams.Ki, "Yaw rate PID Ki", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Kd_yaw", _yawParams.Kd, "Yaw rate PID Kd", "", pid_gain_from, pid_gain_to, pid_gain_step);
    _parameterRegistry.addDoubleParameter("Max_yaw", _yawParams.max, "Yaw rate PID max output", "", -255, 255, 1);
    _parameterRegistry.addDoubleParameter("Min_yaw", _yawParams.min, "Yaw rate PID min output", "", -255, 255, 1);
    _parameterRegistry.addIntParameter("Period_yaw", _yawParams.period, "Yaw rate PID period", "ms", period_from, period_to, period_step);
    _parameterRegistry.addBoolParameter("Direction_yaw", _yawParams.direct, "Yaw rate PID Direction", "");
    _parameterRegistry.addBoolParameter("Auto_yaw", _yawParams.modeAuto, "Yaw rate PID auto", "");

    // Wheel thresholds
    _parameterRegistry.addIntParameter("LW_threshold", 0, "Left wheel starting threshold", "", 0, 255, 1);
    _parameterRegistry.addIntParameter("RW_threshold", 0, "Right wheel starting threshold", "", 0, 255, 1);
    _parameterRegistry.addIntParameter("TicKs_p_r", 292, "Ticks of encoder per full revolution", "", 0, 2000, 1);

    // Now update the local variables with values from registry
    _parameterRegistry.getDouble("Kp_pitch", &_pitchParams.Kp);
    _parameterRegistry.getDouble("Ki_pitch", &_pitchParams.Ki);
    _parameterRegistry.getDouble("Kd_pitch", &_pitchParams.Kd);
    _parameterRegistry.getDouble("Max_pitch", &_pitchParams.max);
    _parameterRegistry.getDouble("Min_pitch", &_pitchParams.min);
    _parameterRegistry.getInt("Period_pitch", &_pitchParams.period);
    _parameterRegistry.getBool("Direction_pitch", &_pitchParams.direct);
    _parameterRegistry.getBool("Auto_pitch", &_pitchParams.modeAuto);

    _parameterRegistry.getDouble("Kp_vel", &_velParams.Kp);
    _parameterRegistry.getDouble("Ki_vel", &_velParams.Ki);
    _parameterRegistry.getDouble("Kd_vel", &_velParams.Kd);
    _parameterRegistry.getDouble("Max_vel", &_velParams.max);
    _parameterRegistry.getDouble("Min_vel", &_velParams.min);
    _parameterRegistry.getInt("Period_vel", &_velParams.period);
    _parameterRegistry.getBool("Direction_vel", &_velParams.direct);
    _parameterRegistry.getBool("Auto_vel", &_velParams.modeAuto);

    _parameterRegistry.getDouble("Kp_yaw", &_yawParams.Kp);
    _parameterRegistry.getDouble("Ki_yaw", &_yawParams.Ki);
    _parameterRegistry.getDouble("Kd_yaw", &_yawParams.Kd);
    _parameterRegistry.getDouble("Max_yaw", &_yawParams.max);
    _parameterRegistry.getDouble("Min_yaw", &_yawParams.min);
    _parameterRegistry.getInt("Period_yaw", &_yawParams.period);
    _parameterRegistry.getBool("Direction_yaw", &_yawParams.direct);
    _parameterRegistry.getBool("Auto_yaw", &_yawParams.modeAuto);

    // Load parameters to ROS parameter server
    _parameterRegistry.loadToParamServer(&_param_server);

    _balance_controller.updatePitchPID();
    _balance_controller.updateVelPID();
    _balance_controller.updateYawPID();
    _balance_controller.setTuningsPitch();
    _balance_controller.setTuningsVel();
    _balance_controller.setTuningsYaw();

    // Update PID modes
    _balance_controller.setPitchPIDOn(_pitchParams.modeAuto);
    _balance_controller.setVelPIDOn(_velParams.modeAuto);
    _balance_controller.setYawPIDOn(_yawParams.modeAuto);

    // Update wheel thresholds
    int64_t leftThreshold, rightThreshold, ticksPerRev;
    if (_parameterRegistry.getInt("LW_threshold", &leftThreshold))
    {
        _balance_controller.setLWSreshold(leftThreshold);
    }
    if (_parameterRegistry.getInt("RW_threshold", &rightThreshold))
    {
        _balance_controller.setRWSreshold(rightThreshold);
    }
    if (_parameterRegistry.getInt("TicKs_p_r", &ticksPerRev))
    {
        _balance_controller.setTicksPerRevolution(ticksPerRev);
    }
}

void controllerNode::spinNode()
{
    RCSOFTCHECK(rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(25)));
};

bool controllerNode::on_parameter_changed(const Parameter *old_param, const Parameter *new_param, void *context)
{
    controllerNode *node = static_cast<controllerNode *>(context);

    if (old_param == NULL && new_param == NULL)
    {
        printf("Callback error, both parameters are NULL\n");
        return false;
    }

    bool ret = true;

    if (new_param == NULL)
    {
        printf("Delete parameter %s rejected\n", old_param->name.data);
        ret = false;
    }
    else
    {
        // Use the parameter registry to update values
        switch (new_param->value.type)
        {
        case RCLC_PARAMETER_BOOL:
            if (!node->_parameterRegistry.setBool(new_param->name.data, new_param->value.bool_value))
            {
                printf("Failed to update bool parameter: %s\n", new_param->name.data);
                ret = false;
            }
            else
            {
                printf("Parameter %s updated to %s\n", new_param->name.data,
                       new_param->value.bool_value ? "true" : "false");
            }
            break;

        case RCLC_PARAMETER_INT:
            if (!node->_parameterRegistry.setInt(new_param->name.data, new_param->value.integer_value))
            {
                printf("Failed to update int parameter: %s\n", new_param->name.data);
                ret = false;
            }
            else
            {
                printf("Parameter %s updated to %d\n", new_param->name.data,
                       new_param->value.integer_value);
            }
            break;

        case RCLC_PARAMETER_DOUBLE:
            if (!node->_parameterRegistry.setDouble(new_param->name.data, new_param->value.double_value))
            {
                printf("Failed to update double parameter: %s\n", new_param->name.data);
                ret = false;
            }
            else
            {
                printf("Parameter %s updated to %f\n", new_param->name.data,
                       new_param->value.double_value);
            }
            break;

        default:
            printf("Unsupported parameter type for %s\n", new_param->name.data);
            ret = false;
            break;
        }

        // If parameter was updated successfully, update bindings and apply changes
        if (ret)
        {
            // Update all bindings to ensure external variables are synchronized
            node->_parameterRegistry.updateAllBindings();

            // Update PID controllers based on parameter changes
            if (strstr(new_param->name.data, "pitch") != NULL)
            {
                node->_balance_controller.updatePitchPID();
                if (strstr(new_param->name.data, "Kp_") != NULL ||
                    strstr(new_param->name.data, "Ki_") != NULL ||
                    strstr(new_param->name.data, "Kd_") != NULL)
                {
                    node->_balance_controller.setTuningsPitch();
                }
                if (strcmp(new_param->name.data, "Auto_pitch") == 0)
                {
                    Serial.println("Auto_pitch changed");
                    node->_balance_controller.setPitchPIDOn(node->_pitchParams.modeAuto);
                }
            }
            else if (strstr(new_param->name.data, "vel") != NULL)
            {
                node->_balance_controller.updateVelPID();
                if (strstr(new_param->name.data, "Kp_") != NULL ||
                    strstr(new_param->name.data, "Ki_") != NULL ||
                    strstr(new_param->name.data, "Kd_") != NULL)
                {
                    node->_balance_controller.setTuningsVel();
                }
                if (strcmp(new_param->name.data, "Auto_vel") == 0)
                {
                    node->_balance_controller.setVelPIDOn(node->_velParams.modeAuto);
                }
            }
            else if (strstr(new_param->name.data, "yaw") != NULL)
            {
                node->_balance_controller.updateYawPID();
                if (strstr(new_param->name.data, "Kp_") != NULL ||
                    strstr(new_param->name.data, "Ki_") != NULL ||
                    strstr(new_param->name.data, "Kd_") != NULL)
                {
                    node->_balance_controller.setTuningsYaw();
                }
                if (strcmp(new_param->name.data, "Auto_yaw") == 0)
                {
                    node->_balance_controller.setYawPIDOn(node->_yawParams.modeAuto);
                }
            }
            else if (strcmp(new_param->name.data, "LW_threshold") == 0)
            {
                int64_t threshold;
                if (node->_parameterRegistry.getInt("LW_threshold", &threshold))
                {
                    node->_balance_controller.setLWSreshold(threshold);
                }
            }
            else if (strcmp(new_param->name.data, "RW_threshold") == 0)
            {
                int64_t threshold;
                if (node->_parameterRegistry.getInt("RW_threshold", &threshold))
                {
                    node->_balance_controller.setRWSreshold(threshold);
                }
            }

            else if (strcmp(new_param->name.data, "RW_threshold") == 0)
            {
                int64_t threshold;
                if (node->_parameterRegistry.getInt("RW_threshold", &threshold))
                {
                    node->_balance_controller.setRWSreshold(threshold);
                }
            }

            else if (strcmp(new_param->name.data, "TicKs_p_r") == 0)
            {
                int64_t value;
                if (node->_parameterRegistry.getInt("TicKs_p_r", &value))
                {
                    node->_balance_controller.setTicksPerRevolution(value);
                }
            }

            // Save parameters to EEPROM after successful update
            node->_parameterRegistry.saveAllParameters();
        }
    }

    return ret;
}

// void controllerNode::initTfMessage()
// {
//     // Initialize transform stamped message
//     _tf_msg.header.frame_id.data = (char *)malloc(strlen(odom_frame_name) + 1);
//     _tf_msg.header.frame_id.size = strlen(odom_frame_name);
//     _tf_msg.header.frame_id.capacity = strlen(odom_frame_name) + 1;
//     strcpy(_tf_msg.header.frame_id.data, odom_frame_name);

//     _tf_msg.child_frame_id.data = (char *)malloc(strlen(base_link_name) + 1);
//     _tf_msg.child_frame_id.size = strlen(base_link_name);
//     _tf_msg.child_frame_id.capacity = strlen(base_link_name) + 1;
//     strcpy(_tf_msg.child_frame_id.data, base_link_name);

//     // Initialize transform values to identity
//     _tf_msg.transform.translation.x = 0.0;
//     _tf_msg.transform.translation.y = 0.0;
//     _tf_msg.transform.translation.z = 0.0;
//     _tf_msg.transform.rotation.x = 0.0;
//     _tf_msg.transform.rotation.y = 0.0;
//     _tf_msg.transform.rotation.z = 0.0;
//     _tf_msg.transform.rotation.w = 1.0;
// }

void controllerNode::initTfMessage() {
    // Allocate memory for the transforms array (initially with 1 transform)
    _tf_msg.transforms.capacity = 1;
    _tf_msg.transforms.size = 1;
    _tf_msg.transforms.data = (geometry_msgs__msg__TransformStamped*)malloc(
        sizeof(geometry_msgs__msg__TransformStamped));
    
    // Initialize the transform
    geometry_msgs__msg__TransformStamped* transform = &_tf_msg.transforms.data[0];
    
    // Set frame IDs
    transform->header.frame_id.data = (char*)malloc(strlen(odom_frame_name) + 1);
    transform->header.frame_id.size = strlen(odom_frame_name);
    transform->header.frame_id.capacity = strlen(odom_frame_name) + 1;
    strcpy(transform->header.frame_id.data, odom_frame_name);
    
    transform->child_frame_id.data = (char*)malloc(strlen(base_link_name) + 1);
    transform->child_frame_id.size = strlen(base_link_name);
    transform->child_frame_id.capacity = strlen(base_link_name) + 1;
    strcpy(transform->child_frame_id.data, base_link_name);
    
    // Initialize transform values to identity
    transform->transform.translation.x = 0.0;
    transform->transform.translation.y = 0.0;
    transform->transform.translation.z = 0.0;
    transform->transform.rotation.x = 0.0;
    transform->transform.rotation.y = 0.0;
    transform->transform.rotation.z = 0.0;
    transform->transform.rotation.w = 1.0;
}


// void controllerNode::timer_callback_tf(rcl_timer_t *timer, int64_t last_call_time)
// {
//     controllerNode *node = controllerNode::getInstance();

//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL)
//     {
//         // Get latest transform from balance controller
//         node->_balance_controller.calculateTransform(node->_tf);

//         // Update timestamp
//         rmw_uros_sync_session(10); // Sync time with agent (timeout 10ms)
//         int64_t time_ns = rmw_uros_epoch_nanos();
//         //_tf_msg.header.frame_id.data = node->odom_frame_name;
//         _tf_msg.header.stamp.sec = time_ns / 1000000000;
//         _tf_msg.header.stamp.nanosec = time_ns % 1000000000;

//         // Update transform with latest values
//         _tf_msg.transform.translation.x = node->_tf.t_x;
//         _tf_msg.transform.translation.y = node->_tf.t_y;
//         _tf_msg.transform.translation.z = node->_tf.t_z;
//         _tf_msg.transform.rotation.x = node->_tf.r_x;
//         _tf_msg.transform.rotation.y = node->_tf.r_y;
//         _tf_msg.transform.rotation.z = node->_tf.r_z;
//         _tf_msg.transform.rotation.w = node->_tf.r_w;

//         // Publish the transform
//         RCSOFTCHECK(rcl_publish(&_tf_publisher, &_tf_msg, NULL));
//     }
// }

void controllerNode::timer_callback_tf(rcl_timer_t *timer, int64_t last_call_time) {
    controllerNode *node = controllerNode::getInstance();
    
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        // Get latest transform from balance controller
        node->_balance_controller.calculateTransform(node->_tf);
        
        // Get pointer to the first transform in the array
        geometry_msgs__msg__TransformStamped* transform = &_tf_msg.transforms.data[0];
        
        // Update timestamp
        rmw_uros_sync_session(10); // Sync time with agent (timeout 10ms)
        int64_t time_ns = rmw_uros_epoch_nanos();
        transform->header.stamp.sec = time_ns / 1000000000;
        transform->header.stamp.nanosec = time_ns % 1000000000;
        
        // Update transform with latest values
        transform->transform.translation.x = node->_tf.t_x;
        transform->transform.translation.y = node->_tf.t_y;
        transform->transform.translation.z = node->_tf.t_z;
        transform->transform.rotation.x = node->_tf.r_x;
        transform->transform.rotation.y = node->_tf.r_y;
        transform->transform.rotation.z = node->_tf.r_z;
        transform->transform.rotation.w = node->_tf.r_w;
        
        // Publish the transform message
        RCSOFTCHECK(rcl_publish(&_tf_publisher, &_tf_msg, NULL));
    }
}

