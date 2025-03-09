#include "MicroRos.h"

controllerNode::controllerNode  (
                                ParameterRegistry& parameterRegistry,
                                BalanceController& balance_controller,
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
                                )
    :  _parameterRegistry(parameterRegistry),_balance_controller(balance_controller) ,_ros_domain_id(ros_domain_id),_agentIP(agentIP), _ssid(ssid), _password(password), _agent_port(agent_port),
        _twist_topic(twist_topic), _targetVel(targetVel), _targetYawRate(targetYawRate),
        _controllerState(contrState), _pitchParams(pitchParams), _velParams(velParams), _yawParams(yawParams)
    {

        instance = this;

        //_msgOut.data = 0;


    };

    rcl_node_t controllerNode::_node;
    geometry_msgs__msg__Twist controllerNode::_msgOut;
    rcl_publisher_t controllerNode::_publisher;
    rclc_parameter_server_t controllerNode::_param_server;
    rclc_parameter_options_t controllerNode::_param_options;
    controllerNode* controllerNode::instance = nullptr; // Definition
    
    

    void controllerNode::connect_to_wifi() {
        char ssidR[_ssid.length() + 1];
        char passwordR[_password.length() + 1];
        
        strcpy(ssidR, _ssid.c_str());
        strcpy(passwordR, _password.c_str());
        
        IPAddress agent_ip;
        if (!agent_ip.fromString(_agentIP)) {
            Serial.println("Invalid IP address format");
            return;
        }
        
        set_microros_wifi_transports(ssidR, passwordR, agent_ip, _agent_port);
    }


    // Static wrapper function
    void controllerNode::timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
            // Assuming you have a way to get the instance of controllerNode
            // For example, if you have a global or static instance:
            controllerNode* node = controllerNode::getInstance();
            RCLC_UNUSED(last_call_time);
        if (timer != NULL) {
            _msgOut.linear.x = node->_controllerState.currentVel;
            _msgOut.angular.y = node->_controllerState.currentPitch;
            RCSOFTCHECK(rcl_publish(&_publisher, &_msgOut, NULL));   
        }
            // controllerNode* node = controllerNode::getInstance(); // You need to implement getInstance()
            // node->timer_callback_impl(timer, last_call_time);
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


    void controllerNode::subscription_callback_twist(const void * msgin, void * context) {
        controllerNode* node = static_cast<controllerNode*>(context);
        // Use node to access non-static members
        // Process the message
        if (node) {
            node->handle_twist_message(msgin);
        }
    }



    void controllerNode::handle_twist_message(const void * msgin) {
        const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
        _targetVel = msg->linear.x;
        _targetYawRate = msg->angular.z;
        Serial.printf("vel_x: %f, yawRate: %f\n", _targetVel, _targetYawRate);
    }
    





    void controllerNode::setup(){

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
        RCCHECK(rclc_support_init_with_options(&_support, 0, NULL, &_init_options, &_allocator));
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

        //create subscriber
        RCCHECK(rclc_subscription_init_default(
            &_subscriber_twist,
            &_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "twist_topic"));

        // create timer,
        Serial.println("Subscr init");
        const unsigned int timer_timeout = 500;
        RCCHECK(rclc_timer_init_default(
            &_timer,
            &_support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

        Serial.println("Timer init");


        //create param server

        // Initialize parameter server options
        _param_options = {
            .notify_changed_over_dds = true,
            .max_params = 40,
            .allow_undeclared_parameters = true,
            .low_mem_mode = false };

        // Initialize parameter server with configured options
        RCCHECK(rclc_parameter_server_init_with_option(
            &_param_server, 
            &_node, 
            &_param_options));

        //rclc_parameter_server_init_default(&_param_server, &_node);
        Serial.println("Param server init");

        // create executor

        RCCHECK(rclc_executor_init(
            &_executor, 
            &_support.context, _handle_count + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, 
            &_allocator
        ));



        //delay(1000);
        RCCHECK(rclc_executor_add_parameter_server_with_context(
            &_executor, 
            &_param_server, 
            on_parameter_changed,//on_parameter_changed
            this
        ));

        RCCHECK(rclc_executor_add_timer(&_executor, &_timer));
        RCCHECK(rclc_executor_add_subscription_with_context(
            &_executor,
            &_subscriber_twist,
            &_twst_msg,
            &subscription_callback_twist,
            this,
            ON_NEW_DATA
        ));

        set_parameters();
        //_msgOut.data = 0;
        //rclc_executor_set_context(&_executor, this);
        
    }

    void controllerNode::set_parameters() {
        const double pid_gain_from = 0.0;
        const double pid_gain_to = 70.0;
        const double pid_gain_step = 0.01;
    
        const unsigned int period_from = 1;
        const unsigned int period_to = 1000;
        const unsigned int period_step = 1;
        
        // Load parameters from registry to local variables first
        bool success = _parameterRegistry.loadAllParameters();
        if (!success) {
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
        _parameterRegistry.addIntParameter("Left_wheel_threshold", 0, "Left wheel starting threshold", "", 0, 255, 1);
        _parameterRegistry.addIntParameter("Right_wheel_threshold", 0, "Right wheel starting threshold", "", 0, 255, 1);
        _parameterRegistry.addIntParameter("Ticks_per_revolution", 292, "Ticks of encoder per full revolution", "", 0, 2000, 1);
        
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
        if (_parameterRegistry.getInt("Left_wheel_threshold", &leftThreshold)) {
            _balance_controller.setLWSreshold(leftThreshold);
        }
        if (_parameterRegistry.getInt("Right_wheel_threshold", &rightThreshold)) {
            _balance_controller.setRWSreshold(rightThreshold);
        }
        if (_parameterRegistry.getInt("Ticks_per_revolution", &ticksPerRev)) {
            _balance_controller.setTicksPerRevolution(ticksPerRev);
        }
    }
    

    // void controllerNode::set_parameters(){

    //     const double pid_gain_from = 0.0;
    //     const double pid_gain_to = 70.0;
    //     const double pid_gain_step = 0.01;

    //     const unsigned int period_from = 1;
    //     const unsigned int period_to = 1000;
    //     const unsigned int period_step = 1000;
        

    //     // struct pidParams
    //     //         {
    //     //             double Kp;
    //     //             double Ki;
    //     //             double Kd;
    //     //             float max;
    //     //             float min;
    //     //             int period;
    //     //             bool direct;
    //     //             bool modeAuto;
    //     //             /* data */
    //     //         };
    //     // Add parameters
    //     //Pitch PID parameters
    //     rclc_add_parameter(&_param_server, "Kp_pitch", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Ki_pitch", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Kd_pitch", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Max_pitch", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Min_pitch", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Period_pitch", RCLC_PARAMETER_INT);
    //     rclc_add_parameter(&_param_server, "Direction_pitch", RCLC_PARAMETER_BOOL);
    //     rclc_add_parameter(&_param_server, "Auto_pitch", RCLC_PARAMETER_BOOL);
    //     //Vel PID parameters
    //     rclc_add_parameter(&_param_server, "Kp_vel", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Ki_vel", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Kd_vel", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Max_vel", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Min_vel", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Period_vel", RCLC_PARAMETER_INT);
    //     rclc_add_parameter(&_param_server, "Direction_vel", RCLC_PARAMETER_BOOL);
    //     rclc_add_parameter(&_param_server, "Auto_vel", RCLC_PARAMETER_BOOL);

    //     //Yaw rate PID parameters
    //     rclc_add_parameter(&_param_server, "Kp_yaw", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Ki_yaw", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Kd_yaw", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Max_yaw", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Min_yaw", RCLC_PARAMETER_DOUBLE);
    //     rclc_add_parameter(&_param_server, "Period_yaw", RCLC_PARAMETER_INT);
    //     rclc_add_parameter(&_param_server, "Direction_yaw", RCLC_PARAMETER_BOOL);
    //     rclc_add_parameter(&_param_server, "Auto_yaw", RCLC_PARAMETER_BOOL);

    //     // Wheels parameters

    //     rclc_add_parameter(&_param_server, "Left_wheel_threshold", RCLC_PARAMETER_INT);
    //     rclc_add_parameter(&_param_server, "Right_wheel_threshold", RCLC_PARAMETER_INT);
    //     rclc_add_parameter(&_param_server, "Ticks_per_revolution", RCLC_PARAMETER_INT);

    //     // Add parameters descriptions 
    //     //Pitch parameters descriptions
    //     rclc_add_parameter_description(&_param_server, "Kp_pitch", "Pitch PID Kp", "");
    //     rclc_add_parameter_description(&_param_server, "Ki_pitch", "Pitch PID Ki", "");
    //     rclc_add_parameter_description(&_param_server, "Kd_pitch", "Pitch PID Kd", "");
    //     rclc_add_parameter_description(&_param_server, "Max_pitch", "Pitch PID max output", "");
    //     rclc_add_parameter_description(&_param_server, "Min_pitch", "Pitch PID min output", "");
    //     rclc_add_parameter_description(&_param_server, "Period_pitch", "Pitch PID period", "ms");
    //     rclc_add_parameter_description(&_param_server, "Direction_pitch", "Pitch PID Direction", "");
    //     rclc_add_parameter_description(&_param_server, "Auto_pitch", "Pitch PID auto", "");

    //     //Vel parameters descriptions
    //     rclc_add_parameter_description(&_param_server, "Kp_vel", "Vel PID Kp", "");
    //     rclc_add_parameter_description(&_param_server, "Ki_vel", "Vel PID Ki", "");
    //     rclc_add_parameter_description(&_param_server, "Kd_vel", "Vel PID Kd", "");
    //     rclc_add_parameter_description(&_param_server, "Max_vel", "Vel PID max output", "");
    //     rclc_add_parameter_description(&_param_server, "Min_vel", "Vel PID min output", "");
    //     rclc_add_parameter_description(&_param_server, "Period_vel", "Vel PID period", "ms");
    //     rclc_add_parameter_description(&_param_server, "Direction_vel", "Vel PID Direction", "");
    //     rclc_add_parameter_description(&_param_server, "Auto_vel", "Vel PID auto", "");

    //     //Yaw rate parameters descriptions
    //     rclc_add_parameter_description(&_param_server, "Kp_yaw", "Yaw rate PID Kp", "");
    //     rclc_add_parameter_description(&_param_server, "Ki_yaw", "Yaw rate PID Ki", "");
    //     rclc_add_parameter_description(&_param_server, "Kd_yaw", "Yaw rate PID Kd", "");
    //     rclc_add_parameter_description(&_param_server, "Max_yaw", "Yaw rate PID max output", "");
    //     rclc_add_parameter_description(&_param_server, "Min_yaw", "Yaw rate PID min output", "");
    //     rclc_add_parameter_description(&_param_server, "Period_yaw", "Yaw rate PID period", "ms");
    //     rclc_add_parameter_description(&_param_server, "Direction_yaw", "Yaw rate PID Direction", "");
    //     rclc_add_parameter_description(&_param_server, "Auto_yaw", "Yaw rate PID auto", "");

    //     // Wheels sresholds
        
    //     rclc_add_parameter_description(&_param_server, "Left_wheel_threshold", "Left whhel starting sreshold", "");
    //     rclc_add_parameter_description(&_param_server, "Right_wheel_threshold", "Right whhel starting sreshold", "");
    //     rclc_add_parameter_description(&_param_server, "Ticks_per_revolution", "Ticks of encoder per full revolution", "");



    //     // Add parameters constrains 
    //     //Pitch parameters constrains
    //     rclc_add_parameter_constraint_double(&_param_server, "Kp_pitch", pid_gain_from, pid_gain_to, pid_gain_step);
    //     rclc_add_parameter_constraint_double(&_param_server, "Ki_pitch", pid_gain_from, pid_gain_to, pid_gain_step);
    //     rclc_add_parameter_constraint_double(&_param_server, "Kd_pitch", pid_gain_from, pid_gain_to, pid_gain_step);
    //     rclc_add_parameter_constraint_double(&_param_server, "Max_pitch", 0, 255, 1);
    //     rclc_add_parameter_constraint_double(&_param_server, "Min_pitch", -255, 0, 1);
    //     rclc_add_parameter_constraint_double(&_param_server, "Period_pitch", period_from, period_to, period_step);
         
    //     //Vel parameters constrains
    //      rclc_add_parameter_constraint_double(&_param_server, "Kp_vel", pid_gain_from, pid_gain_to, pid_gain_step);
    //      rclc_add_parameter_constraint_double(&_param_server, "Ki_vel", pid_gain_from, pid_gain_to, pid_gain_step);
    //      rclc_add_parameter_constraint_double(&_param_server, "Kd_vel", pid_gain_from, pid_gain_to, pid_gain_step);
    //      rclc_add_parameter_constraint_double(&_param_server, "Max_vel", -4.0, 4.0, 0.01);
    //      rclc_add_parameter_constraint_double(&_param_server, "Min_vel", -4.0, 4.0, 0.01);
    //      rclc_add_parameter_constraint_double(&_param_server, "Period_vel", period_from, period_to, period_step);

    //       //Yaw rate parameters constrains
    //     rclc_add_parameter_constraint_double(&_param_server, "Kp_yaw", pid_gain_from, pid_gain_to, pid_gain_step);
    //     rclc_add_parameter_constraint_double(&_param_server, "Ki_yaw", pid_gain_from, pid_gain_to, pid_gain_step);
    //     rclc_add_parameter_constraint_double(&_param_server, "Kd_yaw", pid_gain_from, pid_gain_to, pid_gain_step);
    //     rclc_add_parameter_constraint_double(&_param_server, "Max_yaw", -255, 255, 1);
    //     rclc_add_parameter_constraint_double(&_param_server, "Min_yaw", -255, 255, 1);
    //     rclc_add_parameter_constraint_double(&_param_server, "Period_yaw", period_from, period_to, period_step);

    //       //Wheel sresholds parameters constrains
    //     rclc_add_parameter_constraint_double(&_param_server, "Left_wheel_threshold", 0, 255, 1);
    //     rclc_add_parameter_constraint_double(&_param_server, "Right_wheel_threshold", 0, 255, 1);
    //     rclc_add_parameter_constraint_double(&_param_server, "Ticks_per_revolution", 0, 2000, 1);



    //     // Set parameter initial values
    //     // Pitch PID initial values
    //     rclc_parameter_set_double(&_param_server, "Kp_pitch", _pitchParams.Kp);
    //     rclc_parameter_set_double(&_param_server, "Ki_pitch", _pitchParams.Ki);
    //     rclc_parameter_set_double(&_param_server, "Kd_pitch", _pitchParams.Kd);
    //     rclc_parameter_set_double(&_param_server, "Max_pitch", _pitchParams.max);
    //     rclc_parameter_set_double(&_param_server, "Min_pitch", _pitchParams.min);
    //     rclc_parameter_set_int(&_param_server, "Period_pitch", _pitchParams.period);
    //     rclc_parameter_set_bool(&_param_server, "Direction_pitch", _pitchParams.direct);
    //     rclc_parameter_set_bool(&_param_server, "Auto_pitch", _pitchParams.modeAuto);

    //     // Vel PID initial values
    //     rclc_parameter_set_double(&_param_server, "Kp_vel", _velParams.Kp);
    //     rclc_parameter_set_double(&_param_server, "Ki_vel", _velParams.Ki);
    //     rclc_parameter_set_double(&_param_server, "Kd_vel", _velParams.Kd);
    //     rclc_parameter_set_double(&_param_server, "Max_vel", _velParams.max);
    //     rclc_parameter_set_double(&_param_server, "Min_vel", _velParams.min);
    //     rclc_parameter_set_int(&_param_server, "Period_vel", _velParams.period);
    //     rclc_parameter_set_bool(&_param_server, "Direction_vel", _velParams.direct);
    //     rclc_parameter_set_bool(&_param_server, "Auto_vel", _velParams.modeAuto);

    //     // Yaw Rate PID initial values
    //     rclc_parameter_set_double(&_param_server, "Kp_yaw", _yawParams.Kp);
    //     rclc_parameter_set_double(&_param_server, "Ki_yaw", _yawParams.Ki);
    //     rclc_parameter_set_double(&_param_server, "Kd_yaw", _yawParams.Kd);
    //     rclc_parameter_set_double(&_param_server, "Max_yaw", _yawParams.max);
    //     rclc_parameter_set_double(&_param_server, "Min_yaw", _yawParams.min);
    //     rclc_parameter_set_int(&_param_server, "Period_yaw", _yawParams.period);
    //     rclc_parameter_set_bool(&_param_server, "Direction_yaw", _yawParams.direct);
    //     rclc_parameter_set_bool(&_param_server, "Auto_yaw", _yawParams.modeAuto);
    //     //wheel params
    //     rclc_parameter_set_int(&_param_server, "Ticks_per_revolution", 292);

    //     }

    






    void controllerNode::spinNode(){
        RCSOFTCHECK(rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(100)));

    };


    // bool controllerNode::on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
    // {
    //    // (void) context;
    //     controllerNode* node = static_cast<controllerNode*>(context);
    
    //     if (old_param == NULL && new_param == NULL) {
    //         printf("Callback error, both parameters are NULL\n");
    //         return false;
    //     }
    
    //     bool ret = true;
    
    //     if (new_param == NULL) {
    //         printf("Delete parameter %s rejected\n", old_param->name.data);
    //         ret = false;
    //     } else {
    //         // Check parameter type and assign value
    //         if (strcmp(new_param->name.data, "Kp_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_pitchParams.Kp = new_param->value.double_value;
    //             node->_balance_controller.setTuningsPitch();
    //             printf("Kp_pitch updated to %f\n", node->_pitchParams.Kp);


    //         } else if (strcmp(new_param->name.data, "Ki_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_pitchParams.Ki = new_param->value.double_value;
    //             node->_balance_controller.setTuningsPitch();
    //             printf("Ki_pitch updated to %f\n", node->_pitchParams.Ki);


    //         } else if (strcmp(new_param->name.data, "Kd_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_pitchParams.Kd = new_param->value.double_value;
    //             node->_balance_controller.setTuningsPitch();
    //             printf("Kd_pitch updated to %f\n", node->_pitchParams.Kd);


    //         } else if (strcmp(new_param->name.data, "Max_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             if (new_param->value.double_value >= 0 && new_param->value.double_value <= 255) {
    //                 node->_pitchParams.max = new_param->value.double_value;
    //                 node->_balance_controller.updatePitchPID();
    //                 printf("Max_pitch updated to %f\n", node->_pitchParams.max);
    //             } else {
    //                 printf("Invalid Max_pitch value: %f. Must be between 0 and 255.\n", new_param->value.double_value);
    //                 ret = false;
    //             }


    //         } else if (strcmp(new_param->name.data, "Min_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             if (new_param->value.double_value >= -255 && new_param->value.double_value <= 0) {
    //                 node->_pitchParams.min = new_param->value.double_value;
    //                 node->_balance_controller.updatePitchPID();
    //                 printf("Min_pitch updated to %f\n", node->_pitchParams.min);
    //             } else {
    //                 printf("Invalid Min_pitch value: %f. Must be between -255 and 0.\n", new_param->value.double_value);
    //                 ret = false;
    //             }


    //         } else if (strcmp(new_param->name.data, "Period_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
    //             if (new_param->value.integer_value >= 1 && new_param->value.integer_value <= 1000) {
    //                 node->_pitchParams.period = new_param->value.integer_value;
    //                 node->_balance_controller.updatePitchPID();
    //                 printf("Period_pitch updated to %d\n", node->_pitchParams.period);
    //             } else {
    //                 printf("Invalid Period_pitch value: %d. Must be between 1 and 1000.\n", new_param->value.integer_value);
    //                 ret = false;
    //             }


    //         } else if (strcmp(new_param->name.data, "Direction_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_BOOL) {
    //             node->_pitchParams.direct = new_param->value.bool_value;
    //             printf("Direction_pitch updated to %s\n", (node->_pitchParams.direct) ? "true" : "false");

    //         } else if (strcmp(new_param->name.data, "Auto_pitch") == 0 && new_param->value.type == RCLC_PARAMETER_BOOL) {
    //             node->_pitchParams.modeAuto = new_param->value.bool_value;
    //             node->_balance_controller.setPitchPIDOn(new_param->value.bool_value);
    //             node->_balance_controller.updatePitchPID();
    //             printf("Auto_pitch updated to %s\n", (node->_pitchParams.modeAuto) ? "true" : "false");
    //         }
    
    //         // Vel PID
    //         else if (strcmp(new_param->name.data, "Kp_vel") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_velParams.Kp = new_param->value.double_value;
    //             node->_balance_controller.setTuningsVel();
    //             printf("Kp_vel updated to %f\n", node->_velParams.Kp);


    //         } else if (strcmp(new_param->name.data, "Ki_vel") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_velParams.Ki = new_param->value.double_value;
    //             node->_balance_controller.setTuningsVel();
    //             printf("Ki_vel updated to %f\n", node->_velParams.Ki);


    //         } else if (strcmp(new_param->name.data, "Kd_vel") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_velParams.Kd = new_param->value.double_value;
    //             node->_balance_controller.setTuningsVel();
    //             printf("Kd_vel updated to %f\n", node->_velParams.Kd);


    //         } else if (strcmp(new_param->name.data, "Max_vel") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             if (new_param->value.double_value >= -4.0 && new_param->value.double_value <= 4.0) {
    //                 node->_velParams.max = new_param->value.double_value;
    //                 node->_balance_controller.updateVelPID();
    //                 printf("Max_vel updated to %f\n", node->_velParams.max);
    //             } else {
    //                 printf("Invalid Max_vel value: %f. Must be between -4.0 and 4.0.\n", new_param->value.double_value);
    //                 ret = false;
    //             }


    //         } else if (strcmp(new_param->name.data, "Min_vel") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             if (new_param->value.double_value >= -4.0 && new_param->value.double_value <= 4.0) {
    //                 node->_velParams.min = new_param->value.double_value;
    //                 node->_balance_controller.updateVelPID();
    //                 printf("Min_vel updated to %f\n", node->_velParams.min);
    //             } else {
    //                 printf("Invalid Min_vel value: %f. Must be between -4.0 and 4.0.\n", new_param->value.double_value);
    //                 ret = false;
    //             }


    //         } else if (strcmp(new_param->name.data, "Period_vel") == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
    //             if (new_param->value.integer_value >= 1 && new_param->value.integer_value <= 1000) {
    //                 node->_velParams.period = new_param->value.integer_value;
    //                 node->_balance_controller.updateVelPID();
    //                 printf("Period_vel updated to %d\n", node->_velParams.period);
    //             } else {
    //                 printf("Invalid Period_vel value: %d. Must be between 1 and 1000.\n", new_param->value.integer_value);
    //                 ret = false;
    //             }


    //         } else if (strcmp(new_param->name.data, "Direction_vel") == 0 && new_param->value.type == RCLC_PARAMETER_BOOL) {
    //             node->_velParams.direct = new_param->value.bool_value;
    //             node->_balance_controller.updateVelPID();
    //             printf("Direction_vel updated to %s\n", (node->_velParams.direct) ? "true" : "false");


    //         } else if (strcmp(new_param->name.data, "Auto_vel") == 0 && new_param->value.type == RCLC_PARAMETER_BOOL) {
    //             node->_velParams.modeAuto = new_param->value.bool_value;
    //             node->_balance_controller.setVelPIDOn(new_param->value.bool_value);
    //             node->_balance_controller.updateVelPID();
    //             printf("Auto_vel updated to %s\n", (node->_velParams.modeAuto) ? "true" : "false");
    //         }

    //         //Yaw PID
    //         else if (strcmp(new_param->name.data, "Kp_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_yawParams.Kp = new_param->value.double_value;
    //             node->_balance_controller.setTuningsYaw();
    //             printf("Kp_yaw updated to %f\n", node->_yawParams.Kp);


    //         } else if (strcmp(new_param->name.data, "Ki_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_yawParams.Ki = new_param->value.double_value;
    //             node->_balance_controller.setTuningsYaw();
    //             printf("Ki_yaw updated to %f\n", node->_yawParams.Ki);


    //         } else if (strcmp(new_param->name.data, "Kd_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             node->_yawParams.Kd = new_param->value.double_value;
    //             node->_balance_controller.setTuningsYaw();
    //             printf("Kd_yaw updated to %f\n", node->_yawParams.Kd);


    //         } else if (strcmp(new_param->name.data, "Max_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             if (new_param->value.double_value >= -255 && new_param->value.double_value <= 255) {
    //                 node->_yawParams.max = new_param->value.double_value;
    //                 node->_balance_controller.updateYawPID();
    //                 printf("Max_yaw updated to %f\n", node->_yawParams.max);
    //             } else {
    //                 printf("Invalid Max_yaw value: %f. Must be between -255 and 255.\n", new_param->value.double_value);
    //                 ret = false;
    //             }

    //         } else if (strcmp(new_param->name.data, "Min_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_DOUBLE) {
    //             if (new_param->value.double_value >= -255 && new_param->value.double_value <= 255) {
    //                 node->_yawParams.min = new_param->value.double_value;
    //                 node->_balance_controller.updateYawPID();
    //                 printf("Min_yaw updated to %f\n", node->_yawParams.min);
    //             } else {
    //                 printf("Invalid Min_yaw value: %f. Must be between -255 and 255.\n", new_param->value.double_value);
    //                 ret = false;
    //             }

    //         } else if (strcmp(new_param->name.data, "Period_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
    //             if (new_param->value.integer_value >= 1 && new_param->value.integer_value <= 1000) {
    //                 node->_yawParams.period = new_param->value.integer_value;
    //                 node->_balance_controller.updateYawPID();
    //                 printf("Period_yaw updated to %d\n", node->_yawParams.period);
    //             } else {
    //                 printf("Invalid Period_yaw value: %d. Must be between 1 and 1000.\n", new_param->value.integer_value);
    //                 ret = false;
    //             }

    //         } else if (strcmp(new_param->name.data, "Direction_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_BOOL) {
    //             node->_yawParams.direct = new_param->value.bool_value;
    //             node->_balance_controller.updateYawPID();
    //             printf("Direction_yaw updated to %s\n", (node->_yawParams.direct) ? "true" : "false");

    //         } else if (strcmp(new_param->name.data, "Auto_yaw") == 0 && new_param->value.type == RCLC_PARAMETER_BOOL) {
    //             node->_yawParams.modeAuto = new_param->value.bool_value;
    //             node->_balance_controller.setYawPIDOn(new_param->value.bool_value);
    //             node->_balance_controller.updateYawPID();
    //             printf("Auto_yaw updated to %s\n", (node->_yawParams.modeAuto) ? "true" : "false");
               
    //         //Whheel sresholds
    //         } else if (strcmp(new_param->name.data, "Left_wheel_threshold") == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
    //             if (new_param->value.integer_value >= 0 && new_param->value.integer_value <= 255) {
    //                 node->_balance_controller.setLWSreshold(new_param->value.integer_value);
    //                 printf("Left_wheel_threshold updated to %d\n", new_param->value.integer_value);
    //             } else {
    //                 printf("Invalid Left_wheel_threshold value: %d. Must be between 0 and 255.\n", new_param->value.integer_value);
    //                 ret = false;
    //             }

    //         } else if (strcmp(new_param->name.data, "Right_wheel_threshold") == 0 && new_param->value.type == RCLC_PARAMETER_INT) {
    //             if (new_param->value.integer_value >= 0 && new_param->value.integer_value <= 255) {
    //                 node->_balance_controller.setRWSreshold(new_param->value.integer_value);
    //                 printf("Right_wheel_threshold updated to %d\n", new_param->value.integer_value);
    //             } else {
    //                 printf("Invalid Right_wheel_threshold value: %d. Must be between 0 and 255.\n", new_param->value.integer_value);
    //                 ret = false;
    //             }
    //         }
    //     }

    
    //     return ret;
    // }


//     bool controllerNode::on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
// {
//     controllerNode* node = static_cast<controllerNode*>(context);
    
//     if (old_param == NULL && new_param == NULL) {
//         printf("Callback error, both parameters are NULL\n");
//         return false;
//     }
    
//     bool ret = true;
    
//     if (new_param == NULL) {
//         printf("Delete parameter %s rejected\n", old_param->name.data);
//         ret = false;
//     } else {
//         // Use the parameter registry to update values
//         switch (new_param->value.type) {
//             case RCLC_PARAMETER_BOOL:
//                 if (!node->_parameterRegistry.setBool(new_param->name.data, new_param->value.bool_value)) {
//                     printf("Failed to update bool parameter: %s\n", new_param->name.data);
//                     ret = false;
//                 } else {
//                     printf("Parameter %s updated to %s\n", new_param->name.data, 
//                            new_param->value.bool_value ? "true" : "false");
//                 }
//                 break;
                
//             case RCLC_PARAMETER_INT:
//                 if (!node->_parameterRegistry.setInt(new_param->name.data, new_param->value.integer_value)) {
//                     printf("Failed to update int parameter: %s\n", new_param->name.data);
//                     ret = false;
//                 } else {
//                     printf("Parameter %s updated to %d\n", new_param->name.data, 
//                            new_param->value.integer_value);
//                 }
//                 break;
                
//             case RCLC_PARAMETER_DOUBLE:
//                 if (!node->_parameterRegistry.setDouble(new_param->name.data, new_param->value.double_value)) {
//                     printf("Failed to update double parameter: %s\n", new_param->name.data);
//                     ret = false;
//                 } else {
//                     printf("Parameter %s updated to %f\n", new_param->name.data, 
//                            new_param->value.double_value);
//                 }
//                 break;
                
//             default:
//                 printf("Unsupported parameter type for %s\n", new_param->name.data);
//                 ret = false;
//                 break;
//         }
        
//         // If parameter was updated successfully, apply the changes
//         if (ret) {
//             // Update PID controllers based on parameter changes
//             if (strstr(new_param->name.data, "pitch") != NULL) {
//                 node->_balance_controller.updatePitchPID();
//                 if (strstr(new_param->name.data, "Kp_") != NULL || 
//                     strstr(new_param->name.data, "Ki_") != NULL || 
//                     strstr(new_param->name.data, "Kd_") != NULL) {
//                     node->_balance_controller.setTuningsPitch();
//                 }
//                 if (strcmp(new_param->name.data, "Auto_pitch") == 0) {
//                     bool auto_mode;
//                     if (node->_parameterRegistry.getBool("Auto_pitch", &auto_mode)) {
//                         node->_balance_controller.setPitchPIDOn(auto_mode);
//                     }
//                 }
//             } 
//             else if (strstr(new_param->name.data, "vel") != NULL) {
//                 node->_balance_controller.updateVelPID();
//                 if (strstr(new_param->name.data, "Kp_") != NULL || 
//                     strstr(new_param->name.data, "Ki_") != NULL || 
//                     strstr(new_param->name.data, "Kd_") != NULL) {
//                     node->_balance_controller.setTuningsVel();
//                 }
//                 if (strcmp(new_param->name.data, "Auto_vel") == 0) {
//                     bool auto_mode;
//                     if (node->_parameterRegistry.getBool("Auto_vel", &auto_mode)) {
//                         node->_balance_controller.setVelPIDOn(auto_mode);
//                     }
//                 }
//             }
//             else if (strstr(new_param->name.data, "yaw") != NULL) {
//                 node->_balance_controller.updateYawPID();
//                 if (strstr(new_param->name.data, "Kp_") != NULL || 
//                     strstr(new_param->name.data, "Ki_") != NULL || 
//                     strstr(new_param->name.data, "Kd_") != NULL) {
//                     node->_balance_controller.setTuningsYaw();
//                 }
//                 if (strcmp(new_param->name.data, "Auto_yaw") == 0) {
//                     bool auto_mode;
//                     if (node->_parameterRegistry.getBool("Auto_yaw", &auto_mode)) {
//                         node->_balance_controller.setYawPIDOn(auto_mode);
//                     }
//                 }
//             }
//             else if (strcmp(new_param->name.data, "Left_wheel_threshold") == 0) {
//                 int64_t threshold;
//                 if (node->_parameterRegistry.getInt("Left_wheel_threshold", &threshold)) {
//                     node->_balance_controller.setLWSreshold(threshold);
//                 }
//             }
//             else if (strcmp(new_param->name.data, "Right_wheel_threshold") == 0) {
//                 int64_t threshold;
//                 if (node->_parameterRegistry.getInt("Right_wheel_threshold", &threshold)) {
//                     node->_balance_controller.setRWSreshold(threshold);
//                 }
//             }
            
//             // Save parameters to EEPROM after successful update
//             //node->_parameterRegistry.saveAllParameters();
//         }
//     }
    
//     return ret;
// }



bool controllerNode::on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
    controllerNode* node = static_cast<controllerNode*>(context);
    
    if (old_param == NULL && new_param == NULL) {
        printf("Callback error, both parameters are NULL\n");
        return false;
    }
    
    bool ret = true;
    
    if (new_param == NULL) {
        printf("Delete parameter %s rejected\n", old_param->name.data);
        ret = false;
    } else {
        // Use the parameter registry to update values
        switch (new_param->value.type) {
            case RCLC_PARAMETER_BOOL:
                if (!node->_parameterRegistry.setBool(new_param->name.data, new_param->value.bool_value)) {
                    printf("Failed to update bool parameter: %s\n", new_param->name.data);
                    ret = false;
                } else {
                    printf("Parameter %s updated to %s\n", new_param->name.data, 
                           new_param->value.bool_value ? "true" : "false");
                }
                break;
                
            case RCLC_PARAMETER_INT:
                if (!node->_parameterRegistry.setInt(new_param->name.data, new_param->value.integer_value)) {
                    printf("Failed to update int parameter: %s\n", new_param->name.data);
                    ret = false;
                } else {
                    printf("Parameter %s updated to %d\n", new_param->name.data, 
                           new_param->value.integer_value);
                }
                break;
                
            case RCLC_PARAMETER_DOUBLE:
                if (!node->_parameterRegistry.setDouble(new_param->name.data, new_param->value.double_value)) {
                    printf("Failed to update double parameter: %s\n", new_param->name.data);
                    ret = false;
                } else {
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
        if (ret) {
            // Update all bindings to ensure external variables are synchronized
            node->_parameterRegistry.updateAllBindings();
            
            // Update PID controllers based on parameter changes
            if (strstr(new_param->name.data, "pitch") != NULL) {
                node->_balance_controller.updatePitchPID();
                if (strstr(new_param->name.data, "Kp_") != NULL || 
                    strstr(new_param->name.data, "Ki_") != NULL || 
                    strstr(new_param->name.data, "Kd_") != NULL) {
                    node->_balance_controller.setTuningsPitch();
                }
                if (strcmp(new_param->name.data, "Auto_pitch") == 0) {
                    node->_balance_controller.setPitchPIDOn(node->_pitchParams.modeAuto);
                }
            } 
            else if (strstr(new_param->name.data, "vel") != NULL) {
                node->_balance_controller.updateVelPID();
                if (strstr(new_param->name.data, "Kp_") != NULL || 
                    strstr(new_param->name.data, "Ki_") != NULL || 
                    strstr(new_param->name.data, "Kd_") != NULL) {
                    node->_balance_controller.setTuningsVel();
                }
                if (strcmp(new_param->name.data, "Auto_vel") == 0) {
                    node->_balance_controller.setVelPIDOn(node->_velParams.modeAuto);
                }
            }
            else if (strstr(new_param->name.data, "yaw") != NULL) {
                node->_balance_controller.updateYawPID();
                if (strstr(new_param->name.data, "Kp_") != NULL || 
                    strstr(new_param->name.data, "Ki_") != NULL || 
                    strstr(new_param->name.data, "Kd_") != NULL) {
                    node->_balance_controller.setTuningsYaw();
                }
                if (strcmp(new_param->name.data, "Auto_yaw") == 0) {
                    node->_balance_controller.setYawPIDOn(node->_yawParams.modeAuto);
                }
            }
            else if (strcmp(new_param->name.data, "Left_wheel_threshold") == 0) {
                int64_t threshold;
                if (node->_parameterRegistry.getInt("Left_wheel_threshold", &threshold)) {
                    node->_balance_controller.setLWSreshold(threshold);
                }
            }
            else if (strcmp(new_param->name.data, "Right_wheel_threshold") == 0) {
                int64_t threshold;
                if (node->_parameterRegistry.getInt("Right_wheel_threshold", &threshold)) {
                    node->_balance_controller.setRWSreshold(threshold);
                }
            }

            else if (strcmp(new_param->name.data, "Right_wheel_threshold") == 0) {
                int64_t threshold;
                if (node->_parameterRegistry.getInt("Right_wheel_threshold", &threshold)) {
                    node->_balance_controller.setRWSreshold(threshold);
                }
            }
            
            else if (strcmp(new_param->name.data, "Ticks_per_revolution") == 0) {
                int64_t value;
                if (node->_parameterRegistry.getInt("Ticks_per_revolution", &value)) {
                    node->_balance_controller.setTicksPerRevolution(value);
                }
            }
            
            // Save parameters to EEPROM after successful update
            node->_parameterRegistry.saveAllParameters();
        }
    }
    
    return ret;
}


    

 