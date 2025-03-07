#include "MicroRos.h"

controllerNode::controllerNode  (
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
    :  _ros_domain_id(ros_domain_id),_agentIP(agentIP), _ssid(ssid), _password(password), _agent_port(agent_port),
        _twist_topic(twist_topic), _targetVel(targetVel), _targetYawRate(targetYawRate),
        _controllerState(contrState), _pitchParams(pitchParams), _velParams(velParams), _yawParams(yawParams)
    {

        //_msgOut.data = 0;


    };

    rcl_node_t controllerNode::_node;
    std_msgs__msg__Int32 controllerNode::_msgOut;
    rcl_publisher_t controllerNode::_publisher;
    rclc_parameter_server_t controllerNode::_param_server;
    rclc_parameter_options_t controllerNode::_param_options;
    

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

    void controllerNode::timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
        RCLC_UNUSED(last_call_time);
        if (timer != NULL) {
          RCSOFTCHECK(rcl_publish(&_publisher, &_msgOut, NULL));
          _msgOut.data++;
          //Serial.println(msg.data);
        }
    }


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
        _msgOut.data = 0; 
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
        RCCHECK(rclc_node_init_default(&_node, "micro_ros_platformio_node", "", &_support));
        Serial.println("Node init");
        // create publisher
        RCCHECK(rclc_publisher_init_default(
            &_publisher,
            &_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "micro_ros_platformio_node_publisher"));
        
        Serial.println("Publisher init");

        //create subscriber
        RCCHECK(rclc_subscription_init_default(
            &_subscriber_twist,
            &_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "twist_topic"));

        // create timer,
        Serial.println("Subscr init");
        const unsigned int timer_timeout = 1000;
        RCCHECK(rclc_timer_init_default(
            &_timer,
            &_support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

        Serial.println("Timer init");


        //create param server

        // // Initialize parameter server options
        // _param_options = {
        //     .notify_changed_over_dds = true,
        //     .max_params = 30,
        //     .allow_undeclared_parameters = true,
        //     .low_mem_mode = false };

        // // Initialize parameter server with configured options
        // RCCHECK(rclc_parameter_server_init_with_option(
        //     &_param_server, 
        //     &_node, 
        //     &_param_options));

        RCCHECK(rclc_parameter_server_init_default(&_param_server, &_node));
        Serial.println("Param server init");

        // create executor

        RCCHECK(rclc_executor_init(
            &_executor, 
            &_support.context, _handle_count + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES, 
            &_allocator
        ));



        delay(1000);
        RCCHECK(rclc_executor_add_parameter_server(
            &_executor, 
            &_param_server, 
            NULL//on_parameter_changed
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

        //set_parameters();
        //_msgOut.data = 0;
        //rclc_executor_set_context(&_executor, this);
        
    }

    void controllerNode::set_parameters(){

        const double pid_gain_from = 0.0;
        const double pid_gain_to = 70.0;
        const double pid_gain_step = 0.01;

        const unsigned int period_from = 1;
        const unsigned int period_to = 1000;
        const unsigned int period_step = 1000;
        

        // struct pidParams
        //         {
        //             double Kp;
        //             double Ki;
        //             double Kd;
        //             float max;
        //             float min;
        //             int period;
        //             bool direct;
        //             bool modeAuto;
        //             /* data */
        //         };
        // Add parameters
        //Pitch PID parameters
        rclc_add_parameter(&_param_server, "Kp_pitch", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Ki_pitch", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Kd_pitch", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Max_pitch", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Min_pitch", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Period_pitch", RCLC_PARAMETER_INT);
        rclc_add_parameter(&_param_server, "Direction_pitch", RCLC_PARAMETER_BOOL);
        rclc_add_parameter(&_param_server, "Auto_pitch", RCLC_PARAMETER_BOOL);
        //Vel PID parameters
        rclc_add_parameter(&_param_server, "Kp_vel", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Ki_vel", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Kd_vel", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Max_vel", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Min_vel", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Period_vel", RCLC_PARAMETER_INT);
        rclc_add_parameter(&_param_server, "Direction_vel", RCLC_PARAMETER_BOOL);
        rclc_add_parameter(&_param_server, "Auto_vel", RCLC_PARAMETER_BOOL);

        //Yaw rate PID parameters
        rclc_add_parameter(&_param_server, "Kp_yaw", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Ki_yaw", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Kd_yaw", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Max_yaw", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Min_yaw", RCLC_PARAMETER_DOUBLE);
        rclc_add_parameter(&_param_server, "Period_yaw", RCLC_PARAMETER_INT);
        rclc_add_parameter(&_param_server, "Direction_yaw", RCLC_PARAMETER_BOOL);
        rclc_add_parameter(&_param_server, "Auto_yaw", RCLC_PARAMETER_BOOL);

        // Add parameters descriptions 
        //Pitch parameters descriptions
        rclc_add_parameter_description(&_param_server, "Kp_pitch", "Pitch PID Kp", "");
        rclc_add_parameter_description(&_param_server, "Ki_pitch", "Pitch PID Ki", "");
        rclc_add_parameter_description(&_param_server, "Kd_pitch", "Pitch PID Kd", "");
        rclc_add_parameter_description(&_param_server, "Max_pitch", "Pitch PID max output", "");
        rclc_add_parameter_description(&_param_server, "Min_pitch", "Pitch PID min output", "");
        rclc_add_parameter_description(&_param_server, "Period_pitch", "Pitch PID period", "ms");
        rclc_add_parameter_description(&_param_server, "Direction_pitch", "Pitch PID Direction", "");
        rclc_add_parameter_description(&_param_server, "Auto_pitch", "Pitch PID auto", "");

        //Vel parameters descriptions
        rclc_add_parameter_description(&_param_server, "Kp_vel", "Vel PID Kp", "");
        rclc_add_parameter_description(&_param_server, "Ki_vel", "Vel PID Ki", "");
        rclc_add_parameter_description(&_param_server, "Kd_vel", "Vel PID Kd", "");
        rclc_add_parameter_description(&_param_server, "Max_vel", "Vel PID max output", "");
        rclc_add_parameter_description(&_param_server, "Min_vel", "Vel PID min output", "");
        rclc_add_parameter_description(&_param_server, "Period_vel", "Vel PID period", "ms");
        rclc_add_parameter_description(&_param_server, "Direction_vel", "Vel PID Direction", "");
        rclc_add_parameter_description(&_param_server, "Auto_vel", "Vel PID auto", "");

        //Yaw rate parameters descriptions
        rclc_add_parameter_description(&_param_server, "Kp_yaw", "Yaw rate PID Kp", "");
        rclc_add_parameter_description(&_param_server, "Ki_yaw", "Yaw rate PID Ki", "");
        rclc_add_parameter_description(&_param_server, "Kd_yaw", "Yaw rate PID Kd", "");
        rclc_add_parameter_description(&_param_server, "Max_yaw", "Yaw rate PID max output", "");
        rclc_add_parameter_description(&_param_server, "Min_yaw", "Yaw rate PID min output", "");
        rclc_add_parameter_description(&_param_server, "Period_yaw", "Yaw rate PID period", "ms");
        rclc_add_parameter_description(&_param_server, "Direction_yaw", "Yaw rate PID Direction", "");
        rclc_add_parameter_description(&_param_server, "Auto_yaw", "Yaw rate PID auto", "");


        // Add parameters constrains 
        //Pitch parameters constrains
        rclc_add_parameter_constraint_double(&_param_server, "Kp_pitch", pid_gain_from, pid_gain_to, pid_gain_step);
        rclc_add_parameter_constraint_double(&_param_server, "Ki_pitch", pid_gain_from, pid_gain_to, pid_gain_step);
        rclc_add_parameter_constraint_double(&_param_server, "Kd_pitch", pid_gain_from, pid_gain_to, pid_gain_step);
        rclc_add_parameter_constraint_double(&_param_server, "Max_pitch", 0, 255, 1);
        rclc_add_parameter_constraint_double(&_param_server, "Min_pitch", -255, 0, 1);
        rclc_add_parameter_constraint_double(&_param_server, "Period_pitch", period_from, period_to, period_step);
         
        //Vel parameters constrains
         rclc_add_parameter_constraint_double(&_param_server, "Kp_vel", pid_gain_from, pid_gain_to, pid_gain_step);
         rclc_add_parameter_constraint_double(&_param_server, "Ki_vel", pid_gain_from, pid_gain_to, pid_gain_step);
         rclc_add_parameter_constraint_double(&_param_server, "Kd_vel", pid_gain_from, pid_gain_to, pid_gain_step);
         rclc_add_parameter_constraint_double(&_param_server, "Max_vel", -4.0, 4.0, 0.01);
         rclc_add_parameter_constraint_double(&_param_server, "Min_vel", -4.0, 4.0, 0.01);
         rclc_add_parameter_constraint_double(&_param_server, "Period_vel", period_from, period_to, period_step);

          //Yaw rate parameters constrains
        rclc_add_parameter_constraint_double(&_param_server, "Kp_yaw", pid_gain_from, pid_gain_to, pid_gain_step);
        rclc_add_parameter_constraint_double(&_param_server, "Ki_yaw", pid_gain_from, pid_gain_to, pid_gain_step);
        rclc_add_parameter_constraint_double(&_param_server, "Kd_yaw", pid_gain_from, pid_gain_to, pid_gain_step);
        rclc_add_parameter_constraint_double(&_param_server, "Max_yaw", -255, 255, 1);
        rclc_add_parameter_constraint_double(&_param_server, "Min_yaw", -255, 255, 1);
        rclc_add_parameter_constraint_double(&_param_server, "Period_yaw", period_from, period_to, period_step);


        // Set parameter initial values
        // Pitch PID initial values
        rclc_parameter_set_double(&_param_server, "Kp_pitch", _pitchParams.Kp);
        rclc_parameter_set_double(&_param_server, "Ki_pitch", _pitchParams.Ki);
        rclc_parameter_set_double(&_param_server, "Kd_pitch", _pitchParams.Kd);
        rclc_parameter_set_double(&_param_server, "Max_pitch", _pitchParams.max);
        rclc_parameter_set_double(&_param_server, "Min_pitch", _pitchParams.min);
        rclc_parameter_set_int(&_param_server, "Period_pitch", _pitchParams.period);
        rclc_parameter_set_bool(&_param_server, "Direction_pitch", _pitchParams.direct);
        rclc_parameter_set_bool(&_param_server, "Auto_pitch", _pitchParams.modeAuto);

        // Vel PID initial values
        rclc_parameter_set_double(&_param_server, "Kp_vel", _velParams.Kp);
        rclc_parameter_set_double(&_param_server, "Ki_vel", _velParams.Ki);
        rclc_parameter_set_double(&_param_server, "Kd_vel", _velParams.Kd);
        rclc_parameter_set_double(&_param_server, "Max_vel", _velParams.max);
        rclc_parameter_set_double(&_param_server, "Min_vel", _velParams.min);
        rclc_parameter_set_int(&_param_server, "Period_vel", _velParams.period);
        rclc_parameter_set_bool(&_param_server, "Direction_vel", _velParams.direct);
        rclc_parameter_set_bool(&_param_server, "Auto_vel", _velParams.modeAuto);

        // Yaw Rate PID initial values
        rclc_parameter_set_double(&_param_server, "Kp_yaw", _yawParams.Kp);
        rclc_parameter_set_double(&_param_server, "Ki_yaw", _yawParams.Ki);
        rclc_parameter_set_double(&_param_server, "Kd_yaw", _yawParams.Kd);
        rclc_parameter_set_double(&_param_server, "Max_yaw", _yawParams.max);
        rclc_parameter_set_double(&_param_server, "Min_yaw", _yawParams.min);
        rclc_parameter_set_int(&_param_server, "Period_yaw", _yawParams.period);
        rclc_parameter_set_bool(&_param_server, "Direction_yaw", _yawParams.direct);
        rclc_parameter_set_bool(&_param_server, "Auto_yaw", _yawParams.modeAuto);

        }

    






    void controllerNode::spinNode(){
        RCSOFTCHECK(rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(100)));

    };


    bool controllerNode::on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
        {
            // (void) context;

            // if (old_param == NULL && new_param == NULL) {
            //     printf("Callback error, both parameters are NULL\n");
            //     return false;
            // }

            bool ret = true;
            // if (new_param == NULL) {
            //     printf("Delete parameter %s rejected\n", old_param->name.data);
            //     ret = false;
            // } else if (strcmp(
            //     new_param->name.data,
            //     "publish_toogle") == 0 && new_param->value.type == RCLC_PARAMETER_BOOL)
            // {
            //     publish = new_param->value.bool_value;
            //     printf("Publish %s\n", (publish) ? "ON" : "OFF");
            // } else if (strcmp(
            //     new_param->name.data,
            //     "publish_rate_ms") == 0 && new_param->value.type == RCLC_PARAMETER_INT)
            // {
            //     int64_t old;
            //     RCSOFTCHECK(rcl_timer_exchange_period(&timer, RCL_MS_TO_NS(new_param->value.integer_value), &old));
            //     printf("Publish rate %ld ms\n", new_param->value.integer_value);
            // }

            return ret;
        }

 