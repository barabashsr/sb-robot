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

    std_msgs__msg__Int32 controllerNode::_msgOut;
    rcl_publisher_t controllerNode::_publisher;

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

    void controllerNode::subscription_callback_twist(const void * msgin){
        const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
	    Serial.printf("Received: %d\n", msg->data);

    };

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
        //create init_options
        //RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


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
        RCCHECK(rclc_subscription_init_default(
            &_subscriber_twist,
            &_node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "micro_ros_platformio_node_publisher"));

        // create timer,
        Serial.println("Subscr init");
        const unsigned int timer_timeout = 1000;
        RCCHECK(rclc_timer_init_default(
            &_timer,
            &_support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

        Serial.println("Timer init");

        // create executor

        RCCHECK(rclc_executor_init(&_executor, &_support.context, _handle_count, &_allocator));
        delay(1000);
        RCCHECK(rclc_executor_add_timer(&_executor, &_timer));
        RCCHECK(rclc_executor_add_subscription(&_executor, &_subscriber_twist, &_twst_msg, &subscription_callback_twist, ON_NEW_DATA));
        

        //_msgOut.data = 0;
        
    }

    void controllerNode::spinNode(){
        RCSOFTCHECK(rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(100)));

    };

 