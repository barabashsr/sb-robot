#include <Arduino.h>
#include <WiFi.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// WiFi credentials
char* ssid = "Beeline_2G_F13F37";
char* password = "1122334455667788";

// micro-ROS agent IP and port
IPAddress agent_ip(192, 168, 1, 74);
uint16_t agent_port = 8888;

// micro-ROS entities
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Error checking macro
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){Serial.printf("Failed at line %d: %d\n", __LINE__, (int)temp_rc); return;}}

// Timer callback function
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    msg.data++;
    RCCHECK(rcl_publish(&publisher, &msg, NULL));
    Serial.print("Publishing: ");
    Serial.println(msg.data);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());
  
  // Set up micro-ROS
  Serial.println("Connecting to micro-ROS agent...");
  
  // Initialize micro-ROS with UDP transport
  Serial.println("Initializing transport...");
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
  
  delay(2000);
  
  // Initialize micro-ROS allocator
  Serial.println("Initializing allocator...");
  allocator = rcl_get_default_allocator();
  
  // Initialize support object
  Serial.println("Initializing support...");
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Create node
  Serial.println("Creating node...");
  RCCHECK(rclc_node_init_default(&node, "esp32_test_node", "", &support));
  
  // Create publisher
  Serial.println("Creating publisher...");
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "esp32_test_topic"));
  
  // Create timer
  Serial.println("Creating timer...");
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  // Create executor
  Serial.println("Creating executor...");
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  msg.data = 0;
  
  Serial.println("micro-ROS setup completed successfully!");
}

void loop() {
  // Spin executor to process callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(100);
}
