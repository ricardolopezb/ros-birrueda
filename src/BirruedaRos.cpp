// 

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

#define LED_PIN 2  // Pin del LED en el ESP
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_heartbeat;
rclc_executor_t executor_pub;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    Serial.println("Error in micro_ros_platformio_node. Halting...");
    delay(100);
  }
}

// Callback function to process received messages
void subscription_callback(const void * msgin) {
  // RCSOFTCHECK(rcl_publish(&publisher, &msg_heartbeat, NULL));
  // msg_heartbeat.data++;
  // Serial.println("CALLED CALLBACK ############");
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  if (msg->data % 2 == 0) {
    digitalWrite(LED_PIN, HIGH);  // Enciende LED
    delay(500);
    digitalWrite(LED_PIN, LOW);  // Enciende LED
    delay(500);
    digitalWrite(LED_PIN, HIGH);  // Enciende LED
    delay(500);
    digitalWrite(LED_PIN, LOW);  // Enciende LED
  } else {
    digitalWrite(LED_PIN, HIGH);  // Enciende LED
    delay(500);
    digitalWrite(LED_PIN, LOW);  // Apaga LED
  }
  // Serial.print("Received message: ");
  // Serial.println(msg->data);
}
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  Serial.println("SENDING VALUES");
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher, &msg_heartbeat, NULL));
    msg_heartbeat.data++;
  }
}

void setup() {
  // Configure serial transport
  Serial.println("STARTING SETUP");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);
  Serial.println("After serial setup");
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));

  // // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  // pub!

  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  msg_heartbeat.data = 0;
  Serial.println("micro-ROS subscriber setup complete.");
}

void loop() {
  //RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}