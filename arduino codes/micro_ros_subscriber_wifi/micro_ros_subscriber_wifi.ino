#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <ESP32Servo360.h>

ESP32Servo360 rear;
ESP32Servo360 left;
ESP32Servo360 right;
ESP32Servo360 pen; // create servo object to control a servo
// twelve servo objects can be created on most boards


rcl_subscription_t int_subscriber;
rcl_subscription_t vel_subscriber;
geometry_msgs__msg__Twist vel_1;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_subscription_callback_t subscription_calback_int;
rclc_subscription_callback_t subscription_calback_vel;

#define PEN_PIN 13


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(PEN_PIN, !digitalRead(PEN_PIN));
    delay(100);
  }
}

void subscription_callback_int(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // digitalWrite(PEN_PIN, (msg->data == 0) ? LOW : HIGH);
  if (msg->data == 0)
  {
    digitalWrite(PEN_PIN, LOW);
  }
  else if (msg->data == 1)
  {
    digitalWrite(PEN_PIN, HIGH);
  }
}
void subscription_callback_vel(const void * msgin)
{
  const geometry_msgs__msg__Twist * vel_1 = (const geometry_msgs__msg__Twist *)msgin;

  
}

void setup() {
  set_microros_wifi_transports("Jiten Topiwala's wifi", "123456789", "192.168.236.31", 8888);
  Serial.begin(9600);
  pinMode(PEN_PIN, OUTPUT);
  rear.attach(12,11); // attaches the servo on pin 12 to the servo object
  digitalWrite(PEN_PIN, HIGH);
  delay(2000);

  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "subscriber_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &int_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/Integer"));
  RCCHECK(rclc_subscription_init_default(
      &vel_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel/bot1"));
  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &int_subscriber, &msg, &subscription_callback_int, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &vel_subscriber, &vel_1, &subscription_callback_vel, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  Serial.print("Int32 data: ");
  Serial.println(msg.data);
  Serial.print("Twist data: ");
  Serial.println(vel_1.linear.x); // Assuming Twist has a linear.x field
}
