#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <ESP32Servo360.h>
#include <ESP32Servo.h>

//servo objects created
ESP32Servo360 rear;
ESP32Servo360 left;
ESP32Servo360 right;
Servo pen; 

//pin values to servos
int left_wheel_pin = 33;
int right_wheel_pin = 25;
int rear_wheel_pin = 27;

rcl_subscription_t int_subscriber;
rcl_subscription_t vel_subscriber;
geometry_msgs__msg__Twist vel_msg;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_subscription_callback_t subscription_calback_int;
rclc_subscription_callback_t subscription_calback_vel;

#define PEN_PIN 26 //33,25,26,27 either of these pins to be given voltage


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//changes the pen states on error
void error_loop(){
  while(1){
    digitalWrite(PEN_PIN, !digitalRead(PEN_PIN));
    delay(100);
  }
}

//callback for integer subscription
void subscription_callback_int(const void * msgin)
{
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // digitalWrite(PEN_PIN, (msg->data == 0) ? LOW : HIGH);
  if (msg->data == 0)
  {
//i    digitalWrite(PEN_PIN, LOW);
    pen.write(0);
  }
  else if (msg->data == 1)
  {
//    digitalWrite(PEN_PIN, HIGH);
    pen.write(1);
  }
}

//callback for /cmd_vel/bot1 subscription
void subscription_callback_vel(const void * msgin)
{
  const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Extract linear and angular velocity from the Twist message
  double linear_vel = vel_msg->linear.x;
  double angular_vel = vel_msg->angular.z;

  // Two wheels for driving and one for steering
  double left_wheel_speed, right_wheel_speed, steering_angle;

  // Convert linear and angular velocity to left and right wheel speeds
  const double wheelbase = 0.2;  //distance between 2 wheels
  //inverse kinematics to be added if this doesn't work
  left_wheel_speed = linear_vel - angular_vel * wheelbase / 2.0;
  right_wheel_speed = linear_vel + angular_vel * wheelbase / 2.0;

  // Steering angle for the single steering wheel
  steering_angle = atan2(angular_vel * wheelbase, linear_vel);

  // Map wheel speeds to servo angles
  int left_servo_angle = map(left_wheel_speed, -1.0, 1.0, 0, 360);
  int right_servo_angle = map(right_wheel_speed, -1.0, 1.0, 0, 360);
  int steering_servo_angle = map(steering_angle, -M_PI / 2.0, M_PI / 2.0, 0, 360);

  // Set servo positions based on calculated angles
  left.setSpeed(left_servo_angle);
  right.setSpeed(right_servo_angle);
  rear.setSpeed(steering_servo_angle);
}

//setup the configuration accordingly
void setup() {
  //wifi setup
  set_microros_wifi_transports("OPPO A31", "12345678", "192.168.236.31", 8888);
  Serial.begin(9600);
  
  //initialise the servos
  left.attach(left_wheel_pin);
  right.attach(right_wheel_pin);
  rear.attach(rear_wheel_pin);

  //initialise the pen
  pinMode(PEN_PIN, OUTPUT);
  pen.attach(12,11); // attaches the servo on pin 12 to the servo object
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

//code to be executed
void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  Serial.print("Int32 data: ");
  Serial.println(msg.data);
  Serial.print("Twist data: ");
  Serial.println(vel_msg.linear.x); // Assuming Twist has a linear.x field
}
