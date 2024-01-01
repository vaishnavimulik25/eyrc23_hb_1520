#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <ESP32Servo.h>
#include <ESP32Servo360.h>

//servo objects created
ESP32Servo360 rear;
ESP32Servo360 left;
ESP32Servo360 right;
Servo pen; 

//pin values to servos
int left_wheel_pin = 33;
int right_wheel_pin = 26;
int rear_wheel_pin = 27;

rcl_subscription_t int_subscriber;
rcl_subscription_t vel_subscriber;
geometry_msgs__msg__Twist vel_msg;
std_msgs__msg__Int32 msg;
rclc_executor_t executor_int;
rclc_executor_t executor_vel;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define PEN_PIN 25 //33,25,26,27 either of these pins to be given voltage
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//changes the pen states on error
void error_loop(){
  while(1){
    digitalWrite(PEN_PIN, !digitalRead(PEN_PIN));
    Serial.print("error\n");
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
    pen.write(0);
  }
  else if (msg->data == 1)
  {
    pen.write(1);
  }
}

int findrpm(double velocity) {
  int rpm;
  if (velocity > 500)
    rpm = 140;

  if (velocity < 500 && velocity > -500) 
    rpm = map(velocity,-500,500,-140,140);

  if (velocity < -500)
      rpm = -140;

  return rpm;
}

//callback for /cmd_vel/bot1 subscription
void subscription_callback_vel(const void * msgin)
{
  Serial.print("Entering into vel subscription");
  const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Extract linear and angular velocity from the Twist message
  double linear_velx = vel_msg->linear.x;
  double linear_vely = vel_msg->linear.y;
  double angular_vel = vel_msg->angular.z;

    
  // Set servo positions based on calculated angle
  left.setSpeed(findrpm(linear_velx));
  right.setSpeed(findrpm(linear_vely));
  rear.setSpeed(findrpm(angular_vel));
}

//initialise the servos
void servo_init(void){

  left.attach(left_wheel_pin,10);
  right.attach(right_wheel_pin,11);
  rear.attach(rear_wheel_pin,12);
  pen.attach(PEN_PIN);

  /*left.attach(left_wheel_pin);
  right.attach(right_wheel_pin);
  rear.attach(rear_wheel_pin);
  pen.attach(PEN_PIN);*/
}

void ros2_init(void){

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
  RCCHECK(rclc_executor_init(&executor_int, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_init(&executor_vel, &support.context, 1, &allocator));
  
  // executor attached to specfic topic
  RCCHECK(rclc_executor_add_subscription(&executor_int, &int_subscriber, &msg, &subscription_callback_int, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor_vel, &vel_subscriber, &vel_msg, &subscription_callback_vel, ON_NEW_DATA));
}

//setup the configuration accordingly
void setup() {
  //wifi setup
  set_microros_wifi_transports("OPPO A31", "12345678","192.168.43.229", 8888);
  
  Serial.begin(9600);
  delay(2000);

  servo_init();
  ros2_init();
}

//code to be executed
void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor_int, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_vel, RCL_MS_TO_NS(100)));
}
