#define ROSLIB_SERIAL_BUFFER_SIZE 1024  // Adjust buffer size based on your board

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

// Define trigger pin
#define triggerPin 53

#define left_RPWM 4
#define left_LPWM 3
#define left_R_EN 5
#define left_L_EN 6

#define right_RPWM1 7
#define right_LPWM1 8
#define right_R_EN1 9
#define right_L_EN1 10

float linear_x = 0;
float angular_z = 0;
const int max_pwm = 255; // Maximum PWM value

// ROS setup
ros::NodeHandle nh;
geometry_msgs::Quaternion quat_msg;
ros::Publisher quat_pub("/imu_publisher", &quat_msg);

// IMU setup
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Servo setup
Servo panServo;
Servo tiltServo;
float pan, tilt, previous_pan = 95, previous_tilt = 120;

// Trigger callback
void trigger_fire_callback(const std_msgs::Int32& trigger) {
  if (trigger.data == 1) {
    digitalWrite(triggerPin, LOW);   // Fire ON (Active Low)
    delay(700);
    digitalWrite(triggerPin, HIGH);  // Fire OFF
  }
}

// Pan and tilt callback
void pan_and_tilt_callback(const std_msgs::Float64MultiArray& pan_and_tilt) {
  pan = previous_pan + pan_and_tilt.data[0];
  tilt = previous_tilt + pan_and_tilt.data[1];

  // Constrain movement to valid servo range
  tilt = constrain(tilt, 70, 180);
  pan = constrain(pan, 0, 180);

  panServo.write(pan);
  tiltServo.write(tilt);

  previous_pan = pan;
  previous_tilt = tilt;
}

void cmdVelCallback(const geometry_msgs::Twist& cmd_vel) {
  linear_x = cmd_vel.linear.x;
  angular_z = cmd_vel.angular.z;
  
  int left_speed = (linear_x - angular_z) * max_pwm;
  int right_speed = (linear_x + angular_z) * max_pwm;
  
  left_speed = constrain(left_speed, -max_pwm, max_pwm);
  right_speed = constrain(right_speed, -max_pwm, max_pwm);

  if (left_speed >= 0) {
    analogWrite(left_RPWM, left_speed);
    analogWrite(left_LPWM, 0);
  } else {
    analogWrite(left_RPWM, 0);
    analogWrite(left_LPWM, -left_speed);
  }

  if (right_speed >= 0) {
    analogWrite(right_RPWM1, right_speed);
    analogWrite(right_LPWM1, 0);
  } else {
    analogWrite(right_RPWM1, 0);
    analogWrite(right_LPWM1, -right_speed);
  }
}

ros::Subscriber<geometry_msgs::Twist> cmdvelSub("/cmd_vel", cmdVelCallback);
ros::Subscriber<std_msgs::Float64MultiArray> sub("/pan_and_tilt", pan_and_tilt_callback);
ros::Subscriber<std_msgs::Int32> fire("/trigger_firing", trigger_fire_callback);

void setup() {
  // Trigger pin setup
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, HIGH);  // Default: trigger off

  // ROS init
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(quat_pub);
  nh.subscribe(sub);
  nh.subscribe(fire);
  nh.subscribe(cmdvelSub);

  // Servo init
  panServo.attach(11);
  tiltServo.attach(12);
  panServo.write(95);
  tiltServo.write(120);

  delay(5000);  // Give time for hardware to stabilize

  // BNO055 init
  if (!bno.begin()) {
    // Skip ROS log to save memory
    while (1);  // Halt if sensor not found
  }
  bno.setExtCrystalUse(true);

  pinMode(left_RPWM, OUTPUT);
  pinMode(left_LPWM, OUTPUT);
  pinMode(left_R_EN, OUTPUT);
  pinMode(left_L_EN, OUTPUT);
  
  pinMode(right_RPWM1, OUTPUT);
  pinMode(right_LPWM1, OUTPUT);
  pinMode(right_R_EN1, OUTPUT);
  pinMode(right_L_EN1, OUTPUT);
  
  digitalWrite(left_R_EN, HIGH);
  digitalWrite(left_L_EN, HIGH);
  digitalWrite(right_R_EN1, HIGH);
  digitalWrite(right_L_EN1, HIGH);
}

void loop() {
  // Read and publish IMU quaternion
  imu::Quaternion quat = bno.getQuat();
  quat_msg.w = quat.w();
  quat_msg.x = quat.x();
  quat_msg.y = quat.y();
  quat_msg.z = quat.z();
  quat_pub.publish(&quat_msg);

  nh.spinOnce();
  delay(10);  // Prevent overflow
}
