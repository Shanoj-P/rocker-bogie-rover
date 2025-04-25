#include <ros.h>
#include <geometry_msgs/Twist.h>

#define left_RPWM 4
#define left_LPWM 3
#define left_R_EN 5
#define left_L_EN 6

#define right_RPWM1 7
#define right_LPWM1 8
#define right_R_EN1 9
#define right_L_EN1 10

ros::NodeHandle nh;

float linear_x = 0;
float angular_z = 0;
const int max_pwm = 255; // Maximum PWM value

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

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", cmdVelCallback);

void setup() {
  nh.getHardware()->setBaud(115200); // Set baud rate
  nh.initNode();
  nh.subscribe(sub);
  
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
  nh.spinOnce();
  delay(10);
}
