#include <ros.h>
#include <geometry_msgs/Twist.h>

// #define ENA 5
// #define ENB 11
#define ENA 9
#define ENB 6
#define IN1 12
#define IN2 11
#define IN3 7
#define IN4 8

ros::NodeHandle  nh;

void cmdVelCB( const geometry_msgs::Twist& twist)
{
  int a = 220;
  int b = 480;

  float left_wheel_data = a*twist.linear.x + b*twist.angular.z;
  float right_wheel_data = -a*twist.linear.x + b*twist.angular.z;

  // if(left_wheel_data > 255){
  //   left_wheel_data = 255;
  // }
  if(right_wheel_data > 255){
    right_wheel_data = 255;
  }

  if (left_wheel_data >= 0)
  {
    analogWrite(ENA, abs(left_wheel_data));
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    analogWrite(ENA, abs(left_wheel_data));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if (right_wheel_data >= 0)
  {
    analogWrite(ENB, abs(right_wheel_data));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    analogWrite(ENB, abs(right_wheel_data));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("/onga_velocity_controller/cmd_vel", cmdVelCB);
  
void setup()
{
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);  
    
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
    
  nh.initNode();
  nh.subscribe(subCmdVel);
    
 }
  
 void loop()
 {
  nh.spinOnce();
 }