#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>


ros::NodeHandle  nh;
std_msgs::String str_msg;

void cmd_vel_cb(const geometry_msgs::Twist &cmd_vel){
  printf("subscribed");
  analogWrite(11, 255*0.0);
  delay(100);
  analogWrite(11, 255*0.5);
  delay(100);
  analogWrite(11, 255*1.0);
  delay(100);
  analogWrite(11, 255*0.5);
  delay(100);
  analogWrite(11, 255*0.0);
  delay(100);
}

ros::Subscriber<geometry_msgs::Twist>cmd_vel_sub("/onga_velocity_controller/cmd_vel", &cmd_vel_cb);

void setup()
{
  nh.initNode();
  // nh.subscribe(sub0);
  nh.subscribe(cmd_vel_sub);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop()
{
  nh.spinOnce();
  delay(500);
}