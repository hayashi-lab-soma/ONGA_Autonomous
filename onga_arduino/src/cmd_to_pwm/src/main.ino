
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[13] = "hello world!";

void led_cb(const std_msgs::Bool& msg){
  if(msg.data){
    analogWrite(11, 255*0.0);
    delay(1000);
    analogWrite(11, 255*0.2);
    delay(1000);
    analogWrite(11, 255*0.4);
    delay(1000);
    analogWrite(11, 255*0.6);
    delay(1000);
    analogWrite(11, 255*0.8);
    delay(1000);
    analogWrite(11, 255*1.0);
    delay(1000);
  } else {
    analogWrite(11, 255*0.0);
  }

}
ros::Subscriber<std_msgs::Bool> sub0("led", &led_cb);

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub0);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(11, OUTPUT);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}