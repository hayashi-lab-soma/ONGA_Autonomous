#include <ros.h>
// #include <geometry_msgs/Twist.h>
#include <std_msgs/Int64.h>

// #define ENA 5
// #define ENB 11
#define ENA 9
#define ENB 6
#define IN1 12
#define IN2 11
#define IN3 7
#define IN4 8

ros::NodeHandle  nh;

void leftCB(const std_msgs::Int64& left_pwm)
{
  int pwm = left_pwm.data;
  if (pwm >= 0)
  {
    analogWrite(ENA, abs(pwm));
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    analogWrite(ENA, abs(pwm));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
}

void rightCB( const std_msgs::Int64& right_pwm)
{
  int pwm = right_pwm.data;
  if (pwm <= 0)
  {
    analogWrite(ENB, abs(pwm));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    analogWrite(ENB, abs(pwm));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

ros::Subscriber<std_msgs::Int64> subLeft("/left_motor/pwm", leftCB);
ros::Subscriber<std_msgs::Int64> subRight("/right_motor/pwm", rightCB);
  
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
  nh.subscribe(subLeft);
  nh.subscribe(subRight);
    
}
  
void loop()
{
  nh.spinOnce();
}