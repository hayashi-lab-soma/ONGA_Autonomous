#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>

#define L_PULSE_START 141 //駆動に必要な最低PCMパルス数
#define R_PULSE_START 141 //駆動に必要な最低PCMパルス数
#define PULSE_1MPS 141 //1.0m/sに必要なPCMパルス数

ros::NodeHandle  nh;
int cmdvel_cnt=0;
int l_motor=0, r_motor=0;
std_msgs::Int32 count_msg;

void CmdVelCallback(const geometry_msgs::Twist &msg){
  printf("subscribed");
  cmdvel_cnt = 0;
  // 進行方向設定
  if(msg.linear.x > 0.0){
    l_motor = (int)(L_PULSE_START + PULSE_1MPS * msg.linear.x);
    r_motor = (int)(R_PULSE_START + PULSE_1MPS * msg.linear.x);
  }else if(msg.linear.x < 0.0){
    l_motor = (int)(-1*L_PULSE_START + PULSE_1MPS * msg.linear.x);
    r_motor = (int)(-1*R_PULSE_START + PULSE_1MPS * msg.linear.x);
  }else{
    l_motor = 0;
    r_motor = 0;
  }

  // 角度方向設定
  if(msg.angular.z > 0.0){
    if(msg.linear.x == 0.0){
      l_motor = (int)(-1*L_PULSE_START - PULSE_1MPS * msg.angular.z * 0.75 /2);
      r_motor = (int)( R_PULSE_START + PULSE_1MPS * msg.angular.z * 0.75 /2); 
    }else if(msg.linear.x > 0.0){
      r_motor += PULSE_1MPS * msg.angular.z;
      l_motor -= PULSE_1MPS * msg.angular.z*0.5;
    }else{
      r_motor -= PULSE_1MPS * msg.angular.z;
      l_motor += PULSE_1MPS * msg.angular.z*0.5;
    }
  }else if(msg.angular.z < 0.0){
    if(msg.linear.x == 0.0){
      l_motor = (int)( L_PULSE_START + PULSE_1MPS * msg.angular.z * -0.75 /2);
      r_motor = (int)( -1 * R_PULSE_START - PULSE_1MPS * msg.angular.z * -0.75 /2); 
    }else if(msg.linear.x > 0.0){
      l_motor += PULSE_1MPS * msg.angular.z * -1;
      r_motor -= PULSE_1MPS * msg.angular.z*0.5 * -1;
    }else{
      l_motor -= PULSE_1MPS * msg.angular.z * -1;
      r_motor += PULSE_1MPS * msg.angular.z*0.5 * -1;
    }
  }

  //限度設定
  if(l_motor > 255) l_motor = 255;
  if(l_motor < -255) l_motor = -255;
  if(r_motor > 255) r_motor = 255;
  if(r_motor < -255) r_motor = -255;

  return;
}

bool MotorRun(int LS,int RS){ 
  if(LS >= 0 && LS <= 255){
    analogWrite(5, LS);
  }
  if(LS < 0 && LS >= -255){
    analogWrite(5, abs(LS)); 
  } 
  if(RS >= 0 && RS <= 255){
    analogWrite(6, RS);
  }
  if(RS < 0 && RS >= -255){
    analogWrite(6, RS); 
  }
  if  (RS > 255 || RS < -255 || LS > 255 || LS < -255){
    return false;
  } 
  return true;
}

bool MotorBrake(){ 
  analogWrite(5,0);
  analogWrite(6,0);
}

ros::Subscriber<geometry_msgs::Twist>cmdvel_sub("/onga_velocity_controller/cmd_vel", &CmdVelCallback);

void setup()
{
  nh.initNode();
  nh.subscribe(cmdvel_sub);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop()
{
  //cmd_vel設定の反映
  if(cmdvel_cnt <= 20){ //cmd_vel命令が 1000ms(50*20)以内の場合
    MotorRun(l_motor, r_motor); 
    cmdvel_cnt++; 
  }else{
    MotorBrake();
  }
  nh.spinOnce();
  delay(50);
}