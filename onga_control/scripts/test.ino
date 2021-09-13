
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <Wire.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <Adafruit_PWMServoDriver.h>

//-----ROS変数---------------------------------------------------------------------------------------
ros::NodeHandle nh;
std_msgs::String str_msg;
nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom",&odom);
geometry_msgs::TransformStamped tf_arduino;
tf::TransformBroadcaster broadcaster;
const float pi=3.14159;

//cmd_vel用
float linear_x=0.000;//speed x [m]
float linear_y=0.0;//speed y [m]
float linear_z=0.0;//speed y [m]
float angle_z=0.00;//speed z [rad/s]

//odom用
float x;
float y;
float theta=0;
float dx;
float dy;
float dtheta;
unsigned long current_time,last_time;

//座標変換用
float w[4]         = {0.0001,0.0001,0.0001,0.0001} ;//回転速度w [rad/s]
float motor_vel[4] = {0.001 ,0.001 ,0.001 ,0.001 } ;// [mm/s]
float a[4][3];//スピード→モータ回転　変換行列
float b[3][4];//モータ回転→スピード　変換行列
const float alpha=45;//メカナムホイールのローラ角度[deg]

//Arduino用CNC拡張ボード用ピン番号
const int motor_step_pin[4]={2,3,4,12};//ステッピングモータ　ピン番号
const int direct_pin[4]={5,6,7,13};    //CW,CCWピン番号

//pidパラメータ
float target[4]={0.1,0.1,0.1,0.1};
float duty[4] = {0.1,0.1,0.1,0.1};
float preTime[4]={100,100,100,100};
float P[4],I[4],D[4];
float preP[4]={0,0,0,0};
float dt[4]={0.001,0.001,0.001,0.001};
float Kp=0.001,Ki=0.00004 ,Kd=0.00098;
float motor_target_speed[4]={0,0,0,0};

//cmd_vel から メカナムホイール回転数に変換するための変数
const float wheel_radius=0.050;//ホイール半径0.050[m]　単位はmとする
const float ww=0.13;//width/2 [m]
const float lw=0.115;//length/2 [m]
const float motor_step=3200;//ステッピングモータの1回転当たりのステップ数(マイクロステップ制御込みのステップ数)
//(ステッピングモータ1回転200パルス)÷(マイクロステップ1/16)=3200
float step_time[4]={5000,5000,5000,5000};//各モータのステップパルスのON/OFF間隔[μs]
float step_current_time[4];
float step_last_time[4];
boolean step_flip[4]={LOW,LOW,LOW,LOW};//各モータのステップパルスのON/OFF状態
float step_count[4]={0,0,0,0};//各モータのステップパルス量をカウントする
float last_step_count[4]={0,0,0,0};
float delta_step_count[4]={0,0,0,0};
float wx[4];
float wy[4];
float wz[4];

//ロボットアーム用(サーボモジュール PCA9685使用)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define ARM_SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define ARM_SERVOMAX  500 // this is the 'maximum' pulse length count (out of 4096)
#define GRIP_SERVOMIN  110 // this is the 'minimum' pulse length count (out of 4096)
#define GRIP_SERVOMAX  300 // this is the 'maximum' pulse length count (out of 4096)
uint8_t servonum = 0;
float arm_angle=0;
//----------------------------ROS  callback----------------------------------------------------
void messageCb(const geometry_msgs::Twist& twist) {
  linear_x = twist.linear.x;
  linear_y = twist.linear.y;
  linear_z = twist.linear.z;
  angle_z = twist.angular.z;  
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb);


void setup(){
  //出力ピン設定
  pinMode(motor_step_pin[0], OUTPUT);
  pinMode(motor_step_pin[1], OUTPUT);
  pinMode(motor_step_pin[2], OUTPUT);
  pinMode(motor_step_pin[3], OUTPUT);

  pinMode(direct_pin[0], OUTPUT);
  pinMode(direct_pin[1], OUTPUT);
  pinMode(direct_pin[2], OUTPUT);
  pinMode(direct_pin[3], OUTPUT);
  //ROSの初期設定
  nh.getHardware()->setBaud(1000000);//baudrateを1Mbpsに設定
  nh.initNode();
  broadcaster.init(nh);
  nh.advertise(odom_pub);
  initOdom();
  nh.subscribe(sub); 

  //アーム用サーボモータの初期値を設定する
  pwm.begin();
  pwm.setPWMFreq(60); 
  pwm.setPWM(0, 0, map(172, 0, 180, ARM_SERVOMIN  , ARM_SERVOMAX));
  pwm.setPWM(1, 0, map(  0, 0, 180, ARM_SERVOMIN  , ARM_SERVOMAX));
  pwm.setPWM(4, 0, map( 15, 0, 180, GRIP_SERVOMIN , GRIP_SERVOMAX));
  pwm.setPWM(5, 0, map(150, 0, 180, GRIP_SERVOMIN , GRIP_SERVOMAX));
  delay(100); 
  nh.loginfo("ROS wake up!!");

  //cmd_velの速度情報をモータの回転数に変換するための行列式 
  a[0][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[0][1]= 1/wheel_radius ; a[0][2]=(-1*(lw+ww)/(wheel_radius));
  a[1][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[1][1]=-1/wheel_radius ; a[1][2]=    (lw+ww)/(wheel_radius) ;
  a[2][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[2][1]=-1/wheel_radius ; a[2][2]=(-1*(lw+ww)/(wheel_radius));
  a[3][0]=  1/(wheel_radius*tan(alpha/180*pi)) ; a[3][1]= 1/wheel_radius ; a[3][2]=    (lw+ww)/(wheel_radius) ;

  //モータの回転数からtfの座標を作り出すための行列式 
  b[0][0]=   wheel_radius             ; b[0][1]=   wheel_radius          ; b[0][2]=   wheel_radius          ; b[0][3]=  wheel_radius        ;
  b[1][0]=   wheel_radius             ; b[1][1]= - wheel_radius          ; b[1][2]= - wheel_radius          ; b[1][3]=  wheel_radius        ;
  b[2][0]= - wheel_radius/(lw+ww)     ; b[2][1]=  wheel_radius/(lw+ww)  ; b[2][2]=  -wheel_radius/(lw+ww)  ;  b[2][3]=  wheel_radius/(lw+ww);

  last_time = micros();
  current_time = micros();
  step_current_time[0]=step_current_time[1]=step_current_time[2]=step_current_time[3]= micros();
  step_last_time[0]=step_last_time[1]=step_last_time[2]=step_last_time[3]= micros();
}

//speed_cal()でcmd_velからの要求スピードを各モータの移動スピードmotor_vel[]に変換する
void speed_cal(){
  motor_vel[0]=(a[0][0]*linear_x + a[0][1]*linear_y + a[0][2]*angle_z) *wheel_radius*pi ; //[m/s]
  motor_vel[1]=(a[1][0]*linear_x + a[1][1]*linear_y + a[1][2]*angle_z) *wheel_radius*pi;  //[m/s]
  motor_vel[2]=(a[2][0]*linear_x + a[2][1]*linear_y + a[2][2]*angle_z) *wheel_radius*pi ; //[m/s]
  motor_vel[3]=(a[3][0]*linear_x + a[3][1]*linear_y + a[3][2]*angle_z) *wheel_radius*pi ; //[m/s]
}

//motor_vel[]を目標スピードとしてモータ速度duty[]を制御する
//モータ速度duty[]をステッピングモータのON/OFFタイミングstep_time[][μs]に変換する
void motor_pid(int num,float motor_call_speed){
  duty[num] =motor_call_speed;

  if(motor_call_speed>0 && duty[num]<0){
    duty[num]=-0.0001;
  }
  else if(motor_call_speed<0 && duty[num]>0){
    duty[num]=0.0001;
  }
  if(duty[num]==0){
    duty[num]=0.0001;
  }

  //motor_move(num,duty[num]); //pidで計算されたスピード値[mm]をmotor_moveに渡す
  //　      (1秒/2)/(スピード[m/s]/(ホイール半径[m]*2*pi=周長[m])*1周のステップ数 )
  //↑の解釈:(1秒/2)/(1秒間でのホイール回転数[回転/s]*1周のステップ数[ステップ/回転]
  //↑の解釈:(1,000,000μ秒/2)/(1秒当たりのステップ数[ステップ/ｓ])
  step_time[num]=abs(500000/(duty[num]/(wheel_radius*2*pi)*motor_step));

//以下はスピード制限
  if(step_time[num]<40){
    step_time[num]=40;    
  }
  else if(step_time[num]>100000){     
    step_time[num]=100000;   
  } 

}

void loop(){  
  nh.spinOnce(); //cmd_velから要求されるロボットのスピードを取得する     
  speed_cal();  //要求されるロボットのスピードをホイールの回転数に変換する
  motor_pid(0,motor_vel[0]);//ホイールの回転数をステッピングモータのパルススピードに変換する
  motor_pid(1,motor_vel[1]);
  motor_pid(2,motor_vel[2]);
  motor_pid(3,motor_vel[3]);

  step_current_time[0]=step_current_time[1]=step_current_time[2]=step_current_time[3]= micros();

  //micros()時間が0に戻った時を想定し、クリアするs
  if(step_current_time[0]<step_last_time[0]){
    step_last_time[0]=step_last_time[1]=step_last_time[2]=step_last_time[3]= micros();
    delayMicroseconds(100);
    step_current_time[0]=step_current_time[1]=step_current_time[2]=step_current_time[3]= micros();
  }

//------------------モーターのステップパルスをコントロール----------------------------------------  
  if(step_time[0]<(step_current_time[0]-step_last_time[0])){
    step_flip[0]=!step_flip[0];
    if(motor_vel[0]>0){
      digitalWrite(direct_pin[0],1);
      step_count[0]++;
    }else{
      digitalWrite(direct_pin[0],0);
      step_count[0]--;
    }
    step_last_time[0]=step_current_time[0];
  }

  if(step_time[1]<(step_current_time[1]-step_last_time[1])){
    step_flip[1]=!step_flip[1];
    if(motor_vel[1]>0){
      digitalWrite(direct_pin[1],0);
      step_count[1]++;
    }else{
      digitalWrite(direct_pin[1],1);
      step_count[1]--;
    }
    step_last_time[1]=step_current_time[1];
  }

  if(step_time[2]<(step_current_time[2]-step_last_time[2])){
    step_flip[2]=!step_flip[2];
    if(motor_vel[2]>0){
      digitalWrite(direct_pin[2],0);
      step_count[2]++;
    }else{
      digitalWrite(direct_pin[2],1);
      step_count[2]--;
    }
    step_last_time[2]=step_current_time[2];
  }

  if(step_time[3]<(step_current_time[3]-step_last_time[3])){
    step_flip[3]=!step_flip[3];
    if(motor_vel[3]>0){
      digitalWrite(direct_pin[3],1);
      step_count[3]++;
    }else{
      digitalWrite(direct_pin[3],0);
      step_count[3]--;
    }
    step_last_time[3]=step_current_time[3];
  }

  digitalWrite(motor_step_pin[0],step_flip[0]);
  digitalWrite(motor_step_pin[1],step_flip[1]);
  digitalWrite(motor_step_pin[2],step_flip[2]);
  digitalWrite(motor_step_pin[3],step_flip[3]);
}