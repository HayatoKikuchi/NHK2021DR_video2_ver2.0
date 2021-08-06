#include <Arduino.h>
#include<MsTimer2.h>
#include "RoboClaw.h"
#include "table_visit_class.h"

table_visit_class table(0.01);
RoboClaw robo(&Serial0, 10000);//RoboClawからエンコーダの値を読み取るときにはタイムカウントを10000以上にするといい

double target = PI / 4.0;//テーブル回転の目標値
double check_current;//現在の値
int check_num = 0;
bool print_flag = false;
bool stop_flag = false;
bool sd_flag = false;

int enc1 = 0;
int enc2 = 0;

void robo_pos(int encount1,int encount2, int vel1, int vel2){//速度付き位置制御
  robo.SpeedAccelDeccelPositionM1(128, 0, vel1, 0, encount1, true);
  robo.SpeedAccelDeccelPositionM2(128, 0, vel2, 0, encount2, true);
}

void func(){
   static int count = 0;
   count++;
   table.calc_angle_status(target - 14.0 / 360.0 * 2.0 * PI, check_current);   //回転軌道の計算
   
   if(count % 1 == 0) sd_flag = true; 
   if(count % 10 == 0){
     print_flag = true;
   }
   if(count % 400 == 0){//目標値を反対側に持って行っている
     //target = -table.check_table_angle();
     target = -target;
     check_current = table.check_table_angle(enc1, enc2);//エンコーダ―の値をRoboClawから読み取って現在のテーブル角度を返す
     
     count = 0;
   }
   if(check_num % 500 == 0) stop_flag = true;
   check_num++;
}
    
void setup() {
  table.init_or_stop(0.5, 1.0, 0.5, 0.0, 1);//初期化：加速時間　速度時間　減速時間　初期位置　true or false
  Serial.begin(115200);
  robo.begin(115200);
  
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  
  /*while(digitalRead(A4)){
    robo.BackwardM2(130, 15);
    delay(15);
  }
  robo.BackwardM2(130, 0);
  robo.ResetEncoders(130);
  
  while(digitalRead(A3)){
    robo.ForwardM1(130, 15);
    delay(10);
  }
  
  robo.ForwardM1(130, 0);
  
  robo.SetEncM1(130, 5300);
  */
  
  MsTimer2::set(10, func);
  MsTimer2::start();

}

void loop() {
  static bool a = true;
  
  bool valid1, valid2;
  uint8_t data1, data2;
  int vel1, vel2, encount1, encount2;
  
  if(print_flag){
    
    Serial.print(encount1);
    Serial.print("\t");
    Serial.print(encount2);
    Serial.print("\t");
    Serial.print(vel1);
    Serial.print("\t");
    Serial.print(vel2);
    Serial.print("\t");
    Serial.print(enc1);
    Serial.print("\t");
    Serial.print(enc2);
    Serial.print("\t");
    //Serial.print((angle), 5);
    //Serial.print("\t");
    Serial.println(table.check_d_print, 5);
    print_flag = false;
  }

  if(sd_flag){
    encount1 = -table.get_slide_cmd();//直動位置
    encount2 = table.get_rotate_cmd();//回転位置
    vel1 = table.get_vel_slide();//直動速度
    vel2 = table.get_vel_rotate();//回転速度
    
    enc1 = -robo.ReadEncM1(128, &data1, &valid1);//直動のカウント値
    enc2 = robo.ReadEncM2(128, &data2, &valid2);//回転のカウント値
    double angle = table.check_table_angle(enc1, enc2);//テーブルの角度
    robo_pos(encount1, encount2, vel1, vel2);//速度付き位置制御指令
    sd_flag = false;
  }
  
  //////////////特に使う必要がない//////////////////
  if(stop_flag){
    //table.init_or_stop(1, 2.0, 1, 0.0, a);//5番目の引数に0を入れるとクラス内の関数が使えなくなる　再び1を入れると使えるようになる
    a = !a;
    stop_flag = false;
  }
  
}
