#include <Arduino.h>
#include<MsTimer2.h>
//#include "RoboClaw.h"
#include "table_visit_class.h"

table_visit_class table(0.01);
//RoboClaw robo(&Serial5, 10000);

double target = PI / 6.0;
double check_current;
int check_num = 0;
bool print_flag = false;

void robo_pos(int encount1,int encount2, int vel1, int vel2){
  //robo.SpeedAccelDeccelPositionM1(130, 0, vel1, 0, encount1, true);
  //robo.SpeedAccelDeccelPositionM2(130, 0, vel2, 0, encount2, true);
}

void func(){
   static int count = 0;
   count++;
   table.calc_angle_status(target, check_current);   
   
   if(count % 10 == 0){
     print_flag = true;
   }
   if(count % 400 == 0){
     //target = -table.check_table_angle();
     check_current = -target;
     
     count = 0;
   }
   check_num++;
}
    
void setup() {
  table.init_or_stop(1, 2.0, 1, 0.0, 1);
  Serial.begin(115200);
  //robo.begin(115200);
  MsTimer2::set(10, func);
  MsTimer2::start();

}

void loop() {
  if(print_flag){
    int encount1 = table.get_slide_cmd();
    int encount2 = table.get_rotate_cmd();
    int vel1 = table.get_vel_slide();
    int vel2 = table.get_vel_rotate();
    double angle = table.check_table_angle(check_num, 30);
    //robo_pos(encount1, encount2, vel1, vel2);
//    Serial.print(table.print_d[0]);
//    Serial.print("\t");
//    Serial.print(table.print_d[3]);
//    Serial.print("\t");
//    Serial.print(vel1);
//    Serial.print("\t");
//    Serial.print(vel2);
//    Serial.print("\t");
    Serial.print(check_num);
    Serial.print("\t");
    Serial.println((angle), 5);
    print_flag = false;
  }
  
}
