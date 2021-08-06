#ifndef TABLE_VISIT_CLASS_H
#define TABLE_VISIT_CLASS_H

#include <Arduino.h>
// #include "parameters.h"

class table_visit_class{
    public:
    table_visit_class(double);
    bool init_or_stop(double, double, double, double, bool);//初期化
    void calc_angle_status(double, double);//計算
    int get_slide_cmd();//直動機構の位置
    int get_rotate_cmd();//回転機構の位置
    int get_vel_slide();//直動機構の速度
    int get_vel_rotate();//回転機構の速度
    double check_table_angle(int slide_count, int rotate_count);//角度の計算
    int print_i[2];//int型変数確認用
    double print_d[10];//double型変数確認用
    double current_table_angle;//現在のテーブルの角度

    private:
   
    double sin_angle;
    
    double aim_table_angle = PI / 4.0;//テーブル目標角度
    double pre_aim_table_angle = PI / 4.0;//1つ前のテーブル目標角度
    double current_angle_vel;//現在のテーブル角速度
    double pre_table_angle;//1つ前のテーブル角度
    double pre_arm_length;//ひとつ前のarm_length

    double x_el, y_el;//X成分，Y成分

    double time_count;//時間経過
    double angle_vel_cmd[2];//速度指令値
    double angle_cmd[2];//位置指令値
    double point_cmd[2];

    double basic_accel_time;
    double basic_const_time;
    double basic_deccel_time;
    double all_time;
    // accel_time　加速時間
    // const_time   一定速度時間
    // deccel_time　減速時間

    double T;

    double check_d_print;//double型変数観察用
    int check_i_print;//int型変数観察用
    int rate_t;//目標までの周期回数
    //int enc[2] = {};//エンコーダ―の値


    bool init_flag;

};

#endif
