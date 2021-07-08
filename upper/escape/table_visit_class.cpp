#include <Arduino.h>
#include "table_visit_class.h"
#include "parameters.h"
#include <math.h>

table_visit_class::table_visit_class(double freq){
    T = freq;
    init_flag = false;
}

//基礎加速時間，基礎定速時間，基礎減速時間，初期目標値，初期化:1 or 動作停止:0 の順に引数を入れていく
bool table_visit_class::init_or_stop(double input_accel_time = 0.5, double input_const_time = 0.5, double input_deccel_time = 0.5, double first_angle = 0.0, bool command_status = 0){
    if(command_status){
      basic_accel_time = input_accel_time;//基礎加速時間
      basic_const_time = input_const_time;//基礎定速時間
      basic_deccel_time = input_deccel_time;//基礎減速時間
      all_time = basic_accel_time + basic_const_time + basic_deccel_time;//基礎時間の合計値
      current_table_angle = pre_table_angle = first_angle;//角度合わせ
      x_el = R_MM * (1 - cos(first_angle)) + U_MM;
      y_el = R_MM * sin(first_angle);
      pre_arm_length = sqrt(pow(x_el, 2.0) + pow(y_el, 2.0));
      pre_aim_table_angle = first_angle;
      init_flag = true;//関数の実行可
    } else {
        init_flag = false;
    }
    time_count = 0.0;
    return init_flag;//実行可：1　実行不可：0
}

void table_visit_class::calc_angle_status(double aim_table_angle, double calc_angle){
    double rate;//基準との比
    double aim_time;//目標時間
    double aim_dist;//目標までの角度
    double accel_time;//加速時間
    double deccel_time;//減速時間
    double max_angle_vel;//最大速度
    double angle_accel;//角加速度
    double angle_deccel;//角減速度
    double slide_vel, rotate_vel;//直動と回転の速度成分
    double table_angle;//テーブルの角度
    double tangent_vel;//テーブルの円周移動速度
    double theta_b;//回転機構が回転する角度
    double theta_c;//計算に使用する要素の角度
    double arm_length;//回転中心から取っ手までの長さ

    if(init_flag){

    if(aim_table_angle != pre_aim_table_angle) time_count = 0.0;

    time_count += T;
    current_table_angle = calc_angle;
    aim_dist = aim_table_angle - current_table_angle;//目標までの距離
    if(aim_dist < 0) print_d[2] = aim_dist = -aim_dist;

    // if (aim_dist == 0.0) neat_flag = true;

    rate = (aim_dist) / (BASIC_DIST);
    aim_time = all_time * rate;//到達目標時間
    accel_time = basic_accel_time * rate;//加速時間
    deccel_time = basic_deccel_time * rate;//減速時間
    check_i_print = rate_t = (int)(aim_time / T);

    max_angle_vel = 2.0 * (aim_table_angle - current_table_angle) / (aim_time - accel_time + aim_time - deccel_time);//最大速度
    angle_accel = max_angle_vel / accel_time;//加速度
    angle_deccel = max_angle_vel / deccel_time;//減速度

    table_angle = current_table_angle;

        if((int)(time_count * rate_t) > 0 && (int)(time_count * rate_t) < (int)(accel_time * rate_t) + 1) {//加速時の計算
        table_angle = current_table_angle + 0.5 * angle_accel * pow(time_count, 2.0);
        } 
        else if((int)(time_count * rate_t) >= (int)(accel_time * rate_t) + 1 && (int)(time_count * rate_t) < (int)((aim_time - deccel_time) * rate_t) + 1) {//等速円運動時の計算
        table_angle = current_table_angle + 0.5 * angle_accel * pow(accel_time, 2.0) + max_angle_vel * (time_count - accel_time);
        } 
        else if((int)(time_count * rate_t) >= (int)((aim_time - deccel_time) * rate_t) + 1 && (int)(time_count * rate_t) <= (int)(aim_time * rate_t) ) {//減速時の計算
        table_angle = current_table_angle + 0.5 * angle_accel * pow(accel_time, 2.0) + max_angle_vel * (aim_time - accel_time - deccel_time) + max_angle_vel * (time_count - (aim_time - deccel_time)) - 0.5 * angle_deccel * pow((time_count - (aim_time - deccel_time)), 2.0);
        }
        else if((int)(time_count * rate_t) > (int)(aim_time * rate_t)){
        table_angle = current_table_angle + 0.5 * angle_accel * pow(accel_time, 2.0) + max_angle_vel * (aim_time - accel_time - deccel_time) + max_angle_vel * (aim_time - (aim_time - deccel_time)) - 0.5 * angle_deccel * pow((aim_time - (aim_time - deccel_time)), 2.0);
        }    

    x_el = R_MM * (1 - cos(table_angle)) + U_MM;//ⅹ方向の距離
    y_el = R_MM * sin(table_angle);//y方向の距離
    arm_length = pow(pow(x_el, 2.0) + pow(y_el, 2.0), 0.5);//ハンド先端からアームの回転軸までの直線の長さ
    theta_b = atan2(y_el, x_el);//ハンドの回転角度
    theta_c = PI / 2.0 - table_angle - theta_b;
    current_angle_vel = (table_angle - pre_table_angle) / T;//最新の角速度
    tangent_vel = R_MM * current_angle_vel;//取っ手の角速度
    slide_vel = tangent_vel * cos(theta_c);//直線運動
    print_d[3] = rotate_vel = tangent_vel * sin(theta_c);//回転運動
    pre_table_angle = table_angle;//1つ前の角度
    pre_arm_length = arm_length;//1つ前のlength
    pre_aim_table_angle = aim_table_angle;

    angle_vel_cmd[0] = abs(slide_vel / GEAR_R / 2.0 / PI * RESOLUTION);//速度コマンド値
    angle_vel_cmd[1] = abs(rotate_vel / arm_length / 2.0 / PI * ROTATE_RES  * GEAR_ROTATE);
    angle_cmd[0] = (arm_length - U_MM) / 36.0 / PI * RESOLUTION;//位置コマンド値
    angle_cmd[1] = theta_b / 2.0 / PI * ROTATE_RES  * GEAR_ROTATE;
    }
    print_d[0] = time_count;
    print_d[1] = aim_dist;
}

 double table_visit_class::check_table_angle(int slide_count, int rotate_count){

    double pay;
    //int enc_holder[2] = {(int)slide_count , (int)rotate_count};
    //enc_holder[0] = 5200;
    //double sim_b = 2.0 * PI * (double)enc[1] * 2.0 * 3.0 * PI / 48.0 / 4.0 / 10.0;
    double calc_arm_length = 36.0 * PI * (double)(slide_count) / (double)RESOLUTION + (double)U_MM;//直動した長さの計算
    double square_calc_arm_length = calc_arm_length * calc_arm_length;
    double cos_angle = (pow((double)R_MM, 2.0) + pow((double)(R_MM + U_MM), 2.0) - square_calc_arm_length) / 2.0 / (double)R_MM / (double)(R_MM + U_MM);
    double cos_angle_2 = pow(cos_angle, 2.0);
    double sin_angle_2 = 1.0 - cos_angle_2;
    double sin_angle = pow(sin_angle_2, 0.5);
    //float tan_angle = sin_angle / cos_angle;
    double tan_angle_2 = sin_angle_2 / cos_angle_2;

//    int tab = (int)multi;
//    double tab_2 = (float)tab;
//    double tan_angle = pow(multi, 0.5);
    //calc_arm_length = 36.0 * PI * 7000.0 / (double)RESOLUTION + (double)U_MM;//予想値
    if(init_flag){
    if(rotate_count == 0){
        pay = 0.0;
    } 
    else if (rotate_count > 0){
        pay = acos(cos_angle);
    } 
    else {
        pay = -acos(cos_angle);
    }
    check_d_print = pay;
    return pay;
    //return (cos_angle);
    //return acos((pow(R, 2.0) + pow(R + U, 2.0) - pow(calc_arm_length. 2.0)) / 2.0 / R / (R + U));
    }
    else {
      return init_flag;
    }
}

int table_visit_class::get_slide_cmd(){
    if(init_flag) return angle_cmd[0];
}

int table_visit_class::get_rotate_cmd(){
    if(init_flag) return angle_cmd[1];
}

int table_visit_class::get_vel_slide(){
    if(init_flag) return angle_vel_cmd[0];
}

int table_visit_class::get_vel_rotate(){
    if(init_flag) return angle_vel_cmd[1];
}
