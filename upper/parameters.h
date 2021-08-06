// 速度設定
// #define ALL_TIME 2.0//目標時間
// #define BASIC_ACCEL_TIME 0.5//加速時間
// #define BASIC_DECCEL_TIME 0.5//減速時間

// #define ORIGINAL_POINT 0.0
// #define FIRST_POS -14.0 / 360.0 * 2 * PI//-1.0 * PI / 4.0//初期値
// #define AIM_RAD 0.0//PI / 4.0//今は使用していない

#define R_MM 544.0//テーブル半径
#define U_MM 360//直動初期長さ
//#define T 0.01//周期

//分解能・ギア比
#define RESOLUTION 2000//直動機構の分解能
#define ROTATE_RES 100 * 4//回転機構の分解能
#define GEAR_R 18
#define GEAR_RATE 24
#define GEAR_ROTATE 1//10.0 / 3.0

//基礎値
#define BASIC_AN_VEL PI / 4.0//基礎角速度
#define BASIC_POS PI / 4.0//基礎移動角度
#define BASIC_DIST PI / 2.0//基礎移動量
