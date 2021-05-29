#ifndef DEFINE_h
#define DEFINE_h

#include "Arduino.h"

struct coords{
    double x;
    double y;
    double z;
};

#define SERIAL_PC       Serial
#define SERIAL_UPPER    Serial0
#define SERIAL_LPMSME1  Serial1
#define SERIAL_ROBOCLAW Serial4
#define SERIAL_LEONARDO Serial5
#define SERIAL_LCD      Serial6
#define SERIAL_XBEE     Serial7

#define PIN_XBEERESET 66

// スイッチやLEDのピン設定
#define PIN_DIP1 25
#define PIN_DIP2 24
#define PIN_DIP3 69
#define PIN_DIP4 70

#define PIN_SW_UP    30
#define PIN_SW_LEFT  31
#define PIN_SW_RIGHT 33
#define PIN_SW_DOWN  32

#define PIN_SW_BLACK    29
#define PIN_SW_RED      28

#define PIN_ENC_A  26
#define PIN_ENC_B  27

#define PIN_LED_1 20
#define PIN_LED_2 36
#define PIN_LED_3 37
#define PIN_LED_4 38
#define PIN_LED_ENC 40

#define PIN_CSB 10

// Lernardo からのコントローラ用データのマスクデータ
// #define BUTTON_X  0x0001
// #define BUTTON_Y  0x0002
// #define BUTTON_A  0x0004
// #define BUTTON_B  0x0008

// #define BUTTON_L1     0x0010
// #define BUTTON_R1     0x0020
// #define BUTTON_L2     0x0040
// #define BUTTON_R2     0x0080

// #define BUTTON_JOY_L   0x0100
// #define BUTTON_JOY_R   0x0200
// #define BUTTON_BACK    0x0400
// #define BUTTON_START   0x0800

// #define BUTTON_UP     0x1000
// #define BUTTON_RIGHT  0x2000
// #define BUTTON_DOWN   0x4000
// #define BUTTON_LEFT   0x8000

// 制御周期
#define INT_TIME			( 0.01 )//( 0.001 )

// >>> AutoControlで使用 >>>>>>>>>>>>>>>>>>
// フェーズ管理
#define STATE0  ( 7 )
#define STATE1  ( 8 )
#define STATE2  ( 9 )
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>> ManualControlで使用 >>>>>>>>>>>>>>>>>>
#define JOY_DEADBAND    ( 5 )
#define JOY_MAXVEL      ( 1.0 )
#define JOY_MAXANGVEL   ( radians(0.0) )
#define MANUAL_LOWPASS_T  ( 0.25 )
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>> PathTrackingで使用 >>>>>>>>>>>>>>>>>>
#define POSI_X_KP    ( 0.5 )
#define POSI_X_KI    ( 0.0 )
#define POSI_X_KD    ( 0.0 )

#define POSI_Y_KP    ( 0.5 )
#define POSI_Y_KI    ( 0.0 )
#define POSI_Y_KD    ( 0.0 )

#define POSI_Z_KP    ( 0.5 )
#define POSI_Z_KI    ( 0.0 )
#define POSI_Z_KD    ( 0.0 )

#define YOKOZURE_KP    ( 3.0 )
#define YOKOZURE_KI    ( 0.0 )
#define YOKOZURE_KD    ( 1.5 )

#define KAKUDO_KP    ( 4.0 )
#define KAKUDO_KI    ( 0.0 )
#define KAKUDO_KD    ( 0.0 )

#define FILT_SOKUDO_OMEGA ( 22.0 )
#define FILT_SOKUDO_DZETA ( 1.0 )

#define FILT_KAKUDO_OMEGA ( 10.0 )
#define FILT_KAKUDO_DZETA ( 1.0 )
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>> Platformで使用 >>>>>>>>>>>>>>>>>>>>>
// Platformの種類を設定
#define PLATFORM_MECHANUM      ( 0 )
#define PLATFORM_OMNI4WHEEL    ( 1 )
#define PLATFORM_OMNI3WHEEL    ( 2 )
#define PLATFORM_DUALWHEEL     ( 3 )

#define DRIVE_UNIT  ( PLATFORM_MECHANUM )

#if DRIVE_UNIT == PLATFORM_DUALWHEEL
    // 双輪キャスター関連
    #define PIN_CSB     ( 10 )    // turntableのPIN(CSB)
    #define RADIUS_R    ( 0.04 )    // wheel radius
    #define RADIUS_L    ( 0.04 )    // wheel radius
    #define W           ( 0.265 )    // tread
    #define GEARRATIO   ( 5.5 )
    #define TT_RES4     ( 4096 )    // turntableの分解能
    #define _2RES_PI    ( 2 * 2048 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
    #define _2RES_PI_T  ( 2 * 500 / 3.141592 ) //  ターンテーブルの角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(500はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
#elif DRIVE_UNIT == PLATFORM_MECHANUM
    // メカナム関連
    #define WHEEL_R		( 0.05 )    //車輪半径
    #define TREAD_2 	( 0.15561 ) //車両中心から車輪接地点までのY軸方向距離(トレッド/2)
    #define WHEELBASE_2	( 0.26023 ) //車両中心から車輪接地点までのX軸方向距離(ホイールベース/2)
    #define _2RES_PI  ( 2.0 * 2048.0 / PI ) //  駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
#elif DRIVE_UNIT == PLATFORM_OMNI3WHEEL  
    #define WHEEL_R		( 0.019 )
    #define DIST2WHEEL  ( 0.120 )
    #define GEARRATIO   ( 51.45 )
    #define COS_PI_6    ( 0.86602540378 )
    #define SIN_PI_6    ( 0.5 )
    #define _2RES_PI    ( 2.0 * 2048 / 3.141592 ) // 駆動輪の角速度[rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(2048はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
#elif DRIVE_UNIT == PLATFORM_OMNI4WHEEL
    #define WHEEL_R (0.0635)
    #define DIST2WHEEL (0.421185)               //中心からホイールまでの距離
    #define GEARRATIO   ( 1.0 )           // 車輪からエンコーダの計測軸までのギヤ比(1:1 なら 1)
    #define COS_PI_4    ( 0.70711 )         // cos(pi/4)
    #define SIN_PI_4    ( 0.70711 )         // sin(pi/4)
    #define _2RES_PI (2.0 * 1024.0 / 3.141592 ) //  [rad/s]からRoboClawの指令値[pulses/s]に変換するための定数(1024はエンコーダ分解能) 4逓倍しているが，分母は元は2*piで，通分されている
#endif

// ロボットの移動速度のPID制御
#define VEL_X_KP    ( 0.5 )
#define VEL_X_KI    ( 0.0 )
#define VEL_X_KD    ( 0.0 )

#define VEL_Y_KP    ( 0.5 )
#define VEL_Y_KI    ( 0.0 )
#define VEL_Y_KD    ( 0.0 )

#define VEL_Z_KP    ( 0.5 )
#define VEL_Z_KI    ( 0.0 )
#define VEL_Z_KD    ( 0.0 )

// RoboClaw関連
#define ADR_MD1             ( 128 )
#define ADR_MD2             ( 129 )

// 自己位置推定用エンコーダ関連
#define _2PI_RES4   ( 2 * PI / 800 ) // res = 200 分母は res*4
#define RADIUS_X    ( 0.024 ) // X軸計測輪の半径[m]
#define RADIUS_Y    ( 0.024 ) // Y軸計測輪の半径[m]
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

//#define STATE_ALL   ( STATE1 + STATE2 + STATE3 + STATE4 )

#endif
