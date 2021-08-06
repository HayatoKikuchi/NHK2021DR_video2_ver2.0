#ifndef DEFINE_h
#define DEFINE_h

#include <Arduino.h>
#include <HardwareSerial.h>

struct coords
{
    double x;
    double y;
    double z;
};

/* シリアル通信の定義 */
#define SERIAL_PC       Serial
#define SERIAL_ROBOCLAW Serial0
#define SERIAL_MASTER   Serial5
#define SERIAL_CON      Serial7

/* 入力ピンの定義 */
#define PIN_DIP1 26
#define PIN_DIP2 27
#define PIN_DIP3 28
#define PIN_DIP4 29

#define PIN_M1_ORIGIN 17
#define PIN_M2_ORIGIN 18
#define PIN_M3_ORIGIN 19

#define PIN_LIMIT_1 41
#define PIN_LIMIT_2 40

#define PIN_SW1 43
#define PIN_SW2 42

#define PIN_INDEX_1 13
#define PIN_INDEX_2 4
#define PIN_INDEX_3 12
#define PIN_INDEX_4 5
#define PIN_INDEX_5 11
#define PIN_INDEX_6 8
#define PIN_INDEX_7 10
#define PIN_INDEX_8 9

/* 出力ピンの定義 */
#define PIN_EXPAND  36
#define PIN_HAND    37
#define PIN_FET_INDEX_1 38
#define PIN_FET_INDEX_2 39

#define PIN_LED_1 30
#define PIN_LED_2 31
#define PIN_LED_3 32
#define PIN_LED_4 33

#define PIN_XBEERESET 66

/* 出力の定義 */
#define LED_OLL_LOW  0b0000
#define LED_OLL_HIGH 0b1111
#define LED1_HIGH 0b0001
#define LED2_HIGH 0b0010
#define LED3_HIGH 0b0100
#define LED4_HIGH 0b1000

/* 制御（処理）に関する定義 */
#define INT_TIME ( 0.01 )
#define INT_TIME_MS ( INT_TIME * 1000 )

/* セッティングタイムのフェーズ処理 */
#define SET_PHASE1 1 //チームメンバがロボットのセッティングを行う
#define SET_PHASE2 2 //ロボットが自動でモータの原点合わせをする
#define SET_PHASE3 3 //テーブル回転機構を初期位置に合わせる．
#define SET_PHASE4 4 //全てのセッティングが終わったため，処理開始まち

/* テーブル回転機構の位置に関する定義 */
#define TURN_FIRST_POINT -30.0 // 度 //ダミー
#define SLIDE_FIRST_POINT 0.02 // 0.02m //ダミー

/* モータドライバに関する定義 */
#define ADR_MD1 128
//#define ADR_MD2 129

#define SLIDE_RADIAS 0.018 //m //モジュール1，歯数(36)

#define TURN_RES 48.0
#define SLIDE_RES 500.0
#define TURN_GEAR ( 10.0/3.0 )

#define TURN_2RES_PI  ( 2.0 * TURN_RES / PI ) 
#define SLIDE_2RES_PI  ( 2.0 * SLIDE_RES / PI ) 

#define SET_TURN 0.0 //度 (原点復帰用)
#define SET_SLIDE -0.51 //[m] (原点復帰用)

// >>> Controllerまわりで使用 >>>>>>>>>>>>>>>>>>>>>
#define PUSHE 1
#define RELEASE 0
#define PUSHED 2
#define RELEASED -1

#define CON_ADACHI    ( 0 )
#define CON_ELECOM    ( 1 )
#define CON_DS4       ( 2 )

#define CON_TYPE  ( CON_DS4 )

#if CON_TYPE == CON_ADACHI
    #define MASK_BUTTON_UP    0x01
    #define MASK_BUTTON_RIGHT 0x02
    #define MASK_BUTTON_DOWN  0x04
    #define MASK_BUTTON_LEFT  0x08
    #define MASK_BUTTON_R1    0x10
    #define MASK_BUTTON_R2    0x20
    #define MASK_BUTTON_L1    0x40
    #define MASK_BUTTON_L2    0x80

    #define BUTTON_UP    1
    #define BUTTON_RIGHT 2
    #define BUTTON_DOWN  3
    #define BUTTON_LEFT  4
    #define BUTTON_R1    5
    #define BUTTON_R2    6
    #define BUTTON_L1    7
    #define BUTTON_L2    8
#elif CON_TYPE == CON_ELECOM || CON_TYPE == CON_DS4
    #define MASK_BUTTON_X  0x0001 //1
    #define MASK_BUTTON_Y  0x0002 //2
    #define MASK_BUTTON_A  0x0004 //3
    #define MASK_BUTTON_B  0x0008 //4
    #define MASK_BUTTON_SIKAKU   0x0001 //1
    #define MASK_BUTTON_SANKAKU  0x0002 //2
    #define MASK_BUTTON_BATU     0x0004 //3
    #define MASK_BUTTON_MARU     0x0008 //4

    #define MASK_BUTTON_L1     0x0010 //5
    #define MASK_BUTTON_R1     0x0020 //6
    #define MASK_BUTTON_L2     0x0040 //7
    #define MASK_BUTTON_R2     0x0080 //8

    #define MASK_BUTTON_PS     0x0100 // PS4のときはPSボタン
    #define MASK_BUTTON_PAD    0x0200 // PS4のときはパッド
    #define MASK_BUTTON_JOY_L   0x0100 //9
    #define MASK_BUTTON_JOY_R   0x0200 //10
    #define MASK_BUTTON_BACK    0x0400 //11
    #define MASK_BUTTON_START   0x0800 //12

    #define MASK_BUTTON_UP     0x1000 //13
    #define MASK_BUTTON_RIGHT  0x2000 //14
    #define MASK_BUTTON_DOWN   0x4000 //15
    #define MASK_BUTTON_LEFT   0x8000 //16

    #define BUTTON_SIKAKU   1
    #define BUTTON_SANKAKU  2
    #define BUTTON_BATU     3
    #define BUTTON_MARU     4
    
    #define BUTTON_L1     5
    #define BUTTON_R1     6
    #define BUTTON_L2     7
    #define BUTTON_R2     8

    #define BUTTON_JOY_L   9
    #define BUTTON_JOY_R   10
    #define BUTTON_PAD      9
    #define BUTTON_PS       10
    #define BUTTON_BACK     11
    #define BUTTON_START    12
    #define BUTTON_OPTION  11
    #define BUTTON_SHARE   12
    
    #define BUTTON_UP     13
    #define BUTTON_RIGHT  14
    #define BUTTON_DOWN   15
    #define BUTTON_LEFT   16
/*
    #define BUTTON_UP    12
    #define BUTTON_RIGHT 13
    #define BUTTON_DOWN  14
    #define BUTTON_LEFT  15
    #define BUTTON_R1    5
    #define BUTTON_R2    7
    #define BUTTON_L1    4
    #define BUTTON_L2    6
*/
#endif
// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

/* 指定した時間でフラグを切り替えるクラス */
class FlagChange
{
private:
    int loop_count;
    double seconds, int_time;
    
public:

    // @param _seconds 切り替えたい時間
    // @param _int_time 制御周期
    FlagChange(double _seconds, double _int_time);

    // @param flag 切り替えたいフラグ
    bool change(bool *flag);
    void setSeconds(double setNum);

};

#endif
