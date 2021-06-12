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
#define SERIAL_ROBOCLAW Serial1 //ダミー
#define SERIAL_MASTER   Serial2 //ダミー
#define SERIAL_CON      Serial3 //ダミー

/* 入力ピンの定義 */
#define PIN_DIP1 36 //ダミー
#define PIN_DIP2 37 //ダミー
#define PIN_DIP3 38 //ダミー
#define PIN_DIP4 39 //ダミー

#define PIN_M1_ORIGIN 44 //ダミー
#define PIN_M2_ORIGIN 45 //ダミー

#define PIN_SW1 46 //ダミー
#define PIN_SW2 47 //ダミー

/* 出力ピンの定義 */
#define PIN_EXPAND 40 //ダミー
#define PIN_HAND 41 //ダミー

#define PIN_LED_1 30 //ダミー
#define PIN_LED_2 31 //ダミー
#define PIN_LED_3 32 //ダミー
#define PIN_LED_4 33 //ダミー

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
#define TURN_FIRST_POINT 0.0 // 0.0m
#define SLIDE_FIRST_POINT 0.0 // 0.0度

/* モータドライバに関する定義 */
#define ADR_MD1 128
#define ADR_MD2 129

#define TURN_RADIAS 10.0 //ダミー
#define SLIDE_RADIAS 10.0 //ダミー

#define TURN_RES 1024.0 //ダミー
#define SLIDE_RES 1024.0 //ダミー

#define TURN_2RES_PI  ( 2.0 * TURN_RES / PI ) 
#define SLIDE_2RES_PI  ( 2.0 * SLIDE_RES / PI ) 

#define SET_ENC_M1 0 //ダミー
#define SET_ENC_M2 0 //ダミー

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
