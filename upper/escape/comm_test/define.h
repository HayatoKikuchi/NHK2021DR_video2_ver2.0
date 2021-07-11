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
#define SERIAL_CON      Serial6

/* 入力ピンの定義 */
#define PIN_DIP1 29
#define PIN_DIP2 28
#define PIN_DIP3 27
#define PIN_DIP4 26

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
#define TURN_FIRST_POINT 30.0 // 30.0度
#define SLIDE_FIRST_POINT 0.2 // 0.2m

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
