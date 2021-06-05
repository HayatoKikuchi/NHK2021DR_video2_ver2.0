// HNK学生ロボコン2021 DR親機上半身の制御
// Ⅱ型ポット防御（テーブル回転）の具体的な処理は赤崎が担当
// マスターとのやその他処理は菊池が担当
// 現在のバージョンはマスターと独立した制御になっている

#include <Arduino.h>
#include <MsTimer2.h>
#include "define.h"
#include "DIPclass.h"
#include "DualShock4.h"
#include "Button.h"
#include "master.h"
#include "RoboClaw.h"

DipSW dip;
Button userSW(PIN_SW);
DualSchok4 Con(&SERIAL_CON);
Master master(&SERIAL_MASTER);
RoboClaw MD(&SERIAL_ROBOCLAW, 1);
FlagChange FlagBoardLED(0.5, INT_TIME);
FlagChange FlagUserLED(0.5, INT_TIME);

bool flag_10ms = false;
bool flag_expand = false;
int led_emitting = 0;
int dipState = 0;

coords position = {0.0, 0.0, radians(0.0)};
uint8_t order = 0;

/* マイコンボード上の4つのLEDを光らせる関数 */
// @param emitting_num int型下位4ビットで制御
void UserLED(int emitting_num)
{
    digitalWrite(PIN_LED0, emitting_num & 0x01);
    digitalWrite(PIN_LED1, emitting_num & 0x02);
    digitalWrite(PIN_LED2, emitting_num & 0x04);
    digitalWrite(PIN_LED3, emitting_num & 0x08);
}

/* 基板上の4つのLEDを光らせる関数 */
// @param emitting_num int型下位4ビットで制御
void BoardLED(int emitting_num)
{
    digitalWrite(PIN_LED_1, emitting_num & 0x01);
    digitalWrite(PIN_LED_2, emitting_num & 0x02);
    digitalWrite(PIN_LED_3, emitting_num & 0x04);
    digitalWrite(PIN_LED_4, emitting_num & 0x08);
}

/* RoboClawへの指令値に変換する関数（位置，速度，加速度に対応） */
// @param ref 単位は deg, deg/s, deg/ss のどれか
int getTurnCmd(double ref)
{
    return (int)(radians(ref) *  TURN_2RES_PI);
}

/* RoboClawへの指令値に変換する関数（位置，速度，加速度に対応） */
// @param deg 単位は m, m/s, m/ss のどれか
int getSlideCmd(double ref)
{
    return (int)(ref / SLIDE_RADIAS * SLIDE_2RES_PI);
}


void timer_warikomi()
{
    flag_10ms = true;

    dipState = dip.getDipState();

    /* 展開の処理 */
    if(flag_expand)
    {
        static int expand_seconds = 0;
        if(expand_seconds++ == 99)
        {
            expand_seconds = 0;
            flag_expand = false;
        }
    }

    /* ボード上のLEDを奇麗に光らせるための処理 */
    static int led_time_count = 0;
    if(led_time_count++ == 99)
    {
        led_time_count = 0;
        if(led_emitting++ == 15) led_emitting = 0;
        SERIAL_PC.println(led_emitting);
    }
}

void setup()
{
    delay(1000);

    pinMode(PIN_M1_ORIGIN, INPUT);
    pinMode(PIN_M2_ORIGIN, INPUT);

    pinMode(PIN_LED0, OUTPUT);
    pinMode(PIN_LED1, OUTPUT);
    pinMode(PIN_LED2, OUTPUT);
    pinMode(PIN_LED3, OUTPUT);
    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LED_3, OUTPUT);
    pinMode(PIN_LED_4, OUTPUT);

    pinMode(PIN_EXPAND, OUTPUT);

    UserLED(LED_OLL_HIGH);
    
    delay(1000);

    SERIAL_PC.begin(115200);
    SERIAL_MASTER.begin(115200);
    Con.begin(115200);
    MD.begin(115200);

    bool ready_to_start = false;
    while (!ready_to_start)
    {
        Con.update();
        master.updateMasterCmd(&order, &position);

        static int setup_phase = 4;//SET_PHASE1;
        static bool m1_setup = false, m2_setup = false;
        bool moter_setup = (m1_setup == true && m2_setup == true);
        static bool flag_led = false;
        FlagBoardLED.change(&flag_led);

        if(Con.readButton(BUTTON_PS, PUSHED) && (setup_phase < SET_PHASE4)) setup_phase++;
        //if(!moter_setup) setup_phase = 1;

        switch (setup_phase)
        {
        case SET_PHASE1:
            /* ここで人間がロボットのセッティングを行う． */
            if(flag_led) BoardLED(LED1_HIGH);
            else BoardLED(LED_OLL_LOW);

            dipState = dip.getDipState(); //このタイミングでのみディップスイッチの変更が可能

            break;

        case SET_PHASE2:
            /* 主軸回転の原点出し */
            if(moter_setup || flag_led) BoardLED(LED2_HIGH);
            else if(!moter_setup && flag_led) BoardLED(LED_OLL_LOW);

            if (!digitalRead(PIN_M1_ORIGIN) && !m1_setup)
            {
                MD.SpeedM1(ADR_MD1, getTurnCmd(30.0)); //30.0deg/s
            }
            else if(!digitalRead(PIN_M1_ORIGIN))
            {
                MD.ForwardM1(ADR_MD1, 0);
                MD.SetEncM1(ADR_MD1, SET_ENC_M1);
                m1_setup = true;
            }

            /* 直動スライダの原点出し */
            if(digitalRead(PIN_M2_ORIGIN) && m2_setup)
            {
                MD.SpeedM2(ADR_MD1, getSlideCmd(0.1));
            }
            else if(!digitalRead(PIN_M2_ORIGIN))
            {
                MD.ForwardM1(ADR_MD1, 0);
                MD.SetEncM1(ADR_MD1, SET_ENC_M2);
                m2_setup = true;
            }

            break;
        
        case SET_PHASE3:
            /* テーブル回転機構を所定の位置に合わせる */
            if(flag_led) BoardLED(LED3_HIGH);
            else BoardLED(LED_OLL_LOW);
            
            MD.SpeedAccelDeccelPositionM1(ADR_MD1, getTurnCmd(180.0), getTurnCmd(30.0), getTurnCmd(180.0), getTurnCmd(TURN_2RES_PI), true);
            MD.SpeedAccelDeccelPositionM2(ADR_MD1, getSlideCmd(0.5), getSlideCmd(0.1), getSlideCmd(0.5), getSlideCmd(SLIDE_FIRST_POINT), true);

            break;

        case SET_PHASE4:
            /* コントローラの処理待ち */
            BoardLED(LED_OLL_HIGH);

            if(Con.readButton(BUTTON_PS,PUSHED) || userSW.button_fall())
            {
                ready_to_start = true;
            }

            break;
        
        default:
            break;
        }

        /* setup中のLチカ */
        static bool emitting = false;
        FlagUserLED.change(&emitting);
        if(emitting) UserLED(0b0011);
        else UserLED(0b1100);
        
        delay(INT_TIME_MS);
    }

    /* 初期化が終わったことを示すブリング */
    for(int i = 0; i < 4; i++)
    {
        UserLED(LED_OLL_LOW);
        delay(100);
        UserLED(LED_OLL_HIGH);
        delay(100);
    }

    delay(500);
    
    MsTimer2::set(INT_TIME_MS, timer_warikomi);
    MsTimer2::start();
}

void loop()
{

    if(flag_10ms)
    {
        Con.update();
        master.updateMasterCmd(&order, &position);

        if(Con.readButton(BUTTON_SANKAKU, PUSHED)) flag_expand = true;
        digitalWrite(PIN_EXPAND, flag_expand);

        UserLED(led_emitting);
        BoardLED(dipState);

        flag_10ms = false;
    }

}
