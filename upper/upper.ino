// HNK学生ロボコン2021 DR親機上半身の制御
// Ⅱ型ポット防御（テーブル回転）の具体的な処理は赤崎が担当
// マスターとの通信やその他処理は菊池が担当

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
Button SW1(PIN_SW1);
Button SW2(PIN_SW2);

DualSchok4 Con(&SERIAL_CON);
Master master(&SERIAL_MASTER);
RoboClaw MD(&SERIAL_ROBOCLAW, 1);
FlagChange FlagBoardLED(0.5, INT_TIME);
FlagChange FlagUserLED(0.5, INT_TIME);

bool flag_10ms = false;
bool flag_expand = false;
int led_emitting = 0; //基板上のLEDを奇麗に光らせるための変数
int dipState = 0; //DIPスイッチの情報を格納

unsigned int masterCmd = 0; //masterから送られてくるコマンド
double tableAnlge, tableOmega;

/* セットアップ中に決定しその後変更できない */
#define MODE_CON 1
#define MODE_MASTER 2
unsigned int uppperMode = 0; //上半身の制御モード

/* マイコンボード上の4つのLEDを光らせる関数 */
// @param emitting_num int型下位4ビットで制御
void UserLED(int emitting_num)
{
    digitalWrite(PIN_LED0, emitting_num & 0b0001);
    digitalWrite(PIN_LED1, emitting_num & 0b0010);
    digitalWrite(PIN_LED2, emitting_num & 0b0100);
    digitalWrite(PIN_LED3, emitting_num & 0b1000);
}

/* 基板上の4つのLEDを光らせる関数 */
// @param emitting_num int型下位4ビットで制御
void BoardLED(int emitting_num)
{
    digitalWrite(PIN_LED_1, emitting_num & 0b0001);
    digitalWrite(PIN_LED_2, emitting_num & 0b0010);
    digitalWrite(PIN_LED_3, emitting_num & 0b0100);
    digitalWrite(PIN_LED_4, emitting_num & 0b1000);
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

double getTurnNum(int pulse)
{
    return degrees((double)pulse / TURN_2RES_PI);
}

double getSlideNum(int pulse)
{
    return (double)pulse / SLIDE_2RES_PI * SLIDE_RADIAS;
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
    if(led_time_count++ == 49)
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
    pinMode(PIN_HAND, OUTPUT);

    /* 初期された確認のLチカ */
    for(int i = 0; i < 6; i++)
    {
        for(int j = 1; j <= 8; j *= 2)
        {
            UserLED(j);
            delay(70 - 12*i);
        }
        UserLED(LED3_HIGH);
        delay(70 - 12*i);
        UserLED(LED2_HIGH);
        delay(70 - 12*i);
    }
    UserLED(LED_OLL_HIGH);
    delay(1000);

    SERIAL_PC.begin(115200);
    SERIAL_MASTER.begin(115200);
    Con.begin(115200);
    MD.begin(115200);

    bool ready_to_start = false;

    while (!ready_to_start)
    {
        /* setup中のLチカ */
        static bool emitting = false;
        FlagUserLED.change(&emitting);
        if(emitting) UserLED(0b0011);
        else UserLED(0b1100);

        static bool flag_led = false;
        FlagBoardLED.change(&flag_led); // for UserLED

        /*  */
        static int setup_phase = SET_PHASE1; 
        static bool table_setup = false; //テーブル回転機構の原点合わせ完了の真偽
        static bool table_standby = false; //テーブル回転機構の初期状態の真偽

        if(uppperMode == MODE_CON)
        {
            Con.update();
            if((Con.readButton(BUTTON_PS, PUSHED) || SW1.button_fall()) && setup_phase < SET_PHASE4) setup_phase++;
            if(!table_setup) setup_phase = SET_PHASE2;
            if(!table_standby) setup_phase = SET_PHASE3;

        }
        else if(uppperMode == MODE_MASTER) 
        {
            master.updateMasterCmd(&masterCmd, &tableAnlge, &tableOmega);
            
            if(SW1.button_fall() && setup_phase < SET_PHASE4) setup_phase++;
            if(!table_setup) setup_phase = SET_PHASE2;
            if(!table_standby) setup_phase = SET_PHASE3;
        }

        switch (setup_phase)
        {
        case SET_PHASE1:
            /* ここで人間がロボットのセッティングを行う． */
            static bool master_recv = false;

            if(flag_led) BoardLED(LED1_HIGH);
            else BoardLED(LED_OLL_LOW);

            dipState = dip.getDipState(); //このタイミングでのみディップスイッチの変更が可能
            
            if(SW1.button_fall())
            {
                if(dip.getBool(DIP1, ON)) uppperMode = MODE_CON;
                else uppperMode = MODE_MASTER;

                master_recv = true;
            }

            if(master_recv)
            {
                master.updateMasterCmd(&masterCmd, &tableAnlge, &tableOmega);
                if(masterCmd & MASTER_ON) setup_phase = SET_PHASE2;
            }

            break;

        case SET_PHASE2:

            static int table_pahse = 1; // 1：直動，2：回転

            if(table_setup || flag_led) BoardLED(LED2_HIGH);
            else if(!table_setup && flag_led) BoardLED(LED_OLL_LOW);


            /* 直動スライダの原点出し */
            if(digitalRead(PIN_M2_ORIGIN) && (table_pahse == 1))
            {
                MD.SpeedM2(ADR_MD1, getSlideCmd(0.1));
            }
            else if(!digitalRead(PIN_M2_ORIGIN))
            {
                MD.ForwardM1(ADR_MD1, 0);
                MD.SetEncM1(ADR_MD1, SET_ENC_M2);
                table_pahse = 2;
                delay(500);
            }

            /* 主軸回転の原点出し */
            if (!digitalRead(PIN_M1_ORIGIN) && (table_pahse == 2))
            {
                MD.SpeedM1(ADR_MD1, getTurnCmd(30.0)); //30.0deg/s
            }
            else if(!digitalRead(PIN_M1_ORIGIN))
            {
                MD.ForwardM1(ADR_MD1, 0);
                MD.SetEncM1(ADR_MD1, SET_ENC_M1);
                delay(500); 
                table_setup = true;
            }

            break;
        
        case SET_PHASE3:
            /* テーブル回転機構を所定の位置に合わせる */
            if(flag_led) BoardLED(LED3_HIGH);
            else BoardLED(LED_OLL_LOW);
            
            MD.SpeedAccelDeccelPositionM1(ADR_MD1, getTurnCmd(180.0), getTurnCmd(30.0), getTurnCmd(180.0), getTurnCmd(TURN_FIRST_POINT), true);
            MD.SpeedAccelDeccelPositionM2(ADR_MD1, getSlideCmd(0.5), getSlideCmd(0.1), getSlideCmd(0.5), getSlideCmd(SLIDE_FIRST_POINT), true);

            static int time_count = 0;
            if(time_count++ >= 99)
            {
                double turn_posi, slide_posi;
                turn_posi = getTurnNum(MD.ReadEncM1(ADR_MD1));
                slide_posi = getSlideNum(MD.ReadEncM2(ADR_MD2));

                if(turn_posi > TURN_FIRST_POINT && slide_posi > SLIDE_FIRST_POINT) table_standby = true;

                time_count = 0; 
            }

            break;

        case SET_PHASE4:
            /* コントローラの処理待ち */
            BoardLED(LED_OLL_HIGH);

            switch (uppperMode)
            {
            case MODE_CON:
                if(Con.readButton(BUTTON_PS,PUSHED) || userSW.button_fall()) ready_to_start = true;
                break;
            
            case MODE_MASTER:
                if(masterCmd & MASTER_IS_OK) ready_to_start = true;
            
            default:
                break;
            }

            break;
        
        default:
            break;
        }

        if(userSW.button_fall()) ready_to_start = true;
        
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
        master.updateMasterCmd(&masterCmd, &tableAnlge, &tableOmega);

        if(Con.readButton(BUTTON_SANKAKU, PUSHED) || (masterCmd & EXPAND)) flag_expand = true;
        digitalWrite(PIN_EXPAND, flag_expand);

        UserLED(led_emitting);
        BoardLED(dipState);

        /* テーブル回転機構をハンドル把持前の状態にする */
        static bool table_set = true;
        if(table_set && (masterCmd & TABLE_POSITION)) 
        {
            //////////////////////////////////////////////////////////////////
            /* ここにテーブル回転機構をハンドル把持前の状態にする処理を書く */
            //////////////////////////////////////////////////////////////////
            table_set = false;
        }

        /* ハンドル把持する処理 */
        if(masterCmd & HANDLE)
        {
            digitalWrite(PIN_HAND, HIGH);
            master.add_upper_cmd(HOLD_HANDLE);
        }
        else
        {
            digitalWrite(PIN_HAND, LOW);
        }

        master.sendMasterCmd();

        flag_10ms = false;
    }

}
