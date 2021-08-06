// HNK学生ロボコン2021 DR親機上半身の制御
// Ⅱ型ポット防御（テーブル回転）の具体的な処理は赤崎が担当
// マスターとの通信やその他処理は菊池が担当

/*
DIPスイッチの設定
 ・DIP1：ONで上半身独立，OFFでマスターからの制御
 ・DIP2：
 ・DIP3：
 ・DIP4：ONでテーブル回転機構の現在位置を疑似的に生成（setupで使用）
*/

/* 
手動制御の場合のボタン
　・ BUTTON_PS　でloop関数へ移行
　・ BUTTON_SANKAKU　で展開（電磁弁を1秒間解放する）
　・ BUTTON_MARU　でテーブル回転機構をハンドルは自前の状態にする
　//・ BUTTON_BATU　でテーブル回転の開始と停止を行う
　//・ BUTTON_SIKAKU　でハンドル把持を切り替える(現在は使用していない)
*/  

#include <Arduino.h>
#include <MsTimer2.h>

#include "Controller.h"
#include "define.h"
#include "DIPclass.h"
#include "Button.h"
#include "master.h"
#include "RoboClaw.h"
#include "table_visit_class.h"

table_visit_class table(INT_TIME);

DipSW dip;
Button userSW(PIN_SW);
Button SW1(PIN_SW1);
Button SW2(PIN_SW2);

Controller Con;
Master master(&SERIAL_MASTER);
RoboClaw MD(&SERIAL_ROBOCLAW, 10000);
FlagChange FlagBoardLED(0.5, INT_TIME);

bool flag_10ms = false;
bool flag_expand = false;
bool flag_table_start = false; //テーブル回転開始のフラグ
bool flag_table = false; //テーブル回転開始のフラグ
int led_emitting = 0; //基板上のLEDを奇麗に光らせるための変数
int dipState = 0; //DIPスイッチの情報を格納

unsigned int masterCmd = 0; //masterから送られてくるコマンド
double tableAnlge, tableOmega;

double target = 0.0 - radians(21.18);
double pre_target = 0.0; //取り敢えず0.0度で初期化
uint8_t val1, val2;
bool state1, state2;
int encount1, encount2;
int vel1 , vel2;
int enc1, enc2;

/* セットアップ中に決定しその後変更できない */
#define MODE_CON 1
#define MODE_MASTER 2
unsigned int uppperMode = 0; //上半身の制御モード

/* 基板上の4つのLEDを光らせる関数 */
// @param emitting_num int型下位4ビットで制御
void BoardLED(int emitting_num)
{
    digitalWrite(PIN_LED_1, emitting_num & 0b0001);
    digitalWrite(PIN_LED_2, emitting_num & 0b0010);
    digitalWrite(PIN_LED_3, emitting_num & 0b0100);
    digitalWrite(PIN_LED_4, emitting_num & 0b1000);
}


void robo_pos(int encount1,int encount2, int vel1, int vel2)//速度付き位置制御
{
  MD.SpeedAccelDeccelPositionM1(ADR_MD1, 0, vel1, 0, encount1, true);
  MD.SpeedAccelDeccelPositionM2(ADR_MD1, 0, vel2, 0, encount2, true);
}

void func(){
   static int count = 0;
   static double check_current = 0.0;
   count++;    

   if(target != pre_target)
   {
       check_current = table.check_table_angle(enc1, enc2);//エンコーダ―の値をRoboClawから読み取って現在のテーブル角度を返す
       pre_target = target;
       //count = 0;
    //    SERIAL_PC.println("CON");
   }

   table.calc_angle_status(target, check_current);   //回転軌道の計算
    encount1 = -table.get_slide_cmd();//直動位置
    encount2 = table.get_rotate_cmd();//回転位置
    vel1 = table.get_vel_slide();//直動速度
    vel2 = table.get_vel_rotate();//回転速度

    SERIAL_PC.print(vel1);
    SERIAL_PC.print("\t");
    SERIAL_PC.print(vel2);
    SERIAL_PC.print("\t");
    SERIAL_PC.print(enc1);
    SERIAL_PC.print("\t");
    SERIAL_PC.print(enc2);
    SERIAL_PC.print("\n");
   //目標角度を変更したら check_current も変更
   /*
   if((count % 350 == 0) && flag_table_auto){//目標値を反対側に持って行っている
     //target = -table.check_table_angle();
     //if(target == 0.0 - radians(21.18)) target = -PI / 4.0;
     target = -target;
     
     //count = 0;
   }
*/
    if((masterCmd & CON_RIGHT) || (Con.readButton(BUTTON_RIGHT) == PUSHED)) target = PI / 4.0;
    if((masterCmd & CON_UP) || (Con.readButton(BUTTON_UP) == PUSHED)) target = 0.0 - radians(21.18);
    if((masterCmd & CON_LEFT) || (Con.readButton(BUTTON_LEFT) == PUSHED)) target = -PI / 4.0;
}

/* RoboClawへの指令値に変換する関数（位置，速度，加速度に対応） */
// @param ref 単位は deg, deg/s, deg/ss のどれか
int getTurnCmd(double ref)
{
    return (int)(radians(ref) *  TURN_2RES_PI * TURN_GEAR);
}

/* RoboClawへの指令値に変換する関数（位置，速度，加速度に対応） */
// @param deg 単位は m, m/s, m/ss のどれか
int getSlideCmd(double ref)
{
    return (int)(ref / SLIDE_RADIAS * SLIDE_2RES_PI);
}

double getTurnNum(int pulse)
{
    return degrees((double)pulse / (TURN_2RES_PI * TURN_GEAR));
}

double getSlideNum(int pulse)
{
    return (double)pulse / SLIDE_2RES_PI * SLIDE_RADIAS;
}

// LEDをチカチカさせるための関数
void LEDblink(byte pin, int times, int interval)
{
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}


void timer_warikomi()
{
    flag_10ms = true;


    // RGB LED を良い感じに光らせるための処理
    static int count = 0;
    count += 2; // ここで光る周期を変えられる(はず)
    if(count < 255){
        analogWrite(PIN_LED_RED, count);
        analogWrite(PIN_LED_BLUE, 255 - count);
    }
    else if(count < 255 * 2){
        analogWrite(PIN_LED_GREEN, count - 255);
        analogWrite(PIN_LED_RED, 255 * 2 - count);
    }
    else if(count < 255 * 3){
        analogWrite(PIN_LED_BLUE, count - 255 * 2);
        analogWrite(PIN_LED_GREEN, 255 * 3 - count);
    }
    else{
        count = 0;
    }


    /* テーブル回転の処理 */
    if(flag_table_start) func();
    

    /* 展開の処理 */
    if(flag_expand)
    {
        static int expand_seconds = 0;
        if(expand_seconds++ == 99) //1秒間だけ電磁弁を解放する
        {
            expand_seconds = 0;
            flag_expand = false;
        }
    }
}

void setup()
{
    delay(1000);

    pinMode(PIN_LED_USER, OUTPUT);
    pinMode(PIN_M1_ORIGIN, INPUT);
    pinMode(PIN_M2_ORIGIN, INPUT);

    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
    pinMode(PIN_LED_3, OUTPUT);
    pinMode(PIN_LED_4, OUTPUT);

    pinMode(PIN_EXPAND, OUTPUT);
    pinMode(PIN_HAND, OUTPUT);
    
    /* LOWの出力にして待機 */
    digitalWrite(PIN_LED_USER, LOW);
    digitalWrite(PIN_LED_1, LOW);
    digitalWrite(PIN_LED_2, LOW);
    digitalWrite(PIN_LED_3, LOW);
    digitalWrite(PIN_LED_4, LOW);
    digitalWrite(PIN_EXPAND, LOW);
    digitalWrite(PIN_HAND, LOW);

    /* XBeeのリセット */
    pinMode(PIN_XBEERESET, OUTPUT);
    digitalWrite(PIN_XBEERESET, LOW);
    delay(10);
    digitalWrite(PIN_XBEERESET, HIGH);
    delay(10);

    SERIAL_PC.begin(115200);
    SERIAL_MASTER.begin(115200);
    SERIAL_CON.begin(115200);
    MD.begin(115200);

    table.init_or_stop(0.2, 1.0, 0.2, 0.0, true);//初期化：加速時間　速度時間　減速時間　初期位置　true or false

    /* セットアップ開始 */
    LEDblink(PIN_LED_RED, 1, 100);
    LEDblink(PIN_LED_GREEN, 1, 100);
    LEDblink(PIN_LED_BLUE, 1, 100);

    bool ready_to_start = false;

    while (!ready_to_start)
    {
        digitalWrite(PIN_LED_USER, Con.update());

        /* setup中のLチカ */

        static bool flag_led = false;
        FlagBoardLED.change(&flag_led);

        static int setup_phase = SET_PHASE1; 
        static bool table_setup = false; //テーブル回転機構の原点合わせ完了の真偽
        static bool table_standby = false; //テーブル回転機構の初期状態の真偽

        static int countUserSW = 0;
        bool userSW_fall = userSW.button_fall();
        if(userSW_fall) countUserSW++;

        // if(uppperMode == MODE_CON)
        // {
        // }
        else if(uppperMode == MODE_MASTER) 
        {
            master.updateMasterCmd(&masterCmd, &tableAnlge, &tableOmega);
            master.sendMasterCmd();
        }

        switch (setup_phase)
        {
        case SET_PHASE1:
            //SERIAL_PC.println("phase1");
            /* ここで人間がロボットのセッティングを行う． */
            /*
                ・DIPスイッチ1を操作して，マスターと通信(OFF)するかコントローラで操作する(OFF)のかを決める．
                ・SW1を押すことで決定する．
            */
            static bool master_recv = false;

            if(flag_led) BoardLED(LED1_HIGH);
            else BoardLED(LED_OLL_LOW);

            dipState = dip.getDipState(); //このタイミングでのみディップスイッチの変更が可能
            
            if(SW1.button_fall() || (Con.readButton(BUTTON_RIGHT) == PUSHED))
            {
                if(dip.getBool(DIP1, ON)) uppperMode = MODE_CON;
                else uppperMode = MODE_MASTER;

                switch (uppperMode)
                {
                case MODE_CON: setup_phase = SET_PHASE2; break;
                case MODE_MASTER: 
                    master_recv = true; 
                    setup_phase = SET_PHASE2;
                    break;
                
                default:
                    break;
                }
            }

            if(master_recv)
            {
                master.updateMasterCmd(&masterCmd, &tableAnlge, &tableOmega);
                if(masterCmd & MASTER_ON) setup_phase = SET_PHASE2; //マスターが起動していたら次のフェーズへ移行する．
            }

            break;

        case SET_PHASE2:
            //SERIAL_PC.println("phase2");
            /* テーブル回転機構の原点復帰を行う */
            /*
                ・初めに直動スライダの原点復帰を行う．
                ・次に主軸の原点復帰を行う．
            */

            static int table_pahse = 1; // 1：回転，2：直動

            if(flag_led) BoardLED(LED2_HIGH);
            else BoardLED(LED_OLL_LOW);

            /* 主軸回転の原点出し */
            if (digitalRead(PIN_M2_ORIGIN) && (table_pahse == 1))
            {
                MD.SpeedM2(ADR_MD1, getTurnCmd(10.0)); //30.0deg/s
            }
            else if(!digitalRead(PIN_M2_ORIGIN) || (countUserSW == 1))
            {
                MD.ForwardM2(ADR_MD1, 0);
                //delay(100); 
                MD.SetEncM2(ADR_MD1, /*getTurnCmd(SET_TURN)*/ -8);
                //delay(100);
                table_pahse = 2;
            }

            /* 直動スライダの原点出し */
            if(digitalRead(PIN_M1_ORIGIN) && (table_pahse == 2))
            {
                MD.SpeedM1(ADR_MD1, getSlideCmd(-0.03));
            }
            else if(!digitalRead(PIN_M1_ORIGIN) || (countUserSW == 2))
            {
                MD.ForwardM1(ADR_MD1, 0);
                MD.SetEncM1(ADR_MD1, /*getSlideCmd(SET_SLIDE)*/ -7251);
                table_setup = true;
                setup_phase = SET_PHASE3;
            }

            break;
        
        case SET_PHASE3:
            //SERIAL_PC.println("phase3");
            /* テーブル回転機構を所定の位置に合わせる */
            if(flag_led) BoardLED(LED3_HIGH);
            else BoardLED(LED_OLL_LOW);
            
            MD.SpeedAccelDeccelPositionM2(ADR_MD1, getTurnCmd(180.0), getTurnCmd(30.0), getTurnCmd(180.0), getTurnCmd(TURN_FIRST_POINT), true);
            MD.SpeedAccelDeccelPositionM1(ADR_MD1, getSlideCmd(0.5), getSlideCmd(0.1), getSlideCmd(0.5), getSlideCmd(0.0), true);

            static int time_count = 0;
            if(time_count++ >= 9) //0.1s周期
            {
                double turn_posi, slide_posi;
                if(dip.getBool(DIP4, ON)) //現在位置を疑似的に生成
                {
                    turn_posi += 3.0;
                    slide_posi += 0.01;
                }
                else
                {
                    turn_posi = getTurnNum((int)MD.ReadEncM1(ADR_MD1, &val1, &state1));
                    slide_posi = getSlideNum((int)MD.ReadEncM2(ADR_MD1, &val2, &state2));
                }

                if(turn_posi > TURN_FIRST_POINT && slide_posi > SLIDE_FIRST_POINT)
                {
                    table_standby = true;
                    setup_phase = SET_PHASE4;
                    master.add_upper_cmd(UPPER_IS_OK);
                }

                time_count = 0; 
            }

            break;

        case SET_PHASE4:
            //SERIAL_PC.println("phase4");
            /* コントローラの処理待ち */
            BoardLED(LED_OLL_HIGH);

            switch (uppperMode)
            {
            case MODE_CON:
                if(Con.readButton(BUTTON_SHARE) == PUSHED || userSW_fall) ready_to_start = true;
                break;
            
            case MODE_MASTER:
                if(masterCmd & MASTER_IS_OK) ready_to_start = true;
                if((Con.readButton(BUTTON_SHARE) == PUSHED) || userSW_fall) ready_to_start = true;
                break;
            
            default:
                break;
            }

            break;
        
        default:
            break;
        }
        
        delay(INT_TIME_MS);
    }

    /* setup完了のお知らせ */
    LEDblink(PIN_LED_GREEN, 3, 100);


    delay(500);
    
    MsTimer2::set(INT_TIME_MS, timer_warikomi);
    MsTimer2::start();
}

void loop()
{

    if(flag_10ms)
    {
        BoardLED(dipState);

        switch (uppperMode)
        {
        case MODE_CON:
            ConModeControl();
            break;
        
        case MODE_MASTER:
            MasterModeControl();
            break;
        
        default:
            break;
        }

        flag_10ms = false;
    }

}

void ConModeControl(void)
{
    digitalWrite(PIN_LED_USER, Con.update());
    enc1 = -(int)MD.ReadEncM1(ADR_MD1, &val1, &state1);//直動のカウント値
    enc2 = (int)MD.ReadEncM2(ADR_MD1, &val2, &state2);//回転のカウント値

    /* 展開の処理 */
    if(Con.readButton(BUTTON_SANKAKU) == PUSHED) flag_expand = true;
    digitalWrite(PIN_EXPAND, flag_expand);

    /* テーブル回転機構をハンドル把持前の状態にする */
    static bool table_set = false;
    if(!table_set && Con.readButton(BUTTON_MARU) == PUSHED) 
    {
        MD.SpeedAccelDeccelPositionM1(ADR_MD1, getSlideCmd(0.5), getSlideCmd(0.2), getSlideCmd(0.5), /*getSlideCmd(0.0)*/-799, true);
        MD.SpeedAccelDeccelPositionM2(ADR_MD1, getTurnCmd(180.0), getTurnCmd(30.0), getTurnCmd(180.0), getTurnCmd(-21.18), true);
        flag_table_start = true;
        table_set = true;
    }

    /* テーブル回転の開始と停止(現在は使用していない) */
    if(table_set && (Con.readButton(BUTTON_BATU) == PUSHED)) flag_table_start = !flag_table_start;
    
    if(flag_table_start) robo_pos(encount1, encount2, vel1, vel2);//速度付き位置制御指令

    /* ハンドル把持する処理 */
    if(0 && Con.readButton(BUTTON_SIKAKU) == PUSHED) digitalWrite(PIN_HAND, !digitalRead(PIN_HAND));

}

void MasterModeControl(void)
{
    master.updateMasterCmd(&masterCmd, &tableAnlge, &tableOmega);
    enc1 = -(int)MD.ReadEncM1(ADR_MD1, &val1, &state1);//直動のカウント値
    enc2 = (int)MD.ReadEncM2(ADR_MD1, &val2, &state2);//回転のカウント値

    //SERIAL_PC.println(masterCmd);

    if((masterCmd & EXPAND)) flag_expand = true;
    digitalWrite(PIN_EXPAND, flag_expand);

    /* テーブル回転機構をハンドル把持前の状態にする */
    static bool table_set = false;
    if(!table_set && (masterCmd & TABLE_POSITION)) 
    {
        MD.SpeedAccelDeccelPositionM1(ADR_MD1, getSlideCmd(0.5), getSlideCmd(0.1), getSlideCmd(0.5), /*getSlideCmd(0.0)*/-799, true);
        MD.SpeedAccelDeccelPositionM2(ADR_MD1, getTurnCmd(180.0), getTurnCmd(30.0), getTurnCmd(180.0), getTurnCmd(-21.18), true);
        table_set = true;
    }

    /* テーブル回転の開始と停止(現在は使用していない) */
    if(table_set && (masterCmd & SENDING_TABLE_CMD)) flag_table_start = true;
    else if(table_set && !(masterCmd & SENDING_TABLE_CMD)) flag_table_start = false;

    if(flag_table_start) robo_pos(encount1, encount2, vel1, vel2);//速度付き位置制御指令

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
}