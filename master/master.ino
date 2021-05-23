#include <Arduino.h>
#include <MsTimer2.h>

#include "define.h"
#include "Platform.h"
#include "lpms_me1Peach.h"
#include "phaseCounterPeach.h"
#include "ControllerForDR.h"
#include "ManualControl.h"
#include "PIDclass.h"
#include "operator.h"
#include "LCDclass.h"
#include "Setting.h"
#include "Button.h"
#include "RoboClaw.h"

Platform mechanum;
lpms_me1 lpms(&SERIAL_LPMSME1);
phaseCounter encX(1);
phaseCounter encY(2);
Controller Con(&SERIAL_XBEE);
ManualControl ManuCon;
Operator DR;
myLCDclass lcd(&SERIAL_LCD);

Encorder enc;
DipSW dipsw;
Button userSW(PIN_SW);

PID PIDvelX(0.5, 0.0, 0.0, INT_TIME);
PID PIDvelY(0.5, 0.0, 0.0, INT_TIME);
PID PIDvelZ(0.5, 0.0, 0.0, INT_TIME);
PID PIDPosiX(0.5, 0.0, 0.0, INT_TIME);
PID PIDPosiY(0.5, 0.0, 0.0, INT_TIME);
PID PIDPosiZ(0.5, 0.0, 0.0, INT_TIME);

/* ボード上のボタンとエンコーダでPID制御のゲイン調整を行いLCDに値を表示させるクラス */
PIDsetting settingVx(&PIDvelX, &lcd, &enc, SETVEL_X);
PIDsetting settingVy(&PIDvelY, &lcd, &enc, SETVEL_Y);
PIDsetting settingVz(&PIDvelZ, &lcd, &enc, SETVEL_Z);
PIDsetting settingPx(&PIDPosiX, &lcd, &enc, SETPOSI_X);
PIDsetting settingPy(&PIDPosiY, &lcd, &enc, SETPOSI_Y);
PIDsetting settingPz(&PIDPosiZ, &lcd, &enc, SETPOSI_Z);

Button SW_RED(PIN_SW_RED);
Button SW_BLACK(PIN_SW_BLACK);
Button SW_UP(PIN_SW_UP);
Button SW_DOWN(PIN_SW_DOWN);
Button SW_RIGHT(PIN_SW_RIGHT);
Button SW_LEFT(PIN_SW_LEFT);

coords position = {0.0, 0.0, radians(0.0)}; //ロボットの自己位置
coords prePosition = position;
coords refPosition = {0.0, 0.0, radians(0.0)}; //位置制御の目標位置
coords dummyPosition = {0.0,0.0,radians(0.0)}; //ゲイン調整時の仮の自己位置
coords velocity = {0.0, 0.0, radians(0.0)}; //ロボットの速度
coords C_vel = {1.0, 1.0, 1.0}; //ロボットの移動速度の倍数

bool flag_10ms = false, flag_500ms = false;
int enc_count = 0, dipsw_state = 0; //各値を格納
bool sw_red = false;
bool sw_black = false;
bool sw_up = false;
bool sw_down = false;
bool sw_right = false;
bool sw_left = false;
int setting_num = SET_MAXVELOMEGA;


/* コントローラの受信と確認のLチカを行う関数 */
void controller_update()
{
    if(Con.update()) digitalWrite(PIN_LED_USER, HIGH);
    else digitalWrite(PIN_LED_USER, LOW);
}

/* ロボットの移動速度を得る関数 */
coords getRobotVelocity(coords robotPosition)
{
    coords vel;
    vel.x = (prePosition.x - position.x)/INT_TIME;
    vel.y = (prePosition.y - position.y)/INT_TIME;
    vel.z = (prePosition.z - position.z)/INT_TIME;

    return vel;
}

void timer_warikomi()
{
    /* 10ms，500msのフラグを生成 */
    flag_10ms = true;  
    static int count_500ms = 0;
    if(50 <= count_500ms)
    {
        flag_500ms = true;
        count_500ms = 0;
    }
    count_500ms++;

    DR.RGB_led(2); //フルカラーLEDを光らせる

    int encX_count = encX.getCount();
    int encY_count = encY.getCount();
    double Z_angle = lpms.get_z_angle();
    position = mechanum.getPosi(encX_count, encY_count, Z_angle); //ロボットの自己位置を更新
    velocity = getRobotVelocity(position); //ロボットの移動速度を更新

    coords conVel = ManuCon.getRawVel(Con.readJoy(LX), Con.readJoy(LY), Con.readJoy(RY));
    coords refVel; //最終的な足回りの目標速度
    bool flag_setting = SETPOSI_X <= setting_num && setting_num <= SETPOSI_Z; //位置PIDのゲイン調整をしている場合はtrue
    dummyPosition = getDummyPosition(flag_setting); //位置PID制御のゲイン調整を開始したら仮の原点を得る
    if(dipsw.getBool(DIP3,ON) || dipsw.getBool(DIP4,ON))
    {
        coords rawVel;
        if(flag_setting) //ゲイン調整が位置PID制御の場合
        {
            rawVel.x = PIDPosiX.getCmd(refPosition.x, dummyPosition.x, C_vel.x*JOY_MAXVEL);
            rawVel.y = PIDPosiY.getCmd(refPosition.y, dummyPosition.y, C_vel.y*JOY_MAXVEL);
            rawVel.z = PIDPosiZ.getCmd(refPosition.z, dummyPosition.z, C_vel.z*JOY_MAXANGVEL);
        }
        else
        {
            rawVel = conVel; //コントローラの値が速度制御の目標速度
        }
        //速度PID制御を行う
        refVel.x = PIDvelX.getCmd(C_vel.x*rawVel.x, velocity.x, C_vel.x*(JOY_MAXVEL*1.1));
        refVel.y = PIDvelY.getCmd(C_vel.y*rawVel.y, velocity.y, C_vel.y*(JOY_MAXVEL*1.1));
        refVel.z = PIDvelZ.getCmd(C_vel.z*rawVel.z, velocity.z, C_vel.z*(JOY_MAXANGVEL*1.1));
    }
    else
    {
        //コントローラの値をそのまま使用
        refVel.x = C_vel.x*conVel.x;
        refVel.y = C_vel.y*conVel.y;
        refVel.z = C_vel.z*conVel.z; 
    }

    mechanum.VelocityControl(refVel); //RoboClawへ速度指令を送る
}

void setup()
{   
    SERIAL_PC.begin(115200);
    SERIAL_LCD.begin(115200);
    Con.begin(115200);
    
    lcd.clear_display();
    lcd.color_red();
    analogWrite(PIN_LED_RED, 255);
    lcd.write_line("    Setting Time    ", LINE_1);
    lcd.write_line("        |  |        ", LINE_2);
    lcd.write_line("        |  |        ", LINE_3);
    lcd.write_line("===PUSH BUTTON_PS===", LINE_4);

    //PSボタンが押されるまで待機（ボード上のスイッチでも可）
    bool ready_to_start = false; 
    while (!ready_to_start)
    {
        controller_update();
        /* この間にロボットの位置合わせなどを行う */
        if(Con.readButton(BUTTON_PS,PUSHED) || userSW.button_fall())
        {
            mechanum.platformInit(position);
            if(lpms.init())
            {
                analogWrite(PIN_LED_RED, 0);
                DR.LEDblink(PIN_LED_GREEN, 3, 100);
                delay(100);
            }
            encX.init();
            encY.init();
            lcd.clear_display();
            lcd.color_green();
            ready_to_start = true;
        }
        delay(10);
    }
    
    delay(1000);

    MsTimer2::set(10, timer_warikomi); // 10ms period
    MsTimer2::start();
}

void loop()
{
    enc_count = enc.getEncCount(); //エンコーダのカウント値を更新（10msでは読み飛ばしが起こった）
    if(flag_10ms)
    {
        controller_update();

        dipsw_state = dipsw.getDipState();
        sw_red = SW_RED.button_fall();
        sw_black = SW_BLACK.button_fall();
        sw_up = SW_UP.button_fall();
        sw_down = SW_DOWN.button_fall();
        sw_right = SW_LEFT.button_fall();
    
        pid_gain_setting();

        flag_10ms = false;
    }
    
    

    if (flag_500ms)
    {
        if(dipsw.getBool(DIP4,OFF))
        {
            lcd.clear_display();
            lcd.write_str("X:", LINE_1, 1);
            lcd.write_str("Y:", LINE_2, 1);
            lcd.write_str("Z:", LINE_3, 1);
            lcd.write_str("Enc:", LINE_4, 1);
            lcd.write_double(position.x, LINE_1, 4);
            lcd.write_double(position.y, LINE_2, 4);
            lcd.write_double(degrees(position.z), LINE_3, 4);
            lcd.write_int(enc_count,LINE_4,5);
        }

        digitalWrite(PIN_LED_1, dipsw_state & DIP1);
        digitalWrite(PIN_LED_2, dipsw_state & DIP2);
        digitalWrite(PIN_LED_3, dipsw_state & DIP3);
        digitalWrite(PIN_LED_4, dipsw_state & DIP4);

        flag_500ms = false;
    }
}

/*　PID制御のゲイン調整　*/
void pid_gain_setting()
{  
  if(dipsw.getBool(DIP4, ON)) //DIP4がONの場合
  {
    digitalWrite(PIN_LED_ENC, HIGH);
    if(sw_right)
    {
      setting_num++;
      if(SETTINGNUM + 1 <= setting_num) setting_num = 1;
    }
    else if(sw_left)
    {
      setting_num--;
      if(setting_num <= 0) setting_num = SETTINGNUM;
    }

    char velx_moji[] = "velocity.X PID";
    char vely_moji[] = "velocity.Y PID";
    char velz_moji[] = "velocity.Z PID";
    char posix_moji[] = "position.X PID";
    char posiy_moji[] = "position.Y PID";
    char posiz_moji[] = "position.Z PID";
    settingVx.task(flag_500ms, sw_up, sw_down, velx_moji, setting_num);
    settingVy.task(flag_500ms, sw_up, sw_down, vely_moji, setting_num);
    settingVz.task(flag_500ms, sw_up, sw_down, velz_moji, setting_num);
    settingPx.task(flag_500ms, sw_up, sw_down, posix_moji, setting_num);
    settingPy.task(flag_500ms, sw_up, sw_down, posiy_moji, setting_num);
    settingPz.task(flag_500ms, sw_up, sw_down, posiz_moji, setting_num); 
  }
  else
  {
    digitalWrite(PIN_LED_ENC, LOW);
    settingVx.init_variable();
    settingVy.init_variable();
    settingVz.init_variable();
    settingPx.init_variable();
    settingPy.init_variable();
    settingPz.init_variable();
  }
}

/* 位置PID制御のゲイン調整を開始したら，その場所を仮の原点とする関数（Z軸は変更しない） */
coords getDummyPosition(bool positionSetting)
{
    static coords positionOffset = {0.0, 0.0, radians(0.0)};
    if(!positionSetting)
    {
        positionOffset = position;
        return position;
    }
    else
    {
        coords DummyPosition;
        DummyPosition.x = position.x - positionOffset.x;
        DummyPosition.y = position.y - positionOffset.y;
        DummyPosition.z = position.z;
        return DummyPosition;
    }
}
