#include <Arduino.h>
#include <MsTimer2.h>

#include "define.h"
#include "Platform.h"
#include "lpms_me1Peach.h"
#include "phaseCounterPeach.h"
#include "controllerForDR.h"
#include "ManualControl.h"
#include "PIDclass.h"
#include "operater.h"
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
operater DR;
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

PIDsetting settingVx(&PIDvelX, &lcd, &enc);
PIDsetting settingVy(&PIDvelY, &lcd, &enc);
PIDsetting settingVz(&PIDvelZ, &lcd, &enc);
PIDsetting settingPx(&PIDPosiX, &lcd, &enc);
PIDsetting settingPy(&PIDPosiY, &lcd, &enc);
PIDsetting settingPz(&PIDPosiZ, &lcd, &enc);

Button SW_RED(PIN_SW_RED);
Button SW_BLACK(PIN_SW_BLACK);
Button SW_UP(PIN_SW_UP);
Button SW_DOWN(PIN_SW_DOWN);
Button SW_RIGHT(PIN_SW_RIGHT);
Button SW_LEFT(PIN_SW_LEFT);

coords position = {0.0, 0.0, radians(0.0)}; //ロボットの自己位置
coords prePosition = position;
coords velocity = {0.0, 0.0, radians(0.0)}; //ロボットの速度
bool flag_10ms = false, flag_500ms = false;
int enc_count = 0, dipsw_state = 0; //各値を格納
bool sw_red = false;
bool sw_black = false;
bool sw_up = false;
bool sw_down = false;
bool sw_right = false;
bool sw_left = false;


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
    position = mechanum.getPosi(encX_count, encY_count, Z_angle);
    velocity = getRobotVelocity(position);

    coords refvel = ManuCon.getRawVel(Con.readJoy(LX), Con.readJoy(LY), Con.readJoy(RY));
    mechanum.VelocityControl(refvel);
}

void setup()
{   
    SERIAL_PC.begin(115200);
    SERIAL_LCD.begin(115200);
    lcd.clear_display();
    lcd.color_red();
    analogWrite(PIN_LED_RED, 255);
    lcd.write_line("    Setting Time    ", LINE_1);
    lcd.write_line("         ||         ", LINE_2);
    lcd.write_line("         ||         ", LINE_3);
    lcd.write_line("   PUSH BUTTON_PS   ", LINE_4);
    Con.begin(115200);

    //PSボタンが押されるまで待機（ボード上のスイッチでも可）
    bool ready_to_start = false; 
    while (!ready_to_start)
    {
        controller_update();
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
    enc_count = enc.getEncCount();
    if(flag_10ms)
    {
        controller_update();

        dipsw_state = dipsw.getDipState();
        sw_red = SW_RED.button_fall();
        sw_black = SW_BLACK.button_fall();
        sw_up = SW_UP.button_fall();
        sw_down = SW_DOWN.button_fall();
        sw_right = SW_LEFT.button_fall();

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
    
    pid_gain_setting();
}

/*　PID制御のゲイン調整　*/
void pid_gain_setting()
{  
  if(dipsw.getBool(DIP4, ON)) //DIP4がONの場合
  {
    digitalWrite(PIN_LED_ENC, HIGH);
    static int pid_setting_num = 1;
    if(sw_right)
    {
      pid_setting_num++;
      if(7 <= pid_setting_num) pid_setting_num = 1;
    }
    else if(sw_left)
    {
      pid_setting_num--;
      if(pid_setting_num <= 0) pid_setting_num = 6;
    }

    char velx_moji[] = "velocity.X PID";
    char vely_moji[] = "velocity.Y PID";
    char velz_moji[] = "velocity.Z PID";
    char posix_moji[] = "position.X PID";
    char posiy_moji[] = "position.Y PID";
    char posiz_moji[] = "position.Z PID";
    settingVx.task(flag_500ms, sw_up, sw_down, velx_moji, pid_setting_num == 1);
    settingVy.task(flag_500ms, sw_up, sw_down, vely_moji, pid_setting_num == 2);
    settingVz.task(flag_500ms, sw_up, sw_down, velz_moji, pid_setting_num == 3);
    settingPx.task(flag_500ms, sw_up, sw_down, posix_moji, pid_setting_num == 4);
    settingPy.task(flag_500ms, sw_up, sw_down, posiy_moji, pid_setting_num == 5);
    settingPz.task(flag_500ms, sw_up, sw_down, posiz_moji, pid_setting_num == 6); 
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
