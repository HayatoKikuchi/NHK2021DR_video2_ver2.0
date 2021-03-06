#include <Arduino.h>
#include <MsTimer2.h>
#include "RoboClaw.h"

#include "AutoControl.h"
#include "Button.h"
//#include "DualShock4.h" 
#include "Controller.h"
#include "define.h"
#include "LCDclass.h"
#include "lpms_me1Peach.h"
#include "ManualControl.h"
#include "operator.h"
#include "phaseCounterPeach.h"
#include "Platform.h"
#include "PIDclass.h"
#include "Setting.h"
#include "SDclass.h"


AutoControl Auto;
//DualSchok4 Con(&SERIAL_CON);
Controller Con;
lpms_me1 lpms(&SERIAL_LPMSME1);
phaseCounter encX(1);
phaseCounter encY(2);
Platform mechanum(1,1,1,1);
ManualControl ManuCon;
Operator DR(&SERIAL_UPPER);
myLCDclass lcd(&SERIAL_LCD);
mySDclass mySD;

Encorder enc;
DipSW dipsw;
Button userSW(PIN_SW);

PID PIDvelX(VEL_X_KP, VEL_X_KI, VEL_X_KD, INT_TIME);
PID PIDvelY(VEL_Y_KP, VEL_Y_KI, VEL_Y_KD, INT_TIME);
PID PIDvelZ(VEL_Z_KP, VEL_Z_KI, VEL_Z_KD, INT_TIME);
PID PIDPosiX(POSI_X_KP, POSI_X_KI, POSI_X_KD, INT_TIME);
PID PIDPosiY(POSI_Y_KP, POSI_Y_KI, POSI_Y_KD, INT_TIME);
PID PIDPosiZ(POSI_Z_KP, POSI_Z_KI, POSI_Z_KD, INT_TIME);

/* ボード上のボタンとエンコーダでPID制御のゲイン調整を行いLCDに値を表示させるクラス */
PIDsetting settingVx(&lcd, &enc, SETVEL_X);
PIDsetting settingVy(&lcd, &enc, SETVEL_Y);
PIDsetting settingVz(&lcd, &enc, SETVEL_Z);
PIDsetting settingPx(&lcd, &enc, SETPOSI_X);
PIDsetting settingPy(&lcd, &enc, SETPOSI_Y);
PIDsetting settingPz(&lcd, &enc, SETPOSI_Z);

Button SW_RED(PIN_SW_RED);
Button SW_BLACK(PIN_SW_BLACK);
Button SW_UP(PIN_SW_UP);
Button SW_DOWN(PIN_SW_DOWN);
Button SW_RIGHT(PIN_SW_RIGHT);
Button SW_LEFT(PIN_SW_LEFT);

coords position = {0.0, 0.0, radians(0.0)}; //ロボットの自己位置
coords gPosi = position; // PathTrakingクラスで使用する
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

/* 上半身への送信 */
unsigned int upper_cmd = 0;
double tableAngle = 0, tableOmega = 0; //値は[度]


/* ロボットの移動速度を得る関数 */
coords getRobotVelocity()
{
    coords vel;
    vel.x = (prePosition.x - position.x)/INT_TIME;
    vel.y = (prePosition.y - position.y)/INT_TIME;
    vel.z = (prePosition.z - position.z)/INT_TIME;

    prePosition = position; //前の値を更新

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

    /* スイッチのチャタリング対策 */
    dipsw_state = dipsw.getDipState();
    sw_red = SW_RED.button_fall();
    sw_black = SW_BLACK.button_fall();
    sw_up = SW_UP.button_fall();
    sw_down = SW_DOWN.button_fall();
    sw_right = SW_RIGHT.button_fall();
    sw_left = SW_LEFT.button_fall();

    /* 自己位置およびロボットの移動速度の取得処理 */
    int encX_count = encX.getCount();
    int encY_count = encY.getCount();
    double Z_angle = lpms.get_z_angle();
    position = mechanum.getPosi(encX_count, encY_count, Z_angle); gPosi = position; //ロボットの自己位置を更新
    velocity = getRobotVelocity(); //ロボットの移動速度を更新

    /* 足回りの速度制御または位置制御の処理 */
    coords conVel = ManuCon.getRefVel(Con.readJoyLXbyte(), Con.readJoyLYbyte(), Con.readJoyRYbyte());
    // coords conVel = ManuCon.getRefVel(Con.readJoy(LX), Con.readJoy(LY), Con.readJoy(RY));
    //coords conVel = ManuCon.getRefVel(Con.LJoyX, Con.LJoyY, Con.RJoyY);
    coords refVel; //最終的な足回りの目標速度
    bool flag_setting = (SETPOSI_X <= setting_num) && (setting_num <= SETPOSI_Z); //位置PIDのゲイン調整をしている場合はtrue
    dummyPosition = getDummyPosition(flag_setting); //位置PID制御のゲイン調整を開始したら仮の原点を得る

    if(dipsw.getBool(DIP3,ON))
    {
        coords rawVel;
        if(flag_setting && dipsw.getBool(DIP4,ON)) //ゲイン調整が位置PID制御の場合
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
        refVel.x = PIDvelX.getCmd(C_vel.x*rawVel.x, velocity.x, C_vel.x*(JOY_MAXVEL));
        refVel.y = PIDvelY.getCmd(C_vel.y*rawVel.y, velocity.y, C_vel.y*(JOY_MAXVEL));
        refVel.z = PIDvelZ.getCmd(C_vel.z*rawVel.z, velocity.z, C_vel.z*(JOY_MAXANGVEL));
    }
    else
    {
        //コントローラの値をそのまま使用
        refVel.x = C_vel.x*conVel.x;
        refVel.y = C_vel.y*conVel.y;
        refVel.z = C_vel.z*conVel.z; 
    }

        SERIAL_PC.print(refVel.x);
        SERIAL_PC.print("\t");
        SERIAL_PC.print(Con.readJoyLXbyte());
        SERIAL_PC.print("\t");
        SERIAL_PC.print(refVel.y);
        SERIAL_PC.print("\t");
        SERIAL_PC.print(Con.readJoyLYbyte());
        SERIAL_PC.print("\t");
        SERIAL_PC.print(refVel.z);
        SERIAL_PC.print("\t");
        SERIAL_PC.print(Con.readJoyRYbyte());
        SERIAL_PC.print("\n");

    coords refLocalVel = ManuCon.getLocalVel(refVel, position.z);
    mechanum.VelocityControl(refVel); //RoboClawへ速度指令を送る
    //mechanum.VelocityControl(refLocalVel); //RoboClawへ速度指令を送る
}

void setup()
{   
    delay(1000);
    SERIAL_PC.begin(115200);
    SERIAL_LCD.begin(115200);
    SERIAL_UPPER.begin(115200);
    SERIAL_CON.begin(115200);
    //Con.begin(115200);
    DR.setup();

    DR.sendUpperCmd(); //上半身に受信準備ができたことを知らせる

    lcd.clear_display();
    lcd.color_red();
    analogWrite(PIN_LED_RED, 255);
    lcd.write_line("      Waiting       ", LINE_1);
    lcd.write_line("      For           ", LINE_2);
    lcd.write_line("      GR-SAKURA     ", LINE_3);

    /* 軌道追従関連の初期化 */
    // mySD.init();
    // int actpathnum = Auto.init(&mySD, BLUE); //SDカードから各値を取得，戻り値はパスの数
    // Auto.gPosiInit();
    // Auto.initSettings(); // これをやっていないと足回りの指令速度生成しない
    // Auto.setConvPara(0.02, 0.997); // 初期化
    // Auto.setMaxPathnum(actpathnum); // パス数の最大値

    bool ready_to_start = false;
    
    while (!ready_to_start)
    {
        DR.updateUpperCmd(&upper_cmd);
        dipsw_state = dipsw.getDipState();
        if(upper_cmd & UPPER_IS_OK) ready_to_start = true; //上半身の初期化が完了したら次に進む

        if(SW_RED.button_fall() && dipsw.getBool(DIP1, OFF)) ready_to_start = true; //DIP1がOFFのときは上半身の制御をしない
        
        delay(10);
    }
    
    DR.LEDblink(PIN_LED_BLUE, 3 , 100); //GR-SAKURAの受信確認
    
    lcd.clear_display();
    lcd.write_line("    Setting Time    ", LINE_2);
    lcd.write_line("   PUSH BUTTON_PS   ", LINE_3);

    /* PID制御のゲイン調整のクラスを初期化 */
    settingVx.init(VEL_X_KP, VEL_X_KI, VEL_X_KD);
    settingVy.init(VEL_Y_KP, VEL_Y_KI, VEL_Y_KD);
    settingVz.init(VEL_Z_KP, VEL_Z_KI, VEL_Z_KD);
    settingPx.init(POSI_X_KP, POSI_X_KI, POSI_X_KD);
    settingPy.init(POSI_Y_KP, POSI_Y_KI, POSI_Y_KD);
    settingPz.init(POSI_Z_KP, POSI_Z_KI, POSI_Z_KD);

    /* 速度PID制御のクラスの初期化 */
    PIDvelX.PIDinit(velocity.x, velocity.x);
    PIDvelY.PIDinit(velocity.y, velocity.y);
    PIDvelZ.PIDinit(velocity.z, velocity.z);
    PIDPosiX.PIDinit(position.x, position.x);
    PIDPosiY.PIDinit(position.y, position.y);
    PIDPosiZ.PIDinit(position.z, position.z);

    /* 軌道追従の初期化 */

    //PSボタンが押されるまで待機（ボード上のスイッチでも可）
    ready_to_start = false; 
    while (!ready_to_start)
    {
        //ControllerUpdate(); //コントローラの更新と受信確認のLチカ
        /* この間にロボットの位置合わせなどを行う */
        if(/*(Con.readButton(BUTTON_PS, PUSHED)) ||*/ userSW.button_fall())
        {
            if(lpms.init())
            {
                analogWrite(PIN_LED_RED, 0);
                DR.LEDblink(PIN_LED_GREEN, 3, 100);
                delay(100);
            }
            encX.init();
            encY.init();
            ManuCon.init();
            mechanum.platformInit(position);

            lcd.clear_display();
            lcd.color_green();
            ready_to_start = true;
        }

        delay(10);
    }

    /* 上半身にsetupが完了した事を伝える */
    DR.add_upper_cmd(MASTER_IS_OK);
    DR.sendUpperCmd();
    
    delay(1000);

    MsTimer2::set(10, timer_warikomi); // 10ms period
    MsTimer2::start();
}

void loop()
{
    enc_count = enc.getEncCount(); //エンコーダのカウント値を更新（10msでは読み飛ばしが起こった）
    //ControllerUpdate(); //コントローラの更新と受信確認のLチカ
    //Con.update();
    if(flag_10ms)
    {    
        //ControllerUpdate(); //コントローラの更新と受信確認のLチカ
        Con.update();
        DR.updateUpperCmd(&upper_cmd); //上半身との通信

        /* ボタンの処理（10ms周期でチャタリング対策） */

        /* 上半身と連携し，自動制御でない場合 */
        
        if(dipsw.getBool(DIP1, ON) && dipsw.getBool(DIP2, OFF))
        {
            /* 展開の処理 */
            if(/*Con.readButton(BUTTON_PAD,PUSHED)*/0) DR.add_upper_cmd(EXPAND);
            else DR.sub_upper_cmd(EXPAND);

            /* インナーエリアに進入後にテーブル回転機構を所定の位置に移動させる */
            if(/*Con.readButton(BUTTON_MARU,PUSHED)*/0) DR.add_upper_cmd(TABLE_POSITION);

            /* ハンドル把持の処理 */
            if(/*Con.readButton(BUTTON_SANKAKU,PUSHED)*/0)
            {
                static int hand_count = 0;
                if((hand_count % 2) == 0) DR.add_upper_cmd(HANDLE);
                else DR.sub_upper_cmd(HANDLE);
                hand_count++;
            }
            if(upper_cmd & HOLD_HANDLE)
            {
                ////////////////////////////////////////////////
                /* ここに位置制御へ以降する処理を書く */
                /* 動作確認でない場合はここに展開の処理を書く */
                ////////////////////////////////////////////////

                DR.add_upper_cmd(SENDING_TABLE_CMD);
            }

            /* 送信テスト用 */
            
            /* ハンドル把持の処理 */
            if(/*Con.readButton(BUTTON_UP,PUSHED)*/0)
            {
                static int table_count = 0;
                table_count++;
                switch (table_count)
                {
                case 1:
                    tableAngle = 15.5;
                    tableOmega = 10.5;
                    break;
                case 2:
                    tableAngle = 30.5;
                    tableOmega = 20.5;
                    break;
                case 3:
                    tableAngle = -15.5;
                    tableOmega = -10.5;
                    break;
                case 4:
                    tableAngle = -30.5;
                    tableOmega = -20.5;
                    break;
                case 5:
                    tableAngle = 0.0;
                    tableOmega = 0.0;
                    table_count = 0;
                    break;
                default:
                    break;
                }
            }

            DR.sendUpperCmd(tableAngle, tableOmega);
        }
        /* 上半身と連携し，自動制御の場合 */
        else if(dipsw.getBool(DIP1, ON) && dipsw.getBool(DIP2, ON))
        {

        }

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

void ControllerUpdate(void)
{
    if(Con.update()) digitalWrite(PIN_LED_USER, HIGH);
    else digitalWrite(PIN_LED_USER, LOW);
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
      if((SETTINGNUM + 1) <= setting_num) setting_num = 1;
    }
    else if(sw_left)
    {
      setting_num--;
      if(setting_num <= 0) setting_num = SETTINGNUM;
    }
    
    if((setting_num == 1) && flag_500ms) lcd.clear_display();

    /* エンコーダを用いてPID制御のゲイン調整をする */
    settingVx.task("velocity.x PID", flag_500ms, sw_up, sw_down, setting_num);
    settingVy.task("velocity.y PID", flag_500ms, sw_up, sw_down, setting_num);
    settingVz.task("velocity.z PID", flag_500ms, sw_up, sw_down, setting_num);
    settingPx.task("position.x PID", flag_500ms, sw_up, sw_down, setting_num);
    settingPy.task("position.y PID", flag_500ms, sw_up, sw_down, setting_num);
    settingPz.task("position.z PID", flag_500ms, sw_up, sw_down, setting_num);

    /* 値を代入 */
    PIDvelX.setPara(settingVx.getKp(), settingVx.getKi(), settingVx.getKd());
    PIDvelY.setPara(settingVy.getKp(), settingVy.getKi(), settingVy.getKd());
    PIDvelZ.setPara(settingVz.getKp(), settingVz.getKi(), settingVz.getKd());
    PIDPosiX.setPara(settingPx.getKp(), settingPx.getKi(), settingPx.getKd());
    PIDPosiY.setPara(settingPy.getKp(), settingPy.getKi(), settingPy.getKd());
    PIDPosiZ.setPara(settingPz.getKp(), settingPz.getKi(), settingPz.getKd());

  }
  else
  {
    digitalWrite(PIN_LED_ENC, LOW);
  }
}

coords getRefPosition(bool buttonPosiPush, bool buttonAnglePush, bool init_ref = false)
{
    coords ref;
    static int phase_posi = 1, phase_angle = 0;
    if(buttonPosiPush) phase_posi++;

    if(init_ref)
    {
        phase_posi = 1;
        refPosition.z = ref.z = position.z;
    }

    if(buttonAnglePush) 
    { 
        phase_angle++;
        switch (phase_angle)
        {
        case 1:
        case 2: 
            ref.z += radians(90.0);
            break;
        case 3:
        case 4:
            ref.z -= radians(90.0);
            phase_angle = 0;
            break;
        default:
            break;
        }
    }
    
    switch (phase_posi)
    {
    case 1: ref = {0.0, 0.0, ref.z}; break;
    case 2: ref = {0.5, 0.0, ref.z}; break;
    case 3: ref = {0.5, 0.5, ref.z}; break;
    case 4: ref = {0.0, 0.5, ref.z}; break;
    default: phase_posi = 1; break;
    }
}

/* 位置PID制御のゲイン調整を開始したら，その場所を仮の原点とする関数（Z軸は変更しない） */
coords getDummyPosition(bool positionSetting)
{
    static coords positionOffset = {0.0, 0.0, radians(0.0)};
    static bool do_once = true;
    
    if(!positionSetting)
    {
        do_once = true;
        positionOffset = position;
        return position;
    }
    else
    {
        coords DummyPosition;
        DummyPosition.x = position.x - positionOffset.x;
        DummyPosition.y = position.y - positionOffset.y;
        DummyPosition.z = position.z;

        if(do_once)
        {
            PIDPosiX.PIDinit(DummyPosition.x, DummyPosition.x);
            PIDPosiY.PIDinit(DummyPosition.y, DummyPosition.y);
            PIDPosiZ.PIDinit(DummyPosition.z, DummyPosition.z);

            refPosition = getRefPosition(false, false, true);

            do_once = false;
        }

        return DummyPosition;
    }
}
