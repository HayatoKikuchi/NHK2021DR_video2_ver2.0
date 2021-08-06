// NHK学生ロボコン2021DR2次ビデオでのマスターマイコンのコード
// 2021年7月7日版のManualControlのサンプルコードをベースに作成

/*
DIPスイッチの設定
 ・DIP1：フィールドの設定（今回はどちらでもよいので取り敢えずON）
 ・DIP2：ONのときに上半身を制御する
 ・DIP3：ONのときに自動制御をしOFFのときに手動制御をする
 ・DIP4：SDカードへの書き込み設定（取り敢えずON）
*/

/* 
  手動制御の場合のボタン
　・ BUTTON_RIGHT　でloop関数へ移行
　・ BUTTON_SANKAKU　で展開（電磁弁を1秒間解放する）
　・ BUTTON_MARU　でテーブル回転機構をハンドルは自前の状態にする
　//・ BUTTON_BATU　でテーブル回転の開始と停止を行う
　//・ BUTTON_SIKAKU　でハンドル把持を切り替える(現在は使用していない)
*/

#include <Arduino.h>
#include <MsTimer2.h>
#include <SPI.h>

#include "define.h"

#include "AutoControl.h"
#include "Button.h"
#include "Controller.h"
#include "LpmsMe1Peach.h"
#include "ManualControl.h"
#include "operator.h"
#include "PhaseCounterPeach.h"
#include "PIDclass.h"
#include "Platform.h"
#include "SDclass.h"
#include "RoboClaw.h"
#include "Filter.h"

PhaseCounter enc1(1);
PhaseCounter enc2(2);
ManualControl manualCon;
AutoControl autonomous;
Platform platform(-1,-1,-1,-1); // 括弧内の引数で回転方向を変えられる
Controller CON;
Operator DR(&SERIAL_UPPER);
Filter Cvel(INT_TIME);

Button UserSW(PIN_SW);

//AMT203V amt203(&SPI, PIN_CSB);
LpmsMe1 lpms(&SERIAL_LPMSME1);
mySDclass mySD;
bool SDwrite = false; // trueでSDカードに書き出し

coords gPosi = {0.0, 0.0, radians(0.0)};
coords gRefV = {0.0, 0.0, radians(0.0)};
coords gRefP = {0.0, 0.0, radians(0.0)}; //位置制御の目標位置
coords position = gPosi;
coords prePosition = position;
coords dummyPosition = {0.0,0.0,radians(0.0)}; //ゲイン調整時の仮の自己位置
coords velocity = {0.0, 0.0, radians(0.0)}; //ロボットの速度
int zone = RED;
bool flag_10ms = false; // loop関数で10msごとにシリアルプリントできるようにするフラグ
bool flag_100ms = false;

/* 上半身への送信 */
unsigned int upper_cmd = 0;
double tableAngle = 0, tableOmega = 0; //値は[度]

/* 足回りのPID制御 */
PID velXPID(VEL_X_KP, VEL_X_KI, VEL_X_KD, INT_TIME);
PID velYPID(VEL_Y_KP, VEL_Y_KI, VEL_Y_KD, INT_TIME);
PID velZPID(VEL_Z_KP, VEL_Z_KI, VEL_Z_KD, INT_TIME);
PID posiXPID(POSI_X_KP, POSI_X_KI, POSI_X_KD, INT_TIME);
PID posiYPID(POSI_Y_KP, POSI_Y_KI, POSI_Y_KD, INT_TIME);
PID posiZPID(POSI_Z_KP, POSI_Z_KI, POSI_Z_KD, INT_TIME);

unsigned int robotState = 0; // ロボットの状態
#define STATE_LPMS_ENABLE 0x01
#define STATE_SD_INIT     0x02
#define STATE_SD_WRITE    0x04
#define STATE_ZONE        0x08
#define STATE_READY       0x10

Button button_up(PIN_SW_UP);
Button button_down(PIN_SW_DOWN);
Button button_left(PIN_SW_LEFT);
Button button_right(PIN_SW_RIGHT);
Button button_red(PIN_SW_RED);
Button button_black(PIN_SW_BLACK);


// 最大最小範囲に収まるようにする関数
double min_max(double value, double minmax)
{
  if(value > minmax) value = minmax;
  else if(value < -minmax) value = -minmax;
  return value;
}

// LEDをチカチカさせるための関数
void LEDblink(byte pin, int times, int interval){
  analogWrite(pin, 0);
  for(int i = 0; i < times; i++){
    delay(interval);
    analogWrite(pin, 255);
    delay(interval);
    analogWrite(pin, 0);
  }
}

/* ロボットの移動速度を得る関数 */
coords getRobotVelocity()
{
    coords vel;
    vel.x = (position.x - prePosition.x)/INT_TIME;
    vel.y = (position.y - prePosition.y)/INT_TIME;
    vel.z = (position.z - prePosition.z)/INT_TIME;

    prePosition = position; //前の値を更新

    return vel;
}

// setupで有効にされるタイマ割り込み処理が書いてある場所
void timer_warikomi(){    
  // RGB LED を良い感じに光らせるための処理
  static int count = 0;
  static int count_flag = 0;
  count += 2; // ここで光る周期を変えられる(はず)
  count_flag++;

  if(count < 255){
    analogWrite(PIN_LED_RED, count);
    analogWrite(PIN_LED_BLUE, 255 - count);
  }else if(count < 255 * 2){
    analogWrite(PIN_LED_GREEN, count - 255);
    analogWrite(PIN_LED_RED, 255 * 2 - count);
  }else if(count < 255 * 3){
    analogWrite(PIN_LED_BLUE, count - 255 * 2);
    analogWrite(PIN_LED_GREEN, 255 * 3 - count);
  }else{
    count = 0;
  }

  // フラグ立てるための処理
  flag_10ms = true;
  if(count_flag >= 10){
    flag_100ms = true;
    count_flag = 0;
  }

  double angle_rad;
  int encX, encY; // X,Y軸エンコーダのカウント値
  // 自己位置推定用エンコーダのカウント値取得
  encX = -enc1.getCount();
  encY = -enc2.getCount();

  // LPMS-ME1のから角度を取得
  angle_rad = (double)lpms.get_z_angle();
  gPosi = platform.getPosi(encX, encY, angle_rad);
  position = gPosi;
  velocity = getRobotVelocity();
}

// エラーが発生したら無限ループで停止
void error_stop(){
  SERIAL_M5STACK.println("!!ERROR!");
  while(1){
    analogWrite(PIN_LED_RED, 255);
    analogWrite(PIN_LED_BLUE, 0);
    wait(0.25);
    analogWrite(PIN_LED_RED, 0);
    analogWrite(PIN_LED_BLUE, 255);
    wait(0.25);
  }
}

void setup()
{
  delay(1000);

  Serial.begin(115200);
  SERIAL_CON.begin(115200);
  SERIAL_M5STACK.begin(115200);
  SERIAL_UPPER.begin(115200);
  
  pinMode(PIN_XBEERESET, OUTPUT); // XBeeのリセット
  digitalWrite(PIN_XBEERESET, 0);
  delay(10);
  digitalWrite(PIN_XBEERESET,1);
  delay(10);
  
  // pinMode(PIN_SW, INPUT); // オンボードのスイッチ
  // pinMode(PIN_LED_USER, OUTPUT);

  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);
  pinMode(PIN_LED_4, OUTPUT);
  pinMode(PIN_LED_ENC, OUTPUT);
  
  pinMode(PIN_DIP1, INPUT);
  pinMode(PIN_DIP2, INPUT);
  pinMode(PIN_DIP3, INPUT);
  pinMode(PIN_DIP4, INPUT);

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  analogWrite(PIN_LED_RED, 0); // 消しちゃダメ，ぜったい →　LPMSのために
  analogWrite(PIN_LED_BLUE, 0);
  analogWrite(PIN_LED_GREEN, 0);

  // LPMS-ME1の初期化
  if(lpms.init() != 1) error_stop(); // 理由はわからないが，これをやる前にLEDblinkかanalogWriteを実行していないと初期化できない
  robotState |= STATE_LPMS_ENABLE;
  LEDblink(PIN_LED_BLUE, 2, 100);  // 初期化が終わった証拠にブリンク
  SERIAL_M5STACK.println("!LPMS-ME1 init done!"); // M5stackのデバッグエリアに出力「!」を先頭に付ければ改行コードの前までを出力できる
  Serial.println("LPMS-ME1 init done!");
  Serial.flush();
  
  if(mySD.init() == 0){
    robotState |= STATE_SD_INIT;
    SERIAL_M5STACK.println("!SD-card init done!");
  }else{
    SERIAL_M5STACK.println("!SD-card init failed!!!");
  }
  delay(10);
  
  SDwrite = digitalRead(PIN_DIP4) == 0 ? true : false;

  if(SDwrite){
    mySD.make_logfile();
    robotState |= STATE_SD_WRITE;
    SERIAL_M5STACK.println("!Log file created!");
  }
  
  zone = digitalRead(PIN_DIP1) == 0 ? BLUE : RED;
  int actpathnum = autonomous.init(&mySD, zone);//←mySD.path_read(BLUE, motion->Px, motion.Py, motion.refvel, motion.refangle, motion.acc_mode, motion.acc_count, motion.dec_tbe);
  Serial.print("path num: ");
  Serial.println(actpathnum + 1);
  SERIAL_M5STACK.print("!Path num: ");
  SERIAL_M5STACK.println(actpathnum + 1);
  if(zone == BLUE) robotState |= STATE_ZONE;

  autonomous.gPosiInit();

  velXPID.PIDinit(0.0, 0.0);
  velYPID.PIDinit(0.0, 0.0);
  velZPID.PIDinit(0.0, 0.0);
  posiXPID.PIDinit(0.0, 0.0);
  posiYPID.PIDinit(0.0, 0.0);
  posiZPID.PIDinit(0.0, 0.0);

  Cvel.setSecondOrderPara(22.0, 1.0, 1.0);

  LEDblink(PIN_LED_RED, 2, 100);
  
  //robotState |= STATE_WAIT_INPUT;

  bool ready_for_upper = false;
  while (!ready_for_upper)
  {
      if(!digitalRead(PIN_DIP2))
      {
        DR.updateUpperCmd(&upper_cmd);

        if(upper_cmd & UPPER_IS_OK) 
        {
          ready_for_upper = true; //上半身の初期化が完了したら次に進む
          LEDblink(PIN_LED_BLUE, 2 , 100);
        }

        DR.sendUpperCmd();
      }
      else  
      {  
        digitalWrite(PIN_LED_USER, CON.update()); // コントローラからの受信
        if((CON.readButton(BUTTON_SHARE) == PUSHED) || UserSW.button_fall())
        {
          ready_for_upper = true; //DIP2がOFFのときは上半身の制御をしない
          LEDblink(PIN_LED_BLUE, 2 , 100);
        }
      }
      
      delay(10);
  }

  SERIAL_M5STACK.println("!Waiting controller input");

  // コントローラの"右"ボタンが押されるまで待機
  while((robotState & STATE_READY) == 0){
    delay(10);
    digitalWrite(PIN_LED_USER, CON.update());
    send_state();

    if((CON.readButton(BUTTON_SHARE) == PUSHED) || UserSW.button_fall())
    {
      robotState |= STATE_READY;
      SERIAL_M5STACK.println("!READY TO GO !!!!!!!!!!");
      if(!digitalRead(PIN_DIP2))
      {
        DR.add_upper_cmd(MASTER_IS_OK);
        DR.sendUpperCmd();
      }
    }
  }

  enc1.init();
  enc2.init();

  manualCon.init();
  platform.platformInit(gPosi);
  
  autonomous.initSettings(); // これをやっていないと足回りの指令速度生成しない
  autonomous.setConvPara(0.02, 0.997); // 初期化
  autonomous.setMaxPathnum(actpathnum); // パス数の最大値

  LEDblink(PIN_LED_GREEN, 2, 100);

  MsTimer2::set(10, timer_warikomi); // 10ms period
  MsTimer2::start();
}

void loop()
{
  // 10msに1回ピン情報を出力する
  if(flag_10ms)
  {
    digitalWrite(PIN_LED_USER, CON.update()); // コントローラからの受信
    
    //ユーザ編集部分 >>>>>>>>>>>>>>>>>

    if(!digitalRead(PIN_DIP2)) //DIP2がONで上半身を動かす
    {
      DR.updateUpperCmd(&upper_cmd); //上半身との通信

      if(digitalRead(PIN_DIP3)) //DIP3がOFFのときに上半身の手動制御をする．
      {
        /* 展開の処理 */
        if(CON.readButton(BUTTON_SANKAKU) == PUSHED) DR.add_upper_cmd(EXPAND);
        else DR.sub_upper_cmd(EXPAND);

        /* インナーエリアに進入後にテーブル回転機構を所定の位置に移動させる */
        if(CON.readButton(BUTTON_MARU) == PUSHED) DR.add_upper_cmd(TABLE_POSITION);

        /* テーブル回転の開始と停止 */
        /*
        static int count_table = 0;
        if(CON.readButton(BUTTON_BATU) == PUSHED)
        {
            if((count_table % 2) == 0) DR.add_upper_cmd(SENDING_TABLE_CMD);
            else DR.sub_upper_cmd(SENDING_TABLE_CMD);
            count_table++;
        }
        */

        /* テーブルの位置制御に使用 */
        if(CON.readButton(BUTTON_RIGHT) == PUSHED) DR.add_upper_cmd(CON_RIGHT);
        else DR.sub_upper_cmd(CON_RIGHT);
        if(CON.readButton(BUTTON_UP) == PUSHED) DR.add_upper_cmd(CON_UP);
        else DR.sub_upper_cmd(CON_UP);
        if(CON.readButton(BUTTON_LEFT) == PUSHED) DR.add_upper_cmd(CON_LEFT);
        else DR.sub_upper_cmd(CON_LEFT);

        /* ハンドル把持の処理(現在は使用していない) */
        if(0 && (CON.readButton(BUTTON_SIKAKU) == PUSHED))
        {
            static int hand_count = 0;
            if((hand_count % 2) == 0) DR.add_upper_cmd(HANDLE);
            else DR.sub_upper_cmd(HANDLE);
            hand_count++;
        }

      }

      DR.sendUpperCmd(tableAngle, tableOmega);
    }

    // 手動操作するための処理 >>>>
    static int mode_platform_controll = 0; //足回りの速度制御と位置制御を切り替える
    static bool init_posi = false;
    static double rawC = 1.0;
    coords rawV;

    if(CON.readButton(BUTTON_L2) == PUSHE) rawC = 0.3;
    else if(CON.readButton(BUTTON_R2) == PUSHE) rawC = 2.0;
    else rawC = 1.0;
    
    double Cv = Cvel.SecondOrderLag(rawC);

    if(CON.readButton(BUTTON_DOWN) == PUSHED) mode_platform_controll++;
    switch (mode_platform_controll % 2)
    {
    case 0:
      rawV = manualCon.getRefVel(CON.readJoyLXbyte(), CON.readJoyLYbyte(), CON.readJoyRYbyte()); // ジョイスティックの値から，目標速度を生成
      gRefV = {Cv*rawV.x, Cv*rawV.y, rawV.z};

      gRefP.z += gRefV.z * INT_TIME * 0.9;
      init_posi = false;
      DR.sub_upper_cmd(SENDING_TABLE_CMD);
      break;
    
    case 1:
      if(!init_posi)
      {
        gRefP = position;
        //gRefP.z = radians(0.0);
        posiXPID.PIDinit(gRefP.x, position.x);
        posiYPID.PIDinit(gRefP.y, position.y);
        init_posi = true;
      }
/*
      if(CON.readButton(BUTTON_R2) == PUSHED)
      {
        gRefP.x += 0.5;
        gRefP.y += 0.5;
      }

      if(CON.readButton(BUTTON_L2) == PUSHED)
      {
        gRefP.x -= 0.5;
        gRefP.y -= 0.5;
      }
*/
      gRefV.x = posiXPID.getCmd(gRefP.x, position.x, Cv*JOY_MAXVEL);
      gRefV.y = posiYPID.getCmd(gRefP.y, position.y, Cv*JOY_MAXVEL);

      DR.add_upper_cmd(SENDING_TABLE_CMD);

      break;
    
    default:
      rawV = {0.0, 0.0, radians(0.0)};
      break;
    }

    gRefV.z = posiZPID.getCmd(gRefP.z, position.z, JOY_MAXANGVEL);

    coords refV;
    refV.x = gRefV.x;//velXPID.getCmd(rawV.x, velocity.x, JOY_MAXVEL);
    refV.y = gRefV.y;//velYPID.getCmd(rawV.y, velocity.y, JOY_MAXVEL);
    refV.z = gRefV.z;//velZPID.getCmd(rawV.z, velocity.z, JOY_MAXANGVEL);

    coords refLocalVel = manualCon.getLocalVel(gRefV, position.z);

    platform.VelocityControl(refLocalVel); // 目標速度に応じて，プラットフォームを制御
    // <<<<
    
    // シリアル出力する
    // Serial.print(CON.getButtonState(),BIN);
    // Serial.print(" ");
    // Serial.print(CON.readJoyLXbyte());
    // Serial.print(" ");
    // Serial.print(CON.readJoyLYbyte());
    // Serial.print(" ");
    // Serial.print(CON.readJoyRYbyte());
    // Serial.print(" ");
    // Serial.print(refV.x);
    // Serial.print(" ");
    // Serial.print(refV.y);
    // Serial.print(" ");
    // Serial.print(refV.z);
    // Serial.print(" ");
    // Serial.print(gPosi.x);
    // Serial.print("\t");
    // Serial.print(gPosi.y);
    // Serial.print("\t");
    // Serial.print(gPosi.z);
    // Serial.print("\t");
    // Serial.print(velocity.x);
    // Serial.print("\t");
    // Serial.print(velocity.y);
    // Serial.print("\t");
    // Serial.println(velocity.z);

    // SDカードにログを吐く
    if(SDwrite){ // 変数が追加されています!!!!
      String dataString = "";
      static bool first_write = true;
      if(first_write){
        dataString += "gPosix,gPosiy,gPosiz,gRefVx,gRefVy,gRefVz";
        mySD.write_logdata(dataString);
        first_write = false;
        dataString = "";
      }
      dataString += String(gPosi.x, 4) + "," + String(gPosi.y, 4) + "," + String(gPosi.z, 4);
      dataString += "," + String(gRefV.x, 4) + "," + String(gRefV.y, 4) + "," + String(gRefV.z, 4);

      mySD.write_logdata(dataString);
    }
    
    //ユーザ編集部分 <<<<<<<<<<<<<<<<<

    flag_10ms = false;
  }

  // 100msごとにLCDを更新する
  if(flag_100ms){
    send_state(); // M5stackへ状態の送信 ※不要な場合はコメントアウトを

    //ユーザ編集部分 >>>>>>>>>>>>>>>>>

    //ユーザ編集部分 <<<<<<<<<<<<<<<<<
    
    flag_100ms = false;
  }
}

// M5stackに送るデータ
void send_state(){
  unsigned int checksum = 0;
  char sendStr[25] = {0};
  int sendaData[6] = { (int)(gRefV.x * 100),  (int)(gRefV.y * 100),  (int)(gRefV.z * 100), (int)(gPosi.x * 100),  (int)(gPosi.y * 100),  (int)(gPosi.z * 100)};
  bool flagMinus[6] = {0};

  for(int i = 0; i < 6; i++){
    if(sendaData[i] < 0){
      flagMinus[i] = true;
      sendaData[i] = abs(sendaData[i]);
    }
  }
  sendStr[0] = 0; // ゼロで状態送信を指定
  sendStr[1] = robotState;
  sendStr[2] = sendaData[0] & 0x3F;
  sendStr[3] = ( sendaData[0] >> 6 ) & 0x1F;
  if(flagMinus[0]) sendStr[3] |= 0x20;

  sendStr[4] = sendaData[1] & 0x3F;
  sendStr[5] = ( sendaData[1] >> 6 ) & 0x1F;
  if(flagMinus[1]) sendStr[6] |= 0x20;
  
  sendStr[7] = sendaData[2] & 0x3F;
  sendStr[7] = ( sendaData[2] >> 6 ) & 0x1F;
  if(flagMinus[2]) sendStr[7] |= 0x20;

  sendStr[8] = sendaData[3] & 0x3F;
  sendStr[9] = ( sendaData[3] >> 6 ) & 0x1F;
  if(flagMinus[3]) sendStr[9] |= 0x20;
  
  sendStr[10] = sendaData[4] & 0x3F;
  sendStr[11] = ( sendaData[4] >> 6 ) & 0x1F;
  if(flagMinus[4]) sendStr[11] |= 0x20;
  
  sendStr[12] = sendaData[5] & 0x3F;
  sendStr[13] = ( sendaData[5] >> 6 ) & 0x1F;
  if(flagMinus[5]) sendStr[13] |= 0x20;

  sendStr[14] = (int)(CON.readJoyLXbyte() * 0.247) & 0x3F;
  sendStr[15] = (int)(CON.readJoyLYbyte() * 0.247) & 0x3F;
  sendStr[16] = (int)(CON.readJoyRXbyte() * 0.247) & 0x3F;
  sendStr[17] = (int)(CON.readJoyRYbyte() * 0.247) & 0x3F;
  
  unsigned int ButtonState = CON.getButtonState(); // コントローラデータ格納用
  sendStr[18] = ButtonState & 0x3F;
  sendStr[19] = (ButtonState >> 6) & 0x3F;
  sendStr[20] = (ButtonState >> 12) & 0x3F; // ここはボタン数によって書き換える

  for(int i = 0; i < 21; i++){
    checksum += (unsigned int)sendStr[i];
    SERIAL_M5STACK.write(sendStr[i] + 0x20);
  }
  sendStr[20] = (char)checksum & 0x3F;
  SERIAL_M5STACK.write(sendStr[20] + 0x20);
  //SERIAL_M5STACK.write((checksum & 0x3F) + 0x20);  
  SERIAL_M5STACK.print("\n");
}