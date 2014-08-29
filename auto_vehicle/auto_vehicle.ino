#include <Servo.h>
#include <Wire.h>    // I2C通信用ライブラリ

Servo myservo;

// ピン番号
const int servoPin = 10;
const int dcPin1 = 8;
const int dcPin2 = 7;
const int pmPin = 6;
const int analogpin0 = 0;
const int analogpin1 = 1;
const int tSwitch1 = 2;
const int tSwitch2 = 3;

// モーター用パラメータ変数
int iMotor;        // DCモータ駆動用のパラメータ
int iServo;        // サーボーモータのパラメータ(角度)

// モータ用パラメータ定数
const int incS =  12;     // 操作一回当りのサーボモータパラメータ変化量
const int minM = -100;   // DCモータパラメータの最小値
const int maxM = 150;    // DCモータパラメータの最大値
const int minS =  78;    // サーボモータパラメータの最小値
const int maxS = 102;    // サーボモータパラメータの最大値

unsigned int stcnt;      // 直進時間のカウント
bool lavoidf, ravoidf;

const String voiceRight    = "ga'tte/magaru.";
const String voiceLeft     = "shu'tte/magaru.";
const String voiceStraight = "buu'a-tte/iku'-.";
const String voiceBack     = "hokennkinnta'ka/na'ruyannke/-'.";

#ifdef _DEBUG_
#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define SERIAL_PRINT(...) 
#define SERIAL_PRINTLN(...) 
#endif

//#define _FAST_CODE_

//---------------------------------------------------
// セットアップ関数
//---------------------------------------------------
void setup()
{
#ifdef _DEBUG_
  Serial.begin(9600);
#endif
  Wire.begin();  // ArduinoをI2C Masterとして初期化
  
  pinMode(dcPin1 ,OUTPUT);
  pinMode(dcPin2 ,OUTPUT);
  pinMode(tSwitch1 ,INPUT);
  pinMode(tSwitch2 ,INPUT);
  
  myservo.attach(servoPin);
  
  iMotor = maxM;    // DCモータの初期値
  iServo = 90;      // サーボの初期値
  
  stcnt = 0;
  lavoidf = ravoidf = false;
  
  while(AquesTalk_IsBusy()) ; // Ready待ち
  AquesTalk_Synthe("#J"); // 「ポン！」チャイム音出力
}
//---------------------------------------------------
// 音声合成にる声出し
//---------------------------------------------------
void vvoice( int n )
{
  String strMsg;
  
  switch ( n ) {
    case 1: {
      strMsg = voiceStraight;
      break;
    }
    case 2: {
      strMsg = voiceRight;
      break;
    }
    case 3: {
      strMsg = voiceLeft;
      break;
    }
    case 4: {
      strMsg = voiceBack;
      break;
    }
    case 0: {
      strMsg = "";
      break;
    }
  }
  
  while( AquesTalk_IsBusy() ) ;
  AquesTalk_Synthe(strMsg);

}
  
//---------------------------------------------------
// DCモータ駆動関数
//---------------------------------------------------
void MotorDrive( int iIn1Pin, int iIn2Pin, int iMotor )
{
  if( iMotor == 0 ) { // 
    digitalWrite(iIn1Pin, LOW);
    digitalWrite(iIn2Pin, LOW);
  }
  else if( 0 < iMotor ) {
    digitalWrite(iIn1Pin, LOW);
    digitalWrite(iIn2Pin, HIGH);
    analogWrite(pmPin,iMotor);
  }
  else {
    digitalWrite(iIn1Pin, HIGH);
    digitalWrite(iIn2Pin, LOW);
    analogWrite(pmPin,-iMotor);
  }
}

//---------------------------------------------------
// 測距センサー距離換算関数
//---------------------------------------------------
#ifndef _FAST_CODE_
float voltage2distance(float dVolt) {
  
  float dDist;
  
  if (dVolt < 0.384321) {
    dDist = 999.0;                        /*out of range*/
  } else if (dVolt < 0.430210) {
    dDist = dVolt * (-217.916667) + 163.750000;      /*80 - 70*/
    
  } else if (dVolt < 0.493308) {
    dDist = dVolt * (-158.484848) + 138.181818;    /*70 - 60*/
      
  } else if (dVolt < 0.590822) {
    dDist = dVolt * (-102.549020) + 110.588235;  /*60 - 50*/
        
  } else if (dVolt < 0.722753) {
    dDist = dVolt * (-75.797101) + 94.782609;  /*50 - 40*/
          
  } else if (dVolt < 0.894837) {
    dDist = dVolt * (-58.111111) + 82.000000;  /*40 - 30*/
            
  } else if (dVolt < 1.284895) {
    dDist = dVolt * (-25.637255) + 52.941176;  /*30 - 20*/
              
  } else if (dVolt < 1.623327) {
    dDist = dVolt * (-14.774011) + 38.983051;  /*20 - 15*/
                
  } else if (dVolt < 2.294455) {
    dDist = dVolt * (-7.450142) + 27.094017;  /*15 - 10*/
                  
  } else if (dVolt < 3.131931) {
    dDist = dVolt * (-5.970320) + 23.698630;  /*10 - 5*/
  }
  else {
    dDist = 0.0;                              /*error*/
  }
  return dDist;
}
#endif
//---------------------------------------------------
// 距離の閾値判定関数
//---------------------------------------------------
#ifndef _FAST_CODE_
bool chkThresholdDistance( int dat )
{
  bool chkf = false;
  
  float dVoltage = (float)dat / 1024.0 * 5.0;
  if ( 15 > voltage2distance(dVoltage) ) {
    chkf = true;
  }
  return chkf;
}
#else
bool chkThresholdDistance( int dat )
{
  bool chkf = false;
  if ( dat > 1.623327 && dat < 3.131931 ) {
    chkf = true;
  }
  return chkf;
}
#endif
//---------------------------------------------------
// 測距センサ関数
//---------------------------------------------------
int DistanceSensor()
{  
  int dataL = analogRead(analogpin1);  //read data from analog1 pin
  bool chkL = chkThresholdDistance(dataL);
  
  int dataR = analogRead(analogpin0);  //read data from analog0 pin
  bool chkR = chkThresholdDistance(dataR);
    
  int iservo = 90;
  
  if ( chkL ) {
    iservo = 102 ;
  } else if ( chkR ) {
    iservo = 78 ;
  } 
  
  return iservo;
}
//---------------------------------------------------
// タッチセンサ関数(グローバル変数に直接代入)
//---------------------------------------------------
int TouchSensor( int iIn3Pin, int iIn4Pin )
{
  int dtime = 10;
  int lban = digitalRead(iIn3Pin);
  int rban = digitalRead(iIn4Pin);
  int imotor = maxM;
  int iservo = iServo;
  
  if ( lban == LOW && rban == LOW ) {
    imotor = -100;
    dtime = 5000;
    iservo = 90;
  } else if ( lban == LOW ) {
    imotor = -100;
    dtime = 5000;
    iServo = 102;
  } else if ( rban == LOW ) {
    imotor = -100;
    dtime = 5000;
    iservo = 78;
  }
  
  iMotor = imotor;
  iServo = iservo;
  
  return dtime;
}

//---------------------------------------------------
// メインループ関数
//---------------------------------------------------
void loop()
{ 
  // 測距センサー情報からサーボ角度を計算
  iServo = DistanceSensor();
  
  // タッチセンサーからの情報で上書き
  int dtime = TouchSensor( tSwitch1, tSwitch2 );
  
  if ( dtime != 5000 ) { // タッチセンサーOFF
    if ( iServo == 102  ) {  // 右距離センサ反応
      stcnt = 0;
      if ( lavoidf == false ) {
        vvoice(2);
        lavoidf = true; ravoidf = false;
        SERIAL_PRINTLN("Right!!");
      }
    } else if ( iServo == 78 ) {  // 左距離センサ反応
      stcnt = 0;
      if ( ravoidf == false ) {  
        vvoice(3);
        ravoidf  = true; lavoidf = false;
        SERIAL_PRINTLN("Left!!");
      }
    } else {                  // 直進
      lavoidf = ravoidf = false;
      SERIAL_PRINTLN("Straight!!");
      if ( stcnt == 100 ) {   // 直進が一定時間経った場合
        vvoice(1);
        stcnt = 0;
      } else {
        vvoice(0);
        stcnt ++;
      }
    }
  } else {
    stcnt = 0;
    SERIAL_PRINTLN("Others!!");
    vvoice(4);
  }

  iMotor = constrain(iMotor,minM,maxM);
  MotorDrive(dcPin1,dcPin2,iMotor);
  
  iServo = constrain(iServo,minS,maxS);
  SERIAL_PRINT("Servo is ");
  SERIAL_PRINTLN(iServo);
  myservo.write(iServo);
  
  delay(dtime);

}

//----------------------------------------------------
//    音声合成用の関数
//----------------------------------------------------
#define I2C_ADDR_AQUESTALK 0x2E // AquesTalk pico LSIのデフォルトのI2Cアドレス

// LSIがコマンドを受信可能かチェック
// 戻り値　0:Ready 1:Busy 2:Error
int AquesTalk_IsBusy()
{
    delay(10); // Busy応答は10msec以上待つ必要がある 連続して呼ばれた場合のため
    Wire.requestFrom(I2C_ADDR_AQUESTALK, 1);
    if(Wire.available()>0){
        byte c = Wire.read();
        if(c=='>')    return 0;    // Ready応答
        else        return 1;    // busy応答
    }
    else {
        return 2; //ERR: NOACK または応答が無い。I2Cの配線をチェックすべし
    }
}


// 音声合成開始    引数に音声記号列を指定
void AquesTalk_Synthe(String &strMsg)
{
    char msg[256];
    strMsg.toCharArray(msg, 256);
    AquesTalk_Synthe(msg);
}

// 音声合成開始    引数に音声記号列を指定
// 最後に"¥r"を送信
void AquesTalk_Synthe(const char *msg)
{
    AquesTalk_Cmd(msg);
    AquesTalk_Cmd("¥r");
}

// LSI にコマンド送信
void AquesTalk_Cmd(const char *msg)
{
    // Wireの制約で、一度に送れるのは32byteまで
    // AquesTalk picoへは一度に128byteまで送れるので、
    // Wire.beginTransmission()～Wire.endTransmission()を複数回に分けて呼び出す
    const char *p = msg;
    for(;*p!=0;){
        Wire.beginTransmission(I2C_ADDR_AQUESTALK);
        // Wireの制約で、一度に送れるのは32byteまで
        for(int i=0;i<32;i++){
#ifdef _DEBUG_
            Serial.println(*p);
#endif
            Wire.write(*p++);
            if(*p==0) break;
        }
#ifdef _DEBUG_
        Serial.println("endTransmission");
#endif
        Wire.write('\r');
        Wire.endTransmission(); // 実際はこのタイミングで送信される
    }
}
