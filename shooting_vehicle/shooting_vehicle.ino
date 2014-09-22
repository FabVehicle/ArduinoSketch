#include <MsTimer2.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo srvWheel;
Servo srvGun;

#define _DEBUG_

// モータピン定義用構造体
struct pinMotor {
  int servoPin;	// PWM
  int dcPin1;	// Digital
  int dcPin2;	// Ditital
  int pmPin;	// PWM
};

// モータパラメータ用構造体
struct pmMotor {
  int iMotor;	// DCモータ駆動用のパラメータ
  int iServo;	// サーボーモータのパラメータ(角度)
};

// モータ境界パラメータ用構造体
struct pmBoundMotor {
  int ofsM;  // DCモータオフセットパラメータ値(スロット値＝１の時の値)
  int incM;  // DCモータパラメータ変化量
  int incS;  // サーボモータパラメータ変化量
  int minM;  // DCモータパラメータの最小値
  int maxM;  // DCモータパラメータの最大値
  int minS;  // サーボモータパラメータの最小値
  int maxS;  // サーボモータパラメータの最大値
};

const struct pinMotor pinWheel = {10,7,8,6};// 車輪駆動モータピン番号
const struct pinMotor pinGun   = { 9,3,4,5};// 砲台駆動モータピン番号
const int pinPhotoRef = 11;		    // フォトリフレクタピン番号

struct pmMotor pmWheel;
struct pmMotor pmGun;

// 車輪用境界パラメータ定義
const struct pmBoundMotor pmBoundWheel = {100,10,2,-150,150,78,102};

// 砲台用境界パラメータ変数
const struct pmBoundMotor pmBoundGun   = {100, 0,1,   0,100,78,102};

// フォトリフレクタの状態フラグ
volatile int flagPhotoRef;

// 
volatile bool GunOff;

#ifdef _DEBUG_
#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define SERIAL_PRINT(...) 
#define SERIAL_PRINTLN(...) 
#endif

//---------------------------------------------------
// 砲台用DCモータ停止チェック関数
// (フォトリフレクタ信号の立下りエッジ取得)
//---------------------------------------------------
void readPhotoRef()
{
  int cur = digitalRead(pinPhotoRef);
  bool flag = false;
  
  if ( cur != flagPhotoRef ) {
    if ( cur == HIGH ) {
      flag = true;
    } 
    flagPhotoRef = cur;
  }
  
  GunOff = flag;
}
//---------------------------------------------------
// セットアップ関数
//---------------------------------------------------
void setup()
{
  Serial.begin(2400);
  
  pinMode(pinWheel.servoPin ,OUTPUT);
  pinMode(pinWheel.dcPin1 ,OUTPUT);
  pinMode(pinWheel.dcPin2 ,OUTPUT);
  srvWheel.attach(pinWheel.servoPin);

  pinMode(pinGun.servoPin ,OUTPUT);
  pinMode(pinGun.dcPin1 ,OUTPUT);
  pinMode(pinGun.dcPin2 ,OUTPUT);
  srvGun.attach(pinGun.servoPin);

  pmWheel.iMotor = 0;    // DCモータの初期値
  pmWheel.iServo = 90;   // サーボの初期値

  pmGun.iMotor = 0;    // DCモータの初期値
  pmGun.iServo = 90;   // サーボの初期値

  digitalWrite(pinWheel.dcPin1, LOW);
  digitalWrite(pinWheel.dcPin2, LOW);
  analogWrite(pinWheel.pmPin,pmWheel.iMotor);

  digitalWrite(pinGun.dcPin1, LOW);
  digitalWrite(pinGun.dcPin2, LOW);
  analogWrite(pinGun.pmPin,pmGun.iMotor);
  
  pinMode(pinPhotoRef ,INPUT_PULLUP);
  flagPhotoRef = digitalRead(pinPhotoRef);
  GunOff = false;
  // フォトリフレクタデータ取得関数を
  // タイマー割込み起動するように登録
  MsTimer2::set(100,readPhotoRef);
  MsTimer2::start();

  SERIAL_PRINTLN("done setup");
}

//---------------------------------------------------
// DCモータ駆動関数
//---------------------------------------------------
#define _USE_PWM_

#ifdef _USE_PWM_
// PWMピンを使う場合
void MotorDrive( int iIn1Pin, int iIn2Pin, int iPwmPin, int iMotor )
{
  if( iMotor == 0 ) { // 
    digitalWrite(iIn1Pin, LOW);
    digitalWrite(iIn2Pin, LOW);
    analogWrite(iPwmPin,iMotor);
  } 
  else if( 0 < iMotor ) {
    digitalWrite(iIn1Pin, HIGH);
    digitalWrite(iIn2Pin, LOW);
    analogWrite(iPwmPin,iMotor);
  } 
  else {
    digitalWrite(iIn1Pin, LOW);
    digitalWrite(iIn2Pin, HIGH);
    analogWrite(iPwmPin,-iMotor);
  }
}
#else
// PWMピンを使わない場合
void MotorDrive( int iIn1Pin, int iIn2Pin, int iPwmPin, int iMotor )
{
  analogWrite(iPwmPin,150);
  if( iMotor == 0 ) { // 
    digitalWrite(iIn1Pin, LOW);
    digitalWrite(iIn2Pin, LOW);
  } 
  else if( 0 < iMotor ) {
    analogWrite(iIn1Pin, iMotor);
    analogWrite(iIn2Pin, 0);
  } 
  else {
    analogWrite(iIn1Pin, 0);
    analogWrite(iIn2Pin, -iMotor);
  }
}
#endif

//---------------------------------------------------
// PS3コントローラー信号の受信
//---------------------------------------------------
bool ReadCmd(int* cmd)
{
  int bf_cmd[8];
  while( !Serial.available() );
  
  bool readf = false;
  
  do {
    bf_cmd[0] = Serial.read();
  } while( bf_cmd[0] != 0x80 );
    
    unsigned int sum = 0;
    for( int i=1; i<7; i++ ) {
      bf_cmd[i] = Serial.read();
//      SERIAL_PRINTLN(bf_cmd[i]);
      sum += bf_cmd[i];
    }
    bf_cmd[7] = Serial.read();
    sum &= 0x7F;
    /*
    SERIAL_PRINT("[7] ");
    SERIAL_PRINT(cmd[7]);
    SERIAL_PRINT(" ");
    SERIAL_PRINTLN(sum);
    */
    if ( bf_cmd[7] == sum ) {
      readf = true;
      for ( int i=0;i<8;i++ ) {
        cmd[i] = bf_cmd[i];
      }
    }
  return readf;
}
//---------------------------------------------------
// PS3コントローラー信号のデコード
//---------------------------------------------------
void decodeCmd(
   int *cmdStream, struct pmMotor *pmWheel, struct pmMotor *pmGun, 
   const struct pmBoundMotor *pmBoundWheel, const struct pmBoundMotor *pmBoundGun)
{
  if ( cmdStream[1] == 0 ) {
    switch(cmdStream[2]) {
      case 1: {	// ↑ボタン
        pmGun->iServo += pmBoundGun->incS;
        pmGun->iServo = min(pmBoundGun->maxS,pmGun->iServo);
        break;
      }
      case 2: {	// ↓ボタン
        pmGun->iServo -= pmBoundGun->incS;
        pmGun->iServo = max(pmBoundGun->minS,pmGun->iServo);
        break;
      }
      case 16: {// △ボタン
        pmGun->iMotor = pmBoundGun->maxM;
        break;
      }
      case 0: {	// アナログスティック
        SERIAL_PRINTLN(cmdStream[4]);
        /*
        if ( cmdStream[4] > 63 ) {
          pmWheel->iMotor = map(cmdStream[4],64,127,80,pmBoundWheel->maxM);
        } else {
          pmWheel->iMotor = map(cmdStream[4],0,63,pmBoundWheel->minM,-80);
        }
        */
        pmWheel->iMotor = map(cmdStream[4],0,127,pmBoundWheel->minM,pmBoundWheel->maxM);
        pmWheel->iServo = map(cmdStream[5],0,127,pmBoundWheel->minS,pmBoundWheel->maxS);
        break;
      }
      default: {// その他
      }
    } // end of switch
  } // end of if
}

//---------------------------------------------------
// メインループ関数
//---------------------------------------------------
void loop()
{
  int cmdStream[8];

  if ( ReadCmd(cmdStream) ) {				// PS3コントローラーから信号受信
    // PS3コントローラーから信号デコード
    decodeCmd(cmdStream,&pmWheel,&pmGun,&pmBoundWheel,&pmBoundGun);
  }

  // 後輪制御
  pmWheel.iMotor = constrain(pmWheel.iMotor,pmBoundWheel.minM,pmBoundWheel.maxM);
  MotorDrive(pinWheel.dcPin1,pinWheel.dcPin2,pinWheel.pmPin,pmWheel.iMotor);
  
  // 前輪制御
  pmWheel.iServo = constrain(pmWheel.iServo,pmBoundWheel.minS,pmBoundWheel.maxS);
  srvWheel.write(pmWheel.iServo);

  // 打ち出し制御
  if ( GunOff ) {
    pmGun.iMotor = 0;
  }
  pmGun.iMotor = constrain(pmGun.iMotor,pmBoundGun.minM,pmBoundGun.maxM);
  MotorDrive(pinGun.dcPin1,pinGun.dcPin2,pinGun.pmPin,pmGun.iMotor);
  
  // 砲台制御
  pmGun.iServo = constrain(pmGun.iServo,pmBoundGun.minS,pmBoundGun.maxS);
  srvGun.write(pmGun.iServo);

  SERIAL_PRINT("Wheel::");
  SERIAL_PRINT(pmWheel.iMotor);
  SERIAL_PRINT(", ");
  SERIAL_PRINT(pmWheel.iServo);
  SERIAL_PRINT(" Gun::");
  SERIAL_PRINT(pmGun.iMotor);
  SERIAL_PRINT(", ");
  SERIAL_PRINTLN(pmGun.iServo);

  delay(35);

}

