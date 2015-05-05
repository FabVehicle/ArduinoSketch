#include <Servo.h>
#include "enum_stat.h"

Servo srvWheel;

const int servoPin = 10;
const int dcPin1 = 5;
const int dcPin2 = 4;
const int pmPin = 6;
const int rheadlightPin = 12;
const int lheadlightPin = 13;

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
  int ctrS;  // サーボモータセンター値
};

// ヘッドライト変数
int LightStat;

const struct pinMotor pinWheel = {servoPin,dcPin1,dcPin2,pmPin};// 車輪駆動モータピン番号
struct pmMotor pmWheel;

// 車輪用境界パラメータ定義
const struct pmBoundMotor pmBoundWheel = {100,10,2,-150,150,88-12,88+12,88};

// PS3コントローラの状態変数
KeyStat Stat;

#ifdef _DEBUG_
#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define SERIAL_PRINT(...) 
#define SERIAL_PRINTLN(...) 
#endif

//---------------------------------------------------
// セットアップ関数
//---------------------------------------------------
void setup()
{
  Serial.begin(2400);

  pinMode(pinWheel.servoPin ,OUTPUT);
  pinMode(pinWheel.dcPin1 ,OUTPUT);
  pinMode(pinWheel.dcPin2 ,OUTPUT);
  pinMode(rheadlightPin, OUTPUT);
  pinMode(lheadlightPin, OUTPUT);

  srvWheel.attach(pinWheel.servoPin);

  pmWheel.iMotor = 0;					// DCモータの初期値
  pmWheel.iServo = pmBoundWheel.ctrS;	// サーボの初期値

  digitalWrite(pinWheel.dcPin1, LOW);
  digitalWrite(pinWheel.dcPin2, LOW);
  analogWrite(pinWheel.pmPin,pmWheel.iMotor);

  LightStat = LOW;
  digitalWrite(rheadlightPin,LightStat);
  digitalWrite(lheadlightPin,LightStat);

  Stat = ksNeutral;

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
  unsigned int cnt = 1;
  do {
    bf_cmd[cnt] = Serial.peek();
    if ( bf_cmd[cnt] == 0x80 ) {
      // 電文のスタートコードの場合は処理を終了(エラー状態)
      break;
    }

    bf_cmd[cnt] = Serial.read();
    if ( bf_cmd[cnt] == -1 ) 
      continue;

    if ( cnt < 7 )
      sum += bf_cmd[cnt];
    cnt ++;
  } while ( cnt < 8 );

  sum &= 0x7F;

  char str_buf[256];
  sprintf(str_buf,"CMD: %x, %x, %x, %x, %x, %x, %x, %x, SUM: %x  (cnt=%d)",
                   bf_cmd[0],bf_cmd[1],bf_cmd[2],bf_cmd[3],
                   bf_cmd[4],bf_cmd[5],bf_cmd[6],bf_cmd[7], sum, cnt);
  SERIAL_PRINTLN(str_buf);

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
   int *cmdStream, struct pmMotor *pmWheel,
   const struct pmBoundMotor *pmBoundWheel)
{
  if ( cmdStream[1] == 0 ) {
    if ( cmdStream[2] == 0 ) { // アナログスティック
        if ( cmdStream[3] == 0x40 && cmdStream[4] == 0x40 
             && cmdStream[5] == 0x40 && cmdStream[6] == 0x40 ) {
			// 何も押されていない状態
            Stat = ksNeutral;
        } 
        pmWheel->iMotor = map(cmdStream[4],0,127,pmBoundWheel->minM,pmBoundWheel->maxM);
        pmWheel->iServo = map(cmdStream[5],0,127,pmBoundWheel->minS,pmBoundWheel->maxS);

    } else if ( Stat == ksNeutral ) {
      switch(cmdStream[2]) {
        case 1: {	// ↑ボタン
          Stat = ksUpArrow;
          break;
        }
        case 2: {	// ↓ボタン
          Stat = ksDwArrow;
          break;
        }
        case 3: {	// Startボタン
          Stat = ksNeutral;						// 強制的にニュートラルに戻す
          pmWheel->iMotor = 0;   				// DCモータの初期値
          pmWheel->iServo = pmBoundWheel->ctrS;  // サーボの初期値
          break;
        }
        case 4: {	// →ボタン
          Stat = ksRgArrow;
          break;
        }
        case 8: {	// ←ボタン
          Stat = ksLfArrow;
          break;
        }
        case 16: {// △ボタン
          if ( LightStat == LOW ) 
            LightStat = HIGH;
          else 
            LightStat = LOW;
          Stat = ksTriangle;
          break;
        }
        case 32: {// ×ボタン
          pmWheel->iMotor = 0;
          Stat = ksCross;
          break;
        }
        case 12: { // Selectボタン
          break;
        }
        default: {// その他
        }
      } // end of switch
    } // end of if cmdStream[2]
  } // end of if cmdStream[1]
}

//---------------------------------------------------
// メインループ関数
//---------------------------------------------------
void loop()
{

  int cmdStream[8];

  if ( ReadCmd(cmdStream) ) {	// PS3コントローラーから信号受信
    // PS3コントローラーから信号デコード
    decodeCmd(cmdStream,&pmWheel,&pmBoundWheel);
  }

  pmWheel.iMotor = constrain(pmWheel.iMotor,pmBoundWheel.minM,pmBoundWheel.maxM);
  MotorDrive(pinWheel.dcPin1,pinWheel.dcPin2,pinWheel.pmPin,pmWheel.iMotor);
  
  pmWheel.iServo = constrain(pmWheel.iServo,pmBoundWheel.minS,pmBoundWheel.maxS);
  srvWheel.write(pmWheel.iServo);

  digitalWrite(rheadlightPin,LightStat);
  digitalWrite(lheadlightPin,LightStat);

  SERIAL_PRINT(pmWheel.iMotor);
  SERIAL_PRINT(", ");
  SERIAL_PRINTLN(pmWheel.iServo);

}


