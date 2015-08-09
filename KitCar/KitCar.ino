#include <DCMotor.h>
#include <Servo.h>

Servo myservo;
DCMotor mydc;

// ピン番号
const int servoPin = 9;
const int dcPin1 = 7;
const int dcPin2 = 8;
const int pmPin = 6;


// モーター用パラメータ変数
int iMotor;        // DCモータ駆動用のパラメータ
int iServo;        // サーボーモータのパラメータ(角度)

//
int diS;

// モータ用パラメータ定数
const int incS =  6;     // 操作一回当りのサーボモータパラメータ変化量
const int minM = -100;   // DCモータパラメータの最小値
const int maxM = 150;    // DCモータパラメータの最大値
const int minS =  78;    // サーボモータパラメータの最小値
const int maxS = 102;    // サーボモータパラメータの最大値

int change;

//---------------------------------------------------
// セットアップ関数
//---------------------------------------------------
void setup()
{
  Serial.begin(9600);
  myservo.attach(servoPin);
  mydc.attach(dcPin1,dcPin2,pmPin,minM,maxM);
  
  diS    = 1;
  iMotor = maxM;    // DCモータの初期値
  iServo = 90;      // サーボの初期値
  change = 500;
}
//---------------------------------------------------
// メインループ関数
//---------------------------------------------------
void loop()
{  

  mydc.write(iMotor);
    
  iServo = constrain(iServo,minS,maxS);
  myservo.write(iServo);
  
  if ( iServo == 90 ) {
    change = 2000;
  } else {
    change = 1000;
  }  
  delay ( change );
  
  iServo += (incS * diS);
  if ( maxS < iServo ) {
    iServo -= 2*incS;
    diS *= -1;
  } else if ( minS > iServo ) {
    iServo += 2*incS;
    diS *= -1;
  }
  
}
