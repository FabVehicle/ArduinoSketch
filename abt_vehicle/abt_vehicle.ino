#include <Servo.h>
Servo myservo;

#define _DEBUG_

const int servoPin = 10;
const int dcPin1 = 5;
const int dcPin2 = 4;
const int pmPin = 6;

// モーター用パラメータ変数
int iMotor;        // DCモータ駆動用のパラメータ
int iServo;        // サーボーモータのパラメータ(角度)

// コマンドのデリミター
#define DELIMITER (",;")

// コマンド用のバッファ
#define READBUFFERSIZE (32)
char g_szReadBuffer[READBUFFERSIZE] = "";
int  g_iIndexChar = 0;

// モータ用パラメータ定数
const int ofsM = 100;    // DCモータオフセットパラメータ値(スロット値＝１の時の値)
const int incM = 10;     // DCモータパラメータ変化量
const int incS =  2;     // サーボモータパラメータ変化量
const int minM = -150;   // DCモータパラメータの最小値
const int maxM = 150;    // DCモータパラメータの最大値
const int minS =  78;    // サーボモータパラメータの最小値
const int maxS = 102;    // サーボモータパラメータの最大値

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
  Serial.begin(9600);
  pinMode(servoPin ,OUTPUT);
  pinMode(dcPin1 ,OUTPUT);
  pinMode(dcPin2 ,OUTPUT);

  myservo.attach(servoPin);

  iMotor = 0;    // DCモータの初期値
  iServo = 90;   // サーボの初期値

  digitalWrite(dcPin1, LOW);
  digitalWrite(dcPin2, LOW);
  analogWrite(pmPin,iMotor);

}

//---------------------------------------------------
// コマンド取得関数
//---------------------------------------------------
boolean readCmdString( char* szReadBuffer, const int ciReadBuffer, int& riIndexChar,
                       char* szLineString, const int ciLineString)
{
  while( !Serial.available() );
  while ( true ) {
    char c = Serial.read();

    if ( -1 == c ) {
      break;
    } 
    if ( ';' == c ) {
      szReadBuffer[riIndexChar++] = c;
      szReadBuffer[riIndexChar]   = '\0';
      strncpy( szLineString, szReadBuffer, ciLineString-1 );
      szLineString[ciLineString-1] = '\0';
      riIndexChar = 0;
      return true;
    } 
    else if ( '\n' == c ) {
      ;
    } 
    else {
      if ( (ciReadBuffer-1) > riIndexChar ) {
        szReadBuffer[riIndexChar] = c;
        riIndexChar++;
      }
    }
  }
  return false;
}
//---------------------------------------------------
// コマンド解析関数
// $CMD,コマンド(c|s),スロット値,ステアリング値;
// コマンド：c か s の文字
//           s ⇒　ストップ，c ⇒　値設定
// スロット値：-5～5までの整数
// ステアリング値：-5～5までの整数
//---------------------------------------------------
boolean analizeCommadLine(char* szLineString, int& iMotor, int& iServo)
{  
  if ( 0 != strncmp("$CMD", szLineString, 4 ) ) {
    return false;
  }
  
  char* ptr = szLineString;
  ptr = strpbrk( ptr, DELIMITER );
  if ( NULL == ptr ) return false;
  *ptr = '\0'; ptr++;
  
  char* pszCommand = ptr;
  ptr = strpbrk( ptr, DELIMITER );
  if ( NULL == ptr ) return false;
  *ptr = '\0'; ptr++;
  
  char* pszThrottle = ptr;
  ptr = strpbrk( ptr, DELIMITER );
  if ( NULL == ptr ) return false;
  *ptr = '\0'; ptr++;
  
  char* pszSteering = ptr;
  ptr = strpbrk( ptr, DELIMITER );
  if ( NULL == ptr ) return false;
  *ptr = '\0'; ptr++;
  
  if ( ! strcmp(pszCommand,"s") ) {
    iMotor = 0;
    iServo = 90;
  } else {
    if ( strlen(pszThrottle) != 0 ) {  // スロット値が含まれている場合
      int nThrottle = atoi(pszThrottle);
      if ( nThrottle == 0 ) {
        iMotor = 0;
      } else if ( nThrottle > 0 ) {
        iMotor = incM * (nThrottle-1) + ofsM;
      } else {
        iMotor = incM * (nThrottle+1) - ofsM;
      }
    } 
    if ( strlen(pszSteering) != 0 ) {  // ステアリング値が含まれている場合
      iServo = 90 - ( incS * atoi(pszSteering) );
    }
  }
  
  return true;
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
// メインループ関数
//---------------------------------------------------
void loop()
{
  char szLineString[READBUFFERSIZE];
  if ( ! readCmdString(g_szReadBuffer, READBUFFERSIZE, g_iIndexChar, szLineString,  READBUFFERSIZE) ) {
    return ;
  }
  
  for( int i=0; i<READBUFFERSIZE; i++ ) {
    SERIAL_PRINT(szLineString[i]);
  }
  SERIAL_PRINTLN();

  if ( ! analizeCommadLine(szLineString, iMotor, iServo) )  {
    return;
  }

  iMotor = constrain(iMotor,minM,maxM);
  MotorDrive(dcPin1,dcPin2,pmPin,iMotor);
  
  iServo = constrain(iServo,minS,maxS);
  myservo.write(iServo);

  SERIAL_PRINT(iMotor);
  SERIAL_PRINT(", ");
  SERIAL_PRINTLN(iServo);

 delay(100);
}



