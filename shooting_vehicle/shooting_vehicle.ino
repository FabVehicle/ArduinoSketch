#include <MsTimer2.h>
#include <SoftwareSerial.h>
#include <Servo.h>

Servo srvWheel;
Servo srvGun;

// #define _DEBUG_

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

const struct pinMotor pinWheel = {10,7,8,6};// 車輪駆動モータピン番号
const struct pinMotor pinGun   = { 9,3,4,5};// 砲台駆動モータピン番号
const int pinPhotoRef = 11;		    // フォトリフレクタピン番号

struct pmMotor pmWheel;
struct pmMotor pmGun;

// 車輪用境界パラメータ定義
const struct pmBoundMotor pmBoundWheel = {100,10,2,-150,150,78,102,90};

// 砲台用境界パラメータ変数
const struct pmBoundMotor pmBoundGun   = {100, 0,2,   0,150,92,110,92};

// フォトリフレクタの状態フラグ
int flagPhotoRef;

// フォトリフレクタの立上がりエッジ検出フラグ
volatile bool GunOff;

// コマンドのデリミター
#define DELIMITER (",;")

// コマンド用のバッファ
#define READBUFFERSIZE (32)
char g_szReadBuffer[READBUFFERSIZE] = "";
int  g_iIndexChar = 0;

#define _DEBUG_
#ifdef _DEBUG_
#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define SERIAL_PRINT(...) 
#define SERIAL_PRINTLN(...) 
#endif

//---------------------------------------------------
// 砲台用DCモータ停止チェック関数
// (フォトリフレクタ信号の立上がりエッジ取得)
//---------------------------------------------------
void readPhotoRef()
{
  int cur = digitalRead(pinPhotoRef);
  
  if ( cur != flagPhotoRef ) {
    if ( cur == HIGH ) {
      GunOff = true;
      digitalWrite(3, LOW);
      digitalWrite(4, LOW);
      analogWrite(5,0);
    } 
    flagPhotoRef = cur;
  }
  
//  GunOff = flag;
}
//---------------------------------------------------
// セットアップ関数
//---------------------------------------------------
void setup()
{
  Serial.begin(9600);
  
  pinMode(pinWheel.servoPin ,OUTPUT);
  pinMode(pinWheel.dcPin1 ,OUTPUT);
  pinMode(pinWheel.dcPin2 ,OUTPUT);
  srvWheel.attach(pinWheel.servoPin);

  pinMode(pinGun.servoPin ,OUTPUT);
  pinMode(pinGun.dcPin1 ,OUTPUT);
  pinMode(pinGun.dcPin2 ,OUTPUT);
  srvGun.attach(pinGun.servoPin);

  pmWheel.iMotor = 0;   				// DCモータの初期値
  pmWheel.iServo = pmBoundWheel.ctrS;   // サーボの初期値

  pmGun.iMotor = 0;    					// DCモータの初期値
  pmGun.iServo = pmBoundGun.ctrS;		// サーボの初期値

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
  MsTimer2::set(25,readPhotoRef);
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
// $CMD,コマンド(c|s|l),スロット値,ステアリング値;
// コマンド：c か s か l の文字
//           s ⇒  ストップ，c ⇒  モータ値設定，
//           l ⇒ ヘッドライト状態変更
// スロット値：-5～5までの整数
// ステアリング値：-5～5までの整数
//---------------------------------------------------
boolean analizeCommadLine(
   char* szLineString,
   struct pmMotor *pmWheel, struct pmMotor *pmGun, 
   const struct pmBoundMotor *pmBoundWheel, const struct pmBoundMotor *pmBoundGun)
{ 
  SERIAL_PRINTLN(szLineString);
  
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
  
  char* pszGunDig = ptr;
  ptr = strpbrk( ptr, DELIMITER );
  if ( NULL == ptr ) return false;
  *ptr = '\0'; ptr++;
  
  if ( ! strcmp(pszCommand,"S") ) {
    pmWheel->iMotor = 0;
    pmWheel->iServo = pmBoundWheel->ctrS;
  } else if ( ! strcmp(pszCommand,"C" ) ){
    if ( strlen(pszThrottle) != 0 ) {  // スロット値が含まれている場合
      int nThrottle = -atoi(pszThrottle);
      if ( nThrottle == 0 ) {
        pmWheel->iMotor = 0;
      } else if ( nThrottle > 0 ) {
        pmWheel->iMotor = pmBoundWheel->incM * (nThrottle-1) + pmBoundWheel->ofsM;
      } else {
      	pmWheel->iMotor = pmBoundWheel->incM * (nThrottle+1) - pmBoundWheel->ofsM;
      }
    }
    if ( strlen(pszSteering) != 0 ) {  // ステアリング値が含まれている場合
      pmWheel->iServo = pmBoundWheel->ctrS + ( pmBoundWheel->incS * atoi(pszSteering) );
    }
    if ( strlen(pszGunDig) != 0 ) {
      pmGun->iServo = pmBoundGun->ctrS + ( pmBoundGun->incS * atoi(pszGunDig) );
    }
  } else if ( ! strcmp(pszCommand,"L" ) ) {
	pmGun->iMotor = pmBoundGun->maxM;
  }
  
  return true;
}

//---------------------------------------------------
// メインループ関数
//---------------------------------------------------
void loop()
{
  char szLineString[READBUFFERSIZE];
  if ( ! readCmdString(g_szReadBuffer, READBUFFERSIZE, g_iIndexChar, szLineString,  READBUFFERSIZE) ) {
    return ;
  }
  if ( ! analizeCommadLine(szLineString, &pmWheel,&pmGun,&pmBoundWheel,&pmBoundGun) )  {
    return;
  }
  
  // 後輪制御
  pmWheel.iMotor = constrain(pmWheel.iMotor,pmBoundWheel.minM,pmBoundWheel.maxM);
  if ( abs(pmWheel.iMotor) < pmBoundWheel.ofsM ) 
    pmWheel.iMotor = 0;
  MotorDrive(pinWheel.dcPin1,pinWheel.dcPin2,pinWheel.pmPin,pmWheel.iMotor);
  
  // 前輪制御
  pmWheel.iServo = constrain(pmWheel.iServo,pmBoundWheel.minS,pmBoundWheel.maxS);
  srvWheel.write(pmWheel.iServo);

  // 打ち出し制御
  if ( GunOff ) {
    pmGun.iMotor = 0;
    GunOff = false;
  }
  pmGun.iMotor = constrain(pmGun.iMotor,pmBoundGun.minM,pmBoundGun.maxM);
  MotorDrive(pinGun.dcPin1,pinGun.dcPin2,pinGun.pmPin,pmGun.iMotor);
  
  // 砲台制御
  pmGun.iServo = constrain(pmGun.iServo,pmBoundGun.minS,pmBoundGun.maxS);
  srvGun.write(pmGun.iServo);

  char str_log[256];
  sprintf(str_log,"%d  GunOff = %d   : Wheel:: %d, %d  Gun:: %d, %d",
          flagPhotoRef, GunOff, 
          pmWheel.iMotor, pmWheel.iServo, pmGun.iMotor, pmGun.iServo);
  SERIAL_PRINTLN(str_log);

  
//  delay(35);

}
