#include <Servo.h>

Servo srvWheel;

#define _DEBUG_

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

#if 0
// モーター用パラメータ変数
int iMotor;        // DCモータ駆動用のパラメータ
int iServo;        // サーボーモータのパラメータ(角度)

// コマンドのデリミター
#define DELIMITER (",;")

// コマンド用のバッファ
#define READBUFFERSIZE (32)
char g_szReadBuffer[READBUFFERSIZE] = "";
int  g_iIndexChar = 0;

#endif

// ヘッドライト変数
int LightStat;

#if 0
// モータ用パラメータ定数
const int ofsM = 100;    // DCモータオフセットパラメータ値(スロット値＝１の時の値)
const int incM = 10;     // DCモータパラメータ変化量
const int incS =  2;     // サーボモータパラメータ変化量
const int minM = -150;   // DCモータパラメータの最小値
const int maxM = 150;    // DCモータパラメータの最大値
const int ctrM = 88;     //
const int minS = ctrM-12;// サーボモータパラメータの最小値
const int maxS = ctrM+12;// サーボモータパラメータの最大値
#endif

const struct pinMotor pinWheel = {servoPin,dcPin1,dcPin2,pmPin};// 車輪駆動モータピン番号
struct pmMotor pmWheel;

// 車輪用境界パラメータ定義
const struct pmBoundMotor pmBoundWheel = {100,10,2,-150,150,88-12,88+12,88};


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
  analogWrite(pinWheel.pmPin,iMotor);

  LightStat = LOW;
  digitalWrite(rheadlightPin,LightStat);
  digitalWrite(lheadlightPin,LightStat);

  SERIAL_PRINTLN("done setup");
}
#if 0
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
    iServo = ctrM;
  } else if ( ! strcmp(pszCommand,"c" ) ){
    if ( strlen(pszThrottle) != 0 ) {  // スロット値が含まれている場合
      int nThrottle = -atoi(pszThrottle);
      if ( nThrottle == 0 ) {
        iMotor = 0;
      } else if ( nThrottle > 0 ) {
        iMotor = incM * (nThrottle-1) + ofsM;
      } else {
        iMotor = incM * (nThrottle+1) - ofsM;
      }
    } 
    if ( strlen(pszSteering) != 0 ) {  // ステアリング値が含まれている場合
      iServo = ctrM + ( incS * atoi(pszSteering) );
    }
  } else if ( ! strcmp(pszCommand,"l" ) ) {
      if ( LightStat == LOW ) 
        LightStat = HIGH;
      else 
        LightStat = LOW;
  }
  
  return true;
}
#endif
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
          pmWheel->iServo = pmBoundWheel.ctrS;  // サーボの初期値
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
    decodeCmd(cmdStream,&pmWheel,&pmGun,&pmBoundWheel,&pmBoundGun);
  }

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

  digitalWrite(rheadlightPin,LightStat);
  digitalWrite(lheadlightPin,LightStat);

  SERIAL_PRINT(iMotor);
  SERIAL_PRINT(", ");
  SERIAL_PRINTLN(iServo);

 delay(100);
}



