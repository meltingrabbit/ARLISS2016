#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// #########################################
// 関数プロトタイプ宣言
// #########################################
// ミッション遂行系
void InitializeAll();
void SetMissionStatus();
void CheckAndCalibrateSensor();
void JudgeLanding();
int JudgeLandingLightMode();
void JudgeLandingGpsLightMode();
void OpenCasing();
void EscapeLandingPoint();
void RunUpNearGoal(double residualError);
void ApproachGoal(double residualError);
void FinalApproachToGoal(double residualError);
void AvoidStuck();
int ReceiveCmd();
void UpdateEeprom(char str);
int SplitMissionSequence(char *string, char *separator);
// モーター系
void InitializeMoter();
void CheckMoter();
int SetRoverAngle(double targetRad, int maxCount);
void GoStraight(double targetRad, unsigned long msec);
void SpinRover(double LorR, unsigned long msec);
void SetMoter(int l1, int l2, int r1, int r2);
void BurstGoAhead(long msec);
void BurstGoBack(long msec);
void BurstSpin(double LorR, unsigned long msec);
int RecoverRoverPosture(int maxTryCount);						// #####################
// 光系
void InitializeLight();
int GetLight();
int GetLightNTimes(int n);
// 地磁気系
int InitializeMagnet();
int ExecuteMagnetCalibration(int magnetCalError);
double GetMagnetRad();
double GetMagnetRadNTimes(int n);
int MagnetCalibration();
void MagnetCalibrationPriorityQueue(int *arr, int arrLen);
int MagnetIntMax(int a, int b);
int GetRandom1D();
// GPS系
int InitializeGps();
void SendUnable(int datatype);
char ChecksumCalc(char *dt);
int GetGpsData();
int ClearGetGpsData();
void CalcDistanceDirection();
long GpsLL2Deg(char *ddmmStr, char *mmmmStr);
long CutStr2Long(char *strNum, int digit);
int Split(char *string, char *separator);
int SplitLong(char *string, char *separator);
void ClearGpsRxBuf();
// 加速度センサ系
void GetAcc();
int IsRoverUpsideDown();				// #########################
// カメラ系
int InitializeCamera();
void CapturePic(int isCacheOn);
void ClearCameraRxBuf();
void SendCameraCmd(char cmd[], int cmd_len);
int PreCapture();
int Capture();
void GetPicData();
// ブザー系
void ToneAlert();
// 通信系
void XbeeGps();
void XbeeLight();
// Log系
int InitializeSd();
void SdOpenFailed();
void CacheOn();
int CacheOff();
int MoveFullCacheToSd();
int LogCache(const char *string);
int LogSd(const char *string);
int LogSta(const char *string);
int LogComStrDbl(const char *string, double dbl);
int LogComStrLng(const char *string, long lng);
int LogComStrDblLng(const char *string, double dbl, long lng);
int LogLit(int light);
int LogMagRaw();
int LogMagAvg(int n, double rad);
int LogMagCal(int maxRange, int *xMaxs, int *yMaxs, int *xMins, int *yMins, int arrLen);
int LogGps();
int LogAcc();
int LogPho(int ok);
// I2C
unsigned char I2cRead(unsigned char regAddress);
void I2cWrite(unsigned char regAddress, unsigned char data);
// 変換系
double NormalizeRad(double input);
double NormalizeRadSym(double input);
double Deg2Rad(double deg);
double Rad2Deg(double rad);
// double ddmm2dddd(double ddmm);
unsigned long Meter2Msec(double meter);
double Msec2Meter(int msec);
unsigned long Sec2Msec(double Sec);
unsigned long LongPow(int base, int exponent);
void MyDelay(long msec);
void MyPrintlnRemainingTime(unsigned long startTimeMsec, long timeLimitSec);
// #########################################

// #########################################
// 定数定義
// #########################################

#define DEBUG 1
#define SENSOR_CEHCK 1				// 初期化の際，センサー，モーターチェックをするか
#define TIRE_TYPE 2					// 1 : アストロペンギン, 2 : 中空稠密
#define IS_XBEE_TXRX0 0				// 1 TXRX0
#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート

// スイッチ
// #define PIN_SWITCH 30
#define PIN_SWITCH 45

// MOSFET
// #define PIN_FET_1 34
// #define PIN_FET_2 38
#define PIN_FET_1 42
#define PIN_FET_2 46

// モーター
#if TIRE_TYPE == 1
	// #define PIN_MOTER_LEFT1 6
	// #define PIN_MOTER_LEFT2 5
	// #define PIN_MOTER_RIGHT1 10
	// #define PIN_MOTER_RIGHT2 9
	#define PIN_MOTER_LEFT1 10
	#define PIN_MOTER_LEFT2 8
	#define PIN_MOTER_RIGHT1 13
	#define PIN_MOTER_RIGHT2 12
#elif TIRE_TYPE == 2
	#define PIN_MOTER_LEFT1 10
	#define PIN_MOTER_LEFT2 8
	#define PIN_MOTER_RIGHT1 13
	#define PIN_MOTER_RIGHT2 12
#else
	error detect
#endif

// CdS光センサ
#define ANALOG_PIN_CDS 1			// 光センサのアナログin

// 地磁気センサ
#define HMC5883L_ADDRESS 0x1E		// 7bit ADDRESS for I2C通信の素子識別アドレス

// GPS
#if TIRE_TYPE == 1
	#define GpsSerialBegin(baudrate) Serial2.begin(baudrate)
	#define GpsSerialRead() Serial2.read()
	#define GpsSerialAvailable() Serial2.available()
	#define GpsSerialPrint(gpsSendData) Serial2.print(gpsSendData)
#elif TIRE_TYPE == 2
	#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
	#define GpsSerialRead() Serial1.read()
	#define GpsSerialAvailable() Serial1.available()
	#define GpsSerialPrint(gpsSendData) Serial1.print(gpsSendData)
#else
	error detect
#endif


// 加速度センサ
#define PIN_ACC_X 7
#define PIN_ACC_Y 5
#define PIN_ACC_Z 3

// カメラ
#if TIRE_TYPE == 1
	#define CameraSerialBegin(baudrate) Serial1.begin(baudrate)
	#define CameraSerialRead() Serial1.read()
	#define CameraSerialAvailable() Serial1.available()
	#define CameraSerialPrint(string) Serial1.print(string)
	#define CameraSerialSetTimeout(time) Serial1.setTimeout(time)
	#define CameraSerialReadBytes(a, b) Serial1.readBytes(a, b)
#elif TIRE_TYPE == 2
	#define CameraSerialBegin(baudrate) Serial2.begin(baudrate)
	#define CameraSerialRead() Serial2.read()
	#define CameraSerialAvailable() Serial2.available()
	#define CameraSerialPrint(string) Serial2.print(string)
	#define CameraSerialSetTimeout(time) Serial2.setTimeout(time)
	#define CameraSerialReadBytes(a, b) Serial2.readBytes(a, b)
#else
	error detect
#endif
#define PIC_PKT_LEN 128    // data length of each read, dont set this too big because ram is limited
#define PIC_FORMAT 0x07    // VGA

// ブザー
// #define PIN_BUZZER 26
#define PIN_BUZZER 41
#define TONE_FREQUENCY_SD_PREPARING 440
#define TONE_FREQUENCY_SD_WRITE 880
#define TONE_FREQUENCY_SD_CAMERA 998

// 通信
#if IS_XBEE_TXRX0 == 0
	#define XbeeSerialBegin(baudrate) Serial3.begin(baudrate)
	#define XbeeSerialRead() Serial3.read()
	#define XbeeSerialAvailable() Serial3.available()
	#define XbeeSerialPrint(xbeeSendData) Serial3.print(xbeeSendData)
	#define XbeeSerialPrintln(xbeeSendData) Serial3.println(xbeeSendData)
	// MyPrintは安全ではない．引数に演算禁止！
	#define MyPrint(str) do {	\
		Serial.print(str);				\
		XbeeSerialPrint(str);			\
	} while(0)
	#define MyPrintln(str) do {	\
		Serial.println(str);				\
		XbeeSerialPrintln(str);			\
	} while(0)
#elif IS_XBEE_TXRX0 == 1
	#define MyPrint(str) Serial.print(str)
	#define MyPrintln(str) Serial.println(str)
	#define XbeeSerialRead() Serial.read()
	#define XbeeSerialAvailable() Serial.available()
#else
	error detect
#endif

// Log
//SPI通信で使われるピンは次のとおりです。
//Arduino Uno(ATmega168/328を搭載するボード): 10(SS)、11(MOSI)、12(MISO)、>13(SCK)
//Arduino Leonardo: ICSP4(MISO)、ICSP1(MOSI)、ICSP3(SCK)
//Arduino Mega: 50(MISO)、51(MOSI)、52(SCK)、53(SS)
#define PIN_SD_SS 53

// #########################################

// #########################################
// Global変数定義
// #########################################

int INT_MAX   = 32767;
int INT_MAX_R = 30000;

// int Mission_Sequence = -1;								// 再起動用
// -1 初期値
// 0 通常
// 1 ロケットの中
// 2 ロケットの外
//

//着地判定
// long LIGHT_JUDGE_START_DELAY_SEC = 60;		// ロケット封入から，光判定までの空き時間
// long LIGHT_JUDGE_DELAY_SEC = 30;		//放出判定した場合、放出判定してから着地と判断するまでの時間
// long LIGHT_JUDGE_TIMEOUT_SEC = 600;
// long GPS_LIGHT_JUDGE_TIMEOUT_SEC = 0;		//単位は(秒) この二つのTIMEOUT_SECの和が、着地判定自体のタイムアウト時間
long LIGHT_JUDGE_START_DELAY_SEC = 10;		// ロケット封入から，光判定までの空き時間
long LIGHT_JUDGE_DELAY_SEC = 10;		//放出判定した場合、放出判定してから着地と判断するまでの時間
long LIGHT_JUDGE_TIMEOUT_SEC = 120;
long GPS_LIGHT_JUDGE_TIMEOUT_SEC = 0;		//単位は(秒) この二つのTIMEOUT_SECの和が、着地判定自体のタイムアウト時間


// モーター
int Moter_Powers[4];				// 現在のモーター出力を保存するための配列
double Adjust_Moter_Power = 0;		// モーターのパワーを動的に制御するため 主にApproachGoalでBurstするため
double DEFAULT_ADJUST_MOTER_POWER = 5.0 / 8.0;

// 光センサ
int Light_Raw = 0;
int LIGHT_CRITICAL = 400;		//しきい値
// int LIGHT_CRITICAL = 100;		//しきい値


// 地磁気センサ
const double MAGNET_DECLINATION_DEG = 7.0;					// 東京で西偏7度0分
int Magnet_X12 = 0;
int Magnet_Z12 = 0;
int Magnet_Y12 = 0;
double MAGNET_X_OFFSET = 0;
double MAGNET_Y_OFFSET = 0;
double MAGNET_XY_RATIO = 1;

// GPS
const long EARTH_RADIUS = 6378137;
#define GPS_RAW_ARR_LENGTH 30					// Gps_Raw_Arrの長さ
#define GPS_RAW_ARR_BUF_LENGTH 20				// Gps_Raw_Arrの値の長さ
char Gps_Raw_Arr[GPS_RAW_ARR_LENGTH][GPS_RAW_ARR_BUF_LENGTH];
char Gps_LongLong[2][GPS_RAW_ARR_BUF_LENGTH];
long Gps_Lat = 0;									//
long Gps_Long = 0;									//dd.mmmmmmにすること
double Gps_Height = 0;								//
long Gps_Time = 0;
double Gps_Distance = 0;							//センサと目的地間距離
double Gps_Radian = 0;								//センサから目的地までの方位
double Gps_Passing_Distance = 0;							//センサと中継地間距離
double Gps_Passing_Radian = 0;								//センサから中継地までの方位
int Gps_Status = 0;								// GetGpsDataが成功したか
// dd ddd dddにすること！安田講堂塔真ん中(D, D)=(1602.1, 4.48286)
//35.717900    139.765163
//const long TARGET_LATITUDE = 35717900;
//const long TARGET_LONGITUDE = 139765163;
// 工学部広場
// const long TARGET_LATITUDE = 35713704;
// const long TARGET_LONGITUDE = 139760225;


// 農グラ緯度経度
//     緯度（十進法）    経度（十進法）
// 北西    35717928    139759252
// 北東    35718081    139760397
// 南西    35717325    139759394
// 南東    35717500    139760542
// 中心    35717713    139759901
// 東側ペナルティーエリア北	35717963	139760428

// passingテスト点
// const long TARGET_LATITUDE = 35718075;
// const long TARGET_LONGITUDE = 139760221;
const long TARGET_LATITUDE = 35717953;
const long TARGET_LONGITUDE = 139759432;

long Passing_Latitude = 0;
long Passing_Longitude = 0;


// 農学部広場
//const long TARGET_LATITUDE = 35718101;
//const long TARGET_LONGITUDE = 139760037;

// 加速度センサ
long Acc_X = 0;
long Acc_Z = 0;
long Acc_Y = 0;

// カメラ
File My_Pic_File;
const byte CAMERA_ADDRESS = (0 << 5);
// const int buttonPin = 30;
unsigned long picTotalLen = 0;            // picture length
int picNameNum = 79998;
char PIC_NAME[] = "MD000000/PIC00000.JPG";

// ブザー

// 通信

// Log
// SDカードのクラッシュがこわすぎるので，スイッチがOFFの時はSD書き込みができない（処理が止まる）ようにする．
File MY_FILE;
char FLIE_NAME[] = "MD000000/LOG.CSV";
boolean Log_To_Sd = 1;					// SD書き込みか Cache書き込みか
boolean Is_Cache_Full = 0;				// Cacheの75%が書き込まれたらtrue
int Cache_Start_Address = 0;			// 現在のCache書き込み開始アドレス
int Cache_End_Address = 0;				// 現在のCache終了アドレス．次の文字は this+1 にはいる
unsigned int Cache_Size = 0;			// Cache済みサイズ
// const int MAX_EEPROM_ADDRESS = 4096;	// modとるために 4*2**10 にしてある．末尾は-1であることに注意
const int MAX_EEPROM_ADDRESS = 4090;	// 末尾は-1であることに注意．ステータス保存のためにキャッシュサイズ縮小

// ステータス保存用
const int STATUS_EEPROM_ADDRESS = 4092;
int Mission_Status = 0;
// a :  0 : 初期化前，通常起動
// d : 10 : ロケット内，放出，着地判定まち
// g : 20 : 放出判定後，着地待ち
// j : 30 : 着地判定後，ケーシング展開待ち
// m : 40 : ケーシング展開後，パラシュート危険区域離脱待ち
// q : 50 : パラシュート危険区域離脱後，ターゲットその1へ向けて進行中


// #########################################





void setup() {
	int countSwitch = 0;
	unsigned long time = 0;

	MyDelay(1000);
	Serial.begin(SERIAL_BAUDRATE);
#if IS_XBEE_TXRX0 == 0
	XbeeSerialBegin(SERIAL_BAUDRATE);
#endif
	pinMode(PIN_FET_1, OUTPUT);
	pinMode(PIN_FET_2, OUTPUT);
	digitalWrite(PIN_FET_1, LOW);
	digitalWrite(PIN_FET_2, LOW);

	// 全てのスイッチは，ONで抜ける．
	InitializeAll();

	// EEPROM.update(STATUS_EEPROM_ADDRESS, 'a');			// テスト用###############
	SetMissionStatus();

	// CacheOff();				// テスト用###############
	if (Mission_Status == 0) {
		CheckAndCalibrateSensor();
	}

	// ###########################
	// 初期化完了の音を鳴らしたい
	LogSta("Initialization Finish!");
	MyPrintln(F("IF"));
	for (int i = 0; i < 3; i++) {
		tone(PIN_BUZZER, 1320);
		MyDelay(80);
		noTone(PIN_BUZZER);
		MyDelay(400);
	}
	tone(PIN_BUZZER, 1320);
	MyDelay(1500);
	noTone(PIN_BUZZER);

	MyDelay(10);

	CacheOff();
	MyDelay(50);
	CacheOn();

	if (Mission_Status == 0) {
		// ボタンをONにして通過．
		while(1) {
			countSwitch = 0;
			MyPrintln(F("Waiting for starting..."));
			MyPrintln(digitalRead(PIN_SWITCH));
			for (int i = 0; i < 10; i++) {
				countSwitch += digitalRead(PIN_SWITCH);
				MyDelay(50);
			}
			if (countSwitch == 0) {
				break;
			}
		}

		MyPrintln(F("START"));
	}


}


void loop() {
	unsigned long time = 0;
	int count = 0;
	int numOfPassingPoint = 2;
	long passingLatitudes[] = {0, 0};
	long passingLongitudes[] = {0, 0};
	// 西側
	long passingLatitudes1[] = {35717631, 35717646};
	long passingLongitudes1[] = {139759357, 139759467};
	// 東側
	long passingLatitudes2[] = {35717697, 35717658};
	long passingLongitudes2[] = {139759798, 139759544};

	// for debug
	if (!(sizeof(passingLatitudes) == sizeof(passingLongitudes) && sizeof(passingLatitudes) == sizeof(passingLongitudes1) && sizeof(passingLatitudes) == sizeof(passingLongitudes2) && sizeof(passingLatitudes) == sizeof(passingLatitudes1) && sizeof(passingLatitudes) == sizeof(passingLatitudes2)  && sizeof(passingLatitudes) == 4 * numOfPassingPoint)) {
		MyPrintln(F("Passing Point Error..."));
		while(1) {

		}
	}

	if (Mission_Status == 0) {

		// ケーシング梱包待ち
		// コマンド打ち込み
		MyPrintln(F("Waiting for CasingOK Cmd..."));
		LogSta("WfCC");
		time = millis();
		int cmd = 0;
		while (1) {		// コマンド打ち込み待ち
			cmd = ReceiveCmd();
			count++;
			count = count % INT_MAX_R;
			if ( (count % 1) == 0 ) {
				MyPrintln(F("Waiting for CasingOK Cmd..."));
				ClearGetGpsData();
				LogGps();
				XbeeGps();
				GetLight();				// ログ用
			}
			if (cmd == 3) {
				LogSta("CCO");
				MyPrintln(F("CasingOK Cmd OK!"));
				MyPrint(F("LIGHT_JUDGE_START_DELAY_SEC : "));
				MyPrintln(LIGHT_JUDGE_START_DELAY_SEC);
				break;
			}
			MyDelay(100);
		}
		// MyDelay(5000);

		time = millis();
		LogSta("WfLJSDS");
		LogComStrLng("LJSDS", LIGHT_JUDGE_START_DELAY_SEC);
		while (millis() < time + Sec2Msec(LIGHT_JUDGE_START_DELAY_SEC) ) {			// ロケット封入から判定開始までのディレイ
			MyPrintlnRemainingTime(time, LIGHT_JUDGE_START_DELAY_SEC);
			MyPrintln(F("in LIGHT_JUDGE_START_DELAY_SEC"));
			GetLight();				// ログ用
			CapturePic(0);
			// GetGPSはCapturePicに内包
			XbeeGps();
			MyDelay(1000);
		}

		MyDelay(5000);
	}

	UpdateEeprom('d');

	CacheOff();
	if (Mission_Status < 15) {
		JudgeLanding();
	}

	UpdateEeprom('g');


	UpdateEeprom('j');

	CacheOn();
	if (Mission_Status < 35) {
		OpenCasing();
		OpenCasing();
		CacheOff();
		MyDelay(50);
	}

	UpdateEeprom('m');

	if (Mission_Status < 45) {
		CacheOn();
		MyPrintln(F("EscapeLandingPoint"));
		EscapeLandingPoint();
		CacheOff();
		MyDelay(50);
		MyPrintln(F("End EscapeLandingPoint"));
	}

	// 再起動組は，ここらで地磁気のキャリブレーション
	// 調整する
	if (Mission_Status != 0) {
		ExecuteMagnetCalibration(10);
	}

	UpdateEeprom('q');

	// 経由地点設定
	ClearGetGpsData();
	ClearGetGpsData();
	// とりあえず東西で
	// if (Gps_Lat < TARGET_LATITUDE) {			// 南北
	if (Gps_Long < TARGET_LONGITUDE) {			// 東西
		// 西側落下
		MyPrintln(F("NISHINISHINISHI"));
		LogSta("W");
		for (int i = 0; i < numOfPassingPoint; i++) {
			passingLatitudes[i] = passingLatitudes1[i];
			passingLongitudes[i] = passingLongitudes1[i];
		}
	} else {
		// 東側落下
		MyPrintln(F("HIGASHIHIGASHIHIGASHI"));
		LogSta("E");
		for (int i = 0; i < numOfPassingPoint; i++) {
			passingLatitudes[i] = passingLatitudes2[i];
			passingLongitudes[i] = passingLongitudes2[i];
		}
	}


	CacheOn();
	// 中継地点へ進む
	for (int i = 0; i < numOfPassingPoint; i++) {
		Passing_Latitude = passingLatitudes[i];
		Passing_Longitude = passingLongitudes[i];
		MyPrintln(F("RunUpNearGoal(3)"));
		RunUpNearGoal(3);
	}

	Passing_Latitude = TARGET_LATITUDE;
	Passing_Longitude = TARGET_LONGITUDE;

	MyPrintln(F("RunUpNearGoal(5)"));
	RunUpNearGoal(5);

	MyPrintln(F("ApproachGoal(1)"));
	ApproachGoal(1);

	MyPrintln(F("GOAL!"));
	LogSta("GOAL!");
	CapturePic(1);
	CapturePic(1);

	MyPrintln(F("Turn to GOAL!"));
	LogSta("TtG");
	// ローバー角度調整		40(maxCount)は要検討
	if ( SetRoverAngle(Gps_Radian, 25) < 23 ) {
		if (DEBUG) MyPrintln(F("SetRoverAngle OK!"));
	} else {
		if (DEBUG) MyPrintln(F("SetRoverAngle failed..."));
	}

	CapturePic(1);
	CapturePic(1);

	CacheOff();
	MyDelay(50);
	CacheOn();
	while (1) {
	}
}


// #########################################
// ミッション遂行系

// 初期化
void InitializeAll() {
	int countSwitch = 0;
	int error = 1;
	unsigned long time;
	char str;

	// ボタンが押されて，安定したら初期化スタート
	str = EEPROM.read( STATUS_EEPROM_ADDRESS );
	while(1) {
		countSwitch = 0;
		MyPrintln(F("Unstarted...."));
		MyPrintln(digitalRead(PIN_SWITCH));
		MyPrint(F("NOW EPPROM : "));
		MyPrint(str);
		MyPrintln(F("\n"));
		for (int i = 0; i < 10; i++) {
			countSwitch += digitalRead(PIN_SWITCH);
			MyDelay(50);
		}
		if (countSwitch == 0) {
			break;
		}
	}

	time = millis();
	while(1) {
		countSwitch = 0;
		MyPrintln(F("for EEPROM Reset...."));
		MyPrintln(digitalRead(PIN_SWITCH));
		for (int i = 0; i < 10; i++) {
			countSwitch += digitalRead(PIN_SWITCH);
			MyDelay(50);
		}
		if (countSwitch == 10) {
			EEPROM.update(STATUS_EEPROM_ADDRESS, 'a');
			MyPrintln(F("Change EEPROM!"));
			str = EEPROM.read( STATUS_EEPROM_ADDRESS );
			MyPrint(F("NOW EPPROM : "));
			MyPrint(str);
			MyPrintln(F("\n"));
			break;
		}

		if  (millis() > time + Sec2Msec(5) ) {
			break;
		}
	}

	LogSta("=========================");
	CacheOn();

	// LogSta("=========================");
	// LogSta("=========================");
	MyPrintln(F("InitializeAll"));
	LogSta("IA");
	// ほんとは，SwitchONはもっと前だけど，SDの初期化終わらないと書き込めないのよね....w
	// CacheOnなので大丈夫になった．
	LogSta("SO");
	MyPrintln(F("Switch ON!"));
	// 安全策としてもう一度
	// 起動直前のは，スイッチ前に発熱されると困るから
	pinMode(PIN_FET_1, OUTPUT);
	pinMode(PIN_FET_2, OUTPUT);
	digitalWrite(PIN_FET_1, LOW);
	digitalWrite(PIN_FET_2, LOW);
	// 次のトリガーのために，一度スイッチをOFFにする
	// 仮にミッション中に電源が瞬間的に落ちてロジックがリセットされても大丈夫なように
	// 全てのトリガーはスイッチがONの状態で通過する．
	MyPrintln(F("Please Switch OFF."));
	MyPrintln(F("Waiting for 3s..."));
	MyDelay(3000);

	// SD初期化
	error = InitializeSd();
	if (error == 0) {
		LogSta("ISf");
		MyPrintln(F("initialize SD failed!"));
		ToneAlert();
	} else {
		LogSta("ISO");
		MyPrintln(F("Initialize SD OK!"));
	}

	char missionDir[] = "MD000000/";
	long missionNo = 0;
	while ( (SD.exists( missionDir )) ) {
		MyPrint(missionDir);
		MyPrintln(F(" exist..."));
		missionNo++;
		for (int i = 0; i<6; i++) {
			missionDir[i+2] = (missionNo/LongPow(10,5-i)) % 10 + '0';
		}
		MyDelay(50);
	}
	SD.mkdir( missionDir );
	MyPrintln(F("missionDir : "));
	MyPrintln(missionDir);
	MyPrintln(F("OK"));
	for (int i = 0; i<8; i++) {
		FLIE_NAME[i] = missionDir[i];
		PIC_NAME[i] = missionDir[i];
	}
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		MyPrintln(F("SD miss"));
	}
	MY_FILE.print("string\n");
	MY_FILE.close();

	InitializeMoter();
	LogSta("IMO");
	MyPrintln(F("Initialize Moter OK!"));

	// 光センサ初期化
	InitializeLight();
	LogSta("ILO");
	MyPrintln(F("Initialize Light OK!"));

	// 地磁気初期化
	InitializeMagnet();
	LogSta("IMaO");
	MyPrintln(F("Initialize Magnet OK!"));

	// GPS初期化
	error = InitializeGps();
	if (error == 0) {
		LogSta("IGf");
		MyPrintln(F("Initialize GPS failed!"));
		ToneAlert();
	} else {
		LogSta("IGO");
		MyPrintln(F("Initialize GPS OK!"));
	}

	// カメラ初期化
	error = InitializeCamera();
	if (error == 0) {
		LogSta("ICf");
		MyPrintln(F("Initialize Camera failed!"));
		ToneAlert();
	} else {
		LogSta("ICO");
		MyPrintln(F("Initialize Camera OK!"));
	}

	MyPrintln(F("EIA"));
	LogSta("End InitializeAll");
	return;
}

// ステータス確認
void SetMissionStatus() {
	char str;
	char string[] = "a";
	str = EEPROM.read( STATUS_EEPROM_ADDRESS );
	string[0] = str;

	MyPrint(F("Mission_Status_Str : "));
	MyPrintln(str);

	if (str == 'a') {
		Mission_Status = 0;
	} else if (str == 'd') {
		Mission_Status = 10;
	} else if (str == 'g') {
		Mission_Status = 20;
	} else if (str == 'j') {
		Mission_Status = 30;
	} else if (str == 'm') {
		Mission_Status = 40;
	} else if (str == 'q') {
		Mission_Status = 50;
	} else {
		//
	}

	MyPrint(F("Mission_Status : "));
	MyPrintln(Mission_Status);
	LogSta("MSS");
	LogSta(string);
	LogComStrLng("MS", Mission_Status);
	return;
}

// Checkなど．正常起動時のみおこなう
void CheckAndCalibrateSensor() {
	int countSwitch = 0;
	MyDelay(1);
	CapturePic(1);
	// CheckMoter();
	MyDelay(1000);
	// 調整する
	ExecuteMagnetCalibration(10);

	// 地磁気，加速度，GPSセンサーチェック，光
	if (SENSOR_CEHCK) {
		double magnetRad = 0;

		MyPrintln(F("CheckingSencor"));
		LogSta("CS");

		// ボタンをONにして通過．
		while(1) {
			countSwitch = 0;

			// ######################
			// ここにセンサーチェック
			MyDelay(50);
			// MyDelay(1);
			ClearGetGpsData();
			double magnetRad = GetMagnetRad();
			MyPrint(F("magnet deg : "));
			MyPrint(Rad2Deg(magnetRad));
			MyPrint(F("\n"));

			MyPrint(F("GetRandom1D : "));
			MyPrint(GetRandom1D());
			MyPrint(F("\n"));

			MyPrint(F("Acc : "));
			GetAcc();
			MyPrint(Acc_X);
			MyPrint(F(","));
			MyPrint(Acc_Y);
			MyPrint(F(","));
			MyPrint(Acc_Z);
			MyPrint(F("\n"));

			MyPrint(F("light : "));
			MyPrint(analogRead(ANALOG_PIN_CDS));
			MyPrint(F("\n"));

			MyPrint(F("Switch : "));
			MyPrint(digitalRead(PIN_SWITCH));
			MyPrint(F("\n"));

			XbeeGps();
			LogGps();
			LogMagRaw();
			LogAcc();
			GetLight();

			for (int i = 0; i < 10; i++) {
				countSwitch += digitalRead(PIN_SWITCH);
				MyDelay(50);
			}
			if (countSwitch == 0) {
				break;
			}
		}

		MyPrintln(F("End CheckingSencor"));
		LogSta("ECS");

		MyPrintln(F("Please Switch OFF."));
		MyPrintln(F("Waiting for 3s..."));
		MyDelay(3000);
	}

	return;
}


// 着地判定
void JudgeLanding() {
	unsigned long time = 0;
	MyPrintln(F("JudgeLanding"));
	LogSta("JL");

	if (JudgeLandingLightMode() == 1) {
		time = millis();
		MyPrintln(F("JudgeLandingLightMode OK!"));
		LogSta("JLLMO");

		LogSta("WfLJDS");
		LogComStrLng("LJDS", LIGHT_JUDGE_DELAY_SEC);
		while (millis() < time + Sec2Msec(LIGHT_JUDGE_DELAY_SEC) ) {			//放出判定から着地判定までのディレイ
			MyPrintlnRemainingTime(time, LIGHT_JUDGE_DELAY_SEC);
			MyPrintln(F("in LIGHT_JUDGE_DELAY_SEC"));
			GetLight();				// ログ用
			CapturePic(0);
			// GetGPSはCapturePicに内包
			XbeeGps();
			MyDelay(2000);
		}
		return;			//LIGHT_JUDGE_DELAY_SEC秒後、着地判定
	}
	//光判定のタイムアウト後
	JudgeLandingGpsLightMode();

	MyPrintln(F("End JudgeLanding"));
	LogSta("EJL");
}

int JudgeLandingLightMode() {
	unsigned long time = millis();
	MyPrintln(F("JudgeLandingLightMode"));
	LogSta("JLLM");

	while(1) {
		if (GetLightNTimes(20) == 1) {
			unsigned long time2 = millis();
			while (millis() < time2 + Sec2Msec(3) ) {
				MyPrintln(F("Waiting for 3s in JudgeLandingLightMode"));
				GetLight();				// ログ用
				CapturePic(0);
				// GetGPSはCapturePicに内包
				XbeeGps();
				MyDelay(10);
			}
			if (GetLightNTimes(10) == 1) {			//安全のために3秒後にもう一回判定しておく
				MyPrintln(F("JudgeLandingLightMode OK END"));
				LogSta("JLLMOE");
				return 1;
			}
		}

		MyPrintlnRemainingTime(time, LIGHT_JUDGE_TIMEOUT_SEC);
		MyPrintln(F("in LIGHT_JUDGE_TIMEOUT_SEC"));
		if (millis() > time + Sec2Msec(LIGHT_JUDGE_TIMEOUT_SEC) ) {		//タイムアウト
			MyPrintln(F("JudgeLandingLightMode TimeOut"));
			LogSta("JLLMTO");
			return 0;
		}

		GetLight();				// ログ用
		CapturePic(0);
		// GetGPSはCapturePicに内包
		XbeeGps();
		MyDelay(10);
	}
}

void JudgeLandingGpsLightMode() {		//ver.3.04.01から三軸完全一致
	unsigned long time = millis();
	unsigned long lightTime = millis();
	const int sampleNum = 30;			// 判定に使うためのサンプリング回数
	const int sampleNumMin = 25;			// 判定に使うためのサンプリング最低回数
	const double rateJudge = 0.95;		// 判定基準	1以下	一致していれば着地判定
	long gpsLat = 0;
	long gpsLong = 0;
	double gpsHeight = 0;

	int countGpsData = 0;
	int countMatchedGpsData = 0;

	double rateMatchedGpsData = 0;

	MyPrintln(F("JudgeLandingGpsLightMode"));
	LogSta("JLGLM");

	while(1) {			//!!!gps判定ループ!!!

		if (GetLightNTimes(20) == 1) {
			for (int i = 0; i < 3; i++) {
				MyPrintln(F("Waiting for 3s in JudgeLandingGpsLightMode"));
				ClearGetGpsData();
				LogGps();
				XbeeGps();
				GetLight();				// ログ用
				MyDelay(1000);
			}
			if (GetLightNTimes(10) == 1) {			//安全のために30秒後にもう一回判定しておく
				LogSta("WfLJDS");
				LogComStrLng("LJDS", LIGHT_JUDGE_DELAY_SEC);
				lightTime = millis();
				while (millis() < lightTime + Sec2Msec(LIGHT_JUDGE_DELAY_SEC) ) {			//放出判定から着地判定までのディレイ
					MyPrintlnRemainingTime(lightTime, LIGHT_JUDGE_DELAY_SEC);
					MyPrintln(F("in LIGHT_JUDGE_DELAY_SEC"));
					ClearGetGpsData();
					LogGps();
					XbeeGps();
					GetLight();				// ログ用
					MyDelay(2000);
				}
				MyPrintln(F("JudgeLandingGpsLightMode Light OK END"));
				LogSta("JLGLMLOE");
				return;
			}
		}
		if (DEBUG) MyPrintln(F("LightJudge in GpsJudge Failed"));

		MyPrintlnRemainingTime(time, GPS_LIGHT_JUDGE_TIMEOUT_SEC);
		MyPrintln(F("in GPS_LIGHT_JUDGE_TIMEOUT_SEC"));
		if(millis() > time + Sec2Msec(GPS_LIGHT_JUDGE_TIMEOUT_SEC)) {			//タイムアウト
				if (DEBUG) MyPrintln(F("GpsJudgeTIMEOUT"));
				MyPrintln(F("JudgeLandingGpsLightMode TimeOut"));
				LogSta("JLGLMTO");
				return;
		}

		//初期化・再設定
		ClearGetGpsData();
		gpsLat = Gps_Lat;
		gpsLong = Gps_Long;
		gpsHeight = Gps_Height;
		countGpsData = 0;

		for (int i = 0; i < sampleNum; i++) {
			if (ClearGetGpsData() == 1) {
				if (Gps_Height == gpsHeight && Gps_Lat == gpsLat && Gps_Long == gpsLong ) {
					countMatchedGpsData++;
					if (DEBUG) MyPrintln(F("GpsData is matched"));
				} else {
					if (DEBUG) MyPrintln(F("GpsData is not matched"));
				}
				countGpsData++;
			}
			LogGps();
			XbeeGps();
			GetLight();				// ログ用
			MyDelay(50);
		}

		if(countGpsData >= sampleNumMin){		//!!!判定!!!
			rateMatchedGpsData = countMatchedGpsData * 1.0 / sampleNum;
			if(rateMatchedGpsData >= rateJudge){
				if (DEBUG) MyPrintln(F("GpsJudgeSuccess"));
				MyPrintln(F("JudgeLandingGpsLightMode GPS OK END"));
				LogSta("JLGLMGOE");
				return;
			} else {
				MyPrintln(F("GpsJudgeFailed"));
				LogSta("GJF");
			}
		} else {
			MyPrintln(F("GpsJudgeFailed(get GPS failed)"));
			LogSta("GJF(gGf)");
		}

		MyDelay(1000);
	}
}

void OpenCasing() {
	double magnetRad = 0;
	int getGpsData = 0;			//GetGpsData()の返り値保存用
	// unsigned long time = millis();
	double errorDirection;
	long cutTime = 3000;

	MyPrintln(F("OpenCasing"));
	LogSta("OC");

	MyDelay(10);
	tone(PIN_BUZZER, 1047);
	MyPrintln(F("FET1 ON"));
	LogSta("F1O");
	digitalWrite(PIN_FET_1, HIGH);
	MyDelay(cutTime);
	digitalWrite(PIN_FET_1, LOW);
	noTone(PIN_BUZZER);
	MyPrintln(F("FET1 OFF"));
	LogSta("F1Of");
	MyDelay(2000);
	tone(PIN_BUZZER, 659);
	MyPrintln(F("FET2 ON"));
	LogSta("F2O");
	digitalWrite(PIN_FET_2, HIGH);
	MyDelay(cutTime);
	digitalWrite(PIN_FET_2, LOW);
	noTone(PIN_BUZZER);
	MyPrintln(F("FET2 OFF"));
	LogSta("F2Of");
	MyDelay(2000);

	MyPrintln(F("End OpenCasing"));
	LogSta("End OpenCasing");
	return;
}

void EscapeLandingPoint() {
	double magnetRad = 0;
	int getGpsData = 0;			//GetGpsData()の返り値保存用
	unsigned long time = millis();
	double errorDirection;

	MyPrintln(F("EscapeLandingPoint"));
	LogSta("ELP");

	if (DEBUG) MyPrintln(F("OP : BurstGoAhead 5s"));
	BurstGoAhead(5000);
	MyDelay(20);

	// モーター動かした後にもう一度Fet
	OpenCasing();

	if (DEBUG) MyPrintln(F("OP : GoStraight 1, 2m"));
	magnetRad = NormalizeRadSym(GetMagnetRadNTimes(5));
	GoStraight(magnetRad, Meter2Msec(2));
	MyDelay(1000);

	if (DEBUG) MyPrintln(F("OP : GetGPS"));
	getGpsData = 0;
	time = millis();
	// バッファを消すためGPSは5回とる
	// ケーシングopen後なので，gpsスリープの恐れ．
	while(getGpsData < 5) {
		if(millis() > time + Sec2Msec(20)) {		//これで20秒間とれなかったらループから外れる...はず
			break;
		}
		getGpsData += ClearGetGpsData();
		LogGps();
	}
	if (DEBUG) {
		if (getGpsData < 3) {
			MyPrintln(F("OP : getGpsData is under 3"));
		} else {
			MyPrintln(F("OP : GetGpsData OK!"));
		}
		MyPrint(F("OP : Gps_Lat : "));
		MyPrintln(Gps_Lat);
		MyPrint(F("OP : Gps_Long : "));
		MyPrintln(Gps_Long);
		LogGps();
		XbeeGps();
	}

	// 方向距離取得
	CalcDistanceDirection();

	MyDelay(10);
	if (DEBUG) MyPrint(F("OP : Gps_Distance : "));
	if (DEBUG) MyPrintln(Gps_Distance);
	if (DEBUG) MyPrint(F("OP : Gps_Radian to Deg : "));
	if (DEBUG) MyPrintln( Rad2Deg(Gps_Radian) );

	errorDirection = NormalizeRadSym(Gps_Radian - magnetRad);
	if (errorDirection > 0) {
		magnetRad += (M_PI/2);
	} else {
		magnetRad += -(M_PI/2);
	}
	// ローバー角度調整		40(maxCount)は要検討
	if ( SetRoverAngle(magnetRad, 25) < 23 ) {
		if (DEBUG) MyPrintln(F("OP : SetRoverAngle OK!"));
	} else {
		if (DEBUG) MyPrintln(F("OP : SetRoverAngle failed..."));
	}

	if (DEBUG) MyPrintln(F("OP : GO P 2, 2m"));
	GoStraight(magnetRad, Meter2Msec(2));
	MyDelay(1000);

	MyPrintln(F("End EscapeLandingPoint"));
	LogSta("EELP");

	return;
}

// residualErrorは残り半径
void RunUpNearGoal(double residualError) {
	int getGpsData = 0;			//GetGpsData()の返り値保存用
	unsigned long time = millis();
	double goStraightMeter = 0;		// 直進する長さ メートル
	double magnetRad = 0;
	int count = 0;
	double preGpsDistance = 0;		// スタック検知用
	double deltaGpsDistance = 0;		// スタック検知用

	LogComStrDbl("RUNG", residualError);
	LogComStrLng("TLA", TARGET_LATITUDE);
	LogComStrLng("TLO", TARGET_LONGITUDE);
	LogComStrLng("PLa", Passing_Latitude);
	LogComStrLng("PLo", Passing_Longitude);

	while(1) {

		// GPS取得
		getGpsData = 0;
		time = millis();
		// バッファを消すためGPSは2回とる
		while(getGpsData < 2) {
			if(millis() > time + Sec2Msec(10)) {		//これで10秒間とれなかったらループから外れる...はず
				break;
			}
			getGpsData += ClearGetGpsData();
			LogGps();
		}
		XbeeGps();

		if (DEBUG) {
			if (getGpsData < 2) {
				MyPrintln(F("getGpsData is under 3"));
			} else {
				MyPrintln(F("GetGpsData OK!"));
			}
			MyPrint(F("Gps_Lat : "));
			MyPrintln(Gps_Lat);
			MyPrint(F("Gps_Long : "));
			MyPrintln(Gps_Long);
		}

		// 方向距離取得
		CalcDistanceDirection();

		// スタック検知
		// ゴール近傍ではやらない
		if (Gps_Passing_Distance > 6) {
			deltaGpsDistance = preGpsDistance - Gps_Passing_Distance;
			deltaGpsDistance = abs(deltaGpsDistance);
				if (DEBUG) MyPrint(F("deltaGpsDistance : "));
				if (DEBUG) MyPrintln(deltaGpsDistance);
				LogComStrDbl("dGD", deltaGpsDistance);
			if (deltaGpsDistance < 1.5) {
				magnetRad = GetMagnetRadNTimes(5);
				ToneAlert();
				if (DEBUG) MyPrintln(F("STUCK !!! HELP ME !!!"));
				LogSta("S");

				AvoidStuck();

				// GPS取得
				getGpsData = 0;
				time = millis();
				// バッファを消すためGPSは2回とる
				while(getGpsData < 2) {
					if(millis() > time + Sec2Msec(10)) {		//これで10秒間とれなかったらループから外れる...はず
						break;
					}
					getGpsData += ClearGetGpsData();
					LogGps();
				}
				XbeeGps();

				if (DEBUG) {
					if (getGpsData < 2) {
						MyPrintln(F("getGpsData is under 2"));
					} else {
						MyPrintln(F("GetGpsData OK!"));
					}
					MyPrint(F("Gps_Lat : "));
					MyPrintln(Gps_Lat);
					MyPrint(F("Gps_Long : "));
					MyPrintln(Gps_Long);
				}

				// 方向距離取得
				CalcDistanceDirection();
			}
		}

		preGpsDistance = Gps_Passing_Distance;

		MyDelay(10);
		if (DEBUG) MyPrint(F("Gps_Distance : "));
		if (DEBUG) MyPrintln(Gps_Distance);
		if (DEBUG) MyPrint(F("Gps_Radian to Deg : "));
		if (DEBUG) MyPrintln( Rad2Deg(Gps_Radian) );
		if (DEBUG) MyPrint(F("Gps_Passing_Distance : "));
		if (DEBUG) MyPrintln(Gps_Passing_Distance);
		if (DEBUG) MyPrint(F("Gps_Passing_Radian to Deg : "));
		if (DEBUG) MyPrintln( Rad2Deg(Gps_Passing_Radian) );

		// 終了判定
		if (Gps_Passing_Distance < residualError) {
			if (DEBUG) MyPrintln(F("Break RunUpNearGoal"));
			LogSta("BRUNG");
			MyDelay(1000);
			return;
		}

		// ローバー角度調整		40(maxCount)は要検討
		if ( SetRoverAngle(Gps_Passing_Radian, 25) < 23 ) {
			if (DEBUG) MyPrintln(F("SetRoverAngle OK!"));
		} else {
			if (DEBUG) MyPrintln(F("SetRoverAngle failed..."));
		}


		magnetRad = GetMagnetRadNTimes(5);
		LogMagAvg(5, magnetRad);
		if (DEBUG) MyPrint(F("NowRad to deg : "));
		if (DEBUG) MyPrintln( Rad2Deg( NormalizeRad( magnetRad ) ) );
		MyDelay(1000);

		// 進む
		goStraightMeter = Gps_Passing_Distance * 0.8;
		goStraightMeter = max(goStraightMeter, 2);
		goStraightMeter = min(goStraightMeter, 10);

		if (DEBUG) MyPrint(F("goStraightMeter : "));
		if (DEBUG) MyPrintln(goStraightMeter);
		LogComStrDbl("gSM", goStraightMeter);

		BurstGoAhead(1000);
		SetMoter(170,0,170,0);
		MyDelay(500);
		GoStraight(Gps_Passing_Radian, Meter2Msec(goStraightMeter));
		MyDelay(1000);

		// 反転回復
		RecoverRoverPosture(10);

		// 回転わだち回避
		if ((count % 3) == 0) {
			// BurstGoBack(1000);
			// MyDelay(1000);
			BurstSpin(1, 3000);
			MyDelay(1000);
			BurstSpin(-1, 3000);
			MyDelay(1000);
			// BurstGoAhead(1000);
			// MyDelay(1000);


		}

		count++;
		count = count % INT_MAX_R;

		MyDelay(50);
		CapturePic(1);
	}
}

// 最後は全てBurstで．
void ApproachGoal(double residualError) {
	int getGpsData = 0;			//GetGpsData()の返り値保存用
	unsigned long time = millis();
	double goStraightMeter = 0;		// 直進する長さ メートル
	double magnetRad = 0;
	int count = 0;

	LogComStrDbl("AG", residualError);
	LogComStrLng("TLA", TARGET_LATITUDE);
	LogComStrLng("TLO", TARGET_LONGITUDE);

	// 全部Burst
	Adjust_Moter_Power = 1;

	while(1) {
		// GPS取得
		getGpsData = 0;
		time = millis();
		// バッファを消すためGPSは2回とる
		while(getGpsData < 2) {
			if(millis() > time + Sec2Msec(10)) {		//これで10秒間とれなかったらループから外れる...はず
				break;
			}
			getGpsData += ClearGetGpsData();
			LogGps();
		}
		XbeeGps();

		if (DEBUG) {
			if (getGpsData < 2) {
				MyPrintln(F("getGpsData is under 2"));
			} else {
				MyPrintln(F("GetGpsData OK!"));
			}
			MyPrint(F("Gps_Lat : "));
			MyPrintln(Gps_Lat);
			MyPrint(F("Gps_Long : "));
			MyPrintln(Gps_Long);
		}

		// 方向距離取得
		CalcDistanceDirection();

		MyDelay(10);
		if (DEBUG) MyPrint(F("Gps_Distance : "));
		if (DEBUG) MyPrintln(Gps_Distance);
		if (DEBUG) MyPrint(F("Gps_Radian to Deg : "));
		if (DEBUG) MyPrintln( Rad2Deg(Gps_Radian) );

		// 終了判定
		if (Gps_Distance < residualError) {
			if (DEBUG) MyPrintln(F("Break ApproachGoal"));
			LogSta("BAG");
			MyDelay(1000);
			Adjust_Moter_Power = DEFAULT_ADJUST_MOTER_POWER;
			return;
		}

		// ローバー角度調整		40(maxCount)は要検討
		if ( SetRoverAngle(Gps_Radian, 25) < 23 ) {
			if (DEBUG) MyPrintln(F("SetRoverAngle OK!"));
		} else {
			if (DEBUG) MyPrintln(F("SetRoverAngle failed..."));
		}


		magnetRad = GetMagnetRadNTimes(5);
		LogMagAvg(5, magnetRad);
		if (DEBUG) MyPrint(F("NowRad to deg : "));
		if (DEBUG) MyPrintln( Rad2Deg( NormalizeRad( magnetRad ) ) );
		MyDelay(1000);

		// 進む
		goStraightMeter = Gps_Distance * 0.8 - 1;			// SetMoter(170,0,170,0);分だけ引く
		goStraightMeter = max(goStraightMeter, 1);
		goStraightMeter = min(goStraightMeter, 10);

		if (DEBUG) MyPrint(F("goStraightMeter : "));
		if (DEBUG) MyPrintln(goStraightMeter);
		LogComStrDbl("gSM", goStraightMeter);

		GoStraight(Gps_Radian, Meter2Msec(goStraightMeter));
		SetMoter(170,0,170,0);
		MyDelay(500);
		SetMoter(0,0,0,0);
		MyDelay(1000);

		// 回転わだち回避
		if ((count % 3) == 0) {
			BurstSpin(1, 3000);
			MyDelay(1000);
			BurstSpin(-1, 3000);
			MyDelay(1000);
		}

		count++;
		count = count % INT_MAX_R;

		MyDelay(50);
		CapturePic(1);
	}
}

void FinalApproachToGoal(double residualError) {

}

void AvoidStuck() {
	double magnetRad = 0;

	int exTime = 0 ;
	int flag = GetRandom1D();

	MyPrintln(F("AvoidStuck"));
	LogSta("AS");

	magnetRad = GetMagnetRadNTimes(5);

	if (flag < 7) {					//7:3で分岐 要調整
		MyPrintln(F("AvoidStuck Type1"));
		LogSta("AST1");
		exTime = 100 * GetRandom1D();
		BurstGoBack(4500 + exTime);
		MyDelay(1000);
		// 反転した姿勢を元に戻す
		BurstGoAhead(2000);
		MyDelay(500);
		SetMoter(170,0,170,0);
		MyDelay(500);
		SetMoter(0,0,0,0);
		MyDelay(1000);
		// まわる
		exTime = 100 * GetRandom1D();
		BurstSpin(1, 3000 + exTime);
		MyDelay(1000);
		exTime = 100 * GetRandom1D();
		BurstSpin(-1, 3000 + exTime);
		MyDelay(1000);
		double absTemp = 0;
		int count = 0;
		// while ( (absTemp < Deg2Rad(80.0)) && (count < 20) ) {			//########## 80
		while ( (absTemp < Deg2Rad(45.0)) && (count < 20) ) {			//########## 80
			BurstSpin(1, 50);
			absTemp = Gps_Radian - GetMagnetRad();
			absTemp = NormalizeRadSym(absTemp);
			absTemp = abs(absTemp);
			MyDelay(1000);
			count++;
		}

		exTime = 100 * GetRandom1D();
		BurstGoAhead(4000 + exTime);
		MyDelay(1000);

		exTime = 100 * GetRandom1D();
		BurstSpin(1, 3000 + exTime);
		MyDelay(1000);
		exTime = 100 * GetRandom1D();
		BurstSpin(-1, 3000 + exTime);
		MyDelay(1000);

	}else{
		MyPrintln(F("AvoidStuck Type2"));
		LogSta("AST2");
		exTime = 200 * GetRandom1D();
		BurstSpin(1, 1000 + exTime);
		MyDelay(1000);
		exTime = 200 * GetRandom1D();
		BurstSpin(-1, 1000 + exTime);
		MyDelay(1000);

		exTime = 100 * GetRandom1D();
		BurstGoAhead(4000 + exTime);
		MyDelay(1000);

	}

	// 姿勢修正
	MyDelay(500);
	BurstGoAhead(2000);
	MyDelay(500);
	SetMoter(170,0,170,0);
	MyDelay(500);
	SetMoter(0,0,0,0);
	MyDelay(1000);

	MyPrintln(F("End AvoidStuck"));
	LogSta("EAS");

}

int ReceiveCmd() {
	char serialInput[100];		// シリアル通信受信文字列格納用
	unsigned long time = millis();
	int ii = 0;
	int cmd = 0;
	if (DEBUG) MyPrintln(F("Now at the first of while of ReceiveCmd"));

	while(1) {
		// データ受信したとき
		if (XbeeSerialAvailable()) {
			serialInput[ii] = XbeeSerialRead();
			// 文字数が90以上 or 末尾文字
			if (ii > 90 || serialInput[ii] == ':') {
				// 末尾に終端文字の挿入
				serialInput[ii] = '\0';
				// 受信文字列を送信
				MyPrint(F("RawString : "));
				MyPrint(serialInput);
				MyPrint(F(" : END"));
				MyPrint(F("\n"));

				cmd = SplitMissionSequence(serialInput, ",");

				MyPrint(F("result : "));
				// MyPrint(F("cmd : "));
				MyPrint(cmd);
				MyPrint(F("\n"));

				return cmd;
				// カウンタの初期化
//				ii = 0;
//				MyDelay(1000);
			} else {
				ii++;
			}
		}
		MyDelay(10);
		if (millis() > time + Sec2Msec(10) ) {		 // タイムアウト
			return -1;
		}
	}
}

void UpdateEeprom(char str) {
	char string[] = "a";
	string[0] = str;
	EEPROM.update(STATUS_EEPROM_ADDRESS, str);
	MyPrint(F("EEPROM.update "));
	MyPrintln(str);
	LogSta("Eu");
	LogSta(string);
	return;
}


int SplitMissionSequence(char *string, char *separator) {
	char *endAdress;						// 処理する文字列末尾のアドレス
	char *startAdress;						// 切り取る文字列の先頭
	char *pointer;							// 処理する位置を保持
	char splitString[100];					// 切り取った文字 文字数の上限に注意
	int count = 0;							// カウンタ
	int flag = 0;							// 処理終了のため
	int result = -1;
	int stringLength = 0;

	endAdress = string + strlen(string);	// 文字列の末尾のポインタ．\0の位置
	startAdress = string;					// セパレーター探索スタート位置

	while(count < GPS_RAW_ARR_LENGTH) {
		pointer = strpbrk(startAdress, separator);		// セパレーターの検索 見つからなければNULL
		if (pointer == NULL) {							// セパレーターが発見できなかったらtrue
			// セパレーターが発見できなくても最後の文字列を処理する．
			if (flag) {
				break;
			}
			flag = 1;									// 次回LOOPから抜けれるように
		} else {
			*pointer = '\0';							// セパレーターをNULLに置換
			pointer++;									// 処理位置を次へ移動
//			*(pointer++) = 0;
		}

		strcpy(splitString, startAdress);				// 切り取った文字列をコピー って，まだ切り取ってないかw
		startAdress = pointer;							// 次切り取る文字列の先頭を更新

		if (!strcmp(splitString, "")) {					// strcmp 一致 → 0
			strcpy(splitString, "NULL");
		}

		// 表示
		MyPrint(count);
		MyPrint(F(" : "));
		MyPrint(splitString);
		MyPrint(F("\n"));
		stringLength = strlen(splitString);
		MyPrint(F("len "));
		MyPrint(F(" : "));
		MyPrint(stringLength);
		MyPrint(F("\n"));

		if (count == 0) {
			// if ( !(strcmp(splitString, "start")) ) {
			// 	MyPrint(F("start!!!\n"));
			// 	result = 1;
			// }
			// if ( !(strcmp(splitString, "casingok")) ) {
			// 	MyPrint(F("casingok!!!\n"));
			// 	result = 2;
			// }
			if ( stringLength >= 20 ) {
				MyPrint(F("casingok!!! -- len\n"));
				result = 3;
			}
		}

		count++;
		MyDelay(500);
	}

	return result;
}


// #########################################
// モーター系
void InitializeMoter() {
	Adjust_Moter_Power = DEFAULT_ADJUST_MOTER_POWER;
	// モーターのピン設定
	pinMode(PIN_MOTER_LEFT1, OUTPUT);
	pinMode(PIN_MOTER_LEFT2, OUTPUT);
	pinMode(PIN_MOTER_RIGHT1, OUTPUT);
	pinMode(PIN_MOTER_RIGHT2, OUTPUT);
	for (int i = 0; i < 4; i++) {
		Moter_Powers[i] = 0;
	}
	SetMoter(0,0,0,0);
	return;
}

void CheckMoter() {
	// モーター駆動テスト
	if (DEBUG) MyPrintln(F("Checking Moter..."));
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(255,0,0,0)"));
	SetMoter(255,0,0,0);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(0,255,0,0)"));
	SetMoter(0,255,0,0);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,255,0)"));
	SetMoter(0,0,255,0);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,255)"));
	SetMoter(0,0,0,255);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(255,0,255,0)"));
	SetMoter(255,0,255,0);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(0,255,0,255)"));
	SetMoter(0,255,0,255);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(255,0,0,255)"));
	SetMoter(255,0,0,255);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(250);
	if (DEBUG) MyPrintln(F("SetMoter(0,255,255,0)"));
	SetMoter(0,255,255,0);
	MyDelay(2000);
	if (DEBUG) MyPrintln(F("SetMoter(0,0,0,0)"));
	SetMoter(0,0,0,0);
	MyDelay(2000);

	return;
}

// maxCount は 角度調整回数の上限
int SetRoverAngle(double targetRad, int maxCount) {
	int dMSec = 0;				// 最小角度変更の際の回転時間(ミリ秒)
	if (TIRE_TYPE == 1) {
		dMSec = 50;				// アスペ
	} else {
		dMSec = 70;				// 中空稠密
	}
	double epsilonRad = Deg2Rad(10.0);	// 許容誤差 注意スべきは，許容誤差が最小角度変更より小さいと発振
	double dRad;
	int count = 0;
	double nowRad = NormalizeRadSym( GetMagnetRadNTimes(5) );

	MyPrintln(F("SetRoverAngle"));
	LogComStrDblLng("SRA", targetRad, maxCount);

	targetRad = NormalizeRadSym(targetRad);

	LogMagAvg(5, GetMagnetRadNTimes(5));

	// if (DEBUG) MyPrint(F("NowRad deg in SetRoverAngle : "));
	// if (DEBUG) MyPrintln( Rad2Deg(NormalizeRad(nowRad)) );

	dRad = NormalizeRadSym(targetRad - nowRad);
	while(count < maxCount && abs(dRad) > epsilonRad) {
		SpinRover(dRad, dMSec);
		count++;
		nowRad = NormalizeRadSym( GetMagnetRadNTimes(5) );

		// if (DEBUG) MyPrint(F("NowRad deg in SetRoverAngle : "));
		// if (DEBUG) MyPrintln( Rad2Deg(NormalizeRad(nowRad)) );

		dRad = NormalizeRadSym(targetRad - nowRad);
		MyDelay(500);
	}

	LogMagAvg(5, GetMagnetRadNTimes(5));

	MyPrintln(F("End SetRoverAngle"));
	LogComStrLng("ESRA", count);

	return count;
}

// msecはミリ秒
void GoStraight(double targetRad, unsigned long msec) {
	double gainP = 1.5;					//ここの値は適宜変える！正の値
	double gainI = 0.0001;
	double gainD = 0.01;
	double sumError = 0;
	double lastError = 0;
	double inputPid;
	double errorDirection;
	int count = 0;
	int loopTime = 100;
	double magnetRad = 0;
	const int modePid = 1;		//1のときpi制御、0の時p制御
	double controlRange = Deg2Rad(60.0);			// 片側を止めないで制御する範囲
	unsigned long startTime = millis();

	targetRad = NormalizeRadSym(targetRad);

	MyPrintln(F("GoStraight"));
	LogComStrDblLng("GS", targetRad, msec);

	while (millis() < startTime + msec) {

		// 始めっからXbeeはいらんのでこんな感じにした
		if ( (count % 600) == 300 ) {
			ClearGetGpsData();
			XbeeGps();
		}

		//magnetRad = NormalizeRadSym(GetMagnetRadNTimes(5));
		magnetRad = NormalizeRadSym(GetMagnetRad());
		errorDirection = NormalizeRadSym(targetRad - magnetRad);

		if(modePid == 1) {
			sumError += errorDirection*loopTime;
			// inputPid = gainP*errorDirection + gainI*sumError + gainD*(errorDirection - lastError)/loopTime;
			inputPid = gainP*errorDirection + gainI*sumError;
		} else {
			inputPid = gainP*errorDirection;
		}

		// if ( (count % 30) == 0 ) {
		// 	if (DEBUG) MyPrint(F("nowRad deg : "));
		// 	if (DEBUG) MyPrintln(Rad2Deg(magnetRad));
		// 	if (DEBUG) MyPrint(F("error Rad deg : "));
		// 	if (DEBUG) MyPrintln(Rad2Deg(errorDirection));
		// 	if (DEBUG) MyPrint(F("P : "));
		// 	if (DEBUG) MyPrintln(gainP*errorDirection, 6);
		// 	if (DEBUG) MyPrint(F("I : "));
		// 	if (DEBUG) MyPrintln(gainI*sumError, 6);
		// 	if (DEBUG) MyPrint(F("sumError : "));
		// 	if (DEBUG) MyPrintln(sumError, 6);
		// }


		if (0 < errorDirection && errorDirection < controlRange) {
			// if (DEBUG) MyPrintln(F("Turing RIGHT"));
			SetMoter(255,0, 255 * (M_PI - abs(inputPid)) / M_PI,0);
		} else if (-controlRange < errorDirection && errorDirection < 0) {
			// if (DEBUG) MyPrintln(F("Turing LEFT"));
			SetMoter(255 * (M_PI - abs(inputPid)) / M_PI,0, 255,0);		//負なので+
		} else if (errorDirection >= controlRange) {
			SetMoter(255,0,0,0);
		} else if (errorDirection <= -controlRange) {
			SetMoter(0,0,255,0);
		} else {
			SetMoter(255,0,255,0);
		}

		// if(gainI*sumError > M_PI/2 || gainI*sumError < -M_PI/2) {
		if(gainI / gainP * sumError > M_PI/2 || gainI / gainP * sumError < -M_PI/2) {
			if (DEBUG) MyPrintln(F("Reset I controller"));
			sumError = 0;
		}

		MyDelay(loopTime);
		lastError = errorDirection;
		count++;
		count = count % INT_MAX_R;
	}

	SetMoter(0,0,0,0);
	MyPrintln(F("End GoStraight"));
	LogSta("EGS");
	return;
}

// 正(1.0)で右，負(-1.0)で左
void SpinRover(double LorR, unsigned long msec) {
	MyPrintln(F("SpinRover"));
	LogComStrDblLng("SR", LorR, msec);
	if (LorR > 0) {
		SetMoter(255,0,0,255);
//		SetMoter(300,0,0,300);
	} else {
		SetMoter(0,255,255,0);
//		SetMoter(0,300,300,0);
	}
	MyDelay(msec);
	SetMoter(0,0,0,0);
	MyPrintln(F("End SpinRover"));
	LogSta("ESR");
}

void SetMoter(int l1, int l2, int r1, int r2) {

	Moter_Powers[0] = l1;
	Moter_Powers[1] = l2;
	Moter_Powers[2] = r1;
	Moter_Powers[3] = r2;

	// for 7.4 V
	l1 = int(l1 * Adjust_Moter_Power);
	l2 = int(l2 * Adjust_Moter_Power);
	r1 = int(r1 * Adjust_Moter_Power);
	r2 = int(r2 * Adjust_Moter_Power);

	// 安全に
	l1 = min(l1, 255);
	l2 = min(l2, 255);
	r1 = min(r1, 255);
	r2 = min(r2, 255);
	l1 = max(l1, 0);
	l2 = max(l2, 0);
	r1 = max(r1, 0);
	r2 = max(r2, 0);

	analogWrite(PIN_MOTER_LEFT1, l1);
	analogWrite(PIN_MOTER_LEFT2, l2);
	analogWrite(PIN_MOTER_RIGHT1, r1);
	analogWrite(PIN_MOTER_RIGHT2, r2);
	return;
}

void BurstGoAhead(long msec) {
	MyPrintln(F("BurstGoAhead"));
	LogComStrLng("BGA", msec);
	SetMoter(1000,0,1000,0);
	MyDelay(msec);
	SetMoter(0,0,0,0);
	MyPrintln(F("End BurstGoAhead"));
	LogSta("EBGA");
}

void BurstGoBack(long msec) {
	MyPrintln(F("BurstGoBack"));
	LogComStrLng("BGB", msec);
	SetMoter(0,1000,0,1000);
	MyDelay(msec);
	SetMoter(0,0,0,0);
	MyPrintln(F("End BurstGoBack"));
	LogSta("EBGB");
}

// 正(1.0)で右，負(-1.0)で左
void BurstSpin(double LorR, unsigned long msec) {
	MyPrintln(F("BurstSpin"));
	LogComStrDblLng("BS", LorR, msec);
	if (LorR > 0) {
		SetMoter(1000,0,0,1000);
	} else {
		SetMoter(0,1000,1000,0);
	}
	MyDelay(msec);
	SetMoter(0,0,0,0);
	MyPrintln(F("End BurstSpin"));
	LogSta("EBS");
}

int RecoverRoverPosture(int maxTryCount) {
	for(int i = 0; i < maxTryCount; i++) {
		if (IsRoverUpsideDown() == 0) {
			MyPrintln(F("RecoverRoverPosture OK"));
			LogSta("RRPO");
			return 1;
		}
		// 姿勢修正
		BurstGoAhead(2000);
		MyDelay(500);
		SetMoter(170,0,170,0);
		MyDelay(500);
		SetMoter(0,0,0,0);
		MyDelay(1500);
	}
	MyPrintln(F("RecoverRoverPosture failed..."));
	LogSta("RRPf");
	return 0;
}


// #########################################
// 光系
void InitializeLight() {
	analogReference( INTERNAL2V56 );
	// 最初の数回は値が不定
	for(int i = 0; i < 10; i++) {
		analogRead(ANALOG_PIN_CDS);
	}
}

int GetLight() {
	int light = analogRead(ANALOG_PIN_CDS);
	Light_Raw = light;
	LogLit(light);
	XbeeLight();
	if(light < LIGHT_CRITICAL){
		return 1;
	}
	else{
		return 0;
	}
}

//N回すべておっけーなら1
int GetLightNTimes(int n) {
	while(n > 0) {
		if (GetLight() == 0) {
			return 0;
		}
		n--;
		MyDelay(50);
	}
	return 1;
}


// #########################################
// 地磁気系

int InitializeMagnet() {
	Wire.begin();					// I2C通信起動
	I2cWrite(0x02,0x00);			// 地磁気センサの設定

	return 1;

	// 地磁気テスト用
	// while (1) {
	// 	MyPrint(GetMagnetRadNTimes(10));
	// 	MyPrint(F(", "));
	// 	MyPrint(GetMagnetRad());
	// 	MyPrint(F(", "));
	// 	MyPrint(Magnet_X12);
	// 	MyPrint(F(", "));
	// 	MyPrint(Magnet_Y12);
	// 	MyPrint(F(", "));
	// 	MyPrint(Magnet_Z12);
	// 	MyPrint(F(", "));
	// 	MyPrint(NormalizeRadSym(GetMagnetRad()));
	// 	MyPrint(F("\n"));

	// 	MyDelay(25);
	// }

	// return 1;
}


// 返り値はキャリブレーションの返り値
// 引数は地磁気キャリブレーションの許容エラー どんどん増やしていく
int ExecuteMagnetCalibration(int magnetCalError) {
	int magnetTry = 1;
	int magnetCal = 0;

	// ###########################
	// ###########################
	// 後で，キャリブレーションが無限ループにならないようにする
	// つまり，どんどん許容誤差を大きくするか，xMaxsのlenを大きくする

	while (1) {
		magnetCal = MagnetCalibration();
		MyPrint(F("! MAGNET_TRY_NUM : "));
		MyPrintln(magnetTry);
		MyPrint(F("! MAGNET_CAL_ERR : "));
		MyPrintln(magnetCal);
		MyPrint(F("! MAGNET_X_OFFSET : "));
		MyPrintln(MAGNET_X_OFFSET);
		MyPrint(F("! MAGNET_Y_OFFSET : "));
		MyPrintln(MAGNET_Y_OFFSET);
		MyPrint(F("! MAGNET_XY_RATIO : "));
		MyPrintln(MAGNET_XY_RATIO);

		// #####################
		// あとで返り値

		if (magnetCal < magnetCalError) {
			break;
		}
		magnetTry++;
		magnetCalError += 2;
	}
	return magnetCal;
}

double GetMagnetRad() {
	double magnetRad;
	unsigned char xMsb;
	unsigned char xLsb;
	unsigned char zMsb;
	unsigned char zLsb;
	unsigned char yMsb;
	unsigned char yLsb;
	double xDouble;
	double yDouble;

	// 順番がXZYなので注意！
	xMsb = I2cRead(0x03);
	xLsb = I2cRead(0x04);
	zMsb = I2cRead(0x05);
	zLsb = I2cRead(0x06);
	yMsb = I2cRead(0x07);
	yLsb = I2cRead(0x08);

	Magnet_X12 = xMsb;
	Magnet_X12 = ((Magnet_X12 << 8) & 0xFF00) | xLsb; //SHIFT & GET 12bit DATA WITH MSB
	Magnet_Y12 = yMsb;
	Magnet_Y12 = ((Magnet_Y12 << 8) & 0xFF00) | yLsb; //SHIFT & GET 12bit DATA WITH MSB
	Magnet_Z12 = zMsb;
	Magnet_Z12 = ((Magnet_Z12 << 8) & 0xFF00) | zLsb; //SHIFT & GET 12bit DATA WITH MSB

	//CONVERT TO DOUBLE (FOR atan2)
	xDouble = Magnet_X12 + MAGNET_X_OFFSET;
	yDouble = (Magnet_Y12 + MAGNET_Y_OFFSET) * MAGNET_XY_RATIO;

	magnetRad = atan2(yDouble,xDouble);
	magnetRad += Deg2Rad(MAGNET_DECLINATION_DEG);		// 偏角補正
	magnetRad = NormalizeRad(magnetRad);

	return magnetRad;
}

// 地磁気をN回取得し，平均
// あまりにもNが大きいと誤差が積もる
double GetMagnetRadNTimes(int n) {
	double magnetRad = 0;
	double xSum = 0;
	double ySum = 0;

	for(int i = 0; i < n; i++) {
		magnetRad = NormalizeRad( GetMagnetRad() );
		xSum += cos(magnetRad) / (1.0*n);
		ySum += sin(magnetRad) / (1.0*n);
		MyDelay(50);
	}

	magnetRad = atan2(ySum,xSum);
	magnetRad = NormalizeRad(magnetRad);

	return magnetRad;
}

// 返り値は，データ配列の上位半分を抜いたrangeの最大値．
int MagnetCalibration() {
	const int arrLen = 2*5;		// 2で割り切れるのがいい．
	int moterPower = 0;
	if (TIRE_TYPE == 1) {
		moterPower = 220;			// アスペ
	} else {
		moterPower = 255;			// 中密タイヤ
	}
	const unsigned long measuringTime = 6000;
	unsigned long time = millis();
	// PriorityQueue 大きい順に（[0]が最大）
	int xMaxs[arrLen];
	int yMaxs[arrLen];
	int xMins[arrLen];
	int yMins[arrLen];
	int xMax = 0;
	int yMax = 0;
	int xMin = 0;
	int yMin = 0;
	int xMaxRange = 0;
	int yMaxRange = 0;
	int xMinRange = 0;
	int yMinRange = 0;
	int maxRange = 0;

	// 初期化
	for (int i=0; i<arrLen; i++) {
		xMaxs[i] = -1000;
		yMaxs[i] = -1000;
		xMins[i] = 1000;
		yMins[i] = 1000;
	}

	for (int i = 0; i < 2; i++) {
		if (i == 0) {
			SetMoter(moterPower,0,0,moterPower);
			if (DEBUG) MyPrintln(F("calibration 1 moter set"));
		} else {
			SetMoter(0,moterPower,moterPower,0);
			if (DEBUG) MyPrintln(F("calibration 2 moter set"));
		}

		time = millis();
		while( millis() < (time + measuringTime) ) {
			GetMagnetRad();
			if (xMaxs[arrLen-1] < Magnet_X12) {
				xMaxs[arrLen-1] = Magnet_X12;
				MagnetCalibrationPriorityQueue(xMaxs, arrLen);
			}
			if (xMins[0] > Magnet_X12) {
				xMins[0] = Magnet_X12;
				MagnetCalibrationPriorityQueue(xMins, arrLen);
			}
			if (yMaxs[arrLen-1] < Magnet_Y12) {
				yMaxs[arrLen-1] = Magnet_Y12;
				MagnetCalibrationPriorityQueue(yMaxs, arrLen);
			}
			if (yMins[0] > Magnet_Y12) {
				yMins[0] = Magnet_Y12;
				MagnetCalibrationPriorityQueue(yMins, arrLen);
			}
			MyDelay(50);
		}

		SetMoter(0,0,0,0);
		MyDelay(1000);
	}

	xMaxRange = xMaxs[arrLen/2] - xMaxs[arrLen-1];
	yMaxRange = yMaxs[arrLen/2] - yMaxs[arrLen-1];
	xMinRange = xMins[0] - xMins[arrLen/2];
	yMinRange = yMins[0] - yMins[arrLen/2];

	maxRange = MagnetIntMax(MagnetIntMax(xMaxRange, yMaxRange), MagnetIntMax(xMinRange, yMinRange));

	xMax = xMaxs[arrLen/2];
	yMax = yMaxs[arrLen/2];
	xMin = xMins[arrLen/2];
	yMin = yMins[arrLen/2];

	MAGNET_X_OFFSET = (xMax + xMin) /-2.0;
	MAGNET_Y_OFFSET = (yMax + yMin) /-2.0;
	if ((xMax - xMin) == 0 || (yMax - yMin) == 0) {
		return 1000;
	}
	MAGNET_XY_RATIO = ((xMax - xMin)*1.0) / (yMax - yMin);

	// ################
	// あとで返り値の処理
	LogMagCal(maxRange, xMaxs, yMaxs, xMins, yMins, arrLen);

	return maxRange;
}

// PriorityQueueの更新
void MagnetCalibrationPriorityQueue(int *arr, int arrLen) {
	// 変更されたデータは一つなので，バブルソートを上下からそれぞれ１順すればよい
	int temp;
	for (int i=0; i<arrLen - 1; i++) {
		if (arr[i] < arr[i+1]) {
			temp = arr[i];
			arr[i] = arr[i+1];
			arr[i+1] = temp;
		}
	}
	for (int i=arrLen - 1; i>0; i--) {
		if (arr[i] > arr[i-1]) {
			temp = arr[i];
			arr[i] = arr[i-1];
			arr[i-1] = temp;
		}
	}
	return;
}

int MagnetIntMax(int a, int b) {
	if (a > b) {
		return a;
	} else {
		return b;
	}
}

int GetRandom1D() {
	double temp = GetMagnetRad();
	int random = long(temp * 10000) % 10;
	return random;
}


// #########################################
// GPS系
int InitializeGps() {
	int count = 0;
	GpsSerialBegin(SERIAL_BAUDRATE);			// Serial通信の開始
	SendUnable(3);								//GPS:3.GSV,2.GSA,4.RMCの無効化
	SendUnable(2);
	SendUnable(4);

	// NULL で初期化．バッファローオーバーラン対策．
	for(int i = 0; i < GPS_RAW_ARR_LENGTH; i++) {
		for(int j = 0; j < GPS_RAW_ARR_BUF_LENGTH; j++) {
			Gps_Raw_Arr[i][j] = '\0';
		}
	}

	// 3回試行して取れなかったらエラー
	for(int i = 0; i < 3; i++) {
		count += ClearGetGpsData();
	}

	if (count > 0) {
		return 1;
	} else {
		if (DEBUG) MyPrintln(F("GPS Sensor Dead!"));
		return 0;
	}
}

//不要なデータを送らないようにする datatype:0:GGA 1:GGL 2:GSA 3:GSV 4:RMC 5:VTG 6:MSS 8:ZDA
void SendUnable(int datatype){
	char c , buf[24] ;
	memset(buf,0x00,sizeof(buf)) ;
	strcat(buf,"$PSRF103,");
	buf[9] = datatype + 0x30 ;
	strcat(buf,",0,0");
	strcat(buf,",1") ;			// ChecksumEnable (チェックサムは付加する)
	c = ChecksumCalc(&buf[1]) ;			// チェックサムの計算を行う
	strcat(buf,"*") ;				// Checksum
	itoa(c,&buf[strlen(buf)],16) ;
	c = strlen(buf) ;
	buf[c] = 0x0d ;				// CR
	buf[c+1] = 0x0a ;				// LF
	GpsSerialPrint(buf);
	return;
}

//SendUnable用のチェックサム計算関数
char ChecksumCalc(char *dt) {
	char ans ;
	int i , len ;
	len = strlen(dt) ;
	ans = *dt ;
	for (i=1 ; i<len ; i++) {
		dt++ ;
		ans = ans ^ *dt ;
	}
	return ans ;
}

int GetGpsData() {
	char buf[256];			//受信データ用
	int count = 0;
	int gpsArrLength;
	unsigned long time = millis();

	// 初期値は0
	// 最後まで行ったら1に更新
	Gps_Status = 0;

	do {
		if (GpsSerialAvailable()) {
			buf[count] = GpsSerialRead();
			count++;
		}

		// MyPrint(count);
		// MyPrint(F(","));
		//MyDelay(1);						// これを外すと NormalizeRadSym でエラー
		delayMicroseconds(1);
		if (count > 250) break;
		if (millis() > time + 5000) {		// タイムアウト
			if (DEBUG) MyPrintln(F("GPS TIMEOUT..."));
			return 0;
		}
	} while(buf[count - 1] != 0x0A);				//改行コード0x0Aが来たらひとまず受信終わり
	buf[count] = '\0';							//bufにgpsデータ保存

	MyDelay(10);
	if (DEBUG) {
		//受信したbuf確認用
		MyPrint(F("buf : "));
		// bufに\nが入っているので，lnはいらない
		MyPrint(buf);
	}
	MyDelay(10);

	gpsArrLength = Split(buf, ",");

	// 送られてきたデータが15個で，$GPGGAならtrue
	if (gpsArrLength == 14 && !strcmp(Gps_Raw_Arr[0], "$GPGGA")) {
		// if (strcmp(Gps_Raw_Arr[1], "NULL") && strcmp(Gps_Raw_Arr[2], "NULL") && strcmp(Gps_Raw_Arr[3], "NULL") && strcmp(Gps_Raw_Arr[4], "NULL") && strcmp(Gps_Raw_Arr[5], "NULL") && strcmp(Gps_Raw_Arr[11], "NULL")) {
		if (strcmp(Gps_Raw_Arr[1], "NULL") && strcmp(Gps_Raw_Arr[2], "NULL") && strcmp(Gps_Raw_Arr[3], "NULL") && strcmp(Gps_Raw_Arr[4], "NULL") && strcmp(Gps_Raw_Arr[5], "NULL") && strcmp(Gps_Raw_Arr[9], "NULL")) {
			SplitLong(Gps_Raw_Arr[2], ".");
			Gps_Lat = GpsLL2Deg(Gps_LongLong[0], Gps_LongLong[1]);			//char ddmm mmmm → long dddddddd
			SplitLong(Gps_Raw_Arr[4], ".");
			Gps_Long = GpsLL2Deg(Gps_LongLong[0], Gps_LongLong[1]);
			SplitLong(Gps_Raw_Arr[9], ".");
			Gps_Height = atol(Gps_LongLong[0]) + 0.01 * CutStr2Long(Gps_LongLong[1], 2);			//コピーできないので元配列から直接
			SplitLong(Gps_Raw_Arr[1], ".");
			Gps_Time = atol(Gps_LongLong[0]);

			if (strcmp(Gps_Raw_Arr[3], "N")) {			// S だとtrue
				Gps_Lat = -1 * Gps_Lat;					//南緯の場合負の値にする  一応南緯も対応...?(笑)
			}
			if (strcmp(Gps_Raw_Arr[5], "E")) {			// W だとtrue
				Gps_Long = -1 * Gps_Long;				//西経の場合負の値にする
			}

			if (DEBUG) MyPrintln(F("GPS success"));
			Gps_Status = 1;
			return 1;
		} else {
			if (DEBUG) MyPrintln(F("GPS failed 1..."));
			return 0;
		}
	} else {
		if (DEBUG) MyPrintln(F("GPS failed 2..."));
		// MyPrint(F("[0] : "));
		// MyPrintln(Gps_Raw_Arr[0]);
		// MyPrint(F("[2] : "));
		// MyPrintln(Gps_Raw_Arr[2]);
		// MyPrint(F("[4] : "));
		// MyPrintln(Gps_Raw_Arr[4]);
		// MyPrint(F("[5] : "));
		// MyPrintln(Gps_Raw_Arr[5]);
		return 0;
	}
}

int ClearGetGpsData() {
	ClearGpsRxBuf();
	MyDelay(1);
	GetGpsData();
	MyDelay(1);
	return GetGpsData();
}

//ターゲットと現在地の間の	距離の計算
void CalcDistanceDirection(){

	// 緯度経度に10e6倍してない場合のバージョン
	// double gapLat = TARGET_LATITUDE - Gps_Lat;
	// double gapLong = TARGET_LONGITUDE - Gps_Long;

	// double latRad = Deg2Rad( ddmm2dddd(Gps_Lat) );
	// double gapLatRad = Deg2Rad( ddmm2dddd(gapLat) );
	// double gapLongRad = Deg2Rad( ddmm2dddd(gapLong) );

	// double gapX = EARTH_RADIUS * gapLongRad * cos(latRad);			//直交座標として計算
	// double gapY = EARTH_RADIUS * gapLatRad;

	double gapLat = (TARGET_LATITUDE - Gps_Lat) * 1.0 / 1000000;		//差は最大でも100000と見積もって、doubleにする
	double gapLong = (TARGET_LONGITUDE - Gps_Long) * 1.0 / 1000000;
	double gapPassingLat = (Passing_Latitude- Gps_Lat) * 1.0 / 1000000;		//差は最大でも100000と見積もって、doubleにする
	double gapPassingLong = (Passing_Longitude - Gps_Long) * 1.0 / 1000000;

	double latRad = Deg2Rad(Gps_Lat * 1.0 / 1000000);
	double gapLatRad = Deg2Rad(gapLat);
	double gapLongRad = Deg2Rad(gapLong);
	double passingLatRad = Deg2Rad(Gps_Lat * 1.0 / 1000000);
	double gapPassingLatRad = Deg2Rad(gapPassingLat);
	double gapPassingLongRad = Deg2Rad(gapPassingLong);

	double gapX = EARTH_RADIUS * gapLongRad * cos(latRad);			//直交座標として計算
	double gapY = EARTH_RADIUS * gapLatRad;
	double gapPassingX = EARTH_RADIUS * gapPassingLongRad * cos(passingLatRad);			//直交座標として計算
	double gapPassingY = EARTH_RADIUS * gapPassingLatRad;

	Gps_Distance = sqrt(gapX*gapX + gapY*gapY);
	Gps_Radian = NormalizeRad( atan2(gapX, gapY) );
	Gps_Passing_Distance = sqrt(gapPassingX*gapPassingX + gapPassingY*gapPassingY);
	Gps_Passing_Radian = NormalizeRad( atan2(gapPassingX, gapPassingY) );

	// MyPrint(F("gapLat : "));
	// MyPrint(gapLat, 6);
	// MyPrint(F(", gapLong : "));
	// MyPrint(gapLong, 6);
	// MyPrint(F(", gapX : "));
	// MyPrint(gapX);
	// MyPrint(F(", gapY : "));
	// MyPrint(gapY);
	// MyPrintln(F(""));

	LogComStrDbl("GD", Gps_Distance);
	LogComStrDbl("GR", Gps_Radian);
	LogComStrDbl("GRtD", Rad2Deg(Gps_Radian));
	LogComStrDbl("GPD", Gps_Passing_Distance);
	LogComStrDbl("GPR", Gps_Passing_Radian);
	LogComStrDbl("GPRtD", Rad2Deg(Gps_Passing_Radian));
	return;
}

long GpsLL2Deg(char *ddmmStr, char *mmmmStr){
	long gpsDegree = atol(ddmmStr) / 100;					//以下ｍが6桁なのでdは7桁より上にもっていく
	long gpsMinute = atol(ddmmStr) - gpsDegree*100;			//2桁
	long gpsMinuteDec = CutStr2Long(mmmmStr,4);				//4桁
	long ans = gpsDegree*1000000 + gpsMinute*1000000/60 + gpsMinuteDec*100/60;
	return ans;
}

long CutStr2Long(char *strNum, int digit){
	int length = strlen(strNum);		 //頭についた0も数えてくれるはず これで0120みたいなのも大丈夫なはず...
	char strNumDigit[digit];
	for(int i = 0; i < digit; i++) {
		strNumDigit[i] = '\0';
	}
	strncpy(strNumDigit,strNum,digit);
	long ans = atol(strNumDigit);

	while(length < digit){
		ans = ans*10;
		length++;
	}
	return ans;
}

int Split(char *string, char *separator) {
	char *endAdress;						// 処理する文字列末尾のアドレス
	char *startAdress;						// 切り取る文字列の先頭
	char *pointer;							// 処理する位置を保持
	char splitString[100];					// 切り取った文字 文字数の上限に注意
	int count = 0;							// カウンタ
	int flag = 0;							// 処理終了のため

	endAdress = string + strlen(string);	// 文字列の末尾のポインタ．\0の位置
	startAdress = string;					// セパレーター探索スタート位置

	while(count < GPS_RAW_ARR_LENGTH) {
		pointer = strpbrk(startAdress, separator);		// セパレーターの検索 見つからなければNULL
		if (pointer == NULL) {							// セパレーターが発見できなかったらtrue
			// セパレーターが発見できなくても最後の文字列を処理する．
			if (flag) {
				break;
			}
			flag = 1;									// 次回LOOPから抜けれるように
		} else {
			*pointer = '\0';							// セパレーターをNULLに置換
			pointer++;									// 処理位置を次へ移動
			// *(pointer++) = 0;
		}

		strcpy(splitString, startAdress);				// 切り取った文字列をコピー って，まだ切り取ってないかw
		startAdress = pointer;							// 次切り取る文字列の先頭を更新

		if (!strcmp(splitString, "")) {					// strcmp 一致 → 0
			strcpy(splitString, "NULL");
		}

		//###################
		// debug用表示
		// MyPrint(count);
		// MyPrint(F(" : "));
		// MyPrint(splitString);
		// MyPrint(F("\n"));
		//###################

		// GPS RAW DATA配列更新
		// 安全なコピー．バッファローオーバーラン対策．
		strncpy(Gps_Raw_Arr[count], splitString, GPS_RAW_ARR_BUF_LENGTH - 1);
		Gps_Raw_Arr[count][GPS_RAW_ARR_BUF_LENGTH - 1] =  '\0';
		count++;
		// MyDelay(500);
	}

	return (count-1);
}

int SplitLong(char *string, char *separator) {
	char *endAdress;						// 処理する文字列末尾のアドレス
	char *startAdress;						// 切り取る文字列の先頭
	char *pointer;							// 処理する位置を保持
	char splitString[100];					// 切り取った文字 文字数の上限に注意
	int count = 0;							// カウンタ
	int flag = 0;							// 処理終了のため

	endAdress = string + strlen(string);	// 文字列の末尾のポインタ．\0の位置
	startAdress = string;					// セパレーター探索スタート位置

	while(count < GPS_RAW_ARR_LENGTH) {
		pointer = strpbrk(startAdress, separator);		// セパレーターの検索 見つからなければNULL
		if (pointer == NULL) {							// セパレーターが発見できなかったらtrue
			// セパレーターが発見できなくても最後の文字列を処理する．
			if (flag) {
				break;
			}
			flag = 1;									// 次回LOOPから抜けれるように
		} else {
			*pointer = '\0';							// セパレーターをNULLに置換
			pointer++;									// 処理位置を次へ移動
			// *(pointer++) = 0;
		}

		strcpy(splitString, startAdress);				// 切り取った文字列をコピー って，まだ切り取ってないかw
		startAdress = pointer;							// 次切り取る文字列の先頭を更新

		if (!strcmp(splitString, "")) {					// strcmp 一致 → 0
			strcpy(splitString, "NULL");
		}

		//###################
		// debug用表示
		// MyPrint(count);
		// MyPrint(F(" : "));
		// MyPrint(splitString);
		// MyPrint(F("\n"));
		//###################

		// GPS RAW DATA配列更新
		// 安全なコピー．バッファローオーバーラン対策．
		strncpy(Gps_LongLong[count], splitString, GPS_RAW_ARR_BUF_LENGTH - 1);
		Gps_LongLong[count][GPS_RAW_ARR_BUF_LENGTH - 1] = '\0';
		count++;
		// MyDelay(500);
	}

	return (count-1);
}

void ClearGpsRxBuf() {
	while (GpsSerialAvailable()) {
		GpsSerialRead();
	}
}

// #########################################
// 加速度センサ系
void GetAcc() {
	Acc_X = analogRead(PIN_ACC_X); // Ｘ軸
	Acc_Y = analogRead(PIN_ACC_Y); // Ｙ軸
	Acc_Z = analogRead(PIN_ACC_Z); // Ｚ軸
}

int IsRoverUpsideDown() {
	int critical = 700;
	GetAcc();
	if (Acc_Z > critical) {
		return 1;
	} else {
		return 0;
	}
}

// #########################################
// カメラ系
int InitializeCamera() {
	char cmd[] = {0xaa, 0x0d | CAMERA_ADDRESS, 0x00, 0x00, 0x00, 0x00};
	unsigned char resp[6];
	unsigned long time = millis();
	int err;

	CameraSerialBegin(115200);
	CameraSerialSetTimeout(500);
	while (1) {
		if (millis() > time + 10000) {		// タイムアウト
			MyPrintln(F("InitializeCamera TIMEOUT..."));
			LogSta("ICT");
			return 0;
		}
		//ClearCameraRxBuf();
		SendCameraCmd(cmd,6);
		if (CameraSerialReadBytes((char *)resp, 6) != 6) {
			continue;
		}
		if (resp[0] == 0xaa && resp[1] == (0x0e | CAMERA_ADDRESS) && resp[2] == 0x0d && resp[4] == 0x00 && resp[5] == 0x00) {
			if (CameraSerialReadBytes((char *)resp, 6) != 6) continue;
			if (resp[0] == 0xaa && resp[1] == (0x0d | CAMERA_ADDRESS) && resp[2] == 0x00 && resp[3] == 0x00 && resp[4] == 0x00 && resp[5] == 0x00) break;
		}
	}
	cmd[1] = 0x0e | CAMERA_ADDRESS;
	cmd[2] = 0x0d;
	SendCameraCmd(cmd, 6);

	// MyPrintln("\nCamera initialization done.");


	MyDelay(1000);
	err = PreCapture();
	return err;
}

void CapturePic(int isCacheOn) {
	int err;
	MyPrintln(F("CapturePic"));
	LogSta("CP");
	ClearGetGpsData();
	LogGps();
	GetAcc();
	LogAcc();
	LogMagAvg(5, GetMagnetRadNTimes(5));
	// tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
	// MyDelay(500);
	tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
	err = Capture();
	CacheOff();
	tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
	if (err == 0) {
		LogPho(0);
		if (DEBUG) MyPrintln(F("Error at Capture"));
		// return;				// #############################？
	} else {
		GetPicData();
		LogPho(1);
	}
	picNameNum++;
	MyDelay(50);
	noTone(PIN_BUZZER);
	if (isCacheOn == 1) {
		CacheOn();
	}
	MyPrintln(F("End CapturePic"));
	LogSta("ECP");
	return;
}

void ClearCameraRxBuf() {
	while (CameraSerialAvailable()) {
		CameraSerialRead();
	}
}

void SendCameraCmd(char cmd[], int cmd_len) {
	for (char i = 0; i < cmd_len; i++) CameraSerialPrint(cmd[i]);
}

int PreCapture() {
	unsigned long time = millis();
	char cmd[] = {0xaa, 0x01 | CAMERA_ADDRESS, 0x00, 0x07, 0x00, PIC_FORMAT };
	//char cmd[] = {0xaa, 0x01 | CAMERA_ADDRESS, 0x00, 0x06, 0x07, PIC_FORMAT };
	unsigned char resp[6];

	CameraSerialSetTimeout(100);
	while (1) {
		if (millis() > time + 10000) {		// タイムアウト
			MyPrintln(F("PreCapture TIMEOUT..."));
			LogSta("PCT");
			return 0;
		}
		ClearCameraRxBuf();
		SendCameraCmd(cmd, 6);
		if (CameraSerialReadBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == (0x0e | CAMERA_ADDRESS) && resp[2] == 0x01 && resp[4] == 0x00 && resp[5] == 0x00) break;
	}
	return 1;
}

int Capture() {
	unsigned long time = millis();
	char cmd[] = {0xaa, 0x06 | CAMERA_ADDRESS, 0x08, PIC_PKT_LEN & 0xff, (PIC_PKT_LEN>>8) & 0xff, 0x00};
	unsigned char resp[6];

	// MyPrintln(F("Capture"));
	// LogSta("C");
	// tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);

	CameraSerialSetTimeout(100);
	time = millis();
	while (1) {
		if (millis() > time + 10000) {		// タイムアウト
			MyPrintln(F("Capture TIMEOUT 1..."));
			LogSta("CT1");
			tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
			return 0;
		}
		ClearCameraRxBuf();
		SendCameraCmd(cmd, 6);
		if (CameraSerialReadBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == (0x0e | CAMERA_ADDRESS) && resp[2] == 0x06 && resp[4] == 0x00 && resp[5] == 0x00) break;
	}
	cmd[1] = 0x05 | CAMERA_ADDRESS;
	cmd[2] = 0x00;
	cmd[3] = 0x00;
	cmd[4] = 0x00;
	cmd[5] = 0x00;
	time = millis();
	while (1) {
		if (millis() > time + 10000) {		// タイムアウト
			MyPrintln(F("Capture TIMEOUT 2..."));
			LogSta("CT2");
			tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
			return 0;
		}
		ClearCameraRxBuf();
		SendCameraCmd(cmd, 6);
		if (CameraSerialReadBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == (0x0e | CAMERA_ADDRESS) && resp[2] == 0x05 && resp[4] == 0x00 && resp[5] == 0x00) break;
	}
	cmd[1] = 0x04 | CAMERA_ADDRESS;
	cmd[2] = 0x01;
	time = millis();
	while (1) {
		if (millis() > time + 10000) {		// タイムアウト
			MyPrintln(F("Capture TIMEOUT 3..."));
			LogSta("CT3");
			tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
			return 0;
		}
		ClearCameraRxBuf();
		SendCameraCmd(cmd, 6);
		if (CameraSerialReadBytes((char *)resp, 6) != 6) continue;
		if (resp[0] == 0xaa && resp[1] == (0x0e | CAMERA_ADDRESS) && resp[2] == 0x04 && resp[4] == 0x00 && resp[5] == 0x00) {
			CameraSerialSetTimeout(1000);
			if (CameraSerialReadBytes((char *)resp, 6) != 6) {
				continue;
			}
			if (resp[0] == 0xaa && resp[1] == (0x0a | CAMERA_ADDRESS) && resp[2] == 0x01) {
				picTotalLen = (resp[3]) | (resp[4] << 8) | (resp[5] << 16);
				MyPrint(F("picTotalLen:"));
				MyPrintln(picTotalLen);
				LogComStrDbl("pTL", picTotalLen);
				if (picTotalLen > 100000) {
					MyPrintln(F("picTotalLen is too long"));
					LogSta("pTLitl");
					tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
					return 0;
				}
				break;
			}
		}
	}

	// MyPrintln(F("End Capture"));
	// LogSta("EC");
	// tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
	return 1;
}

void GetPicData() {
	// MyPrintln(F("GetPicData"));
	// LogSta("GPiD");
	// tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
	unsigned int pktCnt = (picTotalLen) / (PIC_PKT_LEN - 6);
	if ((picTotalLen % (PIC_PKT_LEN-6)) != 0) pktCnt += 1;

	char cmd[] = { 0xaa, 0x0e | CAMERA_ADDRESS, 0x00, 0x00, 0x00, 0x00};
	unsigned char pkt[PIC_PKT_LEN];

	// char PIC_NAME[] = "MD000000/PIC00000.JPG";
	// PIC_NAME[6] = picNameNum/10 + '0';
	// PIC_NAME[7] = picNameNum%10 + '0';
	for (int i = 0; i<5; i++) {
		PIC_NAME[9+i+3] = (picNameNum/LongPow(10,4-i)) % 10 + '0';
	}

	if (SD.exists(PIC_NAME)) {
		MyPrint(F("\nexsist picName file\n"));
		ToneAlert();
		SD.remove(PIC_NAME);
	}

	if (DEBUG) MyPrintln(F("\nto SD...\n"));
	My_Pic_File = SD.open(PIC_NAME, FILE_WRITE);
	if (!My_Pic_File) {
		if (DEBUG) MyPrintln(F("My_Pic_File open fail..."));
		SdOpenFailed();
		// ToneAlert();
	} else{
		CameraSerialSetTimeout(1000);
		for (unsigned int i = 0; i < pktCnt; i++) {
			cmd[4] = i & 0xff;
			cmd[5] = (i >> 8) & 0xff;

			int retry_cnt = 0;
			retry:
			MyDelay(10);
			ClearCameraRxBuf();
			SendCameraCmd(cmd, 6);
			uint16_t cnt = CameraSerialReadBytes((char *)pkt, PIC_PKT_LEN);

			unsigned char sum = 0;
			for (int y = 0; y < cnt - 2; y++) {
				sum += pkt[y];
			}
			if (sum != pkt[cnt-2]) {
				if (++retry_cnt < 100) goto retry;
				else break;
			}

			My_Pic_File.write((const uint8_t *)&pkt[4], cnt-6);
			//if (cnt != PIC_PKT_LEN) break;
		}
		cmd[4] = 0xf0;
		cmd[5] = 0xf0;
		SendCameraCmd(cmd, 6);
	}
	My_Pic_File.close();
	if (DEBUG) MyPrintln(F("to SD End\n"));
	// MyPrintln(F("End GetPicData"));
	// LogSta("EGPiD");
	// tone(PIN_BUZZER, TONE_FREQUENCY_SD_CAMERA);
}


// #########################################
// ブザー系
void ToneAlert() {
	int dt = 100;
	int loopTime = 1;
	for (int i = 0; i < loopTime; i++) {
		for (int j = 0; j < 3; j++) {
			tone(PIN_BUZZER, 1000);
			MyDelay(dt);
			noTone(PIN_BUZZER);
			MyDelay(dt);
		}
		MyDelay(dt * (6-1));
	}
	return;
}


// #########################################
// 通信系
void XbeeGps() {
	MyPrint(F("XbeeGps : "));
	MyPrint(millis());
	MyPrint(F(", "));
	MyPrint(Gps_Time);
	MyPrint(F(", "));
	MyPrint(Gps_Lat);
	MyPrint(F(", "));
	MyPrint(Gps_Long);
	MyPrint(F(", "));
	MyPrint(Gps_Height);
	MyPrint(F(", "));
	MyPrint(Gps_Status);
	MyPrint(F("\n"));
}

void XbeeLight() {
	MyPrint(F("XbeeLight : "));
	MyPrint(millis());
	MyPrint(F(", "));
	MyPrint(Light_Raw);
	MyPrint(F("\n"));
}

//void GpsBeacon() {
//	MyPrint(F("Do you copy?\n"));
//	if (GetGpsData() == 1) {
//		MyPrint(F("Gps success!\n"));
//	} else {
//		MyPrint(F("Gps failed... but here is the last GpsData\n"));
//		MyDelay(10);
//	}


//
//	XbeeGps();
//}


// #########################################
// ログ系
int InitializeSd() {
	pinMode(PIN_SD_SS, OUTPUT);
	if (!SD.begin(PIN_SD_SS)) {
		if (DEBUG) MyPrintln(F("SD.begin failed!"));
		ToneAlert();
		return 0;
	}
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return 0;
	}
	MY_FILE.close();
	return 1;
}

void SdOpenFailed() {
	if (DEBUG) MyPrintln(F("SdOpenFailed"));
	MY_FILE.close();		// ねんのため
	ToneAlert();
	return;
}

void CacheOn() {
	Log_To_Sd = 0;
	LogSta("CO");
	return;
}

int CacheOff() {
	int err;
	char str;
	LogSta("COf");
	Log_To_Sd = 1;

	if (Cache_Size == 0) {
		return 1;
	}

	// Cacheアドレスの連続化
	if (Cache_End_Address < Cache_Start_Address) {
		Cache_End_Address += MAX_EEPROM_ADDRESS;
	}

	// cacheのSD書き込み処理
	tone(PIN_BUZZER, TONE_FREQUENCY_SD_PREPARING);
	MyDelay(1000);
	noTone(PIN_BUZZER);
	tone(PIN_BUZZER, TONE_FREQUENCY_SD_WRITE);

	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		if (DEBUG) MyPrintln(F("Cache to SD failed..."));
		// SDが死んでいたら，EEPROMのデータは捨てる．
		err = 0;
	} else {
		// SDが生きてるよ！
		if (DEBUG) MyPrintln(F("Cache to SD begin..."));
		for (int i = Cache_Start_Address; i <= Cache_End_Address; i++) {
			str = EEPROM.read( i % MAX_EEPROM_ADDRESS );
			MY_FILE.print(str);
			if (DEBUG) MyPrint(str);
		}
		if (DEBUG) MyPrintln(F("Cache to SD complete!"));
		MY_FILE.close();
		err = 1;
	}

	MyDelay(50);
	noTone(PIN_BUZZER);

	// 終了処理
	Cache_Start_Address = (Cache_End_Address + 1) % MAX_EEPROM_ADDRESS;
	Cache_End_Address = Cache_End_Address % MAX_EEPROM_ADDRESS;
	Cache_Size = 0;
	Is_Cache_Full = 0;

	return err;
}

// Cacheがフルの時に処理を中断してSDに移行させる
int MoveFullCacheToSd() {
	if (Is_Cache_Full == 0) {
		return 1;
	}
	int moterPowersTemp[4];
	int result;

	for (int i = 0; i < 4; i++) {
		moterPowersTemp[i] = Moter_Powers[i];
	}

	SetMoter(0,0,0,0);
	result = CacheOff();
	MyDelay(50);
	CacheOn();

	SetMoter(moterPowersTemp[0], moterPowersTemp[1], moterPowersTemp[2], moterPowersTemp[3]);
	return result;
}

int LogCache(const char *string) {
	for (int i = 0; i < strlen(string); i++) {
		Cache_End_Address++;
		Cache_End_Address %= MAX_EEPROM_ADDRESS;
		Cache_Size++;
		EEPROM.update(Cache_End_Address, string[i]);
	}

	if ( (Cache_Size*4) > (MAX_EEPROM_ADDRESS*3) ) {
		Is_Cache_Full = 1;
		return 0;
	} else {
		return 1;
	}
}

int LogSd(const char *string) {
	int err;

	// tone(PIN_BUZZER, TONE_FREQUENCY_SD_PREPARING);
	// MyDelay(1000);
	// noTone(PIN_BUZZER);
	tone(PIN_BUZZER, TONE_FREQUENCY_SD_WRITE);

	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		if (DEBUG) MyPrintln(F("LogSd failed..."));
		// SDが死んでいたら，データは捨てる．
		err = 0;
	} else {
		MY_FILE.print(string);
		MY_FILE.close();
		err = 1;
	}

	// MyDelay(500);
	noTone(PIN_BUZZER);

	return err;
}


int LogSta(const char *string) {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",Sta,");
	str += String(string);
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogComStrDbl(const char *string, double dbl) {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",ComSD,");
	str += String(string);
	str += String(",");
	str += String((long)(dbl * 1000));
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogComStrLng(const char *string, long lng) {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",ComSL,");
	str += String(string);
	str += String(",");
	str += String(lng);
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogComStrDblLng(const char *string, double dbl, long lng) {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",ComSDL,");
	str += String(string);
	str += String(",");
	str += String((long)(dbl * 1000));
	str += String(",");
	str += String(lng);
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogLit(int light) {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",Lit,");
	str += String(light);
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogMagRaw() {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",MagRaw,");
	str += String(Magnet_X12);
	str += String(",");
	str += String(Magnet_Y12);
	str += String(",");
	str += String(Magnet_Z12);
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogMagAvg(int n, double rad) {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",MagAvg,");
	str += String(n);
	str += String(",");
	str += String((long)(rad * 1000));
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogMagCal(int maxRange, int *xMaxs, int *yMaxs, int *xMins, int *yMins, int arrLen) {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",MagCal,");
	str += String((long)(MAGNET_X_OFFSET * 1000));
	str += String(",");
	str += String((long)(MAGNET_Y_OFFSET * 1000));
	str += String(",");
	str += String((long)(MAGNET_XY_RATIO * 1000));
	str += String(",");
	str += String(maxRange);
	str += String(",xMaxs");
	for (int i=0; i<arrLen; i++) {
		str += String(",");
		str += String(xMaxs[i]);
	}
	str += String(",xMins");
	for (int i=0; i<arrLen; i++) {
		str += String(",");
		str += String(xMins[i]);
	}
	str += String(",yMaxs");
	for (int i=0; i<arrLen; i++) {
		str += String(",");
		str += String(yMaxs[i]);
	}
	str += String(",yMins");
	for (int i=0; i<arrLen; i++) {
		str += String(",");
		str += String(yMins[i]);
	}
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogGps() {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",Gps,");
	str += String(Gps_Time);
	str += String(",");
	str += String(Gps_Lat);
	str += String(",");
	str += String(Gps_Long);
	str += String(",");
	str += String((long)(Gps_Height * 1000));
	str += String(",");
	str += String(Gps_Status);
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogAcc() {
	String str;
	if (Log_To_Sd == 0) {
		MoveFullCacheToSd();
	}
	str = String(millis());
	str += String(",Acc,");
	str += String(Acc_X);
	str += String(",");
	str += String(Acc_Y);
	str += String(",");
	str += String(Acc_Z);
	str += String("\n");

	if (Log_To_Sd) {
		return LogSd(str.c_str());
	} else {
		LogCache(str.c_str());
		return 1;
	}
}

int LogPho(int ok) {
	// こいつはcameraから呼び出しされ，これが呼び出しされているときは音がなってる．
	// cache書き込み不可
	if (Log_To_Sd == 0) {
		return 0;
	}

	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return 0;
	}
	MY_FILE.print(millis());
	MY_FILE.print(",Pho,");
	MY_FILE.print(picNameNum);
	MY_FILE.print(",");
	MY_FILE.print(ok);
	MY_FILE.print("\n");
	MY_FILE.close();
	return 1;
}

// #########################################
// I2C
unsigned char I2cRead(unsigned char regAddress) {
	Wire.beginTransmission(HMC5883L_ADDRESS);
	Wire.write(regAddress);
	Wire.endTransmission(false);
	Wire.requestFrom(HMC5883L_ADDRESS,1);
	return Wire.read();
}

void I2cWrite(unsigned char regAddress, unsigned char data) {
	Wire.beginTransmission(HMC5883L_ADDRESS);
	Wire.write(regAddress);
	Wire.write(data);
	Wire.endTransmission(false);
}


// #########################################
// 変換系
double NormalizeRad(double input) {				//radianを0~2πにする
	if (input >= 0 ) {
		return input - (int)(input / (2*M_PI)) *2*M_PI;
	} else {
		return input + ((int)((-1*input) / (2*M_PI)) + 1)* 2*M_PI;
	}
}

double NormalizeRadSym(double input) {
	double result = NormalizeRad(input);
	if (result > M_PI) {
		result = result - 2*M_PI;
	}
	return result;
}

//degree→radian変換
double Deg2Rad(double deg) {
	return deg * M_PI / 180.0;
}

//radian→degree変換
double Rad2Deg(double rad) {
	return rad * 180.0 / M_PI;
}

//degree値、dd.mmmmmmからdd.ddddddに変換
// double ddmm2dddd(double ddmm){
// 	return (int)(ddmm) + (ddmm - (int)ddmm) / 0.6;
// }

// ローバー走行距離 メートル → ローバー走行時間 msec
unsigned long Meter2Msec(double meter) {
	// 2km/h = 2000/3600 m/s = 0.55556 m/s を仮定
	// 3km/h = 3000/3600 m/s = 0.83333 m/s を仮定
	return (unsigned long)(1000 * meter / 0.83333);
}

// ローバー走行距離 メートル ← ローバー走行時間 msec
double Msec2Meter(int msec) {
	// 2km/h = 2000/3600 m/s = 0.55556 m/s を仮定
	return (msec * 1.0) * 0.5556 / 1000.0;
}

unsigned long Sec2Msec(double Sec) {
	return (unsigned long)(Sec * 1000);
}

unsigned long LongPow(int base, int exponent) {
	unsigned long result = 1;
	if (exponent <= 0) {
		return result;
	}
	for (int i = 0; i < exponent; i++) {
		result *= base;
	}
	return result;
}

void MyDelay(long msec) {
	if (msec >= 0) {
		delay(msec);
	} else {
		delay(1000);
	}
}

void MyPrintlnRemainingTime(unsigned long startTimeMsec, long timeLimitSec) {
	long remainingTimeSec = timeLimitSec - (long)((millis() - startTimeMsec) / 1000);
	if (remainingTimeSec < 0) {
		MyPrint(0);
	} else {
		MyPrint(remainingTimeSec);
	}
	MyPrintln(F(" seconds left"));
}

// To Do List
// 6/15
//	GPS ddmm.mmmmをlong dddddddddに
//	GPS Target座標を入れるときに注意


// ###########
// max min absは注意


// magnetInitはまだ未完成 無限ループの恐れあり



//・末端誘導
//・pidのログ付


// ランディングの判定中のXbeeで，後何秒で動くかとばす

// GPSのバッファはサイズに注意しないといけないので，
// GPSをGetGpsDatによってぴったり1sになるようにしてるとダメ．


// TODO
// piiname = 12345からスタートでかけるか
// rand1Dの確認
// 西経に対応できるか？


// Mission前TODO
// 地磁気キャリブレーションのしきい値
// 光センサしきい値
// Accしきい値
// モーター各種パラメータ
// SetRoverAngle
// Meter2Msec
// Msec2Meter
// IsRoverUpsidedownのcritical
// SetRoverAngleのmac try
// DEFAULT_ADJUST_MOTER_POWER
// 中継地点判定のデフォルト設定
// 中継地点設定


// 着陸判定時間3つ
// OPでの距離



// 関数引数のアウトレンジ確認
// too long によるエラー確認

// 最後の写真



// take photoにgps mag accをいれる． その時，音の確認
// cacheONOFFの確認
// それに伴う，.pyの編集
// captyrepicの内部のcacheOnを選択可能に
// webapp用の.py．写真が成功するかしないかでロジック変わる？？
// GPS judge中にも写真撮りたい

/*
logstrの略記前

スケッチが プログラムストレージ領域の 61,608バイト (24%) を使用しています。最大は 253,952バイト です。
グローバル変数が 4,089バイト (49%) の 動的メモリを使用しており、ローカル変数に 4,103 バイトが残っています。最高は 8,192バイトです。

略記後

スケッチが プログラムストレージ領域の 60,434バイト (23%) を使用しています。最大は 253,952バイト です。
グローバル変数が 3,003バイト (36%) の 動的メモリを使用しており、ローカル変数に 5,189 バイトが残っています。最高は 8,192バイトです。
*/