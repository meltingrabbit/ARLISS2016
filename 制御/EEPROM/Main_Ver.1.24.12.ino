#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

// #########################################
// 関数プロトタイプ宣言
// #########################################
// ミッション遂行系
void LandingJudge();
int LightLandingJudge();
void GpsLightLandingJudge();
void RunUpNearGoal(double residualError);
void ApproachGoal(double residualError);
void FinalApproachToGoal(double residualError);
// モーター系
void InitializeMoter();
int SetRoverAngle(double targetRad, int maxCount);
void GoStraight(double targetRad, unsigned long msec, int loopTime);
void SpinRover(double LorR, unsigned long mSec);
void SetMoter(int l1, int l2, int r1, int r2);
// 光系
int GetLight();
// 地磁気系
int InitializeMagnet();
double GetMagnetRad();
double GetMagnetRadNTimes(int n);
int MagnetCalibration();
void MagnetCalibrationPriorityQueue(int *arr, int arrLen);
int MagnetIntMax(int a, int b);
// GPS系
int InitializeGps();
void SendUnable(int datatype);
char ChecksumCalc(char *dt);
int GetGpsData();
void CalcDistanceDirection();
long GpsLL2Deg(char *ddmmStr, char *mmmmStr);
long CutStr2Long(char *strNum, int digit);
int Split(char *string, char *separator);
int SplitLong(char *string, char *separator);
// 加速度センサ系

// ブザー系
void ToneAlert();
// Log系
int InitializeSd();
void SdOpenFailed();
void CacheOn();
int CacheOff();
int MoveFullCacheToSd();
int LogCache(const char *string);
int LogSta(char *string);
int LogGps();
int LogMagRaw();
int LogMagAvg(int n, double rad);
int LogMagCal(int maxRange, int *xMaxs, int *yMaxs, int *xMins, int *yMins, int arrLen);
int LogAcc();
int LogPho(char *fileName, int ok);
// I2C
unsigned char I2cRead(unsigned char regAddress);
void I2cWrite(unsigned char regAddress, unsigned char data);
// 変換系
double NormalizeRad(double input);
double NormalizeRadSym(double input);
double Deg2Rad(double deg);
double Rad2Deg(double rad);
double ddmm2dddd(double ddmm);
unsigned long Meter2Msec(double meter);
double Msec2Meter(int msec);
unsigned long Sec2Msec(double Sec);
// #########################################

// #########################################
// 定数定義
// #########################################

#define DEBUG 1

#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート

//着地判定
#define LIGHT_JUDGE_TIME_SEC 1800
#define GPS_LIGHT_JUDGE_TIME_SEC 10800		//単位は(秒) この二つのSECの和が、着地判定自体のタイムアウト時間

// スイッチ
#define PIN_SWITCH 35

// モーター
#define PIN_MOTER_LEFT1 12
#define PIN_MOTER_LEFT2 10
#define PIN_MOTER_RIGHT1 7
#define PIN_MOTER_RIGHT2 5

// CdS光センサ
#define ANALOG_PIN_CDS 1			// 光センサのアナログin
#define PIN_LIGHT 15
#define LIGHT_CRITICAL 15		//しきい値

// 地磁気センサ
#define HMC5883L_ADDRESS 0x1E		// 7bit ADDRESS for I2C通信の素子識別アドレス

// GPS
#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
#define GpsSerialRead() Serial1.read()
#define GpsSerialAvailable() Serial1.available()
#define GpsSerialPrint(gpsSendData) Serial1.print(gpsSendData)

// 加速度センサ
#define PIN_ACC_Z 0
#define PIN_ACC_Y 2
#define PIN_ACC_X 4

// ブザー
#define PIN_BUZZER 27

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

// モーター
// 現在のモーター出力を保存するための配列
int Moter_Powers[4];

// 光センサ

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
double Gps_Distance = 0;							//センサと目的地間距離
double Gps_Radian = 0;								//センサから目的地までの方位
long Gps_Time;
// dd ddd dddにすること！安田講堂塔真ん中(D, D)=(1602.1, 4.48286)
//35.717900    139.765163
//const long TARGET_LATITUDE = 35717900;
//const long TARGET_LONGITUDE = 139765163;
// 工学部広場
const long TARGET_LATITUDE = 35713704;
const long TARGET_LONGITUDE = 139760225;

// 加速度センサ
long Acc_X = 0;
long Acc_Z = 0;
long Acc_Y = 0;

// ブザー

// Log
// SDカードのクラッシュがこわすぎるので，スイッチがOFFの時はSD書き込みができない（処理が止まる）ようにする．
File MY_FILE;
char FLIE_NAME[] = "LOG.CSV";
boolean Log_To_Sd = 1;					// SD書き込みか Cache書き込みか
boolean Is_Cache_Full = 0;				// Cacheの75%が書き込まれたらtrue
int Cache_Start_Address = 0;			// 現在のCache書き込み開始アドレス
int Cache_End_Address = 0;				// 現在のCache終了アドレス．次の文字は this+1 にはいる
unsigned int Cache_Size = 0;			// Cache済みサイズ
const int MAX_EEPROM_ADDRESS = 4096;	// modとるために 4*2**10 にしてある．末尾は-1であることに注意
// #########################################


void setup() {
	int countSwitch = 0;
	delay(1000);

	Serial.begin(SERIAL_BAUDRATE);




	// ボタンが押されて，安定したらスタート
	while(1) {
		countSwitch = 0;
		Serial.println("Unstarted....");
		for (int i = 0; i < 10; i++) {
			countSwitch += digitalRead(PIN_SWITCH);
			delay(50);
		}
		if (countSwitch == 0) {
			break;
		}
	}

	int error = 1;
	if (DEBUG) Serial.println("Switch ON!");


	// SD初期化
	// エラーの場合，音を鳴らして警告させたい
	error = InitializeSd();
	if (error == 0) {
		LogSta("Switch ON!");						// ほんとは，SwitchONはもっと前だけど，SDの初期化終わらないと書き込めないのよね....w
		LogSta("Initialize SD failed!");
		if (DEBUG) Serial.println("initialize SD failed!");
		ToneAlert();
	} else {
		LogSta("Switch ON!");						// ほんとは，SwitchONはもっと前だけど，SDの初期化終わらないと書き込めないのよね....w
		LogSta("Initialize SD OK!");
		if (DEBUG) Serial.println("Initialize SD OK!");
	}

	// 地磁気初期化
	error = InitializeMagnet();

	// GPS初期化
	error = InitializeGps();
	if (error == 0) {
		LogSta("initialize GPS failed!");
		if (DEBUG) Serial.println("initialize GPS failed!");
		ToneAlert();
	} else {
		LogSta("Initialize GPS OK!");
		if (DEBUG) Serial.println("Initialize GPS OK!");
	}

	InitializeMoter();
	LogSta("Initialize Moter OK!");
	if (DEBUG) Serial.println("Initialize Moter OK!");


	// ###########################
	// 初期化完了の音を鳴らしたい
	LogSta("Initialization Finish!");
	if (DEBUG) Serial.println("Initialization Finish!");
	for (int i = 0; i < 3; i++) {
		tone(PIN_BUZZER, 1320);
		delay(80);
		noTone(PIN_BUZZER);
		delay(400);
	}
	tone(PIN_BUZZER, 1320);
	delay(1500);
	noTone(PIN_BUZZER);

	delay(10);

	Serial.println("START");


	delay(5000);

	CacheOn();
	SetMoter(255,0,255,0);
}


void loop() {

	while(digitalRead(PIN_SWITCH) == 1) {
		Serial.println("END");
		tone(PIN_BUZZER, 440);
		SetMoter(0,0,0,0);
		delay(1000000);
	}

	double magnetRad;
	int logMagCal[2];
	logMagCal[0] = 1;
	logMagCal[0] = 2;

	LogSta("LogSta");
	GetGpsData();
	LogGps();
	magnetRad = GetMagnetRad();
	LogMagRaw();
	magnetRad = GetMagnetRadNTimes(10);
	LogMagAvg(10, magnetRad);
	LogMagCal(1, logMagCal, logMagCal, logMagCal, logMagCal, 2);

	delay(10);
}




// #########################################
// ミッション遂行系
// 着地判定
void LandingJudge(){
	if (LightLandingJudge() == 1) {
		return;
	}
	GpsLightLandingJudge();
}

int LightLandingJudge() {
	unsigned long time = millis();
	int flag = 0;
	while(GetLight() == 0) {
		if(millis() > time + Sec2Msec(LIGHT_JUDGE_TIME_SEC)) {
			flag = 0;
			break;
		}
		delay( Sec2Msec(10) );
	}
}

void GpsLightLandingJudge() {
	unsigned long time = millis();
	const int rangeGpsJudge = 2*10;		//偶数にする
	int altitude[rangeGpsJudge];
	int count = 0;
	int countGpsData = 0;
	double sumGpsHeight = 0;
	int averageGps = 0;
	int gapGpsHeight = 0;			//配列に入れる用

	for (int i=0; i<rangeGpsJudge; i++) {		//配列初期化
		altitude[i] = 0;
	}

	while(GetLight() == 0) {			//gps判定ループ(始めに光判定)

		if(millis() > time + Sec2Msec(GPS_LIGHT_JUDGE_TIME_SEC)) {			//タイムアウト
			return;
		}

		while(count < 30){				//平均取得
			if (GetGpsData() == 1) {
				sumGpsHeight += Gps_Height;
				countGpsData++;
			};
			if(GetLight() == 1) {					//平均取得中の光判定
				return;
			}
			count++;
		}

		if(countGpsData > 20){		//平均の時に20個以上取れてたら本格的な判定を始める
			averageGps = sumGpsHeight / countGpsData;
			altitude[10] = 1;

			count = 0;				//カウントリセット
			countGpsData = 0;		//
			while(count <30) {					//配列に代入していき、分布を作成
				if (GetGpsData() == 1) {
					gapGpsHeight = Gps_Height - averageGps;
					if (gapGpsHeight < rangeGpsJudge/2 || gapGpsHeight > -rangeGpsJudge/2){			//rangeの中にあるときに分布に追加
						altitude[rangeGpsJudge/2 + gapGpsHeight]++;
					}
					countGpsData++;			//range関係なくカウント
				}
			}			//分布作成終了


		//#########################################
		//GPS未実装
		//分布評価

		}


		delay( Sec2Msec(10) );
	}
}

// residualErrorは残り半径
void RunUpNearGoal(double residualError) {
	int getGpsData = 0;			//GetGpsData()の返り値保存用
	unsigned long time = millis();

	while(1) {

		// ############################
		// ################# for DEBUG
		// SD を安全に取り外すため
		if (DEBUG == 1 && digitalRead(PIN_SWITCH) == 1) {
			MY_FILE.close();
			SetMoter(0,0,0,0);
			Serial.println("END");
			while(1) {}
		}
		// GPS取得
		getGpsData = 0;
		time = millis();
		// バッファを消すためGPSは3回とる
		while(getGpsData < 3) {
			if(millis() > time + Sec2Msec(10)) {		//これで10秒間とれなかったらループから外れる...はず
				break;
			}
			getGpsData += GetGpsData();
		}

		if (DEBUG) {
			if (getGpsData < 3) {
				Serial.println("getGpsData is under 3");
				Serial.println("#getGpsData is under 3");
			} else {
				Serial.println("GetGpsData OK!");
				Serial.println("#GetGpsData OK!");
			}
			Serial.print("Gps_Lat : ");
			Serial.println(Gps_Lat);
			Serial.print("#Gps_Lat : ");
			Serial.println(Gps_Lat);
			Serial.print("Gps_Long : ");
			Serial.println(Gps_Long);
			Serial.print("#Gps_Long : ");
			Serial.println(Gps_Long);
		}

		// 方向距離取得
		CalcDistanceDirection();

		delay(10);
		if (DEBUG) Serial.print("Gps_Distance : ");
		if (DEBUG) Serial.println(Gps_Distance);
		if (DEBUG) Serial.print("#Gps_Distance : ");
		if (DEBUG) Serial.println(Gps_Distance);
		if (DEBUG) Serial.print("Gps_Radian to Deg : ");
		if (DEBUG) Serial.println( Rad2Deg(Gps_Radian) );
		if (DEBUG) Serial.print("#Gps_Radian to Deg : ");
		if (DEBUG) Serial.println( Rad2Deg(Gps_Radian) );

		// 終了判定
		if (Gps_Distance < residualError) {
			if (DEBUG) Serial.println("break RunUpNearGoal()");
			if (DEBUG) Serial.println("#break RunUpNearGoal()");
			delay(10000);
			return;
		}

		// ローバー角度調整		40(maxCount)は要検討
		if ( SetRoverAngle(Gps_Radian, 40) < 38 ) {
			if (DEBUG) Serial.println("SetRoverAngle OK!");
			if (DEBUG) Serial.println("#SetRoverAngle OK!");
		} else {
			if (DEBUG) Serial.println("SetRoverAngle failed...");
			if (DEBUG) Serial.println("#SetRoverAngle failed...");
		}

		if (DEBUG) Serial.print("NowRad to deg : ");
		if (DEBUG) Serial.println( Rad2Deg( NormalizeRad( GetMagnetRad() ) ) );
		if (DEBUG) Serial.print("#NowRad to deg : ");
		if (DEBUG) Serial.println( Rad2Deg( NormalizeRad( GetMagnetRad() ) ) );
		delay(1000);

		// 進む
		SetMoter(255,0,255,0);
		// 進む距離適当
		delay( Meter2Msec(3.0) );

		SetMoter(0,0,0,0);
		delay(1000);

		// 回転，わだち回避
		SpinRover(1.0, Sec2Msec(5));
		delay(1000);
		SpinRover(-1.0, Sec2Msec(5));
		delay(1000);
	}
}

void ApproachGoal(double residualError) {

}

void FinalApproachToGoal(double residualError) {

}


// #########################################
// モーター系
void InitializeMoter() {
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

// maxCount は 角度調整回数の上限
int SetRoverAngle(double targetRad, int maxCount) {
	int dMSec = 50;				// 最小角度変更の際の回転時間(ミリ秒)
	double epsilonRad = Deg2Rad(10.0);	// 許容誤差 注意スべきは，許容誤差が最小角度変更より小さいと発振
	double dRad;
	int count = 0;
	double nowRad = NormalizeRadSym( GetMagnetRadNTimes(5) );
	targetRad = NormalizeRadSym(targetRad);

	dRad = NormalizeRadSym(targetRad - nowRad);
	while(count < maxCount && abs(dRad) > epsilonRad) {
		SpinRover(dRad, dMSec);
		count++;
		nowRad = NormalizeRadSym( GetMagnetRadNTimes(5) );
		dRad = NormalizeRadSym(targetRad - nowRad);
		delay(500);
	}

	return count;
}

void GoStraight(double targetDirection, unsigned long msec) {
	double gainP = 1;					//ここの値は適宜変える！正の値
	double gainI = 0.0001;
	double gainD = 0.01;
	double sumError = 0;
	double lastError = 0;
	int loopTime = 100;
	int count = 0;
	int maxCount = msec / loopTime;		//msecは100ミリ秒単位で指定、ループ回数はそれをlooptimeで割った数

	while(count < maxCount){
		double errorDirection = NormalizeRadSym(targetDirection - GetMagnetRad()) ;
//		if(abs(errorDirection) > M_PI){				//dDirectionをnormalizeしてなかった時の。
//			errorDirection = (-2*M_PI + abs(errorDirection)) * errorDirection / abs(errorDirection);
//		}
//		Serial.println(Rad2Deg( errorDirection ),6);

		sumError += errorDirection*loopTime;

		double inputPid = gainP*errorDirection + gainI*sumError + gainD*(errorDirection - lastError)/loopTime;

		if(inputPid > 0 || inputPid < M_PI/2){
//			if(DEBUG){
//				Serial.println("Turing RIGHT");
//			}
			SetMoter(255,0, 255 * (M_PI - inputPid) / M_PI,0);

		}else if(inputPid < 0 || inputPid > - M_PI/2){
//			if(DEBUG){
//				Serial.println("Turing LEFT");
//			}
			SetMoter(255 * (M_PI + inputPid) / M_PI,0, 255,0);		//負なので+

		}else if (inputPid >= M_PI/2){
			SetMoter(255,0,-255,0);
		}else if (inputPid <= M_PI/2){
			SetMoter(-255,0,255,0);

		}else{
			SetMoter(255,0,255,0);
		}

		if(gainI*sumError > M_PI/2 || gainI*sumError < -M_PI/2){
//			if(DEBUG){
//				Serial.println("Reset I controller");
//			}
			sumError = 0;
		}

		delay(loopTime);
		lastError = errorDirection;
		count ++;
	}
}

// 正(1.0)で右，負(-1.0)で左
void SpinRover(double LorR, unsigned long mSec) {
	if (LorR > 0) {
		SetMoter(255,0,0,255);
	} else {
		SetMoter(0,255,255,0);
	}
	delay(mSec);
	SetMoter(0,0,0,0);
}

void SetMoter(int l1, int l2, int r1, int r2) {
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

	Moter_Powers[0] = l1;
	Moter_Powers[1] = l2;
	Moter_Powers[2] = r1;
	Moter_Powers[3] = r2;
	return;
}


// #########################################
// 光系
int GetLight() {
	if(analogRead(PIN_LIGHT) > LIGHT_CRITICAL){
		return 1;
	}
	else{
		return 0;
	}
}


// #########################################
// 地磁気系

// 返り値はキャリブレーションの返り値
int InitializeMagnet() {
	int magnetTry = 1;
	int magnetCal = 0;
	Wire.begin();					// I2C通信起動
	I2cWrite(0x02,0x00);			// 地磁気センサの設定
	int magnetCalError = 1000;			// 地磁気キャリブレーションの許容エラー


	// ###########################
	// ###########################
	// 後で，キャリブレーションが無限ループにならないようにする
	// つまり，どんどん許容誤差を大きくするか，xMaxsのlenを大きくする


	while (1) {
		magnetCal = MagnetCalibration();
		Serial.print("! MAGNET_TRY_NUM : ");
		Serial.println(magnetTry);
		Serial.print("! MAGNET_CAL_ERR : ");
		Serial.println(magnetCal);
		Serial.print("! MAGNET_X_OFFSET : ");
		Serial.println(MAGNET_X_OFFSET);
		Serial.print("! MAGNET_Y_OFFSET : ");
		Serial.println(MAGNET_Y_OFFSET);
		Serial.print("! MAGNET_XY_RATIO : ");
		Serial.println(MAGNET_XY_RATIO);

		// #####################
		// あとで返り値

		if (magnetCal < magnetCalError) {
			break;
		}
		magnetTry++;
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
		magnetRad = NormalizeRad( GetMagnetRad() ) / (1.0*n);
		xSum += cos(magnetRad);
		ySum += sin(magnetRad);
		delay(50);
	}

	magnetRad = atan2(ySum,xSum);
	magnetRad = NormalizeRad(magnetRad);

	return magnetRad;
}

// 返り値は，データ配列の上位半分を抜いたrangeの最大値．
int MagnetCalibration() {
	const int arrLen = 2*5;		// 2で割り切れるのがいい．
	const int moterPower = 150;
	const unsigned long measuringTime = 8000;
	unsigned long time;
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
		} else {
			SetMoter(0,moterPower,moterPower,0);
		}

		time = millis();
		while(millis() < time + measuringTime) {
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
			delay(50);
		}

		SetMoter(0,0,0,0);
		delay(1000);
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
		count += GetGpsData();
	}

	if (count > 0) {
		return 1;
	} else {
		if (DEBUG) Serial.println("GPS Sencer Dead!");
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

	do{
		if(GpsSerialAvailable()){
			buf[count] = GpsSerialRead();
			count++;
		}

//		Serial.print(count);
//		Serial.print(",");
		//delay(1);						// これを外すと NormalizeRadSym でエラー
		delayMicroseconds(1);
		if (count > 250) break;
		if (millis() > time + 5000) {		// タイムアウト
			if (DEBUG) Serial.println("GPS TIMEOUT...");
			if (DEBUG) Serial.println("#GPS TIMEOUT...");
			return 0;
		}
	}while(buf[count - 1] != 0x0A);				//改行コード0x0Aが来たらひとまず受信終わり
	buf[count] = '\0';							//bufにgpsデータ保存

	if (DEBUG) {
		Serial.print("buf : ");						//受信したbuf確認用
		Serial.println(buf);
	}

	gpsArrLength = Split(buf, ",");

	if (gpsArrLength == 14 && !strcmp(Gps_Raw_Arr[0], "$GPGGA")) {		// 送られてきたデータが15個で，$GPGGAならtrue
		if (strcmp(Gps_Raw_Arr[1], "NULL") && strcmp(Gps_Raw_Arr[2], "NULL") && strcmp(Gps_Raw_Arr[3], "NULL") && strcmp(Gps_Raw_Arr[4], "NULL") && strcmp(Gps_Raw_Arr[5], "NULL") && strcmp(Gps_Raw_Arr[11], "NULL")) {
			SplitLong(Gps_Raw_Arr[2], ".");
			Gps_Lat = GpsLL2Deg(Gps_LongLong[0], Gps_LongLong[1]);			//char ddmm mmmm → long dddddddd
			SplitLong(Gps_Raw_Arr[4], ".");
			Gps_Long = GpsLL2Deg(Gps_LongLong[0], Gps_LongLong[1]);
			SplitLong(Gps_Raw_Arr[11], ".");
			Gps_Height = atol(Gps_LongLong[0]) + 0.01 * CutStr2Long(Gps_LongLong[1], 2);			//コピーできないので元配列から直接
			SplitLong(Gps_Raw_Arr[1], ".");
			Gps_Time = atol(Gps_LongLong[0]);

			if (strcmp(Gps_Raw_Arr[3], "N")) {			// S だとtrue
				Gps_Lat = -1 * Gps_Lat;					//南緯の場合負の値にする  一応南緯も対応...?(笑)
			}
			if (strcmp(Gps_Raw_Arr[5], "E")) {			// W だとtrue
				Gps_Long = -1 * Gps_Long;				//西経の場合負の値にする
			}

			if (DEBUG) Serial.println("GPS success");
			if (DEBUG) Serial.println("#GPS success");
			return 1;
		} else {
			if (DEBUG) Serial.println("GPS failed 1...");
			if (DEBUG) Serial.println("#GPS failed 1...");
			return 0;
		}
	} else {
		if (DEBUG) Serial.println("GPS failed 2...");
		if (DEBUG) Serial.println("#GPS failed 2...");
//		Serial.print("[0] : ");
//		Serial.println(Gps_Raw_Arr[0]);
//		Serial.print("[2] : ");
//		Serial.println(Gps_Raw_Arr[2]);
//		Serial.print("[4] : ");
//		Serial.println(Gps_Raw_Arr[4]);
//		Serial.print("[5] : ");
//		Serial.println(Gps_Raw_Arr[5]);
		return 0;
	}
}

//ターゲットと現在地の間の	距離の計算
void CalcDistanceDirection(){
//	double gapLat = TARGET_LATITUDE - Gps_Lat;
//	double gapLong = TARGET_LONGITUDE - Gps_Long;
//
//	double latRad = Deg2Rad( ddmm2dddd(Gps_Lat) );
//	double gapLatRad = Deg2Rad( ddmm2dddd(gapLat) );
//	double gapLongRad = Deg2Rad( ddmm2dddd(gapLong) );
//
//	double gapX = EARTH_RADIUS * gapLongRad * cos(latRad);			//直交座標として計算
//	double gapY = EARTH_RADIUS * gapLatRad;
	double gapLat = (TARGET_LATITUDE - Gps_Lat) * 1.0 / 1000000;		//差は最大でも100000と見積もって、doubleにする
	double gapLong = (TARGET_LONGITUDE - Gps_Long) * 1.0 / 1000000;

	double latRad = Deg2Rad(Gps_Lat * 1.0 / 1000000);
	double gapLatRad = Deg2Rad(gapLat);
	double gapLongRad = Deg2Rad(gapLong);

	double gapX = EARTH_RADIUS * gapLongRad * cos(latRad);			//直交座標として計算
	double gapY = EARTH_RADIUS * gapLatRad;
	Gps_Distance = sqrt(gapX*gapX + gapY*gapY);

	Gps_Radian = NormalizeRad( atan2(gapX, gapY) );

//	Serial.print("gapLat : ");
//	Serial.print(gapLat, 6);
//	Serial.print(", gapLong : ");
//	Serial.print(gapLong, 6);
//	Serial.print(", gapX : ");
//	Serial.print(gapX);
//	Serial.print(", gapY : ");
//	Serial.print(gapY);
//	Serial.println("");

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
//		Serial.print(count);
//		Serial.print(" : ");
//		Serial.print(splitString);
//		Serial.print("\n");
		//###################

		// GPS RAW DATA配列更新
		// 安全なコピー．バッファローオーバーラン対策．
		strncpy(Gps_Raw_Arr[count], splitString, GPS_RAW_ARR_BUF_LENGTH - 1);
		Gps_Raw_Arr[count][GPS_RAW_ARR_BUF_LENGTH - 1] =  '\0';
		count++;
//		delay(500);
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
//		Serial.print(count);
//		Serial.print(" : ");
//		Serial.print(splitString);
//		Serial.print("\n");
		//###################

		// GPS RAW DATA配列更新
		// 安全なコピー．バッファローオーバーラン対策．
		strncpy(Gps_LongLong[count], splitString, GPS_RAW_ARR_BUF_LENGTH - 1);
		Gps_LongLong[count][GPS_RAW_ARR_BUF_LENGTH - 1] = '\0';
		count++;
//		delay(500);
	}

	return (count-1);
}

// #########################################
// 加速度センサ系


// #########################################
// ブザー系
void ToneAlert() {
	int dt = 100;
	for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 3; j++) {
			tone(PIN_BUZZER, 1000);
			delay(dt);
			noTone(PIN_BUZZER);
			delay(dt);
		}
		delay(dt * (6-1));
	}
	return;
}



// #########################################
// ログ系
int InitializeSd() {
	pinMode(PIN_SD_SS, OUTPUT);
	if (!SD.begin(PIN_SD_SS)) {
		if (DEBUG) Serial.println("SD.begin failed!");
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
	if (DEBUG) Serial.println("SdOpenFailed");
	MY_FILE.close();		// ねんのため
	ToneAlert();
}

void CacheOn() {
	Log_To_Sd = 0;
	return;
}

int CacheOff() {
	char str;
	Log_To_Sd = 1;
	// #########################
	// ここにcacheのSD書き込み処理
	if (Cache_Size == 0) {
		return 1;
	}
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return 0;
	}

	// Cacheアドレスの連続化
	if (Cache_End_Address < Cache_Start_Address) {
		Cache_End_Address += MAX_EEPROM_ADDRESS;
	}

	if (DEBUG) Serial.println("Cache to SD begin...");
	for (int i = Cache_Start_Address; i <= Cache_End_Address; i++) {
		str = EEPROM.read( i % MAX_EEPROM_ADDRESS );
		MY_FILE.print(str);
		if (DEBUG) Serial.print(str);
	}
	if (DEBUG) Serial.println("Cache to SD complete!");
	MY_FILE.close();

	Cache_Start_Address = (Cache_End_Address + 1) % MAX_EEPROM_ADDRESS;
	Cache_End_Address = Cache_End_Address % MAX_EEPROM_ADDRESS;
	Cache_Size = 0;
	Is_Cache_Full = 0;
	return 1;
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

	tone(PIN_BUZZER, 440);
	delay(2000);

	noTone(PIN_BUZZER);
	tone(PIN_BUZZER, 880);

	result = CacheOff();
	CacheOn();
	delay(2000);

	noTone(PIN_BUZZER);
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

int LogSta(char *string) {
	if (Log_To_Sd) {
		MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
		if (!MY_FILE) {
			SdOpenFailed();
			return 0;
		}
		MY_FILE.print(millis());
		MY_FILE.print(",Sta,");
		MY_FILE.print(string);
		MY_FILE.print("\n");
		MY_FILE.close();
		return 1;
	} else {
		MoveFullCacheToSd();
		String str;
		str = String(millis());
		str += String(",Sta,");
		str += String(string);
		str += String("\n");
		LogCache(str.c_str());
		return 1;
	}
}

int LogGps() {
	// 配列の長さを変更する場合，下のfor内も変更すること忘れずに
	char stringGpsHeight[20];
	for (int i=0; i<20; i++) {
		stringGpsHeight[i] = '\0';
	}
	sprintf(stringGpsHeight, "%.3f", Gps_Height);

	if (Log_To_Sd) {
		MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
		if (!MY_FILE) {
			SdOpenFailed();
			return 0;
		}
		MY_FILE.print(millis());
		MY_FILE.print(",Gps,");
		MY_FILE.print(Gps_Time);
		MY_FILE.print(",");
		MY_FILE.print(Gps_Lat);
		MY_FILE.print(",");
		MY_FILE.print(Gps_Long);
		MY_FILE.print(",");
		MY_FILE.print(stringGpsHeight);
		MY_FILE.print("\n");
		MY_FILE.close();
		return 1;
	} else {
		MoveFullCacheToSd();
		String str;
		str = String(millis());
		str += String(",Gps,");
		str += String(Gps_Time);
		str += String(",");
		str += String(Gps_Lat);
		str += String(",");
		str += String(Gps_Long);
		str += String(",");
		str += String(stringGpsHeight);
		str += String("\n");
		LogCache(str.c_str());
		return 1;
	}
}

int LogMagRaw() {
	if (Log_To_Sd) {
		MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
		if (!MY_FILE) {
			SdOpenFailed();
			return 0;
		}
		MY_FILE.print(millis());
		MY_FILE.print(",MagRaw,");
		MY_FILE.print(Magnet_X12);
		MY_FILE.print(",");
		MY_FILE.print(Magnet_Y12);
		MY_FILE.print(",");
		MY_FILE.print(Magnet_Z12);
		MY_FILE.print("\n");
		MY_FILE.close();
		return 1;
	} else {
		MoveFullCacheToSd();
		String str;
		str = String(millis());
		str += String(",MagRaw,");
		str += String(Magnet_X12);
		str += String(",");
		str += String(Magnet_Y12);
		str += String(",");
		str += String(Magnet_Z12);
		str += String("\n");
		LogCache(str.c_str());
		return 1;
	}
}

int LogMagAvg(int n, double rad) {
	if (Log_To_Sd) {
		MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
		if (!MY_FILE) {
			SdOpenFailed();
			return 0;
		}
		MY_FILE.print(millis());
		MY_FILE.print(",MagAvg,");
		MY_FILE.print(n);
		MY_FILE.print(",");
		MY_FILE.print((long)(rad * 1000));
		MY_FILE.print("\n");
		MY_FILE.close();
		return 1;
	} else {
		MoveFullCacheToSd();
		String str;
		str = String(millis());
		str += String(",MagAvg,");
		str += String(n);
		str += String(",");
		str += String((long)(rad * 1000));
		str += String("\n");
		LogCache(str.c_str());
		return 1;
	}
}

int LogMagCal(int maxRange, int *xMaxs, int *yMaxs, int *xMins, int *yMins, int arrLen) {
	if (Log_To_Sd) {
		MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
		if (!MY_FILE) {
			SdOpenFailed();
			return 0;
		}
		MY_FILE.print(millis());
		MY_FILE.print(",MagCal,");
		MY_FILE.print((long)(MAGNET_X_OFFSET * 1000));
		MY_FILE.print(",");
		MY_FILE.print((long)(MAGNET_Y_OFFSET * 1000));
		MY_FILE.print(",");
		MY_FILE.print((long)(MAGNET_XY_RATIO * 1000));
		MY_FILE.print(",");
		MY_FILE.print(maxRange);
		MY_FILE.print(",xMaxs");
		for (int i=0; i<arrLen; i++) {
			MY_FILE.print(",");
			MY_FILE.print(xMaxs[i]);
		}
		MY_FILE.print(",xMins");
		for (int i=0; i<arrLen; i++) {
			MY_FILE.print(",");
			MY_FILE.print(xMins[i]);
		}
		MY_FILE.print(",yMaxs");
		for (int i=0; i<arrLen; i++) {
			MY_FILE.print(",");
			MY_FILE.print(yMaxs[i]);
		}
		MY_FILE.print(",yMins");
		for (int i=0; i<arrLen; i++) {
			MY_FILE.print(",");
			MY_FILE.print(yMins[i]);
		}
		MY_FILE.print("\n");
		MY_FILE.close();
		return 1;
	} else {
		MoveFullCacheToSd();
		String str;
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
		LogCache(str.c_str());
		return 1;
	}
}

int LogAcc() {
	if (Log_To_Sd) {
		MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
		if (!MY_FILE) {
			SdOpenFailed();
			return 0;
		}
		MY_FILE.print(millis());
		MY_FILE.print(",Acc,");
		MY_FILE.print(Acc_X);
		MY_FILE.print(",");
		MY_FILE.print(Acc_Y);
		MY_FILE.print(",");
		MY_FILE.print(Acc_Z);
		MY_FILE.print("\n");
		MY_FILE.close();
		return 1;
	} else {
		MoveFullCacheToSd();
		String str;
		str = String(millis());
		str += String(",Acc,");
		str += String(Acc_X);
		str += String(",");
		str += String(Acc_Y);
		str += String(",");
		str += String(Acc_Z);
		str += String("\n");
		LogCache(str.c_str());
		return 1;
	}
}

int LogPho(char *fileName, int ok) {
	// cache書き込み不可
	if (Log_To_Sd) {
		return 0;
	}

	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return 0;
	}
	MY_FILE.print(millis());
	MY_FILE.print(",Pho,");
	MY_FILE.print(fileName);
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
double ddmm2dddd(double ddmm){
	return (int)(ddmm) + (ddmm - (int)ddmm) / 0.6;
}

// ローバー走行距離 メートル → ローバー走行時間 msec
unsigned long Meter2Msec(double meter) {
	// 2km/h = 2000/3600 m/s = 0.55556 m/s を仮定
	return (unsigned long)(1000 * meter / 0.55556);
}

// ローバー走行距離 メートル ← ローバー走行時間 msec
double Msec2Meter(int msec) {
	// 2km/h = 2000/3600 m/s = 0.55556 m/s を仮定
	return (msec * 1.0) * 0.5556 / 1000.0;
}

unsigned long Sec2Msec(double Sec) {
	return (unsigned long)(Sec * 1000);
}





// To Do List
// 6/15
//	GPS ddmm.mmmmをlong dddddddddに
//	GPS Target座標を入れるときに注意


// ###########
// println と print("\n")の改行コードが一致してるか調べる．
// GPSの時間もログ
// max min absは注意


// magnetInitはまだ未完成 無限ループの恐れあり
// LogMagCalは未テスト


// SDがしんだらずっとFULLっていわれる
// GPS LOG で HIGHT が sprintf