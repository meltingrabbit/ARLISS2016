#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SPI.h>
#include <SD.h>
//#include <SoftwareSerial.h>		// for GPS Serial通信

// #########################################
// 関数プロトタイプ宣言
// #########################################
// ミッション遂行系
void RunUpNearGoal(double residualError);
void ApproachGoal(double residualError);
void FinalApproachToGoal(double residualError);
// 地磁気系
double GetMagnetRad();
double GetMagnetRadNTimes(int n);
// モーター系
int SetRoverAngle(double targetRad, int maxCount);
void GoStraight(double targetDirection, unsigned long msec);
void SpinRover(double LorR, unsigned long mSec);
void SetMoter(int l1, int l2, int r1, int r2);
// GPS系
int InitializeGps();
int GetGpsData();
int Split(char *string, char *separator);
void SendUnable(int datatype);
char ChecksumCalc(char *dt);
void CalcDistanceDirection();
// Log系
int InitializeSd();
void SdOpenFailed();
void LogSta(char *string);
void LogGps();
void LogMagRaw();
void LogMagAvg(int n, double rad);
void LogAcc();
void LogPho(int ok);

// ###########
// println と print("\n")の改行コードが一致してるか調べる．


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
unsigned long Sec2Msec(int Sec);
// #########################################

// #########################################
// 定数定義
// #########################################

#define DEBUG 1

#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート

// スイッチ
#define PIN_SWITCH 35

// 加速度センサ
#define PIN_ACC_Z 0
#define PIN_ACC_Y 2
#define PIN_ACC_X 4

// モーター
#define PIN_MOTER_LEFT1 12
#define PIN_MOTER_LEFT2 10
#define PIN_MOTER_RIGHT1 7
#define PIN_MOTER_RIGHT2 5

// 地磁気センサ
#define HMC5883L_ADDRESS 0x1E		// 7bit ADDRESS for I2C通信の素子識別アドレス

// CdS光センサ
#define ANALOG_PIN_CDS 1			// 光センサのアナログin

// GPS
#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
#define GpsSerialRead() Serial1.read()
#define GpsSerialAvailable() Serial1.available()
#define GpsSerialPrint(gpsSendData) Serial1.print(gpsSendData)

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

// 地磁気センサ
const double MAGNET_DECLINATION = 7.0;					// 東京で西偏7度0分
int Magnet_X12 = 0;
int Magnet_Z12 = 0;
int Magnet_Y12 = 0;

// 加速度センサ
long Acc_X = 0;
long Acc_Z = 0;
long Acc_Y = 0;

// GPS
const long EARTH_RADIUS = 6378137;
#define GPS_RAW_ARR_LENGTH 20					// Gps_Raw_Arrの長さ
#define GPS_RAW_ARR_BUF_LENGTH 20				// Gps_Raw_Arrの値の長さ
char Gps_Raw_Arr[GPS_RAW_ARR_LENGTH][GPS_RAW_ARR_BUF_LENGTH];
double Gps_Lat;									//
double Gps_Long;								//dd.mmmmmmにすること
double Gps_Height;								//
double Gps_Distance;							//センサと目的地間距離
double Gps_Radian;								//センサから目的地までの方位
const double TARGET_LATITUDE = 35.428045;
const double TARGET_LONGITUDE = 139.457297;		//dd,mmmmmm安田講堂塔真ん中(D, D)=(1602.1, 4.48286)

// Log
File MY_FILE;
char FLIE_NAME[] = "LOG.CSV";
// #########################################


void setup() {
	int countSwitch = 0;
	delay(1000);

	// ボタンが押されて，安定したらスタート
	while(1) {
		countSwitch = 0;
		for (int i = 0; i < 10; i++) {
			countSwitch += digitalRead(PIN_SWITCH);
			delay(50);
		}
		if (countSwitch == 0) {
			break;
		}
	}

	int error = 1;
	Serial.begin(SERIAL_BAUDRATE);
	Wire.begin();					// I2C通信起動
	I2cWrite(0x02,0x00);			// 地磁気センサの設定
	if (DEBUG) Serial.println("Switch ON!");

	// SD初期化
	// エラーの場合，音を鳴らして警告させたい
	error = InitializeSd();
	if (error == 0) {
		Serial.println("initialization SD failed!");
		// #########################
		// ここに警告音
	}

	LogSta("Switch ON!");						// ほんとは，SwitchONはもっと前だけど，SDの初期化終わらないと書き込めないのよね....w
	LogSta("Initialize SD OK!");
	if (DEBUG) Serial.println("Initialize SD OK!");

	// GPS初期化
	error = InitializeGps();
	if (error == 0) {
		Serial.println("initialization GPS failed!");
		// #########################
		// ここに警告音
	}

	LogSta("Initialize GPS OK!");
	if (DEBUG) Serial.println("Initialize GPS OK!");

	// モーターのピン設定
	pinMode(PIN_MOTER_LEFT1, OUTPUT);
	pinMode(PIN_MOTER_LEFT2, OUTPUT);
	pinMode(PIN_MOTER_RIGHT1, OUTPUT);
	pinMode(PIN_MOTER_RIGHT2, OUTPUT);
	SetMoter(0,0,0,0);

	// ###########################
	// 初期化完了の音を鳴らしたい
	LogSta("Initialization Finish!");
	if (DEBUG) Serial.println("Initialization Finish!");

	delay(100);
}


void loop() {
	double magnetRad;

//	Serial.print("Switch : ");
//	Serial.println(digitalRead(PIN_SWITCH));
//	GetGpsData();
	Acc_X = analogRead(PIN_ACC_X);
	Acc_Z = analogRead(PIN_ACC_Y);
	Acc_Y = analogRead(PIN_ACC_Z);
	Serial.print("Acc : ");
	Serial.print(Acc_X);
	Serial.print(", ");
	Serial.print(Acc_Y);
	Serial.print(", ");
	Serial.println(Acc_Z);
//	magnetRad = Rad2Deg( NormalizeRad( GetMagnetRad()));
//	Serial.print("Magnet : ");
//	Serial.print(Magnet_X12);
//	Serial.print(", ");
//	Serial.print(Magnet_Y12);
//	Serial.print(", ");
//	Serial.print(Magnet_Z12);
//	Serial.print(", ");
//	Serial.println(magnetRad);

}




// #########################################
// ミッション遂行系
// residualErrorは残り半径
void RunUpNearGoal(double residualError) {
	int getGpsData = 0;			//GetGpsData()の返り値保存用
	unsigned long time = millis();

	while(1) {
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
			} else {
				Serial.println("GetGpsData OK!");
			}
			Serial.print("Gps_Lat : ");
			Serial.println(Gps_Lat);
			Serial.print("Gps_Long : ");
			Serial.println(Gps_Long);
		}

		// 方向距離取得
		CalcDistanceDirection();

		// 終了判定
		if (Gps_Distance < residualError) {
			if (DEBUG) Serial.println("break RunUpNearGoal()");
			delay(10000);
			return;
		}

		// ローバー角度調整		25(maxCount)は要検討
		if ( SetRoverAngle(Gps_Radian, 25) < 20 ) {
			if (DEBUG) Serial.println("SetRoverAngle OK!");
		} else {
			if (DEBUG) Serial.println("SetRoverAngle failed...");
		}

		if (DEBUG) Serial.print("NowRad : ");
		if (DEBUG) Serial.println(NormalizeRadSym( GetMagnetRad() ));
		delay(1000);

		// 進む
		SetMoter(255,0,255,0);
		// 進む距離適当
		delay( Meter2Msec(10.0) );

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
// 地磁気系
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
	xDouble = Magnet_X12;
	yDouble = Magnet_Y12;

	magnetRad = atan2(yDouble,xDouble);
	magnetRad += MAGNET_DECLINATION;		// 偏角補正

	return magnetRad;
}

// 地磁気をN回取得し，平均
double GetMagnetRadNTimes(int n) {
	double result = 0;
	for(int i = 0; i < n; i++) {
		result += NormalizeRad( GetMagnetRad() ) / n;
		delay(10);
	}
	return result;
}



// #########################################
// モーター系
// maxCount は 角度調整回数の上限
int SetRoverAngle(double targetRad, int maxCount) {
	int dMSec = 100;				// 最小角度変更の際の回転時間(ミリ秒)
	double epsilonRad = 0.174533;	// 許容誤差 注意スべきは，許容誤差が最小角度変更より小さいと発振
	double dRad;
	int count = 0;
	double nowRad = NormalizeRadSym( GetMagnetRadNTimes(5) );

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
		double errorDirection = NormalizeRad(-targetDirection + GetMagnetRad()) ;
//		if(abs(errorDirection) > M_PI){				//dDirectionをnormalizeしてなかった時の。
//			errorDirection = (-2*M_PI + abs(errorDirection)) * errorDirection / abs(errorDirection);
//		}
		if(errorDirection > M_PI){
			errorDirection =  errorDirection - 2*M_PI;
		}
	//		Serial.println(Rad2Deg( errorDirection ),6);

		sumError += errorDirection*looptime;

		double inputPid = gainP*errorDirection + gainI*sumError + gainD*(errorDirection + lastError)/loopTime;

		if(inputPid > 0){
			analogWrite(PIN_MOTER_LEFT1,255 * (M_PI - inputPid) / M_PI);
			analogWrite(PIN_MOTER_RIGHT1,255);

		}else if(inputPid < 0){
			analogWrite(PIN_MOTER_LEFT1,255);
			analogWrite(PIN_MOTER_RIGHT1,255 * (M_PI + inputPid) / M_PI);		//負なので+

		}else{
			analogWrite(PIN_MOTER_LEFT1,255);
			analogWrite(PIN_MOTER_RIGHT1,255);
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
	analogWrite(PIN_MOTER_LEFT1, l1);
	analogWrite(PIN_MOTER_LEFT2, l2);
	analogWrite(PIN_MOTER_RIGHT1, r1);
	analogWrite(PIN_MOTER_RIGHT2, r2);
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

int GetGpsData() {
	char buf[256];			//受信データ用
	int count = 0;
	int gpsArrLength;
	unsigned long time = millis();

	// #########################################
	// ここにタイムアウトの実装
	do{
		if(GpsSerialAvailable()){
			buf[count] = GpsSerialRead();
			count++;
		}
		if (count > 250) break;
		if (millis() > time + 5000) {		// タイムアウト
			if (DEBUG) Serial.println("GPS TIMEOUT...");
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
		if (strcmp(Gps_Raw_Arr[2], "NULL") && strcmp(Gps_Raw_Arr[4], "NULL") && strcmp(Gps_Raw_Arr[5], "NULL") && strcmp(Gps_Raw_Arr[5], "NULL") && strcmp(Gps_Raw_Arr[11], "NULL")) {

			Gps_Lat = 0.01 * atof(Gps_Raw_Arr[2]);			//char ddmm.mmmm → double dd.mmmmmm
			Gps_Long = 0.01 * atof(Gps_Raw_Arr[4]);			//コピーできないので元配列から直接
			Gps_Height = atof(Gps_Raw_Arr[11]);			//コピーできないので元配列から直接

			if (strcmp(Gps_Raw_Arr[3], "N")) {			// S だとtrue
				Gps_Lat = -1 * Gps_Lat;					//南緯の場合負の値にする  一応南緯も対応...?(笑)
			}
			if (strcmp(Gps_Raw_Arr[5], "E")) {			// W だとtrue
				Gps_Long = -1 * Gps_Long;				//西経の場合負の値にする
			}

			if (DEBUG) Serial.println("GPS success");
			return 1;
		} else {
			if (DEBUG) Serial.println("GPS failed 1...");
			return 0;
		}
	} else {
		if (DEBUG) Serial.println("GPS failed 2...");
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
	double gapLat = TARGET_LATITUDE - Gps_Lat;
	double gapLong = TARGET_LONGITUDE - Gps_Long;

	double latRad = Deg2Rad( ddmm2dddd(Gps_Lat) );
	double gapLatRad = Deg2Rad( ddmm2dddd(gapLat) );
	double gapLongRad = Deg2Rad( ddmm2dddd(gapLong) );

	double gapX = EARTH_RADIUS * gapLongRad * cos(latRad);			//直交座標として計算
	double gapY = EARTH_RADIUS * gapLatRad;
	Gps_Distance = sqrt(gapX*gapX + gapY*gapY);

	Gps_Radian = NormalizeRad( atan2(gapX, gapY) );
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
	// #########################
	// ここに警告音
}

void LogSta(char *string) {
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return;
	}
	MY_FILE.print(millis());
	MY_FILE.print(",Sta,");
	MY_FILE.print(string);
	MY_FILE.println("");
	MY_FILE.close();
	return;
}

void LogGps() {
	double kazu;
	int seisu;
	long shousu;
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return;
	}
	// 小数は書き込めないので変換必須
	MY_FILE.print(millis());
	MY_FILE.print(",Gps,");

	kazu = Gps_Lat;
	seisu = (int)kazu;
	shousu = abs( (kazu - (int)kazu)*1000000 );
	MY_FILE.print(seisu);
	MY_FILE.print(".");
	MY_FILE.print(shousu);

	MY_FILE.print(",");

	kazu = Gps_Long;
	seisu = (int)kazu;
	shousu = abs( (kazu - (int)kazu)*1000000 );
	MY_FILE.print(seisu);
	MY_FILE.print(".");
	MY_FILE.print(shousu);

	MY_FILE.print(",");

	kazu = Gps_Height;
	seisu = (int)kazu;
	shousu = abs( (kazu - (int)kazu)*1000000 );
	MY_FILE.print(seisu);
	MY_FILE.print(".");
	MY_FILE.print(shousu);

	MY_FILE.println("");
	MY_FILE.close();
	return;
}

void LogMagRaw() {
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return;
	}
	MY_FILE.print(millis());
	MY_FILE.print(",MagRaw,");
	MY_FILE.print(Magnet_X12);
	MY_FILE.print(",");
	MY_FILE.print(Magnet_Y12);
	MY_FILE.print(",");
	MY_FILE.print(Magnet_Z12);
	MY_FILE.println("");
	MY_FILE.close();
	return;
}

void LogMagAvg(int n, double rad) {
	double kazu;
	int seisu;
	long shousu;
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return;
	}
	MY_FILE.print(millis());
	MY_FILE.print(",MagAvg,");
	MY_FILE.print(n);
	MY_FILE.print(",");

	kazu = rad;
	seisu = (int)kazu;
	shousu = abs( (kazu - (int)kazu)*1000000 );
	MY_FILE.print(seisu);
	MY_FILE.print(".");
	MY_FILE.print(shousu);

	MY_FILE.println("");
	MY_FILE.close();
	return;
}

void LogAcc() {
	// #########################
}

void LogPho(int ok) {
	MY_FILE = SD.open(FLIE_NAME, FILE_WRITE);
	if (!MY_FILE) {
		SdOpenFailed();
		return;
	}
	MY_FILE.print(millis());
	MY_FILE.print(",Pho,");
	MY_FILE.print(ok);
	MY_FILE.println("");
	MY_FILE.close();
	return;
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
		result -= 2*M_PI;
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


