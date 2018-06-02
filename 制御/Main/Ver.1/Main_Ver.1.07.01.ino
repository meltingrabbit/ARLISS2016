#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
//#include <SoftwareSerial.h>		// for GPS Serial通信

// #########################################
// 関数プロトタイプ宣言
// #########################################
unsigned char I2cRead(unsigned char regAddress);					// I2C read
void I2cWrite(unsigned char regAddress, unsigned char data);		// I2C write
double GetMagnetRad();											// 地磁気データを取得
double GetMagnetRadNTimes(int n);
void SendUnable(int datatype);
char ChecksumCalc(char *dt);
void CalDistanceDirection();
double NormalizeRad(double input);
double NormalizeRadSym(double input);
double Deg2Rad(double deg);
double Rad2Deg(double rad);
double ddmm2dddd(double ddmm);
int GetGpsData();
int Split(char *string, char *separator);
int SetRoverAngle(double targetRad);
void SpinRover(double LorR, int mSec);
void SetMoter(int l1, int l2, int r1, int r2);
unsigned long Meter2Msec(double meter);
double Msec2Meter(int msec);
// #########################################

// #########################################
// 定数定義
// #########################################

#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート

// モーター
#define PIN_MOTER_LEFT1 2
#define PIN_MOTER_LEFT2 3
#define PIN_MOTER_RIGHT1 4
#define PIN_MOTER_RIGHT2 5

// 地磁気センサ
#define HMC5883L_ADDRESS 0x1E		// 7bit ADDRESS for I2C通信の素子識別アドレス
const double MAGNET_DECLINATION = 7.0;					// 東京で西偏7度0分

// CdS光センサ
#define ANALOG_PIN_CDS 1			// 光センサのアナログin

// GPS
#define GpsSerialBegin(baudrate) Serial3.begin(baudrate)
#define GpsSerialRead() Serial3.read()
#define GpsSerialAvailable() Serial3.available()
#define GpsSerialPrint(gpsSendData) Serial3.print(gpsSendData)
// #########################################

// #########################################
// Global変数定義
// #########################################

// 地磁気センサ
int Magnet_X12 = 0;
int Magnet_Z12 = 0;
int Magnet_Y12 = 0;

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
// #########################################


void setup() {
	delay(1000);
	Wire.begin();					// I2C通信起動
	I2cWrite(0x02,0x00);			// 地磁気センサの設定

	Serial.begin(SERIAL_BAUDRATE);
	GpsSerialBegin(SERIAL_BAUDRATE);			// Serial通信の開始
	SendUnable(3);								//GPS:3.GSV,2.GSA,4.RMCの無効化
	SendUnable(2);
	SendUnable(4);

	pinMode(PIN_MOTER_LEFT1, OUTPUT);		// sets the pin as output
	pinMode(PIN_MOTER_LEFT2, OUTPUT);		// sets the pin as output
	pinMode(PIN_MOTER_RIGHT1, OUTPUT);		// sets the pin as output
	pinMode(PIN_MOTER_RIGHT2, OUTPUT);		// sets the pin as output

	SetMoter(0,0,0,0);

	delay(100);
}


void loop() {


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
	}
	return result;
}



// #########################################
// モーター系
int SetRoverAngle(double targetRad) {
	int dMSec = 100;				// 最小角度変更の際の回転時間(ミリ秒)
	double epsilonRad = 0.174533;	// 許容誤差 注意スべきは，許容誤差が最小角度変更より小さいと発振
	double dRad;
	int maxCount = 30;				// 角度調整回数の上限
	int count = 0;
	double nowRad = NormalizeRadSym( GetMagnetRad() );

	dRad = NormalizeRadSym(targetRad - nowRad);
	while(count < maxCount && abs(dRad) > epsilonRad) {

		SpinRover(dRad, dMSec);

		count++;
		nowRad = NormalizeRadSym( GetMagnetRad() );
		dRad = NormalizeRadSym(targetRad - nowRad);
		delay(500);
	}

	return count;
}

// 正(1.0)で右，負(-1.0)で左
void SpinRover(double LorR, int mSec) {
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
int GetGpsData() {
	char buf[256];			//受信データ用
	int count = 0;
	int gpsArrLength;

	do{
		if(GpsSerialAvailable()){
		buf[count]= GpsSerialRead();
		count++;
		}
	if(count > 250) break;
	}while(buf[count - 1] != 0x0A);				//改行コード0x0Aが来たらひとまず受信終わり
	buf[count] = '\0';							//bufにgpsデータ保存

//	Serial.print("buf : ");						//受信したbuf確認用
//	Serial.println(buf);

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

//			Serial.println("GPS success");
			return 1;
		} else {
//			Serial.println("GPS failed 1...");
			return 0;
		}
	} else {
//		Serial.println("GPS failed 2...");
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
void CalDistanceDirection(){
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
		strcpy(Gps_Raw_Arr[count], splitString);
		count++;
//		delay(500);
	}

	return (count-1);
}


// #########################################
// ログ系


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