/*
【テスト２】
目的：
直進フィードバックのテスト（P制御）
Pゲインの値推定
使用素子：
モーター、モータードライバ、地磁気、マイコン、（Xbee）
概要：
走行中、地磁気の値を用いてP制御をかける。
ゲインの調整を試み、今後の直進フィードバックの改善にいかす。
*/

#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
//#include <SoftwareSerial.h>		// for GPS Serial通信

// #########################################
// 関数プロトタイプ宣言
// #########################################
unsigned char I2cRead(unsigned char regAddress);					// I2C read
void I2cWrite(unsigned char regAddress, unsigned char data);		// I2C write
double GetMagnetRad();								// 地磁気データを取得
// #########################################

// #########################################
// 定数定義
// #########################################
//モータードライバのピン番号指定
#define PIN_MOTER_LEFT1 4	// L1/R1を1、L2/R2を0にしたときに正転するようにL1/R1を設定する
#define PIN_MOTER_LEFT2 5
#define PIN_MOTER_RIGHT1 6
#define PIN_MOTER_RIGHT2 7

#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート

// 地磁気センサ
#define HMC5883L_ADDRESS 0x1E		// 7bit ADDRESS for I2C通信の素子識別アドレス

#define MAGNET_SENSOR_DIRECTION 0.0			//磁気センサ取り付け向き しっぽの軸をx軸正の向きとして、センサのx軸がどれだけずれてるか Radianで

// CdS光センサ
//#define ANALOG_PIN_CDS 1			// 光センサのアナログin

// GPS
//#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
//#define GpsSerialRead() Serial1.read()
//#define GpsSerialAvailable() Serial1.available()
//#define PIN_GPS_RX 10				// GPS Rx (GPSのData Out) のピン番号
//#define PIN_GPS_TX 11				// GPS Tx (GPSのData In) のピン番号
// #########################################

// #########################################
// Global変数定義
// #########################################

// 地磁気センサ


// GPS
double Gps_Radian = 0.0;			//　ここの値も変える！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！　
// #########################################


// #########################################
// GPSセンサの設定
// #########################################
//SoftwareSerial g_gps(PIN_GPS_RX, PIN_GPS_TX);                          //(rx,tx)
//SoftwareSerial g_gps(6, 7);                          //(rx,tx)
// #########################################

void setup() {
	delay(1000);
	Wire.begin();					// I2C通信起動
	I2cWrite(0x02,0x00);			// 地磁気センサの設定

	Serial.begin(SERIAL_BAUDRATE);
//	GpsSerialBegin(SERIAL_BAUDRATE);			// Serial通信の開始

	delay(100);
}


void loop() {

	// arduinoの仕様がよくわからんのだが，
	// loop関数内でnew変数定義して回してるとメモリーリーク起こすかもしれないのでwhileでくくった
	// うーん，測定値データとかはグローバルに宣言してもいいかもなぁ...

	analogWrite(PIN_MOTER_LEFT2,0);
	analogWrite(PIN_MOTER_RIGHT2,0);

	while(1) {
		double nowDirection = NormalizeRad( M_PI/2 - (GetMagnetRad() + MAGNET_SENSOR_DIRECTION) );		//緯度経度（地球座標）での。緯度方向がy軸正  かなり自信ない
		double dDirection = NormalizeRad(Gps_Radian - nowDirection) - M_PI;
		float gain = 0.5;					//　ここの値を変える！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！正の値　

//		if(abs(dDirection) > M_PI){				//dDirectionをnormalizeしてなかった時の。
//			dDirection = (-2*M_PI + abs(dDirection)) * dDirection / abs(dDirection);
//		}

		if(dDirection > 0){
			analogWrite(PIN_MOTER_LEFT1,255*(1-gain*dDirection/M_PI));
			analogWrite(PIN_MOTER_RIGHT1,255);

		}else if(dDirection < 0){
			analogWrite(PIN_MOTER_LEFT1,255);
			analogWrite(PIN_MOTER_RIGHT1,255*(1+gain*dDirection/M_PI));		//負なので+

		}else{
			analogWrite(PIN_MOTER_LEFT1,255);
			analogWrite(PIN_MOTER_RIGHT1,255);
		}
		//delay(1000);				//　いるかも　
	}
}


// 動作確認 未テスト
double GetMagnetRad() {
	double magnetRad;

	unsigned char xMsb;
	unsigned char xLsb;
	unsigned char zMsb;
	unsigned char zLsb;
	unsigned char yMsb;
	unsigned char yLsb;
	int x12;
	int z12;
	int y12;
	double xDouble;
	double yDouble;

	// 順番がXZYなので注意！
	xMsb = I2cRead(0x03);
	xLsb = I2cRead(0x04);
	zMsb = I2cRead(0x05);
	zLsb = I2cRead(0x06);
	yMsb = I2cRead(0x07);
	yLsb = I2cRead(0x08);

	x12 = xMsb;
	x12 = ((x12 << 8) & 0xFF00) | xLsb; //SHIFT & GET 12bit DATA WITH MSB
	y12 = yMsb;
	y12 = ((y12 << 8) & 0xFF00) | yLsb; //SHIFT & GET 12bit DATA WITH MSB
	z12 = zMsb;
	z12 = ((z12 << 8) & 0xFF00) | zLsb; //SHIFT & GET 12bit DATA WITH MSB

	//CONVERT TO DOUBLE (FOR atan2)
	xDouble = x12;
	yDouble = y12;


//	RAD_RESULT = atan2(Y_DOUBLE,X_DOUBLE); //GET RADIAN
//	DEG_RESULT = RAD_RESULT * 180 /3.142;  //GET DEGREE

// 0割り算 tan2

	magnetRad = atan2(yDouble,xDouble);

	return NormalizeRad(magnetRad);			//Normalizeは一応付けた 混乱を防ぐため。
}

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


double NormalizeRad(double input) {				//radianを0~2πにする
	if (input >= 0 ) {
		return input - (int)(input / (2*M_PI)) *2*PI;
	} else {
		return input + ((int)((-1*input) / (2*M_PI)) + 1)* 2*PI;
	}
}

double Deg2Rad(double deg) {			//degree→radian変換
	return deg * M_PI / 180.0;
}
double Rad2Deg(double rad) {			//radian→degree変換
	return rad * 180.0 / M_PI;
}
double ddmm2dddd(double ddmm){				//degree値、dd.mmmmmmからdd.ddddddに変換
	return (int)(ddmm) + (ddmm - (int)ddmm) / 0.6;
}



/*	Serial.pirnt("xMsb:,");
	Serial.pirnt(xMsb);
	Serial.pirnt(",xLsb:,");
	Serial.pirnt(xLsb);
	Serial.pirnt("yMsb:,");
	Serial.pirnt(yMsb);
	Serial.pirnt(",yLsb:,");
	Serial.pirnt(yLsb);
	Serial.pirnt("zMsb:,");
	Serial.pirnt(zMsb);
	Serial.pirnt(",zLsb:,");
	Serial.pirnt(zLsb);
*/