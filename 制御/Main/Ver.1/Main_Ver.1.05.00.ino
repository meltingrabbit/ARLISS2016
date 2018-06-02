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

#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート

// 地磁気センサ
#define HMC5883L_ADDRESS 0x1E		// 7bit ADDRESS for I2C通信の素子識別アドレス
const double MAGNET_DECLINATION = 7.0;					// 東京で西偏7度0分

// CdS光センサ
#define ANALOG_PIN_CDS 1			// 光センサのアナログin

// GPS
#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
#define GpsSerialRead() Serial1.read()
#define GpsSerialAvailable() Serial1.available()
//#define PIN_GPS_RX 10				// GPS Rx (GPSのData Out) のピン番号
//#define PIN_GPS_TX 11				// GPS Tx (GPSのData In) のピン番号
// #########################################

// #########################################
// Global変数定義
// #########################################

// 地磁気センサ


// GPS

// #########################################


// #########################################
// GPSセンサの設定
// #########################################
//SoftwareSerial g_gps(PIN_GPS_RX, PIN_GPS_TX);                          //(rx,tx)
//SoftwareSerial g_gps(6, 7);                          //(rx,tx)
// #########################################

void setup() {
	Wire.begin();					// I2C通信起動
	I2cWrite(0x02,0x00);			// 地磁気センサの設定

	Serial.begin(SERIAL_BAUDRATE);
	GpsSerialBegin(SERIAL_BAUDRATE);			// Serial通信の開始

	delay(100);
}


void loop() {

	// arduinoの仕様がよくわからんのだが，
	// loop関数内でnew変数定義して回してるとメモリーリーク起こすかもしれないのでwhileでくくった
	// うーん，測定値データとかはグローバルに宣言してもいいかもなぁ...

	while(1) {

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
	magnetRad += MAGNET_DECLINATION;		// 偏角補正

	return magnetRad;
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

