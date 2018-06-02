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
unsigned char Magnet_X_Msb;
unsigned char Magnet_X_Lsb;
unsigned char Magnet_Z_Msb;
unsigned char Magnet_Z_Lsb;
unsigned char Magnet_Y_Msb;
unsigned char Magnet_Y_Lsb;
int Magnet_X_12;
int Magnet_Z_12;
int Magnet_Y_12;
double Magnet_X_Double;
double Magnet_Y_Double;

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

	// 順番がXZYなので注意！
	Magnet_X_Msb = i2cRead(0x03);
	Magnet_X_Lsb = i2cRead(0x04);
	Magnet_Z_Msb = i2cRead(0x05);
	Magnet_Z_Lsb = i2cRead(0x06);
	Magnet_Y_Msb = i2cRead(0x07);
	Magnet_Y_Lsb = i2cRead(0x08);

	Magnet_X_12 = Magnet_X_Msb;
	Magnet_X_12 = ((Magnet_X_12 << 8) & 0xFF00) | Magnet_X_Lsb; //SHIFT & GET 12bit DATA WITH MSB
	Magnet_Y_12 = Magnet_Y_Msb;
	Magnet_Y_12 = ((Magnet_Y_12 << 8) & 0xFF00) | Magnet_Y_Lsb; //SHIFT & GET 12bit DATA WITH MSB
	Magnet_Z_12 = Magnet_Z_Msb;
	Magnet_Z_12 = ((Magnet_Z_12 << 8) & 0xFF00) | Magnet_Z_Lsb; //SHIFT & GET 12bit DATA WITH MSB

	//CONVERT TO DOUBLE (FOR atan2)
	Magnet_X_Double = Magnet_X_12;
	Magnet_Y_Double = Magnet_Y_12;


//	RAD_RESULT = atan2(Y_DOUBLE,X_DOUBLE); //GET RADIAN
//	DEG_RESULT = RAD_RESULT * 180 /3.142;  //GET DEGREE


// 0割り算 tan2

	magnetRad = atan2(Magnet_Y_DOUBLE,Magnet_X_DOUBLE);

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

