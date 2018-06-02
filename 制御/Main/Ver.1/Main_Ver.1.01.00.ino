#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
//#include <SoftwareSerial.h>		// for GPS Serial通信

// #########################################
// 関数プロトタイプ宣言
// #########################################
unsigned char I2cRead(unsigned char regAddress);					// I2C read
void I2cWrite(unsigned char regAddress, unsigned char data);		// I2C write
void GetMagnetData();								// 地磁気データを取得
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
double Magnet_Rad_Result;
double Magnet_Deg_Result;

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

