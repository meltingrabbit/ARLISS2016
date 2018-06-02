#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SoftwareSerial.h>		// for GPS Serial通信

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
SoftwareSerial g_gps(6, 7);                          //(rx,tx)
// #########################################

unsigned long time;
char buf[256];

void setup() {

	Serial.begin(SERIAL_BAUDRATE);
	g_gps.begin(SERIAL_BAUDRATE);			// Serial通信の開始

	delay(100);
	Serial.println("START");
}


void loop() {
	time = millis();
	sprintf(buf, "%010lu", time);
	g_gps.write(buf);
//	g_gps.write("\n");
//	g_gps.write('\n');
	g_gps.write(0x0A);
//	g_gps.print(buf);
//	g_gps.print("\n");
	Serial.print(buf);
	Serial.print("\n");


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

