#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <string.h>				// for Split

//#include <SoftwareSerial.h>		// for GPS Serial通信

// #########################################
// 関数プロトタイプ宣言
// #########################################
unsigned char I2cRead(unsigned char regAddress);					// I2C read
void I2cWrite(unsigned char regAddress, unsigned char data);		// I2C write
void GetMagnetData();												// 地磁気データを取得
int Split(char *string, char *separator);							// 返り値は分割数
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

#define GPS_RAW_ARR_LENGTH 20			// Gps_Raw_Arrの長さ
#define GPS_RAW_ARR_BUF_LENGTH 100		// Gps_Raw_Arrの値の長さ
char Gps_Raw_Arr[GPS_RAW_ARR_LENGTH][GPS_RAW_ARR_BUF_LENGTH];
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
double Magnet_Rad;
double Magnet_Deg;

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


	char serialInput[200];		// シリアル通信受信文字列格納用 for split test
	int i = 0;
	int count = 0;
	while(1) {
		// データ受信したとき
		if (Serial.available()) {
			serialInput[i] = Serial.read();
			// 文字数が190以上 or 末尾文字
			if (i > 190 || serialInput[i] == ':') {
				// 末尾に終端文字の挿入
				serialInput[i] = '\0';
				// 受信文字列を送信
				Serial.print("RawString : ");
				Serial.print(serialInput);
				Serial.print("\n");

				count = Split(serialInput, ",");

				Serial.print("Count : ");
				Serial.print(count);
				Serial.print("\n");

				// カウンタの初期化
				i = 0;
				delay(1000);
			} else {
				i++;
			}
		}
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
//			*(pointer++) = 0;
		}

		strcpy(splitString, startAdress);				// 切り取った文字列をコピー って，まだ切り取ってないかw
		startAdress = pointer;							// 次切り取る文字列の先頭を更新

		if (!strcmp(splitString, "")) {					// strcmp 一致 → 0
			strcpy(splitString, "NULL");
		}

		// 表示
		Serial.print(count);
		Serial.print(" : ");
		Serial.print(splitString);
		Serial.print("\n");

		count++;
		delay(500);
	}

	return (count-1);
}

