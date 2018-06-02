#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか

// #########################################
// 関数プロトタイプ宣言
// #########################################
unsigned char i2cRead(unsigned char regAddress);					// I2C read
void i2cWrite(unsigned char regAddress, unsigned char data);		// I2C write
void sendMaget();													// 地磁気データをSerial.print
void sendGPS();														// GPSデータをSerial.print
void getMagnetData(int *magnetData);								// 地磁気データを取得
// #########################################

// #########################################
// 定数定義
// #########################################

// 地磁気センサ
#define HMC5883L_ADDRESS 0x1E		//7bit ADDRESS for I2C通信の素子識別アドレス

// GPS

// #########################################

// #########################################
// Global変数定義
// #########################################

// 地磁気センサ

// GPS

// #########################################



void setup()
{
	Wire.begin();
	Serial.begin(9600);
	delay(10);

	i2cWrite(0x02,0x00);			// 地磁気センサの設定
}


void loop()
{

	// arduinoの仕様がよくわからんのだが，
	// loop関数内でnew変数定義して回してるとメモリーリーク起こすかもしれないのでwhileでくくった
	// うーん，測定値データとかはグローバルに宣言してもいいかもなぁ...

	while(1) {
		sendMaget();
		sendGPS();

		delay(5);
	}
}


void sendMaget() {
	int magnetData[3] = {0,0,0};		// 地磁気データ格納配列初期化
	double magnetXyDegree = 0;			// 地磁気XY方向
	double magnetXyRadian = 0;			// 地磁気XY方向
	Serial.print("<<Mamagnetic Field>>\n");

	getMagnetData(magnetData);		// 配列ポインタを渡す
	magnetXyRadian = atan2((double)magnetData[1], (double)magnetData[0]);		// arctan(Y,X)
	magnetXyDegree = magnetXyRadian * 180 /3.142;

	Serial.print("X=");
	Serial.print(magnetData[0]);
	Serial.print(", Y=");
	Serial.print(magnetData[1]);
	Serial.print(", Z=");
	Serial.print(magnetData[2]);
	Serial.print(", RAD = ");
	Serial.print(magnetXyRadian);
	Serial.print(", DEG = ");
	Serial.print(magnetXyDegree);
	Serial.print("\n");
	return;
}


void sendGPS() {


}


void getMagnetData(int *magnetData)
{

	// 順番がXZYなので注意！
	unsigned char magnetXMsb = i2cRead(0x03);
	unsigned char magnetXLsb = i2cRead(0x04);
	unsigned char magnetZMsb = i2cRead(0x05);
	unsigned char magnetZLsb = i2cRead(0x06);
	unsigned char magnetYMsb = i2cRead(0x07);
	unsigned char magnetYLsb = i2cRead(0x08);

	int magnetX12 = magnetXMsb;
	magnetX12 = ((magnetX12 << 8) & 0xFF00) | magnetXLsb; //SHIFT & GET 12bit DATA WITH MSB
	int magnetY12 = magnetYMsb;
	magnetY12 = ((magnetY12 << 8) & 0xFF00) | magnetYLsb; //SHIFT & GET 12bit DATA WITH MSB
	int magnetZ12 = magnetZMsb;
	magnetZ12 = ((magnetZ12 << 8) & 0xFF00) | magnetZLsb; //SHIFT & GET 12bit DATA WITH MSB

	magnetData[0] = magnetX12;
	magnetData[1] = magnetY12;
	magnetData[2] = magnetZ12;

//	X_DOUBLE = magnetX_12;                       //CONVERT TO DOUBLE (FOR atan2)
//	Y_DOUBLE = magnetY_12;                       //CONVERT TO DOUBLE (FOR atan2)
//
//	RAD_RESULT = atan2(Y_DOUBLE,X_DOUBLE); //GET RADIAN
//	DEG_RESULT = RAD_RESULT * 180 /3.142;  //GET DEGREE

	return;
}


unsigned char i2cRead(unsigned char regAddress)
{
	Wire.beginTransmission(HMC5883L_ADDRESS);
	Wire.write(regAddress);
	Wire.endTransmission(false);
	Wire.requestFrom(HMC5883L_ADDRESS,1);
	return  Wire.read();
}


void i2cWrite(unsigned char regAddress, unsigned char data)
{
	Wire.beginTransmission(HMC5883L_ADDRESS);
	Wire.write(regAddress);
	Wire.write(data);
	Wire.endTransmission(false);
}

