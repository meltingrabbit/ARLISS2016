#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SoftwareSerial.h>		// for GPS Serial通信

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

// CdS光センサ
#define ANALOG_PIN_CDS 1			// 光センサのアナログin

// GPS
#define PIN_GPS_RX 10				//GPS Rx (GPSのData Out) のピン番号
#define PIN_GPS_TX 11				//GPS Tx (GPSのData In) のピン番号
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
SoftwareSerial g_gps(PIN_GPS_RX, PIN_GPS_TX);                          //(rx,tx)
//SoftwareSerial g_gps(6, 7);                          //(rx,tx)
// #########################################

void setup() {
	Wire.begin();
	Serial.begin(9600);
	g_gps.begin(9600);     // g_gps（Serial通信）の開始
	delay(10);

	i2cWrite(0x02,0x00);			// 地磁気センサの設定


}


void loop() {

	// arduinoの仕様がよくわからんのだが，
	// loop関数内でnew変数定義して回してるとメモリーリーク起こすかもしれないのでwhileでくくった
	// うーん，測定値データとかはグローバルに宣言してもいいかもなぁ...

	while(1) {
		sendMaget();
		sendGPS();
		sendCds();

		delay(500);
	}
}

void sendCds() {
	Serial.print("<<CdS Cell>>\n");
	Serial.print(analogRead(ANALOG_PIN_CDS));
	Serial.print("\n");
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


  char buf[256];
  int count = 0;
  char *gpsLatRaw, *gpsLongRaw, *gpsWorE, *gpsAltRaw; // 南半球対応せず  東経西経はgpsWorEに格納
  double gpsLat, gpsLong, gpsAlt;                     // 扱いやすいように ＆ 0.01倍するための変数

  char *temp1, *temp2, *temp3, *temp4, *temp5, *temp6, *temp7, *temp8, *temp9;

  do{                                   // bufにgpsデータ保存
    if(g_gps.available()){
      buf[count]= g_gps.read();
      count++;
    }
    if(count > 250) break;
  }while(buf[count - 1] != 0x0A);
  buf[count] = '\0';

  if(0== strncmp("$GPGGA", buf, 6)){       // 行頭を比較して緯度経度のデータ行抽出
   if(1){                                  // 行の文字数が少ないとき弾く。。。？？（未実装、今のままだとgps取得してないときにバグるかも）
    Serial.println("<<GPS DATA>>");
    Serial.print(buf);                    // ＄GPGGAの生データ表示
    temp1 = strtok(buf,",");                       // ,で区切る
    Serial.print("<1>");
	Serial.print(temp1);
    temp2 = strtok(NULL,",");
    Serial.print("<2>");
	Serial.print(temp2);

    Serial.print("<3>");
    gpsLatRaw = strtok(NULL,",");
	Serial.print(gpsLatRaw);

    temp4 = strtok(NULL,",");
    Serial.print("<4>");
	Serial.print(temp4);

    gpsLongRaw = strtok(NULL,",");
    Serial.print("<5>");
	Serial.print(gpsLongRaw);

    gpsWorE = strtok(NULL,",");
    Serial.print("<6>");
	Serial.print(gpsWorE);


    temp7 = strtok(NULL,",");
    Serial.print("<7>");
	Serial.print(temp7);

    temp8 = strtok(NULL,",");
    Serial.print("<8>");
	Serial.print(temp8);

    temp9 = strtok(NULL,",");
    Serial.print("<9>");
	Serial.print(temp9);

    gpsAltRaw = strtok(NULL,",");
    Serial.print("<10>");
	Serial.print(gpsAltRaw);

    gpsLat = atof(gpsLatRaw);           //char→double
    gpsLong = atof(gpsLongRaw);
    gpsAlt = atof(gpsAltRaw);

    gpsLat= gpsLat*0.01;                 //生データはddmm.mmmmなので0.01倍
    gpsLong = gpsLong*0.01;

    Serial.print("Latitude = ");
    Serial.print(gpsLat,6);
    Serial.print(" N, Longitude = ");
    Serial.print(gpsLong,6);
    Serial.print(" ");
    Serial.print(gpsWorE);
    Serial.print(", Altitude = ");
    Serial.print(gpsAlt);
    Serial.println(" m");

   }
   } else {
   	Serial.println("<<GPS DATA>>");
   	Serial.println(buf);                    // ＄生データ表示
  }


//  if (g_gps.available()) {
//    Serial.write(g_gps.read());
//  }



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

