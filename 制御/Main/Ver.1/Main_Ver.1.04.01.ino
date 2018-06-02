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
//#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
//#define GpsSerialRead() Serial1.read()
//#define GpsSerialPrint(gpssenddata) Serial1.print(gpssenddata)
//#define GpsSerialAvailable() Serial1.available()
#define GpsSerialBegin(baudrate) Serial3.begin(baudrate)
#define GpsSerialRead() Serial3.read()
#define GpsSerialPrint(gpssenddata) Serial3.print(gpssenddata)
#define GpsSerialAvailable() Serial3.available()
//#define PIN_GPS_RX 10				// GPS Rx (GPSのData Out) のピン番号
//#define PIN_GPS_TX 11				// GPS Tx (GPSのData In) のピン番号
// #########################################

// #########################################
// Global変数定義
// #########################################

// 地磁気センサ


// GPS
const long EARTH_RADIUS = 6378137;
#define GPS_RAW_ARR_LENGTH 20			// Gps_Raw_Arrの長さ
#define GPS_RAW_ARR_BUF_LENGTH 20		// Gps_Raw_Arrの値の長さ
double Gps_Lat;				//
double Gps_Long;			//dd.mmmmmmにすること
double Gps_Distance;			//センサと目的地間距離
double Gps_Radian;				//センサから目的地までの方位
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
	SendUnable(3);				//GPS:3.GSV,2.GSA,4.RMCの無効化
	SendUnable(2);
	SendUnable(4);
	delay(100);
}


void loop() {

	// arduinoの仕様がよくわからんのだが，
	// loop関数内でnew変数定義して回してるとメモリーリーク起こすかもしれないのでwhileでくくった
	// うーん，測定値データとかはグローバルに宣言してもいいかもなぁ...

	while(1) {
		int getGpsData = 0;			//GetGpsData()の返り値保存用
		unsigned long time = millis();
		do{
			if(millis() > time + 5000) {		//これで5秒間とれなかったらループから外れる...はず
				break;
			}
			getGpsData = GetGpsData();
	}while(getGpsData != 1);

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

	return magnetRad;
}

unsigneD char I2cRead(unsigned char regAddress) {
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
char ChecksumCalc(char *dt){
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
//###########################################
//###########################################

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

int GetGpsData(){
	//Get_Gps_Data = 0;		//最初にリセット
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
	buf[count] = '\0';			//bufにgpsデータ保存

//	Serial.print("buf : ");			//受信したbuf確認用
//	Serial.println(buf);

	gpsArrLength = Split(buf, ",");
	if (gpsArrLength == 14 && !strcmp(Gps_Raw_Arr[0], "$GPGGA")) {		// 送られてきたデータが15個で，$GPGGAならtrue
		if (strcmp(Gps_Raw_Arr[2], "NULL") && strcmp(Gps_Raw_Arr[4], "NULL") && strcmp(Gps_Raw_Arr[5], "NULL")) {
			//Get_Gps_Data = 1;
//			strcpy(Gps_Lat_Raw, Gps_Raw_Arr[2]);			// なぜかコピーできず (5/30)
//			strcpy(Gps_Long_Raw, Gps_Raw_Arr[4]);
//			strcpy(Gps_W_or_E, Gps_Raw_Arr[5]);

//			Gps_Lat =  0,01 * atof(Gps_Lat_Raw);			//char ddmm.mmmm → double dd.mmmmmm
//			Gps_Long = 0,01 * atof(Gps_Long_Raw);
			Gps_Lat = 0.01 * atof(Gps_Raw_Arr[2]);			//char ddmm.mmmm → double dd.mmmmmm
			Gps_Long = 0.01 * atof(Gps_Raw_Arr[4]);			//コピーできないので元配列から直接


			/*if (strcmp(Gps_Raw_Arr[3], "N")) {			// S だとtrue
				Gps_Lat = -1 * Gps_Lat;				//南緯の場合負の値にする  一応南緯も対応...?(笑)
			}*/
			if (strcmp(Gps_Raw_Arr[5], "E")) {			// W だとtrue
				Gps_Long = -1 * Gps_Long;				//西経の場合負の値にする
			}

			Serial.println("GPS success");

			return 1;

		} else {
			//Get_Gps_Data = 0;
			Serial.println("GPS failed 1...");

			return 0;
		}

	} else {
		//Get_Gps_Data = 0;
		Serial.println("GPS failed 2...");

		Serial.print("[0] : ");
		Serial.println(Gps_Raw_Arr[0]);
		Serial.print("[2] : ");
		Serial.println(Gps_Raw_Arr[2]);
		Serial.print("[4] : ");
		Serial.println(Gps_Raw_Arr[4]);
		Serial.print("[5] : ");
		Serial.println(Gps_Raw_Arr[5]);

		return 0;
	}

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

		//###################
		// debag用表示
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
