//##################################################
//Note 重大な個人情報が含まれてます。
//##################################################
//10 5/31整理 loop内をGetGpsData()とCalDistanceDirection()だけにした
//いずれも家の中なので精度はご愛嬌
//安田講堂(地図では(D, D)=(1602.1, 4.48286) 方向は定規で測った精度が微妙、距離はほぼほぼ正しいはず)
//Latitude = 35.436405 N, Longitude = 139.459671 Distance = 1592.276611 Direction = 4.485278
//Latitude = 35.436519, Longitude = 139.459701 Distance = 1613.998657, Direction = 4.485466
//Latitude = 35.436424, Longitude = 139.459701 Distance = 1596.763549, Direction = 4.482973
//麺や「ひだまり」の角(D, D)=(213.0, 2.44914)
//Latitude = 35.436492, Longitude = 139.459732 Distance = 211.008300, Direction = 2.472194
//11 timerいれて五秒間取れないときの処理を書いた
//12 Get_Gps_Dataに代入するのをやめた GPSをGpsに直した GetGpsData()が0,1を返すようになった
//##################################################
//##################################################

#include <math.h>			// atan2とか
//##################
#define SERIAL_BAUDRATE 9600			// シリアル通信のボーレート
//##################

//######
//##################
//Arduino Uno 用設定		下のとどちらかにすること
//#include <SoftwareSerial.h>
//SoftwareSerial g_gps(6,7);
//#define GpsSerialBegin(baudrate) g_gps.begin(baudrate)
//#define GpsSerialRead() g_gps.read()
//#define GpsSerialPrint(gpssenddata) g_gps.print(gpssenddata)
//#define GpsSerialAvailable() g_gps.available()
//##################

//##################
//Arduino Mega 用設定
#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
#define GpsSerialRead() Serial1.read()
#define GpsSerialPrint(gpssenddata) Serial1.print(gpssenddata)
#define GpsSerialAvailable() Serial1.available()
//##################
//######

const double EARTH_RADIUS = 6378137;
//##################
//目標地点座標
//										!!!!!!!!!!!!東経なら正、西経なら負にすること!!!!!!!!!!!!!!!!!!
//#define TARGET_LATITUDE 35.436867
//#define TARGET_LONGITUDE 139.461983     //dd,mmmmmm,日暮里駅東の角
//const double TARGET_LATITUDE = 35.428045;
//const double TARGET_LONGITUDE = 139.457297;   //dd,mmmmmm安田講堂塔真ん中(D, D)=(1602.1, 4.48286)
const double TARGET_LATITUDE = 35.437200;
const double TARGET_LONGITUDE = 139.458633;   //dd,mmmmmm麺や「ひだまり」の角(D, D)=(213.0, 2.44914)
//const double TARGET_LATITUDE = 35.428333;
//const double TARGET_LONGITUDE = 139.456100;   //dd,mmmmmm工学部広場丸のとこの中心(D, D)=()
//##################

// #################
// 鈴本が新たに定義
#define GPS_RAW_ARR_LENGTH 20			// Gps_Raw_Arrの長さ
#define GPS_RAW_ARR_BUF_LENGTH 20		// Gps_Raw_Arrの値の長さ

char Gps_Raw_Arr[GPS_RAW_ARR_LENGTH][GPS_RAW_ARR_BUF_LENGTH];
// ##############

//####################
//Split未実装時のGlobal変数
//char *Gps_Lat_Raw;
//char *Gps_Long_Raw;
//char *Gps_W_or_E;			//南半球対応せず
//####################

//####################
//int Get_Gps_Data = 0;			//取れたら1or取れなかったら0

double Gps_Lat;				//
double Gps_Long;			//dd.mmmmmmにすること
double Gps_Distance;			//センサと目的地間距離
double Gps_Radian;				//センサから目的地までの方位
//####################


//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//11以降は以下に追加

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$


void setup(){
	Serial.begin(SERIAL_BAUDRATE);
	GpsSerialBegin(SERIAL_BAUDRATE);			// Serial通信の開始
	SendUnable(3);				//GPS:3.GSV,2.GSA,4.RMCの無効化
	SendUnable(2);
	SendUnable(4);
	delay(100);
}

void loop(){
	int getGpsData = 0;			//GetGpsData()の返り値保存用
	unsigned long time = millis();
	do{
		if(millis() > time + 5000) {		//これで5秒間とれなかったらループから外れる...はず
			getGpsData = 0;
			break;
		}
		getGpsData = GetGpsData();
	}while(getGpsData != 1);
//	}while(Get_Gps_Data != 1);

//	if(Get_Gps_Data == 1){
	if(getGpsData == 1){
		Serial.print("<<GPS DATA>>\n");
		Serial.print("Latitude = ");
		Serial.print(Gps_Lat,6);
		Serial.print(", Longitude = ");
		Serial.println(Gps_Long,6);
//		Serial.print(" ");
//		Serial.println(Gps_W_or_E);
		CalDistanceDirection();					//目的地までの距離と方角計算
		Serial.print("Distance = ");
		Serial.print(Gps_Distance, 6);
		Serial.print(", Direction = ");
		Serial.println(Gps_Radian, 6);
		Serial.println();
	}
}


//#############################################
//#############################################
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

	Gps_Radian = NormalizeRad( atan2(gapY, gapX) );
}

double NormalizeRad(double input) {				//radianを0~2πにする
	if (input >= 0 ) {
		return input - (int)(input / (2*M_PI)) *2*PI;
	} else {
		return input + ((int)((-1*input) / (2*M_PI)) + 1)* 2*PI;
	}
}

double Deg2Rad(double deg) {			//degree→radian変換
	return (deg) * M_PI / 180.0;
}
double Rad2Deg(double rad) {			//radian→degree変換
	return (rad) * 180.0 / M_PI;
}
double ddmm2dddd(double ddmm){				//degree値、dd.mmmmmmからdd.ddddddに変換
	return (int)(ddmm) + ((ddmm) - (int)(ddmm)) / 0.6;
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

//################################################################################
//	初期の取得関数
//void GetGpsDataPrototype(){
//	if(0== strncmp("$GPGGA", buf, 6)){			//行頭を比較して緯度経度のデータ行抽出
//		if(1){			//行の文字数が少ないとき弾く。。。？？（未実装）
//			char *checkN;
//			strtok(buf,",");			//,で区切る
//			strtok(NULL,",");
//			Gps_Lat_Raw = strtok(NULL,",");
//			checkN = strtok(NULL,",");
//			if(!strcmp(checkN,"N")){
//				Gps_Long_Raw = strtok(NULL,",");
//				Gps_W_or_E = strtok(NULL,",");

//				Gps_Lat = atof(Gps_Lat_Raw);			//char→double
//				Gps_Long = atof(Gps_Long_Raw);
//
//				Gps_Lat= Gps_Lat*0.01;			//生データはddmm.mmmmなので0.01倍
//				Gps_Long = Gps_Long*0.01;
//
//				Serial.print("<<GPS DATA>>\n");
//
//				Serial.print("Latitude = ");
//				Serial.print(Gps_Lat,6);
//				Serial.print(" N, Longitude = ");
//				Serial.print(Gps_Long,6);
//				Serial.print(" ");
//				Serial.println(Gps_W_or_E);
////
//				CalDistance();
//				Serial.println(Gps_Distance);
//				Serial.println(Gps_Radian);
//			}
//		}
//	}
//}
//################################################################################
