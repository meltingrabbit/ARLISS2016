//#include <SoftwareSerial.h>
#include <math.h>			// atan2とか

//SoftwareSerial g_gps(6,7);
//#define GpsSerialBegin(baudrate) g_gps.begin(baudrate)
//#define GpsSerialRead() g_gps.read()
//#define GpsSerialPrint(gpssenddata) g_gps.print(gpssenddata)
//#define GpsSerialAvailable() g_gps.available()
#define GpsSerialBegin(baudrate) Serial1.begin(baudrate)
#define GpsSerialRead() Serial1.read()
#define GpsSerialPrint(gpssenddata) Serial1.print(gpssenddata)
#define GpsSerialAvailable() Serial1.available()

#define SERIAL_BAUDRATE 9600			// シリアル通信のボーレート

char *Gps_Lat_Raw;
char *Gps_Long_Raw;
char *Gps_W_or_E;			//南半球対応せず
double Gps_Lat;
double Gps_Long;			//扱いやすいように＆0.01倍するための変数

double Gps_Distance;
double Gps_Radian;

//#define TARGET_LATITUDE 35.436867
//#define TARGET_LONGITUDE 139.461983     //dd,mmmmmm,日暮里駅東の角

#define TARGET_LATITUDE 35.428045
#define TARGET_LONGITUDE 139.457297   //dd,mmmmmm安田講堂塔真ん中1602.1, 4.48286

void setup(){
	Serial.begin(SERIAL_BAUDRATE);
	GpsSerialBegin(SERIAL_BAUDRATE);			// Serial通信の開始
	SendUnable(3);	//GPS:GSV,GSA,RMCの無効化
	SendUnable(2);
	SendUnable(4);
	delay(100);
}

void loop(){
	char buf[256];
	int count = 0;

	do{
		if(GpsSerialAvailable()){
		buf[count]= GpsSerialRead();
		count++;
		}
	if(count > 250) break;
	}while(buf[count - 1] != 0x0A);
	buf[count] = '\0';			//bufにgpsデータ保存


	if(0== strncmp("$GPGGA", buf, 6)){			//行頭を比較して緯度経度のデータ行抽出
		if(1){			//行の文字数が少ないとき弾く。。。？？（未実装）
			char *checkN;
			strtok(buf,",");			//,で区切る
			strtok(NULL,",");
			Gps_Lat_Raw = strtok(NULL,",");
			checkN = strtok(NULL,",");
			if(!strcmp(checkN,"N")){
				Gps_Long_Raw = strtok(NULL,",");
				Gps_W_or_E = strtok(NULL,",");

				Gps_Lat = atof(Gps_Lat_Raw);			//char→double
				Gps_Long = atof(Gps_Long_Raw);

				Gps_Lat= Gps_Lat*0.01;			//生データはddmm.mmmmなので0.01倍
				Gps_Long = Gps_Long*0.01;

				Serial.print("<<GPS DATA>>\n");

				Serial.print("Latitude = ");
				Serial.print(Gps_Lat,6);
				Serial.print(" N, Longitude = ");
				Serial.print(Gps_Long,6);
				Serial.print(" ");
				Serial.println(Gps_W_or_E);


				CalDistance();
				Serial.println(Gps_Distance);
				Serial.println(Gps_Radian);
			}
		}
	}
}
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

char ChecksumCalc(char *dt)
{
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

void CalDistance(){
	double gapLat =  TARGET_LATITUDE - Gps_Lat;
	double gapLong = TARGET_LONGITUDE - Gps_Long;

	double latRad = (int)Gps_Lat * M_PI / 180.000000;
	latRad = latRad + (Gps_Lat - (int)Gps_Lat) * M_PI * 100 / (180*60);

	double gapLatRad = (int)gapLat * M_PI / 180.000000;
	gapLatRad = gapLatRad + (gapLat - (int)gapLat) * M_PI * 100 / (180*60);

	double gapLongRad = (int)gapLong * M_PI / 180.000000;
	gapLongRad = gapLongRad + (gapLong - (int)gapLong) * M_PI * 100 / (180*60);

	double gapX = 6378137 * gapLongRad * cos(latRad);
	double gapY = 6378137 * gapLatRad;
	Gps_Distance = sqrt(gapX*gapX + gapY*gapY);

	Gps_Radian = atan2(gapY, gapX);

}

