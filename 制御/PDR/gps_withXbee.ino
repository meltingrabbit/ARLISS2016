#include <SoftwareSerial.h>

SoftwareSerial g_gps(6,7);

void setup()
{
 
  Serial.begin(9600);
  Serial.println("ready");
  g_gps.begin(9600);
  
}

void loop()
{
  char buf[256];
  char c;
  int count = 0;
  char *gpsLatraw, *gpsLongraw, *gpsWorE; //南半球対応せず  
  double gpsLat, gpsLong;
  
  do{
    if(g_gps.available()){
      buf[count]= g_gps.read();
      count++;
    }
    if(count > 250) break;
  }while(buf[count - 1] != 0x0A);
  buf[count] = '\0';

  if(0== strncmp("$GPGGA", buf, 6)){
   if(1){
    strtok(buf,",");
    strtok(NULL,",");
    gpsLatraw = strtok(NULL,",");
    strtok(NULL,",");
    gpsLongraw = strtok(NULL,",");
    gpsWorE = strtok(NULL,",");

    gpsLat = atof(gpsLatraw);
    gpsLong = atof(gpsLongraw);
    
    Serial.print("<<GPS DATA>>\n");

    Serial.print("Latitude = ");
    Serial.print(gpsLat);
   
    Serial.print(" N, Longitude = ");
    Serial.print(gpsLong);
    Serial.print(" ");
    Serial.println(gpsWorE);

   }
  }
}

