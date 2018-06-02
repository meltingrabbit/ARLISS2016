#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>


#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート

// #define MyPrint(str) Serial.print(str)

#define MyPrint3(str) Serial.print(str)
#define MyPrint(str) do {	\
	Serial.print(str);				\
	Serial.print('DEFDEF');				\
	Serial.print(str);				\
} while(0)

void setup() {


	MyDelay(1000);
	Serial.begin(SERIAL_BAUDRATE);
	Serial3.begin(SERIAL_BAUDRATE);

	while (1) {
		Serial.print(F("Serial Port : "));
		Serial.println(millis());
		MyPrint("Serial Port : ");
		MyPrint(F("Serial Port : "));
		MyPrint(millis());

		Serial3.print(F("Xbee : "));
		Serial3.println(millis());
		MyPrint3("Xbee : ");
		MyPrint3(F("Xbee : "));
		MyPrint3(millis());

		MyDelay(1000);
	}


}


void loop() {



}


void MyDelay(long msec) {
	if (msec >= 0) {
		delay(msec);
	} else {
		delay(1000);
	}
}