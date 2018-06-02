#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>


#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート



void setup() {


	MyDelay(1000);
	Serial.begin(SERIAL_BAUDRATE);
	Serial3.begin(SERIAL_BAUDRATE);

	while (1) {
		Serial.print("Serial Port : ");
		Serial.println(millis());
		Serial3.print("Xbee : ");
		Serial3.println(millis());
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