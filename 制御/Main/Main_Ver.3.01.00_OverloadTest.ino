#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>


#define TIRE_TYPE 1

#if TIRE_TYPE == 1
	#define DIRECTIVE_TEST 11
#elif TIRE_TYPE == 2
	#define DIRECTIVE_TEST 22
#else
	#define DIRECTIVE_TEST 33
#endif

#define SERIAL_BAUDRATE 9600		// シリアル通信のボーレート



void setup() {


	MyDelay(1000);
	Serial.begin(SERIAL_BAUDRATE);


	while (1) {
		int a = 3;
		int b = 1;
		double c = 6.3;
		double d = 2.1;
		Serial.print("int : ");
		Serial.println(add(a,b));
		Serial.print("double : ");
		Serial.println(add(c,d));

		// エラー
		// Serial.print("mix : ");
		// Serial.println(add(a,c));

		Serial.print("DIRECTIVE_TEST : ");
		Serial.println(DIRECTIVE_TEST);

		MyDelay(1000);
	}


}


void loop() {



}

// int add(int a, int b) {
// 	return a + b;
// }

int add(int a, int b) {
	return (int) add((double)a, (double)b);
}

double add(double a, double b) {
	return a + b;
}




void MyDelay(long msec) {
	if (msec >= 0) {
		delay(msec);
	} else {
		delay(1000);
	}
}