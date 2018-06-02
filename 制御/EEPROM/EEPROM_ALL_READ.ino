#include <Wire.h>				// for I2C通信
#include <math.h>				// atan2とか
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>


// Log
// SDカードのクラッシュがこわすぎるので，スイッチがOFFの時はSD書き込みができない（処理が止まる）ようにする．
File MY_FILE;
char FLIE_NAME[] = "MD000000/LOG.CSV";
boolean Log_To_Sd = 1;					// SD書き込みか Cache書き込みか
boolean Is_Cache_Full = 0;				// Cacheの75%が書き込まれたらtrue
int Cache_Start_Address = 0;			// 現在のCache書き込み開始アドレス
int Cache_End_Address = 0;				// 現在のCache終了アドレス．次の文字は this+1 にはいる
unsigned int Cache_Size = 0;			// Cache済みサイズ
// const int MAX_EEPROM_ADDRESS = 4096;	// modとるために 4*2**10 にしてある．末尾は-1であることに注意
const int MAX_EEPROM_ADDRESS = 4090;	// 末尾は-1であることに注意．ステータス保存のためにキャッシュサイズ縮小

// ステータス保存用
const int STATUS_EEPROM_ADDRESS = 4092;
// #########################################


void setup() {
	delay(1000);
	Serial.begin(9600);
	Serial.println(F("HEAD"));
	Serial.println(F(""));

	char str;

	for (int i = 0; i<4096; i++) {
		str = EEPROM.read(i);
		Serial.print(str);
	}

	Serial.println(F(""));
	Serial.println(F("END"));

}


void loop() {
	while (1) {

	}
}
