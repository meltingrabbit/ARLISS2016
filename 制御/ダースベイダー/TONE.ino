const int BEAT = 600;	// 音の長さを指定
#define beat(number) BEAT*number
#define PINNO 12	// 圧電スピーカを接続したピン番号


void setup() {

}

void loop() {
	Imperial();
}

void Imperial(){
	G_4(1);
	G_4(1);
	G_4(1);
	Eb_4(0.75);
	Bb_4(0.25);
	G_4(1);
	Eb_4(0.75);
	Bb_4(0.25);
	G_4(1);
	Delay(1);

	D_5(1);	//
	D_5(1);
	D_5(1);
	Eb_5(0.75);
	Bb_4(0.25);
	G_4(1);	//
	Eb_4(0.75);
	Bb_4(0.25);
	G_4(1);
	Delay(1);

	G_5(1);	//
	G_4(0.75);
	G_4(0.25);
	G_5(1);
	Gb_5(0.75);
	F_5(0.25);
	E_5(0.25);	//
	Eb_5(0.25);
	E_5(0.5);
	Delay(0.5);

	Ab_4(0.5);
	Db_5(1);
	C_5(0.75);
	B_4(0.25);
	Bb_4(0.25);	//
	A_4(0.25);
	Bb_4(0.5);
	Delay(0.5);

	Eb_4(0.5);
	Gb_4(1);
	Eb_4(0.75);
	Gb_4(0.25);
	Bb_4(1);	//
	G_4(0.75);
	Bb_4(0.25);
	D_5(1);
	Delay(1);

	G_5(1);	//繰り返し
	G_4(0.75);
	G_4(0.25);
	G_5(1);
	Gb_5(0.75);
	F_5(0.25);
	E_5(0.25);	//
	Eb_5(0.25);
	E_5(0.5);
	Delay(0.5);

	Ab_4(0.5);
	Db_5(1);
	C_5(0.75);
	B_4(0.25);
	Bb_4(0.25);	//
	A_4(0.25);
	Bb_4(0.5);
	Delay(0.5);

	Eb_4(0.5);
	Gb_4(1);
	Eb_4(0.75);
	Bb_4(0.25);
	G_4(1);
	Eb_4(0.75);
	Bb_4(0.25);
	G_4(1);
	Delay(1);
}

void Delay(float time) {
	delay(beat(time));
}
void C_4(float time) {
	tone(PINNO,262,beat(time)) ;		// ド
	delay(beat(time));
}
void Db_4(float time){
	tone(PINNO,277,beat(time)) ;
	delay(beat(time));
}
void D_4(float time) {
	tone(PINNO,294,beat(time)) ;		// レ
	delay(beat(time));
}
void Eb_4(float time){
	tone(PINNO,311,beat(time)) ;
	delay(beat(time));
}
void E_4(float time) {
	tone(PINNO,330,beat(time)) ;		// ミ
	delay(beat(time));
}
void F_4(float time) {
	tone(PINNO,349,beat(time)) ;		// ファ
	delay(beat(time));
}
void Gb_4(float time){
	tone(PINNO,370,beat(time)) ;
	delay(beat(time));
}
void G_4(float time) {
	tone(PINNO,392,beat(time)) ;		// ソ
	delay(beat(time));
}
void Ab_4(float time){
	tone(PINNO,415,beat(time)) ;
	delay(beat(time));
}
void A_4(float time) {
	tone(PINNO,440,beat(time)) ;		// ラ
	delay(beat(time));
}
void Bb_4(float time){
	tone(PINNO,466,beat(time)) ;
	delay(beat(time));
}
void B_4(float time) {
	tone(PINNO,494,beat(time)) ;		// シ
	delay(beat(time));
}
void C_5(float time) {
	tone(PINNO,523,beat(time)) ;		// ド
	delay(beat(time));
}
void Db_5(float time){
	tone(PINNO,554,beat(time)) ;
	delay(beat(time));
}
void D_5(float time) {
	tone(PINNO,587,beat(time)) ;		// レ
	delay(beat(time));
}
void Eb_5(float time){
	tone(PINNO,622,beat(time)) ;
	delay(beat(time));
}
void E_5(float time) {
	tone(PINNO,659,beat(time)) ;		// ミ
	delay(beat(time));
}
void F_5(float time) {
	tone(PINNO,698,beat(time)) ;		// ファ
	delay(beat(time));
}
void Gb_5(float time){
	tone(PINNO,740,beat(time)) ;
	delay(beat(time));
}
void G_5(float time) {
	tone(PINNO,784,beat(time)) ;		// ソ
	delay(beat(time));
}
void Ab_5(float time){
	tone(PINNO,831,beat(time)) ;
	delay(beat(time));
}
void A_5(float time) {
	tone(PINNO,880,beat(time)) ;		// ラ
	delay(beat(time));
}
void Bb_5(float time){
	tone(PINNO,932,beat(time)) ;
	delay(beat(time));
}
void B_5(float time) {
	tone(PINNO,988,beat(time)) ;		// シ
	delay(beat(time));
}
void C_6(float time) {
	tone(PINNO,1047,beat(time)) ;	// ド
	delay(beat(time));
}

