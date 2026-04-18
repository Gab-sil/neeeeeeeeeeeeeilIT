#define ServoIn 18
#define Emitter 19
#define Receiver 20

void setup() {

pinMode(ServoIn, OUTPUT);
pinMode(Emitter, OUTPUT);
pinMode(Receiver, INPUT);

analogWriteFreq(50);
analogWriteRange(20000);

}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(LED_BUILTIN, HIGH);
delay(500);
digitalWrite(LED_BUILTIN, LOW);
delay(500);
}
