#include <SoftwareSerial.h>
#define ledPin 8
int state = 0;

SoftwareSerial BT (12, 13);

void setup() {
 pinMode(ledPin, OUTPUT);
 digitalWrite(ledPin, LOW);
 Serial.begin(9600); 
 BT.begin(9600);
}

void loop() {
 if(BT.available() > 0){ // Checks whether data is comming from the serial port
 state = BT.read(); 
 BT.println("data came in");
 }                 
if (state == '0') {
 digitalWrite(ledPin, LOW); 
 BT.println("LED: OFF");
 state = 0;
 }
 else if (state == '1') {
 digitalWrite(ledPin, HIGH);
 BT.println("LED: ON");;
 state = 0;
 } 
}
