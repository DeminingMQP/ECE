#include <Arduino.h>

void setup() {
Serial.begin(115200);
pinMode(7, OUTPUT);
pinMode(6, OUTPUT);
digitalWrite(7, HIGH);
analogWrite(6, 255);
}

void loop() {
delay(1000);
Serial.println("Testing");

}
