#include "BluetoothSerial.h"



const int IPWM = 21;
const int AICAL = 33;
const int freq = 1000;
const int resolution = 8;

BluetoothSerial SerialBT;

int ch=1;
 
int read_voltage(int chan, int nsamples){
  int s = 0;

  for (int i = 0; i < nsamples; ++i){
    s += analogRead(chan);
  }

  return s / nsamples;
  
}

void setup() {
  // put your setup code here, to run once:
  SerialBT.begin("IPTANEM0");
  Serial.begin(9600);
  ledcSetup(ch, freq, resolution);
  ledcAttachPin(IPWM, ch);
}

void loop() {
  // put your main code here, to run repeatedly:
  int V;
  for (int duty=0; duty<255; duty+=1){
    ledcWrite(ch, duty);
    delay(500);
    Serial.print(duty);
    Serial.print(",");
    V = read_voltage(AICAL, 20000);
    Serial.println(V);
  }
}
