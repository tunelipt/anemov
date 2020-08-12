
/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include "BluetoothSerial.h"

//#define DEBUG_ANEM

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();
BluetoothSerial SerialBT;

const int AI_VEL = 34;
const int AI_TEMP = 33;
const int NCHANS = 2;

const int AI_CHANS[NCHANS] = {AI_VEL, AI_TEMP};
int aivals[NCHANS];

const int NSAMPLES = 5000;
String btresp;


void read_voltage(){
  for (int i = 0; i < NCHANS; ++i){
    aivals[i] = 0;
  }

  for (int i = 0; i < NSAMPLES; ++i){
    for (int k = 0; k < NCHANS; ++k){
      aivals[k] += analogRead(AI_CHANS[k]);
    }
  }

  for (int k = 0; k < NCHANS; ++k){
    aivals[k] /= NSAMPLES;
    }
}
void setup() {
  
  SerialBT.begin("IPTANEM0");
  
#ifdef DEBUG_ANEM  
  Serial.begin(9600);
#endif //DEBUG_ANEM
  
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  //bmp_temp->printSensorDetails();
}

void loop(){
  int idx;
  int nsamples;
  if (SerialBT.available()){
    delay(10);
    
    btresp = SerialBT.readString();
    idx = btresp.indexOf("?");
    if (idx > -1){
      // Ler o número de amostras:
      read_voltage();
      sensors_event_t temp_event, pressure_event;
      bmp_temp->getEvent(&temp_event);
      bmp_pressure->getEvent(&pressure_event);
  
      SerialBT.println("BEGIN");
      SerialBT.println(aivals[0]); 
      SerialBT.println(aivals[1]); 
      SerialBT.println(temp_event.temperature);
      SerialBT.println(pressure_event.pressure/10);
      SerialBT.println("END");
    }
  }

#ifdef DEBUG_ANEM
  if (Serial.available()){
    btresp = Serial.readString();
    if (btresp.indexOf("?") > -1){
  
      read_voltage();
      sensors_event_t temp_event, pressure_event;
      bmp_temp->getEvent(&temp_event);
      bmp_pressure->getEvent(&pressure_event);

      Serial.println("BEGIN");
      Serial.println(aivals[0]);
      Serial.println(aivals[1]);
      Serial.println(temp_event.temperature);
      Serial.println(pressure_event.pressure/10);
      Serial.println("END");
    }
  }

  delay(10);
#endif // DEBUG_ANEM
}
