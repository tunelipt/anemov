
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

//#define USE_BLUETOOTH
#define USE_WIFI

#ifdef USE_BLUETOOTH
#include "BluetoothSerial.h"
#elif defined(USE_WIFI)
#include <WiFi.h>
#endif // USE_BLUETOOTH



Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

#ifdef USE_BLUETOOTH
BluetoothSerial dev;
#elif defined(USE_WIFI)
WiFiServer dev(9590);
const char *ssid = "tunel";
const char *password = "tunelgvento";
#endif


const char *devname = "IPT0";




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

void read_anem(int *vel, int *temp, float *tref, float *Pa){
  read_voltage();

  *vel = aivals[0];
  *temp = aivals[1];
  
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  *tref = temp_event.temperature;
  *Pa = pressure_event.pressure/10;
}


void setup() {


  
  Serial.begin(9600);
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
#ifdef USE_BLUETOOTH
  dev.begin("IPTANEM0");
#elif defined(USE_WIFI)
  Serial.print("Conectando-se a ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Conectado ao WiFi");
  Serial.print("Endereço de IP: ");
  Serial.println(WiFi.localIP());
  
  dev.begin();
#endif
  
}


int parse_header(String hdr){
  int idx1, idx2;
  idx1 = hdr.indexOf("?");
  idx2 = hdr.indexOf("!");

  if (idx1 < 0 || idx2 < 0){
    return -1;
  }
  String ss = hdr.substring(idx1+1, idx2);
  int ttot = ss.toInt();  
  return ttot;
}

#ifdef USE_BLUETOOTH

void loop(){

  int t1, dt, ttot;
  String hdr;
  int vel, temp;
  float tref, Pa;
  
  if (dev.available()){
    delay(20);
    hdr = dev.readString();
    ttot = parse_header(hdr);
    t1 = millis();
    dt = 0;
    while(dt < ttot){
      read_anem(&vel, &temp, &tref, &Pa);
      dt = millis() - t1;
      dev.print(devname); dev.print(",");
      dev.print(dt); dev.print(",");
      dev.print(vel); dev.print(",");
      dev.print(temp); dev.print(",");
      dev.print(tref); dev.print(",");
      dev.print(Pa); dev.println(",END");
      if (dt > ttot) 
        break;
       
    }
  
  }
}


#elif defined(USE_WIFI)

WiFiClient dev1;
void loop(){

  Serial.println("Estamos no wifi loop"); 
  int t1, dt, ttot;
  
  String hdr;
  int vel, temp;
  float tref, Pa;
  if (!dev1){
    dev1= dev.available();
  }else{
    if (dev1.available()){
      delay(20);
      hdr = dev1.readString();
      ttot = parse_header(hdr);
      t1 = millis();
      dt = 0;
      while(dt < ttot){
        read_anem(&vel, &temp, &tref, &Pa);
        dt = millis() - t1;
        dev1.print(devname); dev1.print(",");
        dev1.print(dt); dev1.print(",");
        dev1.print(vel); dev1.print(",");
        dev1.print(temp); dev1.print(",");
        dev1.print(tref); dev1.print(",");
        dev1.print(Pa); dev1.println(",END");
        if (dt > ttot) 
          break;
      }
    }
  }
  
}



#else

void loop(){

  int t1, dt, ttot;
  String hdr;
  int vel, temp;
  float tref, Pa;
  
  if (Serial.available()){
    delay(20);
    hdr = Serial.readString();
    ttot = parse_header(hdr);
    t1 = millis();
    dt = 0;
    while(dt < ttot){
      read_anem(&vel, &temp, &tref, &Pa);
      dt = millis() - t1;
      Serial.print(devname); Serial.print(",");
      Serial.print(dt); Serial.print(",");
      Serial.print(vel); Serial.print(",");
      Serial.print(temp); Serial.print(",");
      Serial.print(tref); Serial.print(",");
      Serial.print(Pa); Serial.println(",END");
      if (dt > ttot) 
        break;
    }
  
  }
}


#endif
