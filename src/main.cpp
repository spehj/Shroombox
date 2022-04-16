/*
Product: Shroombox
Authors: Matic Sedej, Jure Špeh
Date: April 2022
GitHub: 
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include "IO_Defs.h"

//https://docs.blynk.io/en/getting-started/activating-devices/blynk-edgent-wifi-provisioning
//Fill information from your Blynk Template here
//#define BLYNK_TEMPLATE_ID "xxxxx"
//#define BLYNK_DEVICE_NAME "yyyyy"
#define BLYNK_FIRMWARE_VERSION  "0.1.0"
#define BLYNK_PRINT Serial //#define BLYNK_DEBUG
#define APP_DEBUG
#include "BlynkEdgent.h" //Must be below blynk defines!

//#define VERSION "v1.0"
#define CH0 0 //PWM channel number
#define CH1 1 //PWM channel number
#define CH2 2 //PWM channel number
#define CH3 3 //PWM channel number
#define CH4 4 //PWM channel number

#define GROWTH_STAGE1 1
#define GROWTH_STAGE2 2

DHT dht(DHT_PIN, DHT22);
OneWire oneWire1(DS18B20_1_PIN);
OneWire oneWire2(DS18B20_2_PIN);
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
AccelStepper stepper(AccelStepper::FULL4WIRE, STPR_PIN1, STPR_PIN2, STPR_PIN3, STPR_PIN4);
SCD30 co2;

/****************************
Prototypes of functions */

void begin_stepper();
void begin_io();
char begin_dht22();
char begin_ds18b20();
char begin_scd30();
void begin_pwm();
char read_dht22(float &temp, float &hum);
char read_ds18b20(float &temp1, float &temp2);
char time_passed(const unsigned long timemark, const unsigned long delay);
unsigned long time_mark();

/****************************/
void setup() {
  Serial.begin(115200);
  begin_dht22();
  begin_ds18b20();
  begin_stepper();
  begin_io();
  begin_pwm();
  BlynkEdgent.begin();
}


/****************************/

unsigned long time_temp = time_mark();
float DHT_temp, DHT_hum;
float DS_temp1, DS_temp2;
char growth;

void loop() {
  
  if(time_passed(time_temp, 5000)){ //Measure every 5s
    read_dht22(DHT_temp, DHT_hum);
    read_ds18b20(DS_temp1, DS_temp2);
    Serial.print("DS_temp1 "), Serial.println(DS_temp1);
    Serial.print("DS_temp2 "), Serial.println(DS_temp2);
    time_temp = time_mark();
  }
  if(growth==GROWTH_STAGE1){ //Stage 1
    //TO DO...
  }
  if(growth==GROWTH_STAGE2){ //Stage 2
    //TO DO...
  }

  BlynkEdgent.run();
}

/****************************
Functions
*/

/* 
void begin_io();
Setup digital IOs 
*/
void begin_io(){
  pinMode(BTN_1_PIN,INPUT_PULLUP), pinMode(BTN_2_PIN,INPUT_PULLUP); //Enable internal pullup
  //pinMode(MOSFET_1_PIN,OUTPUT), pinMode(MOSFET_2_PIN,OUTPUT), pinMode(MOSFET_3_PIN,OUTPUT);
  //pinMode(MOSFET_4_PIN,OUTPUT), pinMode(MOSFET_5_PIN,OUTPUT);
  pinMode(LED_PIN,OUTPUT);
}

/* 
void begin_stepper();
Setup stepper motor
*/
void begin_stepper(){
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(20);
  stepper.enableOutputs(); //Enable pins
}

/* 
char begin_dht22();
Setup DHT22 sensor
Return 1 if OK, 0 if ERROR 
*/
char begin_dht22(){
  dht.begin();
  float Humidity = dht.readHumidity();
  float Temperature = dht.readTemperature();
    if (isnan(Humidity) || isnan(Temperature)) { //If read from sensor fail (sensor not connected, ...)
      Humidity = 0; Temperature = 0;
      Serial.println(F("Failed to read from DHT sensor!"));
      return 0; //ERROR
    }
  return 1; //OK
}

/* 
char begin_ds18b20();
Setup DS18B20 sensors
Return 1 if OK, 0 if ERROR 
*/
char begin_ds18b20(){
  sensor1.begin();
  sensor2.begin();
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  float temp1 = sensor1.getTempCByIndex(0);
  float temp2 = sensor2.getTempCByIndex(0);
  if(temp1 == DEVICE_DISCONNECTED_C){
    Serial.println(F("Failed to read from DS18B20 sensor1!"));
    return 0; //ERROR sensor1
  }
  if(temp2 == DEVICE_DISCONNECTED_C){
    Serial.println(F("Failed to read from DS18B20 sensor2!"));
    return 0; //ERROR sensor2
  }
  return 1; //OK
}

/* 
char begin_scd30();
Setup SCD30 sensor
Return 1 if OK, 0 if ERROR 
*/
char begin_scd30(){
  Wire.begin();
  if(co2.begin() == 0){
    Serial.println(F("CO2 sensor not detected..."));
    return 0; //ERROR
  }
  return 1; //OK
}

/* 
void begin_pwm();
Setup PWM outputs
*/
void begin_pwm(){
  const int freq = 100; //PWM frequency
  const char res = 8; //PWM resolution in bits
  ledcSetup(CH0, freq, res);
  ledcAttachPin(MOSFET_1_PIN, CH0);
  ledcSetup(CH1, freq, res);
  ledcAttachPin(MOSFET_2_PIN, CH1);
  ledcSetup(CH2, freq, res);
  ledcAttachPin(MOSFET_3_PIN, CH2);
  ledcSetup(CH3, freq, res);
  ledcAttachPin(MOSFET_4_PIN, CH3);
  ledcSetup(CH4, freq, res);
  ledcAttachPin(MOSFET_5_PIN, CH4);
}

/* 
char read_ds18b20();
Read DS18B20 sensors
Return temp1 and temp2
Return 1 if OK, 0 if ERROR 
*/
char read_ds18b20(float &temp1, float &temp2){
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  temp1 = sensor1.getTempCByIndex(0);
  temp2 = sensor2.getTempCByIndex(0);
  if(temp1 == DEVICE_DISCONNECTED_C){
    Serial.println(F("Failed to read from DS18B20 sensor1!"));
    return 0; //ERROR sensor1
  }
  if(temp2 == DEVICE_DISCONNECTED_C){
    Serial.println(F("Failed to read from DS18B20 sensor2!"));
    return 0; //ERROR sensor2
  }
  return 1; //OK
}

/* 
char read_dht22()
Read temp and humidity from DHT22
Return temp and humidity
If both values are 0 => sensor ERROR
Return 1 if OK, 0 if ERROR 
*/
char read_dht22(float &temp, float &hum){
  temp = dht.readTemperature();
  hum = dht.readHumidity();
    if (isnan(temp) || isnan(hum)) { //If read from sensor fail (sensor not connected, ...)
      temp = 0; hum = 0;
      Serial.println(F("Failed to read from DHT sensor!"));
      return 0; //ERROR
    }
  //Serial.print(F("DHT22: ")), Serial.print(temp), Serial.print(F(" ")), Serial.println(hum);
  return 1; //OK
}

/* 
byte time_passed(unsigned long timemark, unsigned long delay)
Check if more (or equal) of "delay" millisecond has passed or not
Return 1 if yes, 0 if not
*/
char time_passed(const unsigned long timemark, const unsigned long delay){
	char x = 0;
  unsigned long new_millis = millis();
	if ((new_millis-timemark)>=delay){
		x = 1;
	}
	return x;
}

/* 
unsigned long time_mark()
Set time mark
Return value of timer 
*/
unsigned long time_mark(){
	return millis();
}