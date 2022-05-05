/*
Product: Shroombox
Authors: Matic Sedej, Jure Špeh
Date: May 2022
GitHub: https://github.com/spehj/Shroombox
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "IO_Defs.h"
#include "Blynk_Virtual_Pins.h"

// https://docs.blynk.io/en/getting-started/activating-devices/blynk-edgent-wifi-provisioning
// https://docs.blynk.io/en/getting-started/updating-devices-firmwares-ota

#define BLYNK_TEMPLATE_ID "TMPLWxVCUiA-" // Copy from Blynk template
#define BLYNK_DEVICE_NAME "Shroombox V1" // Copy from Blynk template
#define BLYNK_FIRMWARE_VERSION "0.1.2" // Change the Firmware version every time, otherwise device will ignore it and won't update OTA!
#define BLYNK_PRINT Serial //#define BLYNK_DEBUG
#define APP_DEBUG
#include "BlynkEdgent.h" // Must be below blynk defines!

#define HUMIDIFIER 0 // PWM channel 0
#define LEDS 1 // PWM channel 1
#define FAN 2 // PWM channel 2
#define HEATING_PAD1 3 // PWM channel 3
#define HEATING_PAD2 4 // PWM channel 4

#define GROWTH_PHASE1 1
#define GROWTH_STAGE2 2



#define DISPLAY_W 128 // OLED display width
#define DISPLAY_H 32 // OLED display height
#define DISPLAY_ADR 0x3D
Adafruit_SSD1306 display(DISPLAY_W, DISPLAY_H, &Wire);

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
char begin_display();
char time_passed(unsigned long timemark, unsigned long delay);
unsigned long time_mark();
void reg_temp(float measured_temp, float desired_temp, float hyst, char pwm_ch);

/****************************/
void setup()
{
  Serial.begin(115200);
  begin_dht22();
  begin_ds18b20();
  //begin_stepper();
  begin_io();
  //begin_pwm();
  begin_display();
  begin_scd30();
  delay(100);
  BlynkEdgent.begin();
}

/****************************/

unsigned long time_temp = time_mark();
float air_temp, air_hum;
float room_temp, heater_temp;
char growth_phase;
unsigned char pwm_duty = 0;

void loop()
{

  if (time_passed(time_temp, 4000))
  { // Measure every 4s
    read_dht22(air_temp, air_hum);
    Serial.print("Air_temp "), Serial.println(air_temp);
    Serial.print("Air_hum "), Serial.println(air_hum);
    read_ds18b20(room_temp, heater_temp);
    Serial.print("Room_temp "), Serial.println(room_temp);
    Serial.print("Heater_temp "), Serial.println(heater_temp);
    time_temp = time_mark();
    // Test Blynk
    Blynk.virtualWrite(AIR_TEMP,air_temp);
    Blynk.virtualWrite(AIR_HUM,air_hum);
    Blynk.virtualWrite(ROOM_TEMP,room_temp);
    Blynk.virtualWrite(HEATER_TEMP,heater_temp);
    // Test PWM outputs
    //ledcWrite(HUMIDIFIER, pwm_duty);
    //ledcWrite(LEDS, pwm_duty);
    //ledcWrite(FAN, pwm_duty);
    //ledcWrite(HEATING_PAD1, pwm_duty);
    //ledcWrite(HEATING_PAD2, pwm_duty);
    //pwm_duty = pwm_duty + 50;
    //if (pwm_duty >= 255)
    //{
    //  pwm_duty = 0;
    //}
  }
  /*
  if (growth_phase == GROWTH_PHASE1)
  { // Phase 1
    // TO DO...
  }
  if (growth_phase == GROWTH_PHASE2)
  { //Phase 2
    // TO DO...
  }*/

  BlynkEdgent.run();

}

/****************************
Functions
*/

/*
void begin_io();
Setup digital IOs
*/
void begin_io()
{
  pinMode(BTN_1_PIN, INPUT_PULLUP), pinMode(BTN_2_PIN, INPUT_PULLUP); // Enable internal pullup
  // pinMode(MOSFET_1_PIN,OUTPUT), pinMode(MOSFET_2_PIN,OUTPUT), pinMode(MOSFET_3_PIN,OUTPUT);
  // pinMode(MOSFET_4_PIN,OUTPUT), pinMode(MOSFET_5_PIN,OUTPUT);
  // pinMode(LED_PIN, OUTPUT);
}

/*
char begin_display();
Setup display
Return 1 if OK, 0 if ERROR
*/
char begin_display()
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADR)) { 
    Serial.println(F("Display error"));
    return 0; // ERROR
  }
  display.clearDisplay(); // Clear display
  display.setTextSize(2); // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0,0); // Start at top-left corner
  display.println(F("Shroombox ")), display.println(BLYNK_FIRMWARE_VERSION);
  display.display(); // Show on display
  return 1; // OK
}

/*
void begin_stepper();
Setup stepper motor
*/
void begin_stepper()
{
  stepper.setMaxSpeed(100);
  stepper.setAcceleration(20);
  stepper.enableOutputs(); // Enable pins
}

/*
char begin_dht22();
Setup DHT22 sensor
Return 1 if OK, 0 if ERROR
*/
char begin_dht22()
{
  dht.begin();
  float Humidity = dht.readHumidity();
  float Temperature = dht.readTemperature();
  if (isnan(Humidity) || isnan(Temperature))
  { // If read from sensor fail (sensor not connected, ...)
    Humidity = 0;
    Temperature = 0;
    Serial.println(F("Failed to read from DHT sensor!"));
    return 0; // ERROR
  }
  return 1; // OK
}

/*
char begin_ds18b20();
Setup DS18B20 sensors
Return 1 if OK, 0 if ERROR
*/
char begin_ds18b20()
{
  sensor1.begin();
  sensor2.begin();
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  float temp1 = sensor1.getTempCByIndex(0);
  float temp2 = sensor2.getTempCByIndex(0);
  if (temp1 == DEVICE_DISCONNECTED_C)
  {
    Serial.println(F("Failed to read from DS18B20 sensor1!"));
    return 0; // ERROR sensor1
  }
  if (temp2 == DEVICE_DISCONNECTED_C)
  {
    Serial.println(F("Failed to read from DS18B20 sensor2!"));
    return 0; // ERROR sensor2
  }
  return 1; // OK
}

/*
char begin_scd30();
Setup SCD30 sensor
Return 1 if OK, 0 if ERROR
*/
char begin_scd30()
{
  //Wire.begin();
  if (!co2.begin())
  {
    Serial.println(F("Failed to read from CO2 sensor!"));
    return 0; // ERROR
  }
  return 1; // OK
}

/*
void begin_pwm();
Setup PWM outputs
*/
void begin_pwm()
{
  const int freq = 100; // PWM frequency
  const char res = 8;   // PWM resolution in bits
  ledcSetup(HUMIDIFIER, freq, res);
  ledcAttachPin(MOSFET_1_PIN, HUMIDIFIER);
  ledcSetup(LEDS, freq, res);
  ledcAttachPin(MOSFET_2_PIN, LEDS);
  ledcSetup(FAN, freq, res);
  ledcAttachPin(MOSFET_3_PIN, FAN);
  ledcSetup(HEATING_PAD1, freq, res);
  ledcAttachPin(MOSFET_4_PIN, HEATING_PAD1);
  ledcSetup(HEATING_PAD2, freq, res);
  ledcAttachPin(MOSFET_5_PIN, HEATING_PAD2);
}

/*
char read_ds18b20();
Read DS18B20 sensors
Return temp1 and temp2
Return 1 if OK, 0 if ERROR
*/
char read_ds18b20(float &temp1, float &temp2)
{
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();
  temp1 = sensor1.getTempCByIndex(0);
  temp2 = sensor2.getTempCByIndex(0);
  if (temp1 == DEVICE_DISCONNECTED_C)
  {
    Serial.println(F("Failed to read from DS18B20 sensor1!"));
    return 0; // ERROR sensor1
  }
  if (temp2 == DEVICE_DISCONNECTED_C)
  {
    Serial.println(F("Failed to read from DS18B20 sensor2!"));
    return 0; // ERROR sensor2
  }
  return 1; // OK
}

/*
char read_dht22()
Read temp and humidity from DHT22
Return temp and humidity
If both values are 0 => sensor ERROR
Return 1 if OK, 0 if ERROR
*/
char read_dht22(float &temp, float &hum)
{
  temp = dht.readTemperature();
  hum = dht.readHumidity();
  if (isnan(temp) || isnan(hum))
  { // If read from sensor fail (sensor not connected, ...)
    temp = 0;
    hum = 0;
    Serial.println(F("Failed to read from DHT sensor!"));
    return 0; // ERROR
  }
  // Serial.print(F("DHT22: ")), Serial.print(temp), Serial.print(F(" ")), Serial.println(hum);
  return 1; // OK
}

/*
byte time_passed(unsigned long timemark, unsigned long delay)
Check if more (or equal) of "delay" millisecond has passed or not
Return 1 if yes, 0 if not
*/
char time_passed(unsigned long timemark, unsigned long delay)
{
  char x = 0;
  unsigned long new_millis = millis();
  if ((new_millis - timemark) >= delay)
  {
    x = 1;
  }
  return x;
}

/*
unsigned long time_mark()
Set time mark
Return value of timer
*/
unsigned long time_mark()
{
  return millis();
}

/*
void reg_temp()
Regulate temperature with hysteresis
*/
void reg_temp(float measured_temp, float desired_temp, float hyst, char pwm_ch)
{
  float dif = measured_temp - desired_temp;
  if (dif > hyst)
  {
    ledcWrite(pwm_ch, 50);
  }
  if (dif < hyst)
  {
    ledcWrite(pwm_ch, 0);
  }
}
