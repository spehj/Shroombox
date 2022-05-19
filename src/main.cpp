/*
Product: Shroombox
Authors: Matic Sedej, Jure Špeh
Date: May 2022
GitHub: https://github.com/spehj/Shroombox
*/

#include <Arduino.h>
#include <AccelStepper.h>
//#include <Adafruit_Sensor.h>
//#include <DHT.h>
#include <Wire.h>
#include <SHT31.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "SparkFun_SCD30_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_SCD30
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h> //<Adafruit_SSD1306.h>
#include "IO_Defs.h"
#include "Blynk_Virtual_Pins.h"
#include "Icons_16x16.h" // Wifi icons for display

// https://docs.blynk.io/en/getting-started/activating-devices/blynk-edgent-wifi-provisioning
// https://docs.blynk.io/en/getting-started/updating-devices-firmwares-ota

#define BLYNK_TEMPLATE_ID "TMPLWxVCUiA-" // Copy from Blynk template
#define BLYNK_DEVICE_NAME "Shroombox V1" // Copy from Blynk template

#define BLYNK_FIRMWARE_VERSION "0.1.26" // Change the Firmware version every time, otherwise device will ignore it and won't update OTA!

#define BLYNK_PRINT Serial //#define BLYNK_DEBUG
#define APP_DEBUG
#include "BlynkEdgent.h" // Must be below blynk defines!

#define HUMIDIFIER 0   // PWM channel 0
#define LEDS 1         // PWM channel 1
#define FAN 2          // PWM channel 2
#define HEATING_PAD1 3 // PWM channel 3
#define HEATING_PAD2 4 // PWM channel 4

#define GP1 1
#define GP2 2

#define DISPLAY_W 128 // OLED display width
#define DISPLAY_H 64  // OLED display height
#define DISPLAY_ADR 0x3C

#define SHT30_ADDRESS 0x44
Adafruit_SH1106 display(SDA, SCL);
// Adafruit_SSD1306 display(DISPLAY_W, DISPLAY_H, &Wire);

//DHT dht(DHT_PIN, DHT22);
OneWire oneWire1(DS18B20_1_PIN);
OneWire oneWire2(DS18B20_2_PIN);
DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
AccelStepper stepper(AccelStepper::FULL4WIRE, STPR_PIN1, STPR_PIN2, STPR_PIN3, STPR_PIN4);

SCD30 sensorco2;

SHT31 sht;

BlynkTimer blynkTimer;
WidgetTerminal terminal(BLYNK_TERMINAL);

/****************************
Variables */

float air_temp, air_hum;
float room_temp, heater_temp;
char wifi_strength;
uint16_t co2;
String substrate_moist;
float hyst_temp = 3;
float hyst_hum = 4;
float hyst_co2 = 200;
unsigned char pwm_duty = 0;

unsigned long light_on_t = 0;
float goal_temp = 0;
unsigned char goal_hum = 0;
unsigned int goal_co2 = 0;

unsigned int heatpad_auto_set_pwm = 100;
unsigned int heatpad_auto_pwm = 0;

unsigned int hum_auto_set_pwm = 150;
unsigned int hum_auto_pwm = 0;

unsigned int fan_auto_set_pwm = 180;
unsigned int fan_auto_pwm = 0;

unsigned int led_auto_set_pwm = 180;
unsigned int led_auto_pwm = 0;

unsigned int heatpad_man_pwm = 0;
unsigned int fan_man_pwm = 0;
unsigned int hum_man = 0;
unsigned char light_on_t_gp1 = 0;
float goal_temp_gp1 = 0;
unsigned char goal_hum_gp1 = 0;
unsigned int goal_co2_gp1 = 0;
unsigned char light_on_t_gp2 = 0;
float goal_temp_gp2 = 0;
unsigned char goal_hum_gp2 = 0;
String shroombox_status;

char auto_man = 0; // 0-auto, 1-man
char growth_phase = 0;
float pwm_scale_factor = 2.55;

// Status variables for app LEDS
int hum_status = 0;
int led_status = 0;
int fan_status = 0;
int heat_status = 0;

int last_hum_status = 0;
int last_led_status = 0;
int last_fan_status = 0;
int last_heat_status = 0;

unsigned int goal_co2_gp2 = 0;

char main_switch = 0; // 0-OFF, 1-ON

/****************************
Prototypes of functions */

void begin_stepper();
void begin_io();
char begin_sht30();
char begin_ds18b20();
char begin_scd30();
void begin_pwm();
char read_sht30(float &temp, float &hum);
char read_ds18b20(float &temp1, float &temp2);
String read_sen0193();
void begin_display();
void display_values();
char time_passed(unsigned long timemark, unsigned long delay);
unsigned long time_mark();
void reg_temp(float measured_temp, float desired_temp, float hyst);
void reg_leds();
void auto_mode();
void manual_mode();
void shutdown();
void mode();
char check_wifi_strength();
void select_setting();
void check_actuators();
int reg_co2(float measured_co2, float desired_co2, float hyst);

// char read_sht30()
/****************************/
void setup()
{
  Serial.begin(115200);
  begin_sht30();
  begin_ds18b20();
  // begin_stepper();
  begin_io();
  begin_pwm();
  begin_display();
  begin_scd30();
  shutdown();
  delay(100);

  BlynkEdgent.begin();
  // Clear the terminal content
  terminal.clear();

  terminal.println(F("Blynk v" BLYNK_FIRMWARE_VERSION ": Shroombox started"));
  terminal.println(F("-------------"));
  terminal.flush();
}

/****************************/
/*** BLYNK WRITE ***/

BLYNK_WRITE(MAIN_ON_OFF) // Executes when the value of virtual pin 0 changes
{
  if (param.asInt() == 0)
  {
    // execute this code if the switch widget is now OFF
    main_switch = 0;
    shutdown();
  }
  else if (param.asInt() == 1)
  {
    // execute this code if the switch widget is now ON
    main_switch = 1;
  }
}

BLYNK_WRITE(AUTO_MAN)
{
  if (param.asInt() == 0)
  {
    // Growbox in AUTO mode
    auto_man = 0;
  }
  else if (param.asInt() == 1)
  {
    // Growbox in MAN mode
    auto_man = 1;
  }
}

BLYNK_WRITE(GROWTH_PHASE)
{
  if (param.asInt() == 1)
  {
    // Growbox in growth phase 1
    growth_phase = GP1;
  }
  else if (param.asInt() == 2)
  {
    // Growbox in growth phase 2
    growth_phase = GP2;
  }
}

unsigned int led_man_pwm = 0; // LED percentage from 0 to 100 times 2.5 to scale from 0 to 255
BLYNK_WRITE(LED_MAN)
{
  led_man_pwm = (int)round((param.asInt()) * pwm_scale_factor);
}

BLYNK_WRITE(HEATPAD_MAN)
{
  heatpad_man_pwm = (int)round((param.asInt()) * pwm_scale_factor);
}

BLYNK_WRITE(VENTILATOR_MAN)
{
  fan_man_pwm = (int)round((param.asInt()) * pwm_scale_factor);
}

BLYNK_WRITE(HUM_MAN)
{
  hum_man = (int)round((param.asInt()) * pwm_scale_factor);
}

/* Settings for automatic control for GP1 and GP2*/
/* GP1 */

BLYNK_WRITE(BRIGHT_TIME_ON_GP1)
{
  light_on_t_gp1 = param.asInt();
}

BLYNK_WRITE(SET_AIR_TEMP_GP1)
{
  goal_temp_gp1 = param.asFloat();
}

BLYNK_WRITE(SET_HUM_GP1)
{
  goal_hum_gp1 = param.asInt();
}

BLYNK_WRITE(SET_CO2_GP1)
{
  goal_co2_gp1 = param.asInt();
}

/* GP2 */

BLYNK_WRITE(BRIGHT_TIME_ON_GP2)
{
  light_on_t_gp2 = param.asInt();
}

BLYNK_WRITE(SET_AIR_TEMP_GP2)
{
  goal_temp_gp2 = param.asFloat();
}

BLYNK_WRITE(SET_HUM_GP2)
{
  goal_hum_gp2 = param.asInt();
}

BLYNK_WRITE(SET_CO2_GP2)
{
  goal_co2_gp2 = param.asInt();
}

BLYNK_WRITE(SHROOMBOX_STATUS)
{
  shroombox_status = param.asString();
}

BLYNK_WRITE(BLYNK_TERMINAL)
{ // Ukaz oblike: co2_h 2.0
  int space1, space2, space3, space4;
  int param1, param2, param3, param4;
  String buf = param.asStr();
  space1 = buf.indexOf(' ');
  space2 = buf.indexOf(' ', space1 + 1);
  // space3 = buf.indexOf(' ', space2 + 1);
  // space4 = buf.indexOf(' ', space3 + 1);
  param1 = buf.substring(space1, space2).toFloat();
  // param2=buf.substring(space2, space3).toInt();
  // param3=buf.substring(space3).toInt();
  // param4=buf.substring(space4).toInt();

  if (buf.startsWith("coh"))
  {
    // Set co2 hysteresis
    terminal.print("CO2 hysteresis was set to +-");
    terminal.println(hyst_co2);
    hyst_co2 = param1;
    terminal.print("CO2 hysteresis is set to +-");
    terminal.println(hyst_co2);
    terminal.flush();
  }

  if (buf.startsWith("teh"))
  {
    // Set temp hysteresis
    terminal.print("Temperature hysteresis was set to +-");
    terminal.println(hyst_temp);
    hyst_temp = param1;
    terminal.print("Temperature hysteresis is set to +-");
    terminal.println(hyst_temp);
    terminal.flush();
  }

  if (buf.startsWith("huh"))
  {
    // Set humidity hysteresis
    terminal.print("Humidity hysteresis was set to +-");
    terminal.println(hyst_hum);
    hyst_hum = param1;
    terminal.print("Humidity hysteresis is set to +-");
    terminal.println(hyst_hum);
    terminal.flush();
  }

  if (buf.startsWith("hep"))
  {
    // Set heatpad pwm power from 0 to 255
    terminal.print("Heatpad pwm was: ");
    terminal.println(heatpad_auto_set_pwm);
    heatpad_auto_set_pwm = param1;
    terminal.print("Heatpad pwm is set to: ");
    terminal.println(heatpad_auto_set_pwm);
    terminal.flush();
  }

  if (buf.startsWith("hup"))
  {
    // Set humidifier auto pwm from 0 to 255
    terminal.print("Humidifier pwm was: ");
    terminal.println(hum_auto_set_pwm);
    hum_auto_set_pwm = param1;
    terminal.print("Humidifier pwm is set to: ");
    terminal.println(hum_auto_set_pwm);
    terminal.flush();
  }

  if (buf.startsWith("fap"))
  {
    // Set fan auto pwm from 0 to 255
    terminal.print("Fan pwm was: ");
    terminal.println(fan_auto_set_pwm);
    fan_auto_set_pwm = param1;
    terminal.print("Fan pwm is set to: ");
    terminal.println(fan_auto_set_pwm);
    terminal.flush();
  }

  if (buf.startsWith("lpw"))
  {
    // List auto pwm values
    terminal.print("Heatpad pwm is set to: ");
    terminal.println(heatpad_auto_set_pwm);
    terminal.print("Humidifier pwm is set to: ");
    terminal.println(hum_auto_set_pwm);
    terminal.print("Fan pwm is set to: ");
    terminal.println(fan_auto_set_pwm);
    terminal.print("LED pwm is set to: ");
    terminal.println(led_auto_set_pwm);
    terminal.flush();
  }

  if (buf.startsWith("ls"))
  {
    // List statuses
    terminal.print("Heatpad status: ");
    terminal.println(heat_status);
    terminal.print("Humidifier status: ");
    terminal.println(hum_status);
    terminal.print("Fan status: ");
    terminal.println(fan_status);
    terminal.print("LED status: ");
    terminal.println(led_status);
    terminal.flush();
  }

  if (buf.startsWith("clc"))
  {
    terminal.clear();
  }

  if(buf.startsWith("lh")){
    // List hysteresis
    terminal.print("CO2 hysteresis: ");
    terminal.println(hyst_co2);
    terminal.print("Temperature hysteresis: ");
    terminal.println(hyst_temp);
    terminal.print("Humidity hysteresis: ");
    terminal.println(hyst_hum);
    terminal.flush();
  }

  if(buf.startsWith("lc")){
    // List commands
    terminal.println("Terminal commands:");
    terminal.println("------------------");
    terminal.println("Co2 hyst: coh <value>");
    terminal.println("Temp hyst: teh <value>");
    terminal.println("Humid hyst: huh <value>");
    terminal.println("------------------");
    terminal.println("Heat pwm: hep <value>");
    terminal.println("Hum pwm: hup <value>");
    terminal.println("Fan pwm: fap <value>");
    terminal.println("------------------");
    terminal.println("List pwm values: lpw");
    terminal.println("------------------");
    terminal.println("List statuses: ls");
    terminal.println("------------------");
    terminal.println("List commands: lc");
    terminal.println("------------------");
    terminal.println("List hysteresis: lh");
    terminal.println("------------------");
    terminal.println("Clear terminal: clc");
    terminal.flush();
  }
}

BLYNK_CONNECTED()
{
  // blynkTimer.setInterval(1011L, check_actuators);
  // blynkTimer.setInterval(1223L, select_setting);
  Blynk.syncAll();
}

/*** BLYNK WRITE END***/
unsigned long time_temp = time_mark();

void loop()
{

  if (time_passed(time_temp, 4000))
  { // Measure every 4s
    read_sht30(air_temp, air_hum);
    read_ds18b20(heater_temp, room_temp);
    if (sensorco2.dataAvailable())
    {
      co2 = sensorco2.getCO2();
    }
    substrate_moist = read_sen0193();
    wifi_strength = check_wifi_strength();
    time_temp = time_mark();
    // Test Blynk
    Blynk.virtualWrite(AIR_TEMP, air_temp);
    Blynk.virtualWrite(AIR_HUM, air_hum);
    Blynk.virtualWrite(ROOM_TEMP, room_temp);
    Blynk.virtualWrite(HEATER_TEMP, heater_temp);
    Blynk.virtualWrite(CO2, co2);
    Blynk.virtualWrite(SUBSTRATE_MOIST, substrate_moist);
    Blynk.virtualWrite(WIFI_STRENGTH, wifi_strength);
    display_values();
  }
  mode();
  select_setting();

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
void begin_display()
{
  display.begin(SH1106_SWITCHCAPVCC, DISPLAY_ADR);
  display.clearDisplay();      // Clear display
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(F("Shroombox ")), display.println(BLYNK_FIRMWARE_VERSION);
  display.display(); // Show on display
}

/*ledcWrite(HUMIDIFIER, 0);
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
char begin_sht30()
{
  Wire.begin();
  sht.begin(SHT30_ADDRESS);
  Wire.setClock(100000);
  sht.read();

  float Humidity = sht.getHumidity();
  float Temperature = sht.getTemperature();
  if (isnan(Humidity) || isnan(Temperature))
  { // If read from sensor fail (sensor not connected, ...)
    Humidity = 0;
    Temperature = 0;
    // Serial.println(F("Failed to read from DHT sensor!"));
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
  float temp1 = sensor1.getTempCByIndex(0);
  sensor2.requestTemperatures();
  float temp2 = sensor2.getTempCByIndex(0);
  if (temp1 == DEVICE_DISCONNECTED_C)
  {
    // Serial.println(F("Failed to read from DS18B20 sensor1!"));
    return 0; // ERROR sensor1
  }
  if (temp2 == DEVICE_DISCONNECTED_C)
  {
    // Serial.println(F("Failed to read from DS18B20 sensor2!"));
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
  // Wire.begin();
  if (!sensorco2.begin())
  {
    // Serial.println(F("Failed to read from CO2 sensor!"));
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
  temp1 = sensor1.getTempCByIndex(0);
  sensor2.requestTemperatures();
  temp2 = sensor2.getTempCByIndex(0);
  if (temp1 == DEVICE_DISCONNECTED_C)
  {
    // Serial.println(F("Failed to read from DS18B20 sensor1!"));
    return 0; // ERROR sensor1
  }
  if (temp2 == DEVICE_DISCONNECTED_C)
  {
    // Serial.println(F("Failed to read from DS18B20 sensor2!"));
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
char read_sht30(float &temp, float &hum)
{
  sht.read();
  temp = sht.getTemperature();
  hum = sht.getHumidity();
  if (isnan(temp) || isnan(hum))
  { // If read from sensor fail (sensor not connected, ...)
    temp = 0;
    hum = 0;
    // Serial.println(F("Failed to read from DHT sensor!"));
    return 0; // ERROR
  }
  // Serial.print(F("DHT22: ")), Serial.print(temp), Serial.print(F(" ")), Serial.println(hum);
  return 1; // OK
}

/*
String read_sen0193()
*/
String read_sen0193()
{
  const int AirValue = 3000;
  const int WaterValue = 1500;

  int intervals = (AirValue - WaterValue) / 3;
  String txt;
  int adc;
  adc = analogRead(SEN0193_PIN);
  if ((adc > WaterValue && adc < (WaterValue + intervals)) || (adc < WaterValue))
  {
    txt = "Very wet";
  }
  else if (adc > (WaterValue + intervals) && adc < (AirValue - intervals))
  {
    txt = "Wet";
  }
  else if (( adc < AirValue && adc > (AirValue - intervals)) || (adc > AirValue))
  {
    txt = "Dry";
  }
  else
  {
    txt = "Undefined";
  }
  return txt;
  // return adc;
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
void display_values()
Display values temp, hum, co2, ... on OLED
*/
void display_values()
{
  display.clearDisplay();      // Clear display
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(WHITE); // Draw white text
  display.setCursor(0, 16);    // Start at top-left corner
  display.print("T "), display.print(air_temp), display.println(" C");
  display.print("H "), display.print(air_hum), display.println(" %");
  display.print("C "), display.print(co2), display.println(" ppm");
  if (wifi_strength < 25)
  {
    display.drawBitmap(110, 0, signal1_icon16x16, 16, 16, 1);
  }
  else if (wifi_strength >= 25 && wifi_strength < 50)
  {
    display.drawBitmap(110, 0, signal2_icon16x16, 16, 16, 1);
  }
  else if (wifi_strength >= 50 && wifi_strength < 75)
  {
    display.drawBitmap(110, 0, signal3_icon16x16, 16, 16, 1);
  }
  else if (wifi_strength >= 75)
  {
    display.drawBitmap(110, 0, signal4_icon16x16, 16, 16, 1);
  }

  display.display(); // Show on display
}

/*
void reg_temp()
Regulate temperature with hysteresis
*/

void reg_temp(float measured_temp, float desired_temp, float hyst)
{

  if (measured_temp <= (desired_temp - hyst / 2.0)) // Lower limit
  {
    heatpad_auto_pwm = heatpad_auto_set_pwm;
    heat_status = 1;
    // ledcWrite(HEATING_PAD1, heatpad_auto_pwm); // 60% duty cycle
    // ledcWrite(HEATING_PAD2, heatpad_auto_pwm); // 60% duty cycle
  }
  else if (measured_temp >= (desired_temp + hyst / 2.0)) // Upper limit
  {
    heatpad_auto_pwm = 0;
    heat_status = 0;
    // ledcWrite(HEATING_PAD1, 0); // 0% duty cycle
    // ledcWrite(HEATING_PAD2, 0); // 0% duty cycle
  }

  ledcWrite(HEATING_PAD1, heatpad_auto_pwm);
  ledcWrite(HEATING_PAD2, heatpad_auto_pwm);
}

/*
void reg_hum()
Regulate humidity with hysteresis
*/

void reg_hum(float measured_hum, float desired_hum, float hyst)
{
  if (measured_hum <= (desired_hum - hyst / 2.0)) // Lower limit
  {
    hum_auto_pwm = hum_auto_set_pwm;
    hum_status = 1;
    // ledcWrite(HUMIDIFIER, hum_auto_pwm); // 60% duty cycle
  }
  else if (measured_hum >= (desired_hum + hyst / 2.0)) // Upper limit
  {
    hum_auto_pwm = 0;
    hum_status = 0;
    // ledcWrite(HUMIDIFIER, 0); // 0% duty cycle
  }
  ledcWrite(HUMIDIFIER, hum_auto_pwm);
}

/*
void reg_co2()
Regulate co2 with hysteresis
*/
int reg_co2(float measured_co2, float desired_co2, float hyst)
{
  int flag = 0;
  if (measured_co2 <= (desired_co2 - hyst / 2.0)) // Lower limit
  {
    fan_auto_pwm = 0;
    fan_status = 0;
    flag = 0;
    // ledcWrite(FAN, 0); // 0% duty cycle
  }
  else if (measured_co2 >= (desired_co2 + hyst / 2.0)) // Upper limit
  {
    fan_auto_pwm = fan_auto_set_pwm;
    fan_status = 1;
    flag = 1;
    // ledcWrite(FAN, 150); // 60% duty cycle
  }
  else
  {
    flag = 1; // When inside hysteresis, set flag to 1 so humidifier doesn't turn on when fan is on
  }
  ledcWrite(FAN, fan_auto_pwm); // 60% duty cycle

  return flag;
}

/*
void reg_leds()
Regulate LEDs with timer
*/
void reg_leds()
{
  unsigned long cycle_t = 120;//60*60*24; // Cycle time unit: seconds
  //unsigned char light_on_t = 2; // Unit: seconds
  unsigned long light_off_t = cycle_t - light_on_t; // Unit: seconds
  static unsigned long timex = time_mark() - cycle_t*1000; // *1000 to convert to milliseconds
  static char led_flag = 0; // First turn LEDs on

  if (time_passed(timex, light_on_t*1000) && (led_flag == 1)) // *1000 to convert to milliseconds
  {
    led_auto_pwm = 0;
    ledcWrite(LEDS,led_auto_pwm);
    led_flag = 0;
    timex = time_mark();
  }
  else if (time_passed(timex, light_off_t*1000) && (led_flag == 0)) // *1000 to convert to milliseconds
  {
    led_auto_pwm = led_auto_set_pwm;
    ledcWrite(LEDS,led_auto_pwm);
    led_flag = 1;
    timex = time_mark();
  }
}

void mode()
{ /* Function checks mode and execute auto/man mode functions */
  if (main_switch == 1)
  {
    if (auto_man == 0)
    {
      // auto mode
      if (shroombox_status != "Automatic Mode")
      {
        Blynk.virtualWrite(SHROOMBOX_STATUS, "Automatic Mode");
      }

      auto_mode();
    }
    else if (auto_man == 1)
    {
      // manual mode
      if (shroombox_status != "Manual Mode")
      {
        Blynk.virtualWrite(SHROOMBOX_STATUS, "Manual Mode");
      }

      manual_mode();
    }
  }
  else if (main_switch == 0)
  {
    if (shroombox_status != "Shroombox OFF")
    {
      Blynk.virtualWrite(SHROOMBOX_STATUS, "Shroombox OFF");
    }
    shutdown();
  }
  else
  {
    if (shroombox_status != "Undefined Mode")
    {
      Blynk.virtualWrite(SHROOMBOX_STATUS, "Undefined Mode");
    }
  }
  check_actuators();
}


void auto_mode()
{
  int flag_co2;

  flag_co2 = reg_co2(co2, goal_co2, hyst_co2);
  if (flag_co2 == 1)
  {
    hum_auto_pwm = 0;
    ledcWrite(HUMIDIFIER, hum_auto_pwm);
  }
  else if (flag_co2 == 0)
  {
    reg_hum(air_hum, goal_hum, hyst_hum);
  }
  reg_temp(air_temp, goal_temp, hyst_temp);
  reg_leds();
}

void manual_mode()
{
  ledcWrite(HUMIDIFIER, hum_man);
  ledcWrite(LEDS, led_man_pwm);
  ledcWrite(FAN, fan_man_pwm);
  ledcWrite(HEATING_PAD1, heatpad_man_pwm);
  ledcWrite(HEATING_PAD2, heatpad_man_pwm);

  if (hum_man > 0)
  {
    hum_status = 1;
  }
  else if (hum_man <= 0)
  {
    hum_status = 0;
  }

  if (led_man_pwm > 0)
  {
    led_status = 1;
  }
  else if (led_man_pwm <= 0)
  {
    led_status = 0;
  }

  if (fan_man_pwm > 0)
  {
    fan_status = 1;
  }
  else if (fan_man_pwm <= 0)
  {
    fan_status = 0;
  }

  if (heatpad_man_pwm > 0)
  {
    heat_status = 1;
  }
  else if (heatpad_man_pwm <= 0)
  {
    heat_status = 0;
  }
}

// If main switch is OFF shutdown all actuators
void shutdown()
{
  ledcWrite(HUMIDIFIER, 0);
  ledcWrite(LEDS, 0);
  ledcWrite(FAN, 0);
  ledcWrite(HEATING_PAD1, 0);
  ledcWrite(HEATING_PAD2, 0);
  hum_status = 0;
  led_status = 0;
  fan_status = 0;
  heat_status = 0;
}

/*
char check_wifi_strength()
Get value of wifi signal strength
Return wifi signal strength in %
*/
char check_wifi_strength()
{
  int dBm = WiFi.RSSI();
  char quality;
  if (dBm <= -100) // Lower limit
  {
    quality = 0;
  }
  else if (dBm >= -50) // Upper limit
  {
    quality = 100;
  }
  else
  {
    quality = 2 * (dBm + 100);
  }
  return quality;
}

void select_setting()
{
  if (growth_phase == 1)
  {
    // Use settings for GP1
    light_on_t = light_on_t_gp1;
    goal_temp = goal_temp_gp1;
    goal_hum = goal_hum_gp1;
    goal_co2 = goal_co2_gp1;
  }
  else if (growth_phase == 2)
  {
    // Use settings for GP2
    light_on_t = light_on_t_gp2;
    goal_temp = goal_temp_gp2;
    goal_hum = goal_hum_gp2;
    goal_co2 = goal_co2_gp2;
  }
  else
  {
    if (shroombox_status != "Undefined growth phase")
    {
      Blynk.virtualWrite(SHROOMBOX_STATUS, "Undefined growth phase");
    }
  }
}

void check_actuators()
{
  if (hum_status != last_hum_status)
  {
    Blynk.virtualWrite(HUM_STATUS, hum_status);
    last_hum_status = hum_status;
  }

  if (led_status != last_led_status)
  {
    Blynk.virtualWrite(LED_STATUS, led_status);
    last_led_status = led_status;
  }

  if (fan_status != last_fan_status)
  {
    Blynk.virtualWrite(FAN_STATUS, fan_status);
    last_fan_status = fan_status;
  }

  if (heat_status != last_heat_status)
  {
    Blynk.virtualWrite(HEATPAD_STATUS, heat_status);
    last_heat_status = heat_status;
  }
}