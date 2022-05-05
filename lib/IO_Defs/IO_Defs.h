#ifndef IO_Defs_h
#define IO_Defs_h

// Connector values on PCB Shroombox v1.0

//IO pin for DHT22
#define DHT_PIN 23 // Air temperature and humidity sensor, connector J23

//IO pins for DS18B20
#define DS18B20_1_PIN 0 // Room temperature, connector J24
#define DS18B20_2_PIN 15 // Heater temperature, connector J25

//IO pins for stepper motor, connector J22
#define STPR_PIN1 19
#define STPR_PIN2 18
#define STPR_PIN3 17
#define STPR_PIN4 16

//IO pins for butons
#define BTN_1_PIN 33 // Button, connector J15
#define BTN_2_PIN 32 // Button, connector J16

//IO pin for LED
//#define LED_PIN 2 //LED used by Blynk Edgent for status indication, connector J19

//IO pins for MOSFET
#define MOSFET_1_PIN 26 // Humidifier, connector J1
#define MOSFET_2_PIN 27 // LEDs, connector J2
#define MOSFET_3_PIN 14 // Fan, connector J3
#define MOSFET_4_PIN 12 // Heating pad 1, connector J4
#define MOSFET_5_PIN 13 // Heating pad 2, connector J5

//IO pin for photoresistor
#define PHOTRES_PIN 35 // Brightness sensor, connector J21

//IO pin for SEN0193
#define SEN0193_PIN 34 // Soil moisture sensor, connector J20

#endif