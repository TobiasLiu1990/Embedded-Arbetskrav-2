/*
  Arbetskrav 2 - Alarmsystem

  Components used:
    IPS LCD-screen from Adafruit (ST7789). 240x135
    Real time clock module ZS-042 (DS3231)
    Membrane Switch module 4x4 (keypad)
    Ultrasonic sensor (HC-SR04)
    Buzzer
    RGB-LED

  References:
    Adafruit_GFX - https://www.arduino.cc/reference/en/libraries/adafruit-gfx-library/
    Adafruit_ST7789 - https://github.com/adafruit/Adafruit-ST7735-Library (ST7789)
    SPI - https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/SPI/src/SPI.h

    Wire.h - 
    RtcD3221 -

    NewPing - https://www.arduino.cc/reference/en/libraries/newping/
    NewTone - https://bitbucket.org/teckel12/arduino-new-tone/wiki/Home
    pitches - Taken from Arduino Examples - Digital - toneMelody
    Keypad Library for Arduino - https://playground.arduino.cc/Code/Keypad/
*/

/*
  Source: https://bitbucket.org/teckel12/arduino-new-ping/wiki/Multiple%20Definition%20of%20%22__vector_7%22%20Error
  Here i used another library: NewTone because i was getting a __vector_7 error, most likely because NewPing and Tone library tries to access the same timer 2.


  https://microcontrollerslab.com/arduino-timer-interrupts-tutorial/
  Timers used for interrupts.
  Interrupts used to handle events that occur a condition.
  Timer interrupts in Arduino - pause the sequential execution of a program loop() function for a predefined number of secounds.
  Arduino has 3 timers: 
    Timer0 (8-bit)
    Timer1 (16-bit) - can count from 0-65537
    Timer2 (8-bit) - can count from 0-255
*/


#include <Adafruit_GFX.h>  // Graphics
#include <Adafruit_ST7789.h>
#include <SPI.h>

//For normal hardware wire
#include <Wire.h>  // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>

#include <NewPing.h>
#include <NewTone.h>
#include "pitches.h"
#include <Keypad.h>


//IPS LCD-screen used with Arduino Uno
#define TFT_CS 10
#define TFT_DC 8
#define TFT_RST 9

//RGB-LED
#define LED_PIN 2

//Ultrasonic sensor
#define TRIGGER_PIN A0
#define ECHO_PIN A1
#define ALARM_DISTANCE 150
#define IS_BLOCKED 0

//Keypad - Some code reused from Arbetskrav 1
const byte rows = 4;
const byte cols = 4;
const byte rowPins[rows] = { 8, 7, 6, 5 };
const byte colPins[cols] = { 4, 3, 2, 1 };
const char keys[rows][cols] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '#', '0', '*', 'D' }
};

//Buzzer
#define SPEAKER_PIN A2
int alarmMelody[] = { NOTE_C4, NOTE_G3 };
int noteDuration = 350;


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, ALARM_DISTANCE);
RtcDS3231<TwoWire> Rtc(Wire);

bool printWarning = true;
unsigned long startMillis;
unsigned long alarmTriggerTime;
const unsigned long pinCodePeriod = 3000;  //should be 10 seconds

void setup() {
  Serial.begin(115200);
  tft.init(135, 240);
  Rtc.Begin();
}

void loop() {
  //Timer for when Alarm system is on
  startMillis = millis();

  int currentDistance = sonar.ping_cm();
  Serial.println(currentDistance);  //remove later, only debug
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);

  if (currentDistance <= IS_BLOCKED) {  //If ultrasonic sensor returns 0 (blocked or signal lost)
    //different warning sound
    noNewTone(SPEAKER_PIN);  //temp
  }

  if (currentDistance < 30) {
    alarmTriggerTime = millis();
    unsigned long duration = alarmTriggerTime - startMillis - 129;  //Seems like there is a delay of 129ms before calculating from 0.

    if (printWarning) {
      printEnterPinPeriod();
      delay(100);
      printWarning = false;
    }

    while (duration < pinCodePeriod) {  //You have 10s until the real alarm goes off to enter right pin
      alarmTriggerTime = millis();
      duration = alarmTriggerTime - startMillis;
      Serial.print("Duration: ");
      Serial.println(duration);

      NewTone(SPEAKER_PIN, 500);
      //can enter pin
    }
    while (pinCodePeriod >= duration) {
      Serial.println("ALAAAARM");
      delay(100);
      noNewTone(SPEAKER_PIN);  //temp

      if (printWarning) {
        printAlarmMessage();
        delay(100);
        printWarning = false;
      }

      playAlarm();
      //PLAY ACTUAL LOUD ALARM
      //must enter master pin to shut off
    }
  } else {
    printWarning = true;
  }
}

void printEnterPinPeriod() {
  tft.setTextColor(ST77XX_WHITE);
  tft.print("Alarm triggered. Enter PIN");
}

void printAlarmMessage() {
  tft.setTextColor(ST77XX_RED);
  tft.print("WARNING!!!!");
}


#define countof(arr) (sizeof(arr) / sizeof(arr[0]))  //Macro to get number of elements in array

void playAlarm() {
  for (int alarmNote = 0; alarmNote < countof(alarmMelody); alarmNote++) {
    NewTone(SPEAKER_PIN, alarmMelody[alarmNote]);
    delay(noteDuration);
  }
}

void printCurrentDate(const RtcDateTime& date) {  //Example code from DS3231_Simple (Rtc by Makuna)
  char dateString[20];

  //snprintf_P - Reads from flash memory (non-volatile), reduced cost. Function formats and stores a series of chars in array buffer. Accepts n arguments.
  //PSTR - Uses flash memory too (?) Only be used in functions (?)

  snprintf_P(dateString,                             //buffer
             countof(dateString),                    //max number of bytes (char), written to buffer
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),  //PSTR reads from flash mem. n (...) is for formating
             date.Month(),                           //rest of params to format
             date.Day(),
             date.Year(),
             date.Hour(),
             date.Minute(),
             date.Second());
  Serial.print(dateString);
}

void printAlarmDates() {
}

void saveAlarmDate() {
}
