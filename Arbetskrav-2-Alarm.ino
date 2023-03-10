/*
  Arbetskrav 2 - Alarmsystem

  Components used:
    IPS LCD-screen from Adafruit (ST7789). 240x135
    Membrane Switch module 4x4 (keypad)
    Ultrasonic sensor (HC-SR04)
    Buzzer
    RGB-LED


  References:
  Adafruit_GFX - https://www.arduino.cc/reference/en/libraries/adafruit-gfx-library/
  Adafruit_ST7789 - https://github.com/adafruit/Adafruit-ST7735-Library (ST7789)
  SPI - https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/SPI/src/SPI.h
  NewPing - https://www.arduino.cc/reference/en/libraries/newping/
  NewTone - https://bitbucket.org/teckel12/arduino-new-tone/wiki/Home
  pitches - Taken from Arduino Examples - Digital - toneMelody
  Keypad Library for Arduino - https://playground.arduino.cc/Code/Keypad/
*/

#include <Adafruit_GFX.h>  // Graphics
#include <Adafruit_ST7789.h>
#include <SPI.h>

#include <NewPing.h>
#include <NewTone.h>
#include "pitches.h"
#include <Keypad.h>


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

bool printWarning = true;
unsigned long startMillis;
unsigned long alarmTriggeredTime;
const unsigned long pinCodePeriod = 2000;  //10 seconds

void setup() {
  Serial.begin(115200);
  tft.init(135, 240);
}

void loop() {
  //Timer for when Alarm system is on
  startMillis = millis();


  int currentDistance = sonar.ping_cm();
  Serial.println(currentDistance);
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);

  if (currentDistance <= IS_BLOCKED) {  //If ultrasonic sensor returns 0 (blocked or signal lost)
    //different warning sound
    noNewTone(SPEAKER_PIN);  //temp
  }

  if (currentDistance < 30) {  //Alarm triggered
    alarmTriggeredTime = millis();
    Serial.println(startMillis);
    Serial.println(alarmTriggeredTime);
    unsigned long duration = alarmTriggeredTime - startMillis - 129;    //Seems like there is a delay of 129ms before calculating from 0.
    Serial.println(duration);

    if (printWarning) {
      printAlarmMessage(duration);
      delay(100);
      printWarning = false;
    }

    while (duration <= pinCodePeriod) {  //You have 10s until the real alarm goes off to enter right pin
      Serial.println(duration);
      alarmTriggeredTime = millis();
      duration = alarmTriggeredTime - startMillis;
      Serial.print("Duration: ");
      Serial.println(duration);
      //Serial.println("Alarm enter pin time");
      playAlarm();
      //regular static beeeeeeeeeep sound
      //can enter pin
    }
    while (pinCodePeriod >= duration) {
      Serial.println("ALAAAARM");
      noNewTone(SPEAKER_PIN);  //temp

      //playAlarm();
      //PLAY ACTUAL LOUD ALARM
      //must enter master pin to shut off
    }
  } else {
    noNewTone(SPEAKER_PIN);  //temp
    printWarning = true;
  }
}

void printAlarmMessage(unsigned long duration) {
  if (duration <= pinCodePeriod) {
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Alarm triggered. Enter PIN");
  } else {
    tft.setTextColor(ST77XX_RED);
    tft.print("WARNING!!!!");
  }
}


void printCurrentDate() {
}

void printAlarmDates() {
}

void saveAlarmDate() {
}

void playAlarm() {
  for (int alarmNote = 0; alarmNote < 2; alarmNote++) {
    NewTone(SPEAKER_PIN, alarmMelody[alarmNote]);
    delay(noteDuration);
  }
}
