/*
  Arbetskrav 2 - Alarmsystem

  Components used:
    IPS LCD-screen from Adafruit (ST7789). 240x135
    Real time clock module ZS-042 (DS3231)
    Membrane Switch module 4x4 (keypad)
    Ultrasonic sensor (HC-SR04)
    Buzzer
    RGB-LED
    220 resistor

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
#include <NewPing.h>
#include <NewTone.h>
#include "pitches.h"
#include <Keypad.h>
//For normal hardware wire
#include <Wire.h>  // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>
//For normal hardware wire


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
int noteDuration = 250;


Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, rows, cols);
NewPing sonar(TRIGGER_PIN, ECHO_PIN, ALARM_DISTANCE);
RtcDS3231<TwoWire> Rtc(Wire);

bool wasError(const char* errorTopic = "") {
  uint8_t error = Rtc.LastError();
  if (error != 0) {
    // we have a communications error
    // see https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
    // for what the number means
    Serial.print("[");
    Serial.print(errorTopic);
    Serial.print("] WIRE communications error (");
    Serial.print(error);
    Serial.print(") : ");

    switch (error) {
      case Rtc_Wire_Error_None:
        Serial.println("(none?!)");
        break;
      case Rtc_Wire_Error_TxBufferOverflow:
        Serial.println("transmit buffer overflow");
        break;
      case Rtc_Wire_Error_NoAddressableDevice:
        Serial.println("no device responded");
        break;
      case Rtc_Wire_Error_UnsupportedRequest:
        Serial.println("device doesn't support request");
        break;
      case Rtc_Wire_Error_Unspecific:
        Serial.println("unspecified error");
        break;
      case Rtc_Wire_Error_CommunicationTimeout:
        Serial.println("communications timed out");
        break;
    }
    return true;
  }
  return false;
}

bool printWarning = true;

unsigned long timeWhenAlarmTriggered;
unsigned long elapsedMillis;
const unsigned long pinEntryTime = 2000;  //should be 10 seconds

unsigned long previousTimeForDateAndTime = 0;
const unsigned long printDateAndTimeInterval = 5000;  //should be 10 seconds

bool runErrorHandlingOnce = true;

//Used in setup() once, and then once in loop() - but in loop() everything inside the 3rd if-statement is not run.
RtcDateTime checkDateTimeErrors() {
  //Error checking code taken from DS3231_Simple (Rtc by Hakuna)
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);

  if (!Rtc.IsDateTimeValid()) {
    if (!wasError("setup IsDateTimeValid")) {
      // Common Causes:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing

      Serial.println("RTC lost confidence in the DateTime!");

      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue

      if (runErrorHandlingOnce) {
        Rtc.SetDateTime(compiled);
        Serial.print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
        runErrorHandlingOnce = false;
        return compiled;
      }
    }
  }
}


void setup() {
  Serial.begin(115200);

  tft.init(135, 240);
  Rtc.Begin();

  RtcDateTime compiled = checkDateTimeErrors();
  printDateTime(compiled);
  Serial.println();

  if (!Rtc.GetIsRunning()) {
    if (!wasError("setup GetIsRunning")) {
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
    }
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (!wasError("setup GetDateTime")) {
    if (now < compiled) {
      Serial.println("RTC is older than compile time, updating DateTime");
      Rtc.SetDateTime(compiled);
    } else if (now > compiled) {
      Serial.println("RTC is newer than compile time, this is expected");
    } else if (now == compiled) {
      Serial.println("RTC is the same as compile time, while not expected all is still fine");
    }
  }

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  wasError("setup Enable32kHzPin");
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
  wasError("setup SetSquareWavePin");
}





void loop() {
  int currentDistance = sonar.ping_cm();
  //Timer for when Alarm system is on
  unsigned long currentMillis = millis();
  //Serial.println(currentDistance);  // DEBUG

  checkDateTimeErrors();
  printInterval(currentMillis);
  resetTftScreen(ST77XX_BLACK);

  //If ultrasonic sensor returns 0 (blocked or signal lost)
  if (currentDistance <= IS_BLOCKED) {
    NewTone(SPEAKER_PIN, 1200);
  } else {
    noNewTone(SPEAKER_PIN);
  }

  //Alarm is triggered
  if (currentDistance < 30) {
    if (printWarning) {
      printEnterPinPeriod();
      delay(100);
      printWarning = false;
    }

    getElapsedAlarmTriggertime(currentMillis);
    
    while (elapsedMillis < pinEntryTime) {  //You have 10s until the real alarm goes off to enter right pin
      getElapsedAlarmTriggertime(currentMillis);

      Serial.print("Elapsed time: ");
      Serial.println(elapsedMillis);

      NewTone(SPEAKER_PIN, 500);
      //ADD CODE LATER FOR KEYPAD - ENTER PIN HERE
    }
    while (elapsedMillis >= pinEntryTime) {
      Serial.println("ALAAAARM");

      if (printWarning) {
        printAlarmMessage();
        delay(100);
        printWarning = false;
      }

      playAlarm();
      //must enter master pin to shut off
    }
  } else {
    noNewTone(SPEAKER_PIN);
    printWarning = true;
  }
}

void getElapsedAlarmTriggertime(unsigned long currentMillis) {
  timeWhenAlarmTriggered = millis();
  elapsedMillis = timeWhenAlarmTriggered - currentMillis;
}

void resetTftScreen(uint16_t color) {
  tft.setCursor(0, 0);
  tft.fillScreen(color);
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

//Should trigger something every x secounds
void printInterval(unsigned long currentMillis) {
  unsigned long printTime = currentMillis - previousTimeForDateAndTime;

  if (currentMillis - previousTimeForDateAndTime >= printDateAndTimeInterval) {
    RtcDateTime date = Rtc.GetDateTime();
    printDateTime(date);
    Serial.println();
    previousTimeForDateAndTime = currentMillis;
  }
}

void printDateTime(const RtcDateTime& date) {  //Example code from DS3231_Simple (Rtc by Makuna)
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
