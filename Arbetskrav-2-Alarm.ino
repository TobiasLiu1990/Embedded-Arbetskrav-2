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

    ---Adafruit ImageReader Library---
    #include <SdFat.h>                 // SD card & FAT filesystem library
    #include <Adafruit_SPIFlash.h>     // SPI / QSPI flash library
    #include <Adafruit_ImageReader.h>  // Image-reading functions
    ---Adafruit ImageReader Library---

    Wire.h - 
    RtcD3221 -

    NewPing - https://www.arduino.cc/reference/en/libraries/newping/
    NewTone - https://bitbucket.org/teckel12/arduino-new-tone/wiki/Home
    pitches - Taken from Arduino Examples - Digital - toneMelody
*/

/*
  Keypad with only 1 pin
    * 7 Resistor
    * 6 with different resistor values
      - Need to be in similar range
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
#include <SdFat.h>                 // SD card & FAT filesystem library
#include <Adafruit_SPIFlash.h>     // SPI / QSPI flash library
#include <Adafruit_ImageReader.h>  // Image-reading functions
#include <SPI.h>
#include <NewPing.h>
#include <NewTone.h>
#include "pitches.h"
//For normal hardware wire
#include <Wire.h>  // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>
//For normal hardware wire

//IPS LCD-screen used with Arduino Uno. Some code for reading SD-card taken from Adafruit ImageReader Library - BreakoutST7789-240x135.ino
#define TFT_CS 10
#define TFT_DC 8
#define TFT_RST 9
#define SD_CS 4    // SD card select pin

#define USE_SD_CARD

#if defined(USE_SD_CARD)
SdFat32 SD;
Adafruit_ImageReader reader(SD);
#endif


//RGB-LED
#define LED_PIN 2

//Ultrasonic sensor
#define TRIGGER_PIN A1
#define ECHO_PIN A2
#define ALARM_DISTANCE 150
#define IS_BLOCKED_DISTANCE 1

//Buzzer
#define SPEAKER_PIN A0
int alarmMelody[] = { NOTE_C4, NOTE_G3 };  // 2k resistor atm
int noteDuration = 250;

//Keypad
char keys[16] = {
  '1', '2', '3', 'A',
  '4', '5', '6', 'B',
  '7', '8', '9', 'C',
  '*', '0', '#', 'D'
};

int keyValues[16] = {
  924, 907, 889, 872,
  847, 833, 818, 803,
  782, 770, 757, 744,
  726, 716, 704, 694
};


//Objects
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
int32_t width = 0;
int32_t height = 0;
ImageReturnCode status;   //Status from image-reading functions

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


#define MAX_NUM_OF_DATES 10
#define DATE_LENGTH 19  //num of chars of date+time
char alarmDates[MAX_NUM_OF_DATES][DATE_LENGTH];
char dateString[20];
int counter = 0;

bool tftCheckIfPrinted = true;

unsigned long timeWhenAlarmTriggered;
unsigned long elapsedTime;
const unsigned long pinEntryTime = 10000;  //should be 10 seconds

unsigned long previousTimeForDateAndTime = 0;
const unsigned long printDateAndTimeInterval = 5000;  //should be 10 seconds

bool runErrorHandlingOnce = true;

long secretPin = 5555;
String inputPin = "";


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
        runErrorHandlingOnce = false;
        return;
      }
    }
  }
  return compiled;
}


void setup() {
  Serial.begin(115200);
  while(!Serial);   //Wait till Serial starts

  Rtc.Begin();
  tft.init(135, 240);
  tft.setRotation(0);
  resetTftScreen(ST77XX_BLACK);
  initAlarmDatesArray();

#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
#endif

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


  //Check SD file system
    Serial.print(F("Initializing filesystem..."));
  #if defined (USE_SD_CARD)
  if (!SD.begin(SD_CS, SD_SCK_MHZ(10))) {
    Serial.println(F("SD begin() failed"));
    for(;;);  //Fatal error, do not continue
  }
  #else {
    if (!flash.begin()) {
      Serial.println(F("Flash begin() failed"));
      for(;;);
    }
    if (!filesys.begin(&flash)) {
      Serial.println(F("filesys begin() failed"));
      for(;;);
    }
  }
  #endif

  //Loading SD successfull here---
  Serial.println(F("OK!"));
}



void loop() {
  int currentDistance = sonar.ping_cm();
  unsigned long currentMillis = millis();
  bool isCorrectCode = false;
  int pinEntryCounter = 0;
  //Serial.println(currentDistance);  // DEBUG

  checkDateTimeErrors();
  printDateInterval(currentMillis);

  //If ultrasonic sensor returns 0 (blocked or signal lost)
  if (currentDistance <= IS_BLOCKED_DISTANCE) {
    NewTone(SPEAKER_PIN, 1200);
  }
  //Alarm is triggered
  else if (currentDistance < 30) {
    if (tftCheckIfPrinted) {
      printEnterPinPeriod();
      delay(100);
      tftCheckIfPrinted = false;
    }

    // Save date when alarm triggered in array
    findArrayIndexForSavingTriggerAlarm();
    getElapsedAlarmTriggertime(currentMillis);

    while (elapsedTime < pinEntryTime && !isCorrectCode && pinEntryCounter < 3) {  //You have 10s until the real alarm goes off to enter right pin
      getElapsedAlarmTriggertime(currentMillis);
      NewTone(SPEAKER_PIN, 500);

      if (pinEntryCounter < 3) {      //3 tries to enter correct PIN
        if (inputPin.length() < 4) {  //Can input 4 chars
          enterPin();
        } else {
          isCorrectCode = validatePin(inputPin.toInt(), pinEntryCounter);  //Check if input pin == correct pin
          pinEntryCounter++;
        }
      }
    }

    while (elapsedTime >= pinEntryTime || !isCorrectCode && pinEntryCounter >= 3) {
      Serial.println("ALAAAARM");

      if (tftCheckIfPrinted) {
        printAlarmMessage();
        delay(100);
        tftCheckIfPrinted = false;
      }

      playAlarm();
      //must enter master pin to shut off
    }
  } else {
    noNewTone(SPEAKER_PIN);
    tftCheckIfPrinted = true;
  }
}


void getImageDimensions() {
  //Load image and get its dimensions
  Serial.print(F("Loading image size: "));
  status = reader.bmpDimensions("/Arnold.bmp", &width, &height);  //Load image, set width, height to images.
  reader.printStatus(status);

  if (status == IMAGE_SUCCESS) {  //Find the center Y pixel to place image in center later.
    int32_t tftHeight = 240;
    int32_t centerImageY = (tftHeight / 2) - (height / 2);
    printBMP(centerImageY);
  } else {
    Serial.println("Failed to load image dimensions");
  }
}

void printBMP(int32_t centerY) {
  //Now load the BMP
  Serial.print(F("Loading Arnold.bmp to screen "));
  status = reader.drawBMP("/Arnold.bmp", tft, 0, centerY);  //Load image to position 0,0 (top left);

  if (status == IMAGE_SUCCESS) {
    reader.printStatus(status);
  } else {
    Serial.println("Failed to print image to TFT");
  }
}



void enterPin() {
  int keyIn = 0;
  int range = 2;

  keyIn = analogRead(A3);

  //Would probably be better with millis(). But using delay for now as it works. Although not optimal as holding the button or a long press adds more than one key-press
  for (int i = 0; i < 16; i++) {
    if (keyIn >= keyValues[i] - range && keyIn <= keyValues[i] + range) {
      inputPin += keys[i];
      delay(250);
      Serial.println(inputPin);
    }
  }
}

bool validatePin(int enteredPin, int pinEntryCounter) {
  Serial.print("Entered pin: ");
  Serial.println(enteredPin);
  Serial.println("ActualPin: ");
  Serial.print(secretPin);
  Serial.println();
  Serial.print("counter: ");
  Serial.println(pinEntryCounter);

  if (enteredPin == secretPin) {
    inputPin = "";
    return true;
  } else {
    Serial.print("Wrong PIN");
    inputPin = "";
    return false;
  }
}

void getElapsedAlarmTriggertime(unsigned long currentMillis) {
  timeWhenAlarmTriggered = millis();
  elapsedTime = timeWhenAlarmTriggered - currentMillis;
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
void printDateInterval(unsigned long currentMillis) {
  unsigned long printTime = currentMillis - previousTimeForDateAndTime;

  if (currentMillis - previousTimeForDateAndTime >= printDateAndTimeInterval) {
    RtcDateTime date = Rtc.GetDateTime();

    if (!wasError("no errors")) {
      previousTimeForDateAndTime = currentMillis;
      printDateTime(date);
    }
  }
}

void printDateTime(const RtcDateTime& date) {  //Example code from DS3231_Simple (Rtc by Makuna)
  resetTftScreen(ST77XX_BLACK);

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
  Serial.print(dateString);  //Remove later
  Serial.println();          //DEBUG
  tft.print(dateString);
}

void initAlarmDatesArray() {  //Sets every first letter to char 9 in every index. Since the date format mm/dd/, it cant start with a 9
  for (int i = 0; i < MAX_NUM_OF_DATES; i++) {
    for (int j = 0; j < 1; j++) {
      alarmDates[i][j] = NULL;
    }
  }
}

void findArrayIndexForSavingTriggerAlarm() {
  bool foundNull = true;

  while (foundNull && counter < 10) {

    if (alarmDates[counter][counter - counter] == NULL) {
      Serial.println("Saved date: ");
      saveTriggeredAlarmDate(counter);
      counter = 0;
      foundNull = false;
    }
    Serial.println("No more nulls");
    counter++;
  }
}

void saveTriggeredAlarmDate(int i) {
  for (int j = 0; j < 19; j++) {
    alarmDates[i][j] = dateString[j];
  }
}

void printAlarmDates() {
}
