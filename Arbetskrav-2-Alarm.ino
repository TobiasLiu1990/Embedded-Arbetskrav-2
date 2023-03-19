//Arbetskrav 2 - Alarmsystem

/*
  Components used:
    Adafruit 240x135 1.14" IPS LCD-Screen
    RTC ZS-042 (DS3231)
    Membrane Switch Module 4x4 (Keypad)
    Ultrasonic Sensor (HC-SR04)
    Buzzer
    330 ohm resistor
*/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SdFat.h>                 // SD card & FAT filesystem library
#include <Adafruit_SPIFlash.h>     // SPI / QSPI flash library
#include <Adafruit_ImageReader.h>  // Image-reading functions
#include <SPI.h>
#include <NewPing.h>
#include <NewTone.h>
#include <Wire.h>  // must be included here so that Arduino library object file references work
#include <RtcDS3231.h>

//IPS LCD-screen with Arduino Uno. Some code taken from Adafruit ImageReader Library - BreakoutST7789-240x135
#define TFT_CS 10
#define TFT_DC 8
#define TFT_RST 9
#define SD_CS 4
#define USE_SD_CARD

#if defined(USE_SD_CARD)
SdFat32 SD;
Adafruit_ImageReader reader(SD);
#endif

//Ultrasonic sensor
#define TRIGGER_PIN A1
#define ECHO_PIN A2
#define ALARM_DISTANCE 150
#define IS_BLOCKED_DISTANCE 1

//Buzzer
#define SPEAKER_PIN A0
//const short alarmMelody[] = { 262, 196 };  // 330 resistor - NOTE_C4 = 262, COTE_G3 = 196 From pitches.h
const short noteDuration = 250;

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
ImageReturnCode status;  //Status from image-reading functions
NewPing sonar(TRIGGER_PIN, ECHO_PIN, ALARM_DISTANCE);
RtcDS3231<TwoWire> Rtc(Wire);

//Keypad
const char keys[16] = {
  '1', '2', '3', 'A',
  '4', '5', '6', 'B',
  '7', '8', '9', 'C',
  '*', '0', '#', 'D'
};

const short keyValues[16] = {
  924, 907, 889, 872,
  847, 833, 818, 803,
  782, 770, 757, 744,
  726, 716, 704, 694
};

//Array for saving dates when alarm trigger
const short MAX_NUM_OF_DATES = 5;
const short DATE_LENGTH = 19;
char alarmDates[MAX_NUM_OF_DATES][DATE_LENGTH];
short alarmCounter = 0;

//Countdown timer for how long you have to enter correct pin
unsigned long timeWhenAlarmTriggered;
unsigned long elapsedTime;
const unsigned long pinEntryTime = 10000;

//How often big alarm changes between two tones.
unsigned long previousTimeForAlarm = 0;
const unsigned long playAlarmInterval = 250;

//Timer for how often date should be printed to TFT-screen
unsigned long previousTimeForDateAndTime = 0;
const unsigned long printDateAndTimeInterval = 5000;
char dateString[20];

int secretPinCode = 5555;
int masterPinCode = 1111;
String inputPinCode = "";


void setup() {
  Serial.begin(115200);

  //Check SD file system
  Serial.print(F("Initializing filesystem..."));
#if defined(USE_SD_CARD)
  if (!SD.begin(SD_CS, SD_SCK_MHZ(10))) {
    Serial.println(F("SD begin() failed"));
    for (;;)
      ;  //Fatal error, do not continue
  }
#else {
  if (!flash.begin()) {
    Serial.println(F("Flash begin() failed"));
    for (;;)
      ;
  }
  if (!filesys.begin(&flash)) {
    Serial.println(F("filesys begin() failed"));
    for (;;)
      ;
  }
}
#endif
  //SD load successful
  Serial.println(F("OK!"));

  Rtc.Begin();
  tft.init(135, 240);
  tft.setRotation(0);
  resetTftScreen();
  initAlarmDatesArray();

#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
#endif

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
}

void loop() {
  short currentDistance = sonar.ping_cm();
  unsigned long currentMillis = millis();
  bool isCorrectCode = false;
  int pinEntryalarmCounter = 0;
  //Serial.println(currentDistance);  // DEBUG

  printDateInterval(currentMillis);
  keypadButtonLetters();

  //If ultrasonic sensor returns 0 (blocked or signal lost)
  if (currentDistance <= IS_BLOCKED_DISTANCE) {
    NewTone(SPEAKER_PIN, 1200);
  }
  //Alarm is triggered
  else if (currentDistance < 30) {    //Distance here can be changed depending on situation. Might need to change ALARM_DISTANCE in that case as well.
    printEnterPin();

    // Save date when alarm triggered in array
    findArrayIndexForSavingTriggerAlarm();
    getElapsedAlarmTriggertime(currentMillis);

    while (elapsedTime < pinEntryTime && !isCorrectCode && pinEntryalarmCounter < 3) {  //10s to enter correct PIN
      getElapsedAlarmTriggertime(currentMillis);
      NewTone(SPEAKER_PIN, 500);

      if (pinEntryalarmCounter < 3) {     //3 tries to enter correct PIN
        if (inputPinCode.length() < 4) {  //Can input 4 chars
          enterPin();
        } else {
          isCorrectCode = validatePin(inputPinCode.toInt(), secretPinCode);
          pinEntryalarmCounter++;
        }
      }
    }

    if (elapsedTime >= pinEntryTime) {
      String inputPinCode = "";  //Reset input if you input something but ran out of time.
      resetTftScreen();

      //Get center position in Y-axis and print image from SD
      printAlarmWarning();
      int32_t centerImageY = getImageDimensions();
      printBMP(centerImageY);
    }

    while (!isCorrectCode) {
      currentMillis = millis();
      playAlarm(currentMillis);  //Sadly this part does not work properly. I tried to get it to jump between 2 tones with 250ms delay inbetween. It did work with millis in a for-loop, but delay wasnt good in this case.

      //must enter master pin to shut off
      if (inputPinCode.length() < 4) {
        enterPin();
      } else {
        isCorrectCode = validatePin(inputPinCode.toInt(), masterPinCode);
      }
    }
    noNewTone(SPEAKER_PIN);
    resetTftScreen();
  }
}

void keypadButtonLetters() {
  int keyIn = 0;
  int range = 2;
  keyIn = analogRead(A3);

  if (keyIn >= 872 - range && keyIn <= 872 + range) {  // Value 872 = A
    printAlarmDatesToTFTScreen();
  } else if (keyIn >= 803 - range && keyIn <= 803 + range) {  //Value 803 = B
    resetTftScreen();
  }
}

void enterPin() {
  int keyIn = 0;
  int range = 2;
  keyIn = analogRead(A3);

  //Would be better with millis(). But using delay for now as it works. Although not optimal as holding the button or a long press adds more than one key-press
  for (int i = 0; i < 16; i++) {
    if (keyIn >= keyValues[i] - range && keyIn <= keyValues[i] + range) {
      inputPinCode += keys[i];
      delay(320);
      Serial.println(inputPinCode);
    }
  }
}

bool validatePin(int enteredPin, int secret) {
  if (enteredPin == secret) {
    inputPinCode = "";
    return true;
  } else {
    Serial.print("Wrong PIN");
    inputPinCode = "";
    return false;
  }
}

void getElapsedAlarmTriggertime(unsigned long currentMillis) {
  timeWhenAlarmTriggered = millis();
  elapsedTime = timeWhenAlarmTriggered - currentMillis;
}

int32_t getImageDimensions() {
  int32_t width = 0;
  int32_t height = 0;

  Serial.print(F("Loading image size: "));
  status = reader.bmpDimensions("/Arnold.bmp", &width, &height);  //Set width, height to images.
  reader.printStatus(status);

  //Find the center Y pixel to place image in center later.
  if (status == IMAGE_SUCCESS) {
    int32_t tftHeight = 240;
    return (tftHeight / 2) - (height / 2);
  } else {
    Serial.println(F("Failed to load image dimensions"));
  }
}

void printBMP(int32_t centerY) {
  Serial.print(F("Loading Arnold.bmp to screen "));
  status = reader.drawBMP("/Arnold.bmp", tft, 0, centerY);  //Load image to position 0,0 (top left);

  if (status == IMAGE_SUCCESS) {
    reader.printStatus(status);
  } else {
    Serial.println(F("Failed to print image to TFT"));
  }
}

void resetTftScreen() {
  tft.setTextColor(ST77XX_WHITE);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(0);
  tft.setCursor(0, 0);
}

void printEnterPin() {
  tft.setTextSize(2);
  tft.setCursor(15, 15);
  tft.print("Enter PIN");
}

void printAlarmWarning() {
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.setCursor(0, 20);
  tft.print("WARNING");
}

//Used delay(250) first in a for-loop with the melody[]. But since it locks down everything by 250ms, it wasnt working well with detecting keypad entries.
//Tried to get millis to work here but couldnt get it to work properly.
void playAlarm(unsigned long currentMillis) {
  if (currentMillis - previousTimeForAlarm >= playAlarmInterval) {
    previousTimeForAlarm = currentMillis;
    NewTone(SPEAKER_PIN, 196);
  } else {
    NewTone(SPEAKER_PIN, 262);
  }
}

//Print every x second
void printDateInterval(unsigned long currentMillis) {
  if (currentMillis - previousTimeForDateAndTime >= printDateAndTimeInterval) {
    RtcDateTime date = Rtc.GetDateTime();
    previousTimeForDateAndTime = currentMillis;
    printDateTime(date);
  }
}

#define countof(arr) (sizeof(arr) / sizeof(arr[0]))  //Macro to get number of elements in array

void printDateTime(const RtcDateTime& date) {  //Example code from DS3231_Simple (Rtc by Makuna)
  resetTftScreen();
  snprintf_P(dateString,                             //buffer
             countof(dateString),                    //max number of bytes (char), written to buffer
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),  //PSTR reads from flash mem. n (...) is for formating
             date.Month(),                           //rest of params to format
             date.Day(),
             date.Year(),
             date.Hour(),
             date.Minute(),
             date.Second());
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

  while (foundNull && alarmCounter < 10) {

    if (alarmDates[alarmCounter][alarmCounter - alarmCounter] == NULL) {
      //Serial.println("Saved date: ");
      saveTriggeredAlarmDate(alarmCounter);
      alarmCounter = 0;
      foundNull = false;
    }
    //Serial.println("No more nulls");
    alarmCounter++;
  }
}

void saveTriggeredAlarmDate(int i) {
  for (int j = 0; j < 19; j++) {
    alarmDates[i][j] = dateString[j];
  }
}

//Bug where it prints out weird things in the end, not sure why.
void printAlarmDatesToTFTScreen() {
  int cursorCounter = 0;
  resetTftScreen();

  for (int i = 0; i < 10; i++) {
    tft.setCursor(0, cursorCounter += 10);
    for (int j = 0; j < 19; j++) {
      if (alarmDates[i][j - j] != NULL) {
        tft.print(alarmDates[i][j]);
      }
    }
  }
}