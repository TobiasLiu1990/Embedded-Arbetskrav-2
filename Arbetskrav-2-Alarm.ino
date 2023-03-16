//Arbetskrav 2 - Alarmsystem

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SdFat.h>                 // SD card & FAT filesystem library
#include <Adafruit_SPIFlash.h>     // SPI / QSPI flash library
#include <Adafruit_ImageReader.h>  // Image-reading functions
#include <SPI.h>
#include <NewPing.h>
#include <NewTone.h>
//For normal hardware wire
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
const short alarmMelody[] = { 262, 196 };  // 2k resistor atm   NOTE_C4 = 262, COTE_G3 = 196 From pitches.h
const short noteDuration = 250;

//Keypad
const char keys[16] PROGMEM = {
  '1', '2', '3', 'A',
  '4', '5', '6', 'B',
  '7', '8', '9', 'C',
  '*', '0', '#', 'D'
};

const short keyValues[16] PROGMEM = {
  924, 907, 889, 872,
  847, 833, 818, 803,
  782, 770, 757, 744,
  726, 716, 704, 694
};


//Objects
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
ImageReturnCode status;  //Status from image-reading functions

NewPing sonar(TRIGGER_PIN, ECHO_PIN, ALARM_DISTANCE);
RtcDS3231<TwoWire> Rtc(Wire);


const short MAX_NUM_OF_DATES = 5;
const short DATE_LENGTH = 19;
char alarmDates[MAX_NUM_OF_DATES][DATE_LENGTH];
char dateString[20];
short counter = 0;

unsigned long timeWhenAlarmTriggered;
unsigned long elapsedTime;
const unsigned long pinEntryTime = 10000;  //should be 10 seconds

unsigned long previousTimeForDateAndTime = 0;
const unsigned long printDateAndTimeInterval = 5000;  //should be 10 seconds

bool runErrorHandlingOnce = true;

short secretPin = 5555;
String inputPin = "";



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
  resetTftScreen(ST77XX_BLACK);
  initAlarmDatesArray();

#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
#endif

  printDateTime(RtcDateTime(__DATE__, __TIME__));
  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);
}

void loop() {
  short currentDistance = sonar.ping_cm();
  unsigned long currentMillis = millis();
  bool isCorrectCode = false;
  int pinEntryCounter = 0;
  //Serial.println(currentDistance);  // DEBUG

  printDateInterval(currentMillis);

  //If ultrasonic sensor returns 0 (blocked or signal lost)
  if (currentDistance <= IS_BLOCKED_DISTANCE) {
    NewTone(SPEAKER_PIN, 1200);
  }
  //Alarm is triggered
  else if (currentDistance < 30) {
    printEnterPinPeriod();

    // Save date when alarm triggered in array
    findArrayIndexForSavingTriggerAlarm();
    getElapsedAlarmTriggertime(currentMillis);

    while (elapsedTime < pinEntryTime && !isCorrectCode && pinEntryCounter < 3) {  //10s to enter correct PIN
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

    int32_t centerImageY = getImageDimensions();
    printBMP(centerImageY);

    while (elapsedTime >= pinEntryTime || !isCorrectCode && pinEntryCounter >= 3) {
      playAlarm();
      //must enter master pin to shut off
    }
  } else {
    noNewTone(SPEAKER_PIN);
  }
}

int32_t getImageDimensions() {
  //Load image and get its dimensions
  int32_t width = 0;
  int32_t height = 0;

  Serial.print(F("Loading image size: "));
  status = reader.bmpDimensions("/Arnold.bmp", &width, &height);  //Load image, set width, height to images.
  reader.printStatus(status);

  if (status == IMAGE_SUCCESS) {  //Find the center Y pixel to place image in center later.
    int32_t tftHeight = 240;
    return (tftHeight / 2) - (height / 2);
  } else {
    Serial.println(F("Failed to load image dimensions"));
  }
}

void printBMP(int32_t centerY) {
  //Now load the BMP
  Serial.print(F("Loading Arnold.bmp to screen "));
  status = reader.drawBMP("/Arnold.bmp", tft, 0, centerY);  //Load image to position 0,0 (top left);

  if (status == IMAGE_SUCCESS) {
    reader.printStatus(status);
  } else {
    Serial.println(F("Failed to print image to TFT"));
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
  tft.print("Enter PIN");
}

void printAlarmMessage() {
  tft.setTextColor(ST77XX_RED);
  //tft.print("WARNING!!!!");
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

    previousTimeForDateAndTime = currentMillis;
    printDateTime(date);
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
      //Serial.println("Saved date: ");
      saveTriggeredAlarmDate(counter);
      counter = 0;
      foundNull = false;
    }
    //Serial.println("No more nulls");
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