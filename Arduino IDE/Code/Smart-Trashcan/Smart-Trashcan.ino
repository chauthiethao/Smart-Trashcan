#define DEFAULT_MQTT_HOST "mqtt1.eoh.io"

// You should get Auth Token in the ERa App or ERa Dashboard
#define ERA_AUTH_TOKEN "1c078e2e-92cc-45f7-9111-ccc3a46b8f85"

#define triggermode    2
#define triggeropen    4
#define servoPin      13
#define relayPin      12
#define echoR         14
#define trigR         27
#define echoL         26
#define trigL         25
#define pirPin        34
#define HX711_DT      A5
#define HX711_SCK     A4

#include <ESP32Servo.h>
#include <HX711_ADC.h>
#include <DFRobotDFPlayerMini.h>

#include <Arduino.h>
#include <ERa.hpp>
#include <ERa/ERaTimer.hpp>

const char ssid[] = "American Study HD";
const char pass[] = "66668888";

#if (defined(ARDUINO_AVR_UNO) || defined(ESP8266))   // Using a soft serial port
#include <SoftwareSerial.h>
SoftwareSerial softSerial(/*rx =*/4, /*tx =*/5);
#define FPSerial softSerial
#else
#define FPSerial Serial1
#endif

#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

Servo servo;
HX711_ADC LoadCell (HX711_DT, HX711_SCK);
DFRobotDFPlayerMini myDFPlayer;

ERaTimer timer;

const int calVal_eepromAdress = 0;
uint64_t tim = 0;
float calibrationValue = 322843.88;
float height    = 0.00;
float load      = 0.00;
float maxWeight = 4.00;
float minHeight = 3.00;

int mode     = 1;
int openthis = 0;

/* This function print uptime every second */
void timerEvent() {
    ERa.virtualWrite(V2, (ERaMillis() / 1000L / 3600L));
    ERa.virtualWrite(V3, ((ERaMillis() / 1000L / 60L) - int(ERaMillis() / 1000L / 3600L) * 60L));
    ERa.virtualWrite(V4, (ERaMillis() / 1000L) % 60L);
    if (openthis != 1) {
      ERa.virtualWrite(V0, load);
      if ((25 - height) >= 0) ERa.virtualWrite(V1, (25 - height));
      else ERa.virtualWrite(V1, 0);
    }
    ERA_LOG("Timer", "Uptime: %d", ERaMillis() / 1000L);
}

void setup() {
    /* Setup debug console */
    Serial.begin(115200);

    ERa.begin(ssid, pass);
    
    /* Setup timer called function every second */
    timer.setInterval(1000L, timerEvent);

  #if (defined ESP32)
  FPSerial.begin(9600, SERIAL_8N1, /*rx =*/16, /*tx =*/17);
  #else
  FPSerial.begin(9600);
  #endif

  #if defined(ESP8266) || defined(ESP32)
  #endif

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
  
  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  //Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while(true){
      delay(0);
    }
  }
  Serial.println(F("DFPlayer Mini online."));

  servo.attach(servoPin);
  pinMode(trigL    , OUTPUT);
  pinMode(echoL    , INPUT );
  pinMode(trigR    , OUTPUT);
  pinMode(echoR    , INPUT );
  pinMode(pirPin   , INPUT );
  pinMode(relayPin , OUTPUT);

  digitalWrite(relayPin , LOW );

  servo.write(80);
  myDFPlayer.volume(30);
}

void loop() {
  height = (HCSR04(trigR, echoR) + HCSR04(trigL, echoL)) / 2;
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > tim + serialPrintInterval) {
      load = LoadCell.getData();
      newDataReady = 0;
      tim = millis();
    }
  }

  if (digitalRead(triggermode) == 1) {
    mode = 1;
  }
  else if (digitalRead(triggermode) == 0) {
    mode = 0;
  }

  if (digitalRead(triggeropen) == 1) {
    openthis = 1;
  }
  else if (digitalRead(triggeropen) == 0) {
    openthis = 0;
  }

  if (mode == 1) {
    if ((height > minHeight) && (load < maxWeight)) {
      if (digitalRead(pirPin) == 1) {
        OpenTrashBin();
        CloseTrashBin();
      }
    }
    else {
      if (digitalRead(pirPin) == 1) {
        myDFPlayer.play(2);
        delay(5000);
      }
    }
  }

  else if (mode == 0) {
    if (openthis == 1) {
      servo.write(0);
      delay(100);
    }
    else if (openthis == 0) {
      servo.write(80);
      delay(100);
    }
  }

  if (myDFPlayer.available()) {
    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  }
  
  // Serial.print(mode);
  // Serial.print(" ");
  // Serial.println(openthis);

    ERa.run();
    timer.run();
}

float HCSR04(int trig, int echo) {
  unsigned long duration;
  float dis;
  digitalWrite(trig, LOW);
  delay(2);
  digitalWrite(trig, HIGH);
  delay(5);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  dis = duration/2/29.412;
  return dis;
}

void OpenTrashBin() {
  servo.write(0);
  delay(1000);
  myDFPlayer.play(1);
  delay(5000);
}

void CloseTrashBin() {
  servo.write(80);
  delay(1000);
  myDFPlayer.play(3);
  digitalWrite(relayPin, HIGH);
  delay(3000);
  digitalWrite(relayPin, LOW);
}

void printDetail(uint8_t type, int value){
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}