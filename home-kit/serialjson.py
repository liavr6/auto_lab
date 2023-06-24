#include <ArduinoJson.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" 
#define SHUTDOWN_PIN 2
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor;

const int SENSOR_1_PIN = A0;
const int UPDATE_INTERVAL_MS = 10;


String inputString = "";         

bool stringComplete = false;     


void setup() {

  Wire.begin();
  Serial.begin(115200);

  while (!Serial) continue; // Wait for serial connection

  inputString.reserve(200);

  if (distanceSensor.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");

}


void loop() {

  static unsigned long last_update_time = 0;

  distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
  while (!distanceSensor.checkForDataReady())
  {
    delay(1);
  }
  int dis = distanceSensor.getDistance(); 
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  unsigned long current_time = millis();

  if (current_time - last_update_time >= UPDATE_INTERVAL_MS) {

    last_update_time = current_time;


    int sensor1_value = dis;


    StaticJsonDocument<64> doc;

    doc["sensor1"] = sensor1_value;

    serializeJson(doc, Serial);

    Serial.println();

  }


  processIncomingString();


}


void processIncomingString() {

  if (stringComplete) {

    StaticJsonDocument<64> incoming_doc;

    DeserializationError error = deserializeJson(incoming_doc, inputString);

    if (!error) {

      if (incoming_doc.containsKey("command")) {

        String command = incoming_doc["command"].as<String>();

        if (command == "reset") {

          Serial.println("{'reset': 0}");

        }

      }

    }

    inputString = "";

    stringComplete = false;

  }

}


void serialEvent() {

  while (Serial.available()) {

    char inChar = (char)
Serial.read
();

    inputString += inChar;

    if (inChar == '\n') {

      stringComplete = true;

    }

  }

} 