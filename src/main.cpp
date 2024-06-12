#include <Arduino.h>
#include <HX711_ADC.h> // weight sensor
#include "Stepper.h"   // stepper motor
#include <WiFi.h>
#include <WebServer.h>

// Loadcell config/constructor
const int HX711_dout = 32; // mcu > HX711 dout pin
const int HX711_sck = 33;  // mcu > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
unsigned long t = 0;
float calibrationValue = 2921.21; // CALIBRATION
boolean dispenseMotor1ACounter = false;
// Relay config/constructor
int motor1Relay = 25; // Relay for motor1

// Motor Config/constructor
const int motorSpeed = 25;
const int stepsPerRevolution = 2048;
const int smallStep = 55;                                     // Small step size for motor control
Stepper motor1 = Stepper(stepsPerRevolution, 12, 14, 27, 26); // motor1

// AmountToDispense
float amountToDispense = 0; // Initialized to 0, will be updated by HTTP request
boolean dataReceived = false; // Flag to check if data is received

// Definitions for WiFi
const char* WIFI_SSID = "Tabaranza_WIFI";
const char* WIFI_PASSWORD = "Gabby1713";

WebServer server(80);

void handleReceive() {
  if (server.hasArg("plain")) {
    String body = server.arg("plain");

    // Print received data to Serial
    Serial.print("Received data: ");
    Serial.println(body);

    // Parse received data to update amountToDispense
    int index = body.indexOf("amountToDispense");
    if (index != -1) {
      int startIndex = body.indexOf(":", index) + 1;
      int endIndex = body.indexOf(",", startIndex);
      if (endIndex == -1) endIndex = body.length();
      String amountStr = body.substring(startIndex, endIndex);
      amountToDispense = amountStr.toFloat();
      Serial.print("Updated amountToDispense: ");
      Serial.println(amountToDispense);
      dataReceived = true; // Set the flag to true
    }

    // Respond with success message
    server.send(200, "application/json", "{\"status\":\"success\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"fail\",\"message\":\"Invalid request\"}");
  }
}

void dispenseMotor1A() {
  for (int i = 0; i < stepsPerRevolution; i += smallStep) {
    motor1.step(smallStep); // Smaller steps for better control
    LoadCell.update();      // Update load cell value
    if (LoadCell.getData() >= amountToDispense)
      break; // Check if threshold is reached
  }
}

void dispenseMotor1B() {
  for (int i = 0; i < stepsPerRevolution; i += smallStep) {
    motor1.step(-smallStep); // Smaller steps for better control
    LoadCell.update();       // Update load cell value
    if (LoadCell.getData() >= amountToDispense)
      break; // Check if threshold is reached
  }
}

void stopDispenseMotor1() {
  digitalWrite(motor1Relay, LOW);
}

void loadCellConfiguration() {
  unsigned long stabilizingtime = 2000; // tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  } else {
    LoadCell.setCalFactor(calibrationValue); // set calibration factor (float)
    Serial.println("Startup is complete");
  }

  while (!LoadCell.update())
    ;
  Serial.print("Calibration value: ");
  Serial.println(LoadCell.getCalFactor());
  Serial.print("HX711 measured conversion time ms: ");
  Serial.println(LoadCell.getConversionTime());
  Serial.print("HX711 measured sampling rate HZ: ");
  Serial.println(LoadCell.getSPS());
  Serial.print("HX711 measured settling time ms: ");
  Serial.println(LoadCell.getSettlingTime());
  Serial.println("Note that the settling time may increase significantly if you use delay() in your sketch!");
  if (LoadCell.getSPS() < 7) {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  } else if (LoadCell.getSPS() > 100) {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }
}

void setup() {
  Serial.begin(115200);

  // Relay Config
  pinMode(motor1Relay, OUTPUT);
  digitalWrite(motor1Relay, LOW);

  // Motor Config
  motor1.setSpeed(motorSpeed);

  Serial.println();
  Serial.println("Starting...");
  LoadCell.begin();
  loadCellConfiguration();

  // Initialize WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize server routes
  server.on("/receive", HTTP_POST, handleReceive);
  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient();
  static boolean newDataReady = 0;
  const int serialPrintInterval = 100; // increase value to slow down serial print activity
  static unsigned long lastPrintTime = 0; // Track the last print time

  // Check if new data is received
  if (LoadCell.update())
    newDataReady = true;

  // Check if data has been received from client
  if (dataReceived) {
    // get smoothed value from the dataset:
    if (newDataReady) {
      if (millis() > t + serialPrintInterval) {
        float petTrayAmount = LoadCell.getData();
        if (petTrayAmount >= amountToDispense) {
          stopDispenseMotor1();
          Serial.println(" -- Reached the Threshold, STOP");
          Serial.print("Load_cell output val: ");
          Serial.println(petTrayAmount);
          dataReceived = false;
        } else if (petTrayAmount < amountToDispense && !dispenseMotor1ACounter) {
          digitalWrite(motor1Relay, HIGH);
          Serial.println(" Rotating Clockwise --");
          dispenseMotor1A();
          Serial.print("Load_cell output val: ");
          Serial.println(petTrayAmount);
          newDataReady = 0;
          t = millis();
          dispenseMotor1ACounter = true;
        } else if (petTrayAmount < amountToDispense && dispenseMotor1ACounter) {
          Serial.println("Counter Clockwise --");
          dispenseMotor1B();
          Serial.print("Load_cell output val: ");
          Serial.println(petTrayAmount);
          newDataReady = 0;
          t = millis();
          dispenseMotor1ACounter = false;
        } else {
          Serial.println("Stepper Motor Not Moving Because the Threshold is Zero");
        }
      }
    }
  } else {
    // Print "waiting for data" every 5 seconds
    if (millis() - lastPrintTime >= 5000) {
      Serial.println("Waiting for data...");
      lastPrintTime = millis();
    }
  }
}
