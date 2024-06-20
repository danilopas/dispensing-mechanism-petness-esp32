#include <Firebase_ESP_Client.h>
#include <Arduino.h>
#include <HX711.h>
#include "Stepper.h" // stepper motor
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include "soc/rtc.h"
#include <deque>

using namespace std;

// Definitions for HX711_petTray
const int petTray_dout = 16;
const int petTray_sck = 4;
const int platformLoadCell_dout = 32;
const int platformLoadCell_sck = 33;
// Calibration values
float calibrationValue_petTray = 2291.28; // petTray Calibration
float calibrationValue_platform = 21.68;  // platform Calibration
// LoadCell objects+
HX711 LoadCell_petTray;
HX711 LoadCell_platform;

boolean petTray_newDataReady = false;

boolean dispenseMotor1ACounter = false;

unsigned long t = 0;

// Relay config/constructor
int motor1Relay = 25; // Relay for motor1
// int motor2Relay = 2; // Relay for motor2

// Motor Config/constructor
const int motorSpeed = 20;
const int stepsPerRevolution = 2048;
const int smallStep = 60;                                     // Small step size for motor control
const float tolerance = 0.5;                                  // Allowable error margin in grams
Stepper motor1 = Stepper(stepsPerRevolution, 12, 14, 27, 26); // motor1

// AmountToDispense
float amountToDispense = 0;           // Initialized to 0, will be updated by HTTP request
String userName = "";                 // Initialized to 0, will be updated by HTTP request
String selectedFood = "";             // Initialized to 0, will be updated by HTTP request
String cageID = "";                   // Initialized to 0, will be updated by HTTP request
String feedingModeType = "";          // Initialized to 0, will be updated by HTTP request
String scheduleDate = "";             // Initialized to 0, will be updated by HTTP request
String scheduledTime = "";            // Initialized to 0, will be updated by HTTP request
const char *clientCageID = "cage_01"; // Predefined cageID
boolean dataReceived = false;         // Flag to check if data is received
float initialWeight = 0.0;            // Initialized to 0, will be updated by load cell

// Fluctuation handling
deque<float> weightReadings;
const int smoothingWindow = 10;            // Number of readings to average
const int consistentReadingsThreshold = 5; // Number of consistent readings before stopping

// Definitions for WiFi
const char *WIFI_SSID = "virusX-2.4g";
const char *WIFI_PASSWORD = "simacmacmagbabayad";

// Definitions for Firebase Database
const char *FIREBASE_HOST = "https://petness-92c55-default-rtdb.asia-southeast1.firebasedatabase.app/";
const char *FIREBASE_AUTH = "bhvzGLuvbjReHlQjk77UwWGtCVdBBUBABE3X4PQ2";
// Definitions for Firestore Database
const char *FIREBASE_FIRESTORE_HOST = "petness-92c55";
// Define the API Key
const char *API_KEY = "AIzaSyDPcMRU9x421wP0cS1sRHwEvi57W8NoLiE";
// Define the user Email and password that already registerd or added in your project
const char *USER_EMAIL = "petnessadmin@gmail.com";
const char *USER_PASSWORD = "petness";

// Declare the FirebaseData objects at the global scope
FirebaseData fbdo1; // For Realtime Database Weight Stream
FirebaseData fbdo2; // For Firestore Record
FirebaseData fbdo3; // For Firestore Read Previous Amount Dispensed
FirebaseConfig config;
FirebaseAuth auth;
FirebaseJson content;

// Declare the paths at the global scope
String pathGetPetWeight = "/trigger/getPetWeight/status";

WebServer server(80);

// function prototypes
void handleReceive();
void dispenseMotor1A();
void dispenseMotor1B();
void stopDispenseMotor1();
// void loadCellConfiguration(HX711 &loadCell, float calibrationValue, const char *loadCellName);
void recordFeedingDataToFirestore(const String &mode, const String &userName, float amount, const String &scheduleDate, const String &scheduledTime, const String &cageID, float initialWeight);
String generateRandomString(int length);
// float calculateFoodConsumption();
void connectToWifi();
void configureLoadCells();
void initializeFirebase();
void initializeMotor();
float samplesForGettingWeight();
void setPetWeight();
void petWeightStream();
void printLoadCellWeights();
bool checkConsistentWeight(float targetWeight, float tolerance);

void setup()
{
  Serial.begin(115200);

  connectToWifi();
  configureLoadCells();
  initializeFirebase();
  initializeMotor();

  // randomSeed(analogRead(0));

  // Initialize server routes
  server.on("/receive", HTTP_POST, handleReceive);
  server.begin();
  Serial.println("Server started");
}

unsigned long lastPetWeightStreamTime = 0;

void loop()
{
  // printLoadCellWeights();

  // delay(1000);

  // Handle Client method provided by WebServer to run the server
  server.handleClient();

  unsigned long currentMillis = millis();
  static boolean newDataReady = 0;
  const int serialPrintInterval = 100;
  static unsigned long lastPrintTime = 0;

  if (LoadCell_petTray.is_ready())
  {
    newDataReady = true;
  }

  if (dataReceived)
  {
    if (newDataReady)
    {
      static int clockwiseCount = 0;

      if (millis() > t + serialPrintInterval)
      {
        float petTrayAmount = LoadCell_petTray.get_units();
        weightReadings.push_back(petTrayAmount);

        if (weightReadings.size() > smoothingWindow)
        {
          weightReadings.pop_front(); // Remove the oldest reading
        }

        float smoothedWeight = accumulate(weightReadings.begin(), weightReadings.end(), 0.0) / weightReadings.size();

        if (checkConsistentWeight(amountToDispense, tolerance))
        {
          stopDispenseMotor1();
          Serial.println(" -- Reached the Threshold, STOP");
          Serial.print("Smoothed load cell output val: ");
          Serial.println(smoothedWeight);
          Serial.println("Dispensing Finished");
          Serial.println("Recording Feeding Data to Firestore");
          recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialWeight);
          Serial.println("Reseting the Data Received Flag");
          dataReceived = false;
        }
        else if (smoothedWeight < amountToDispense)
        {
          if (clockwiseCount < 2)
          {
            digitalWrite(motor1Relay, HIGH);
            Serial.println(" Rotating Clockwise --");
            dispenseMotor1A();
            Serial.print("Smoothed load cell output val: ");
            Serial.println(smoothedWeight);
            clockwiseCount++;
          }
          else
          {
            Serial.println("Counter Clockwise --");
            dispenseMotor1B();
            Serial.print("Smoothed load cell output val: ");
            Serial.println(smoothedWeight);
            clockwiseCount = 0;
          }
          newDataReady = 0;
          t = millis();
        }
        else
        {
          static unsigned long lastThresholdPrintTime = 0;
          if (millis() - lastThresholdPrintTime >= 2000)
          { // Print every 2 seconds
            Serial.println("Stepper Motor Not Moving Because the Threshold is Zero");
            lastThresholdPrintTime = millis();
          }
        }
      }
    }
  }
  else
  {
    if (millis() - lastPrintTime >= 5000)
    {
      Serial.println("Waiting for data...");
      lastPrintTime = millis();
    }
  }

  if (currentMillis - lastPetWeightStreamTime >= 250)
  {
    petWeightStream();
    lastPetWeightStreamTime = currentMillis;
  }
}

bool checkConsistentWeight(float targetWeight, float tolerance)
{
  int consistentCount = 0;
  for (float reading : weightReadings)
  {
    if (abs(reading - targetWeight) <= tolerance)
    {
      consistentCount++;
    }
  }
  return consistentCount >= consistentReadingsThreshold;
}

void printLoadCellWeights()
{
  Serial.print("PET TRAY WEIGHT: ");
  float petTrayWeight = LoadCell_petTray.get_units(10);
  Serial.println(petTrayWeight);

  Serial.print("PLATFORM WEIGHT: ");
  float platformWeight = LoadCell_platform.get_units(10);
  Serial.println(platformWeight);
}

void initializeMotor()
{
  // Relay Config
  pinMode(motor1Relay, OUTPUT);
  digitalWrite(motor1Relay, LOW);

  // Motor Config
  motor1.setSpeed(motorSpeed);
}

void dispenseMotor1A()
{
  motor1.setSpeed(motorSpeed);
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor1.step(smallStep);
    delay(10);

    float petTrayAmount = LoadCell_petTray.get_units();
    weightReadings.push_back(petTrayAmount);
    if (weightReadings.size() > smoothingWindow)
    {
      weightReadings.pop_front();
    }

    float smoothedWeight = accumulate(weightReadings.begin(), weightReadings.end(), 0.0) / weightReadings.size();
    if (abs(smoothedWeight - amountToDispense) <= tolerance)
    {
      stopDispenseMotor1();
      Serial.print("Reached threshold during dispenseMotor1A: ");
      Serial.println(smoothedWeight);

      // Record feeding data to Firestore
      Serial.println("Recording Feeding Data to Firestore");
      recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialWeight);

      dataReceived = false;
      return;
    }
  }
}

void dispenseMotor1B()
{
  motor1.setSpeed(motorSpeed);
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor1.step(-smallStep);
    delay(10);

    float petTrayAmount = LoadCell_petTray.get_units();
    weightReadings.push_back(petTrayAmount);
    if (weightReadings.size() > smoothingWindow)
    {
      weightReadings.pop_front();
    }

    float smoothedWeight = accumulate(weightReadings.begin(), weightReadings.end(), 0.0) / weightReadings.size();
    if (abs(smoothedWeight - amountToDispense) <= tolerance)
    {
      stopDispenseMotor1();
      Serial.print("Reached threshold during dispenseMotor1B: ");
      Serial.println(smoothedWeight);

      // Record feeding data to Firestore
      Serial.println("Recording Feeding Data to Firestore");
      recordFeedingDataToFirestore(feedingModeType, userName, amountToDispense, scheduleDate, scheduledTime, cageID, initialWeight);

      dataReceived = false;
      return;
    }
  }
  dispenseMotor1ACounter = true;
}

void stopDispenseMotor1()
{
  digitalWrite(motor1Relay, LOW); // Turn off the relay to stop the motor
}

void initializeFirebase()
{
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  Firebase.begin(&config, &auth);

  if (Firebase.RTDB.beginStream(&fbdo1, pathGetPetWeight.c_str()))
  {
    Serial.println("Stream getPetWeight begin, success");
  }
  else
  {
    Serial.print("Stream begin failed, reason: ");
    Serial.println(fbdo1.errorReason());
  }
}

void connectToWifi()
{
  // Initialize WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void handleReceive()
{
  if (server.hasArg("plain"))
  {
    String body = server.arg("plain");

    // Print received data to Serial
    Serial.print("Received data: ");
    Serial.println(body);

    // Increase the size of the JsonDocument (adjust as needed)
    StaticJsonDocument<256> jsonDoc;

    // Parse received data to JSON
    DeserializationError error = deserializeJson(jsonDoc, body);

    // Check for errors
    if (error)
    {
      Serial.print("JSON deserialization failed: ");
      Serial.println(error.c_str());
      server.send(400, "application/json", "{\"status\":\"fail\",\"message\":\"Invalid JSON\"}");
      return;
    }

    // Extract fields from JSON
    const char *receivedCageID = jsonDoc["cageID"];
    const char *receivedUserName = jsonDoc["userName"];
    float receivedAmountToDispense = jsonDoc["amountToDispense"].as<float>(); // Extract amountToDispense as float
    const char *receivedSelectedFood = jsonDoc["selectedFood"];
    const char *receivedFeedingModeType = jsonDoc["feedingModeType"];
    const char *receivedScheduleDate = jsonDoc["scheduledDate"]; // Corrected the field name
    const char *receivedScheduledTime = jsonDoc["scheduledTime"];

    // Check if the received cageID matches the predefined cageID
    if (strcmp(receivedCageID, clientCageID) == 0)
    {
      // Update global variables with received data
      cageID = receivedCageID;
      amountToDispense = receivedAmountToDispense;
      userName = receivedUserName;
      selectedFood = receivedSelectedFood;
      feedingModeType = receivedFeedingModeType;
      scheduleDate = receivedScheduleDate; // Corrected the field name
      scheduledTime = receivedScheduledTime;

      Serial.println("Receiving Data...");
      Serial.println("Getting Initial Weight...");
      // Get initial weight before dispensing
      float initialWeightGramsToKG = samplesForGettingWeight() / 1000; // Convert to kilograms
      initialWeight = roundf(initialWeightGramsToKG * 100) / 100;      // Round to nearest 0.01

      dataReceived = true; // Set the flag to true

      // Prepare the response JSON
      StaticJsonDocument<256> responseDoc;
      responseDoc["status"] = "success";
      responseDoc["clientCageID"] = clientCageID;
      responseDoc["userName"] = userName;
      responseDoc["selectedFood"] = selectedFood;
      responseDoc["feedingModeType"] = feedingModeType;
      responseDoc["amountToDispense"] = amountToDispense;
      responseDoc["scheduleDate"] = scheduleDate; // Corrected the field name
      responseDoc["scheduledTime"] = scheduledTime;

      // Serialize response JSON to string
      String response;
      serializeJson(responseDoc, response);

      server.send(200, "application/json", response);
    }
    else
    {
      server.send(403, "application/json", "{\"status\":\"fail\",\"message\":\"Invalid Cage ID\"}");
    }
  }
  else
  {
    server.send(400, "application/json", "{\"status\":\"fail\",\"message\":\"Invalid request\"}");
  }
}

void configureLoadCells()
{
  LoadCell_petTray.begin(petTray_dout, petTray_sck);
  LoadCell_petTray.set_scale(calibrationValue_petTray);
  LoadCell_petTray.tare();
  Serial.println("Pet tray load cell initialized");

  LoadCell_platform.begin(platformLoadCell_dout, platformLoadCell_sck);
  LoadCell_platform.set_scale(calibrationValue_platform);
  LoadCell_platform.tare();
  Serial.println("Platform load cell initialized");
}

String generateRandomString(int length)
{
  String chars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
  String randomString = "";
  for (int i = 0; i < length; i++)
  {
    int index = random(0, chars.length());
    randomString += chars[index];
  }
  return randomString;
}

void recordFeedingDataToFirestore(const String &mode, const String &userName, float amount, const String &scheduleDate, const String &scheduledTime, const String &cageID, float initialWeight)
{
  // Debugging: Print the values of userName and scheduleDate
  Serial.print("userName: ");
  Serial.println(userName);
  Serial.print("scheduleDate: ");
  Serial.println(scheduleDate);

  // Check if userName or scheduleDate is empty
  if (userName.length() == 0 || scheduleDate.length() == 0)
  {
    Serial.println("Failed to add feeding record, reason: userName or scheduleDate is empty.");
    return;
  }

  content.clear(); // Clear previous content to avoid conflicts
  content.set("fields/mode/stringValue", mode);
  content.set("fields/userName/stringValue", userName);
  content.set("fields/amount/doubleValue", amount);
  content.set("fields/date/stringValue", scheduleDate); // Corrected the field name
  content.set("fields/time/stringValue", scheduledTime);
  content.set("fields/cageID/stringValue", cageID);
  content.set("fields/weight/doubleValue", initialWeight);

  // Debugging: Print the content JSON
  Serial.println("Content JSON to be sent:");
  String contentString;
  content.toString(contentString);
  Serial.println(contentString);

  String documentPath = "pets/";
  documentPath.concat(userName);
  documentPath.concat("/records/");
  documentPath.concat(generateRandomString(20)); // Adding a unique identifier to the record path

  Serial.print("Document path: ");
  Serial.println(documentPath);

  if (Firebase.Firestore.createDocument(&fbdo2, FIREBASE_FIRESTORE_HOST, "", documentPath.c_str(), content.raw()))
  {
    Serial.println("Feeding record added successfully.");
    Serial.printf("ok\n%s\n\n", fbdo2.payload().c_str());
  }
  else
  {
    Serial.print("Failed to add feeding record, reason: ");
    Serial.println(fbdo2.errorReason());
  }
}

float samplesForGettingWeight()
{
  const int numSamples = 10;
  float totalWeight = 0.0;
  for (int i = 0; i < numSamples; ++i)
  {
    totalWeight += LoadCell_platform.get_units(10);
    delay(100);
  }
  return totalWeight / numSamples;
}

void setPetWeight()
{
  float petsWeight = samplesForGettingWeight() / 1000;      // Convert to kilograms
  float petsWeightRounded = roundf(petsWeight * 100) / 100; // Round to nearest 0.1

  Serial.println("Weight data received from load cell");
  Serial.print("Weight: ");
  Serial.println(petsWeightRounded);

  String path = "/getWeight/loadCell/weight";
  if (Firebase.RTDB.setFloat(&fbdo1, path.c_str(), petsWeightRounded))
  {
    Serial.println("Weight data updated successfully.");
  }
  else
  {
    Serial.print("Failed to update weight data, reason: ");
    Serial.println(fbdo1.errorReason());
  }

  if (Firebase.RTDB.setBool(&fbdo1, "/trigger/getPetWeight/status", false))
  {
    Serial.println("Status of Weight Stream set to false");
  }
  else
  {
    Serial.print("Failed to set status of Weight Stream, reason: ");
    Serial.println(fbdo1.errorReason());
  }
}

void petWeightStream()
{
  if (Firebase.RTDB.readStream(&fbdo1))
  {
    if (fbdo1.streamTimeout())
    {
      Serial.println("Stream timeout, no data received from Firebase");
    }
    else if (fbdo1.dataType() == "boolean")
    {
      bool status = fbdo1.boolData();
      Serial.print("Status of Weight Stream: ");
      Serial.println(String(status).c_str());
      if (status)
      {
        setPetWeight();
      }
    }
  }
  else
  {
    Serial.print("Failed to read stream, reason: ");
    Serial.println(fbdo1.errorReason());
  }
}

// void foodConsumption() {
//   float foodConsumed = initialWeight - LoadCell_petTray.get_units();
//   Serial.print("Food Consumed: ");
//   Serial.println(foodConsumed);
//   return foodConsumed;
// }