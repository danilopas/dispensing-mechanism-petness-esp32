#include <Arduino.h>
#include <HX711_ADC.h> // weight sensor
#include "Stepper.h"   // stepper motor
#include <esp_now.h>
#include <WiFi.h>

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
float amountToDispense = 0; // Initialized to 0, will be updated by ESP-NOW

// ESP-NOW data structure
typedef struct struct_message
{
  char userName[32];
  float amountToDispense;
  char scheduledDate[20];
  char scheduledTime[20];
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Flag to indicate if new data is received
bool newDataReceived = false;

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  if (len == sizeof(myData))
  {
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Data received: ");
    Serial.print("Amount to dispense: ");
    Serial.println(myData.amountToDispense);
    amountToDispense = myData.amountToDispense;
    newDataReceived = true; // Set the flag to indicate new data received
  }
  else
  {
    Serial.println("Received data length mismatch!");
  }
}

void setupESPNow()
{
  WiFi.mode(WIFI_STA); // Initialize Wi-Fi in station mode
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  Serial.println("ESP-NOW Initialized");
  esp_now_register_recv_cb(onDataRecv);
}

void dispenseMotor1A()
{
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor1.step(smallStep); // Smaller steps for better control
    LoadCell.update();      // Update load cell value
    if (LoadCell.getData() >= amountToDispense)
      break; // Check if threshold is reached
  }
}

void dispenseMotor1B()
{
  for (int i = 0; i < stepsPerRevolution; i += smallStep)
  {
    motor1.step(-smallStep); // Smaller steps for better control
    LoadCell.update();       // Update load cell value
    if (LoadCell.getData() >= amountToDispense)
      break; // Check if threshold is reached
  }
}

void stopDispenseMotor1()
{
  digitalWrite(motor1Relay, LOW);
}

void loadCellConfiguration()
{
  unsigned long stabilizingtime = 2000; // tare precision can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
  }
  else
  {
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
  if (LoadCell.getSPS() < 7)
  {
    Serial.println("!!Sampling rate is lower than specification, check MCU>HX711 wiring and pin designations");
  }
  else if (LoadCell.getSPS() > 100)
  {
    Serial.println("!!Sampling rate is higher than specification, check MCU>HX711 wiring and pin designations");
  }
}

void setup()
{
  Serial.begin(115200);

  // Relay Config
  pinMode(motor1Relay, OUTPUT);
  digitalWrite(motor1Relay, LOW);

  // Initialize ESP-NOW
  setupESPNow();

  // Motor Config
  motor1.setSpeed(motorSpeed);

  Serial.println();
  Serial.println("Starting...");
  LoadCell.begin();
  loadCellConfiguration();
}

void loop()
{
  static boolean newDataReady = 0;
  const int serialPrintInterval = 100; // increase value to slow down serial print activity

  // Check if new data is received
  if (newDataReceived)
  {
    newDataReceived = false; // Reset the flag after processing

    // check for new data/start next conversion:
    if (LoadCell.update())
      newDataReady = true;

    // get smoothed value from the dataset:
    if (newDataReady)
    {
      if (millis() > t + serialPrintInterval)
      {

        float petTrayAmount = LoadCell.getData();
        if (petTrayAmount >= amountToDispense)
        {
          stopDispenseMotor1();
          Serial.println(" -- Reached the Threshold, STOP");
          Serial.print("Load_cell output val: ");
          Serial.println(petTrayAmount);
        }
        else if (petTrayAmount < amountToDispense && !dispenseMotor1ACounter)
        {
          digitalWrite(motor1Relay, HIGH);
          Serial.println(" Rotating Clockwise --");
          dispenseMotor1A();
          Serial.print("Load_cell output val: ");
          Serial.println(petTrayAmount);
          newDataReady = 0;
          t = millis();
          dispenseMotor1ACounter = true;
        }
        else if (petTrayAmount < amountToDispense && dispenseMotor1ACounter)
        {
          Serial.println("Counter Clockwise --");
          dispenseMotor1B();
          Serial.print("Load_cell output val: ");
          Serial.println(petTrayAmount);
          newDataReady = 0;
          t = millis();
          dispenseMotor1ACounter = false;
        }
        else
        {
          Serial.println("Stepper Motor Not Moving Because the Threshold is Zero");
        }
      }
    }
  }
}
