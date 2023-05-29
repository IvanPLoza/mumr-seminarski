#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>

// LED
#define LED_PIN 8

// IMU SENSOR
#define BNO005_SENSOR_ID 55
#define SENSOR_REFRESH_RATE 100
#define I2C_SDA_PIN 1
#define I2C_SCL_PIN 0

// BLE
#define BLE_SERVER_NAME "MUMR_LOZANCIC_SEMINAR"
#define BLE_IMU_SERVICE_UUID "21a64c30-ecee-11ed-a05b-0242ac120003"
#define BLE_R_CHARACTERISTIC_UUID "78ff8b3a-ed1c-11ed-a05b-0242ac120003"
#define BLE_G_CHARACTERISTIC_UUID "84110cb0-ed1c-11ed-a05b-0242ac120003"
#define BLE_B_CHARACTERISTIC_UUID "87d6c182-ed1c-11ed-a05b-0242ac120003"
#define BLE_REFRESH_RATE 1000

#define DEBUG_SERIAL
// #define DEBUG_SERIAL_SENSOR
#define DEBUG_SERIAL_BLE

// Sensor
Adafruit_BNO055 sensor = Adafruit_BNO055(BNO005_SENSOR_ID, 0x28, &Wire);

// BLE
String BLE_CLIENT_NAME = WiFi.macAddress();

BLEClient *bleClient;
BLEAddress *pServerAddress;
BLEScan *bleScannner;

BLERemoteCharacteristic *bleRedColorCharacteristic;
BLERemoteCharacteristic *bleGreenColorCharacteristic;
BLERemoteCharacteristic *bleBlueColorCharacteristic;

bool bleClientConnectedToServer = false;
bool bleScanComplete = false;

void connectBLE(BLEAddress pAddress)
{
#ifdef DEBUG_SERIAL_BLE
  Serial.println("[BLE] Connecting to BLE server...");
#endif
  bleClient->connect(pAddress);

  // Obtain a reference to the remote service
  BLERemoteService *remoteService = bleClient->getService(BLE_IMU_SERVICE_UUID);

#ifdef DEBUG_SERIAL_BLE
  Serial.println("[BLE] Service found, getting characteristics...");
#endif

  // Obtain a references to the characteristics
  bleRedColorCharacteristic = remoteService->getCharacteristic(BLE_R_CHARACTERISTIC_UUID);
  bleGreenColorCharacteristic = remoteService->getCharacteristic(BLE_G_CHARACTERISTIC_UUID);
  bleBlueColorCharacteristic = remoteService->getCharacteristic(BLE_B_CHARACTERISTIC_UUID);

#ifdef DEBUG_SERIAL_BLE
  Serial.println("[BLE] Finished Connecting to BLE server!");
#endif
}

void bleScanCompleteHandler(BLEScanResults scanResults)
{
  bleScanComplete = true;

  // Check if we need to start the scan again
  if (!bleClientConnectedToServer)
  {
#ifdef DEBUG_SERIAL_BLE
    Serial.println("[BLE] Failed to find desired server, starting scanning again!");
#endif
    bleScanComplete = false;
    bleScannner->start(30, bleScanCompleteHandler, false);
  }
}

void bleStartScanning()
{
#ifdef DEBUG_SERIAL_BLE
  Serial.println("[BLE] Started scanning...");
#endif
  bleScanComplete = false;
  bleScannner->start(30, bleScanCompleteHandler, false);
}

class BLE_ClientCallbacks : public BLEClientCallbacks
{
  void onConnect(BLEClient *pClient)
  {
#ifdef DEBUG_SERIAL_BLE
    Serial.println("[BLE] Client connected!");
#endif
    bleClientConnectedToServer = true;
  }

  void onDisconnect(BLEClient *pClient)
  {
#ifdef DEBUG_SERIAL_BLE
    Serial.println("[BLE] Client disconnected!");
#endif
    bleClientConnectedToServer = false;

    bleStartScanning();
  }
};

class BLE_AdvertiseCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    std::string aDeviceName = advertisedDevice.getName();
#ifdef DEBUG_SERIAL_BLE
    Serial.print("[BLE] Scanned new BLE device with name: ");
    Serial.println(aDeviceName.c_str());
#endif
    // Check if scanned device is our server
    if (aDeviceName == BLE_SERVER_NAME)
    {
      // Get address and connect the client, stop scan
#ifdef DEBUG_SERIAL_BLE
      Serial.println("[BLE] Desired client found, stopping the scan...");
#endif
      advertisedDevice.getScan()->stop();
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      connectBLE(*pServerAddress);
    }
  }
};

// LED
Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);

// Timers
struct
{
  int timSensor;
  int timBle;
} timerMillis;

void setupSensor()
{
  // Begin I2C communication
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  int counter = 1;
  while (!sensor.begin())
  {
    counter++;
    if (counter > 5)
    {
      // Restart the device as the I2C failed
      ESP.restart();
    }
    delay(2000);
  }
  delay(1000);
  sensor.setExtCrystalUse(true);
}

void updateLed(int R, int G, int B)
{
  pixels.setPixelColor(0, pixels.Color(R, G, B));
  pixels.show();
}

void setupBLE()
{
  // Init BLE
  BLEDevice::init(BLE_CLIENT_NAME.c_str());

#ifdef DEBUG_SERIAL_BLE
  Serial.println("[BLE] Scanner init");
#endif

  // Init BLE Scanner
  bleScannner = BLEDevice::getScan();
  bleScannner->setAdvertisedDeviceCallbacks(new BLE_AdvertiseCallbacks());
  bleScannner->setActiveScan(true);
  bleStartScanning();

  // Init BLE Client
  bleClient = BLEDevice::createClient();
  bleClient->setClientCallbacks(new BLE_ClientCallbacks());
}

void setupLed()
{
  for (uint8_t i = 0; i < 5; i++)
  {
    updateLed(0, 0, 0);
    delay(500);
    updateLed(255, 255, 255);
    delay(500);
  }
  updateLed(0, 0, 0);
}

uint8_t *parseOrientationToRGB(float x, float y, float z)
{
  uint8_t *array = new uint8_t[3];

  array[0] = x <= 180 ? 255 - map(x, 0, 180, 0, 255) : map(x, 180, 360, 0, 255);
  array[1] = y < 0 ? 255 - map(y, -90, 0, 0, 255) : map(y, 0, 180, 0, 255);
  array[2] = z < 0 ? 255 - map(z, -180, 0, 0, 255) : map(z, 0, 180, 0, 255);

  return array;
}

void collectData()
{
  // Check if we need to collect data
  int currentT = millis();
  if (currentT - timerMillis.timSensor >= SENSOR_REFRESH_RATE)
  {

    // Get sensor values
    sensors_event_t event;
    sensor.getEvent(&event);

    // Parse them to color values
    uint8_t *parsedValues = parseOrientationToRGB(event.orientation.x, event.orientation.y, event.orientation.z);

    // Update led
    updateLed(parsedValues[0], parsedValues[1], parsedValues[2]);

#ifdef DEBUG_SERIAL_SENSOR
    Serial.print("[SENSOR] X: ");
    Serial.print(parsedValues[0]);
    Serial.print("  ");
    Serial.print(event.orientation.x);
    Serial.print("  ");
    Serial.print("\tY: ");
    Serial.print(parsedValues[1]);
    Serial.print("  ");
    Serial.print(event.orientation.y);
    Serial.print("  ");
    Serial.print("\tZ: ");
    Serial.print(parsedValues[2]);
    Serial.print("  ");
    Serial.print(event.orientation.z);
    Serial.print("  ");
    Serial.println("");
#endif

    // Check if we need to send data
    if (currentT - timerMillis.timBle >= BLE_REFRESH_RATE)
    {
      // If client is connected send the data
      if (bleClientConnectedToServer)
      {
#ifdef DEBUG_SERIAL_BLE
        Serial.println("[BLE] Sending values to ble server...");
#endif
        bleRedColorCharacteristic->writeValue(parsedValues[0], sizeof(uint8_t));
        bleGreenColorCharacteristic->writeValue(parsedValues[1], sizeof(uint8_t));
        bleBlueColorCharacteristic->writeValue(parsedValues[2], sizeof(uint8_t));
#ifdef DEBUG_SERIAL_BLE
        Serial.println("[BLE] Sent values to ble server.");
#endif
      }
      timerMillis.timBle = currentT;
    }

    // Free memory
    delete[] parsedValues;

    // Update timer
    timerMillis.timSensor = currentT;
  }
}

void setupTimers()
{
  int start = millis();
  timerMillis.timBle = start;
  timerMillis.timSensor = start;
}

void setup()
{
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  delay(100);
#endif

  // Prepare everything
  setupLed();
  setupSensor();
  setupBLE();
  setupTimers();
}

void loop()
{
  // Start collecting the data
  collectData();
}