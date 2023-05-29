#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>

// LED
#define LED_PIN 8

// BLE
#define BLE_SERVER_NAME "MUMR_LOZANCIC_SEMINAR"
#define BLE_IMU_SERVICE_UUID "21a64c30-ecee-11ed-a05b-0242ac120003"
#define BLE_R_CHARACTERISTIC_UUID "78ff8b3a-ed1c-11ed-a05b-0242ac120003"
#define BLE_G_CHARACTERISTIC_UUID "84110cb0-ed1c-11ed-a05b-0242ac120003"
#define BLE_B_CHARACTERISTIC_UUID "87d6c182-ed1c-11ed-a05b-0242ac120003"

// Configuration
#define DEBUG_SERIAL

// LED
Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);

void updateLed(int R, int G, int B)
{
  pixels.setPixelColor(0, pixels.Color(R, G, B));
  pixels.show();
}

struct BLE_CLIENT_COLORS
{
  uint8_t redColor;
  uint8_t greenColor;
  uint8_t blueColor;
  std::string clientMacAddress;
};
std::vector<BLE_CLIENT_COLORS> bleClientColors;

void bleAddClientColors(std::string macAddress)
{
  BLE_CLIENT_COLORS clientToAdd;
  clientToAdd.blueColor = 0;
  clientToAdd.greenColor = 0;
  clientToAdd.blueColor = 0;
  clientToAdd.clientMacAddress = macAddress;
  bleClientColors.push_back(clientToAdd);
}

void bleRemoveClientColors(std::string macAddress)
{
  for (auto it = bleClientColors.begin(); it != bleClientColors.end(); ++it)
  {
    if (it->clientMacAddress == macAddress)
    {
      bleClientColors.erase(it);
      break;
    }
  }
}

BLE_CLIENT_COLORS *bleFindClientColors(std::string clientAddress)
{
  for (auto &bleClientColor : bleClientColors)
  {
    if (bleClientColor.clientMacAddress == clientAddress)
    {
      return &bleClientColor;
    }
  }
  return nullptr;
}

void bleUpdateClientColors(std::string clientAddress, BLE_CLIENT_COLORS updatedColors)
{
  for (auto &clientColors : bleClientColors)
  {
    if (clientColors.clientMacAddress == clientAddress)
    {
      clientColors = updatedColors;
      break;
    }
  }
}

void bleUpdateLed()
{
  // Now calculate led rgb value based on all clients
  uint32_t rColorCalc = 0;
  uint32_t gColorCalc = 0;
  uint32_t bColorCalc = 0;
  size_t numberOfClients = bleClientColors.size();
  for (const auto &bleClientColor : bleClientColors)
  {
    rColorCalc += bleClientColor.redColor;
    gColorCalc += bleClientColor.greenColor;
    bColorCalc += bleClientColor.blueColor;
  }
  rColorCalc = rColorCalc / numberOfClients;
  gColorCalc = gColorCalc / numberOfClients;
  bColorCalc = bColorCalc / numberOfClients;

  updateLed(rColorCalc, gColorCalc, bColorCalc);
}

void bleUpdateLedValue(uint8_t color, uint8_t value, std::string macAddress)
{
  // First update the value for the device in array
  BLE_CLIENT_COLORS *deviceValueFound = bleFindClientColors(macAddress);
  if (deviceValueFound != NULL)
  {
    switch (color)
    {
    case 0:
    {
      // Red color
      deviceValueFound->redColor = value;
    }
    case 1:
    {
      // Green color
      deviceValueFound->greenColor = value;
    }
    case 2:
    {
      // Blue color
      deviceValueFound->blueColor = value;
    }
    }
  }

  // Update the led
  bleUpdateLed();
}

// BLE
BLEServer *bleServer;
BLECharacteristic *bleRedColorCharacteristic;
BLECharacteristic *bleGreenColorCharacteristic;
BLECharacteristic *bleBlueColorCharacteristic;

std::string convertEspBLEAddressToString(uint8_t *byteArray, size_t length)
{
  std::string result;

  for (size_t i = 0; i < length; i++)
  {
    result += static_cast<char>(byteArray[i]);
  }

  return result;
}

class BLE_ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t param)
  {
#ifdef DEBUG_SERIAL
    Serial.println("[BLE] New client connected!");
#endif

    std::string clientAddress = convertEspBLEAddressToString(param.write.bda, 6);

#ifdef DEBUG_SERIAL
    Serial.print("[BLE] New client address: ");
    Serial.println(clientAddress.c_str());
#endif

    // Add client colors item
    bleAddClientColors(clientAddress);
  };

  void onDisconnect(BLEServer *pServer, esp_ble_gatts_cb_param_t param)
  {
#ifdef DEBUG_SERIAL
    Serial.println("[BLE] Client disconnected!");
#endif

    std::string clientAddress = convertEspBLEAddressToString(param.write.bda, 6);

#ifdef DEBUG_SERIAL
    Serial.print("[BLE] Old client address: ");
    Serial.println(clientAddress.c_str());
#endif

    // Add client colors item
    bleRemoveClientColors(clientAddress);
  }
};

class BLE_RedColorCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t param)
  {
#ifdef DEBUG_SERIAL
    Serial.println("[BLE] Red color callback fired!");
#endif
    // Read value
    std::string receivedData = pCharacteristic->getValue();
    std::string clientAddress = convertEspBLEAddressToString(param.write.bda, 6);
    if (receivedData.length() > 0)
    {
      // Parse data and update led
      uint8_t parsedValue = static_cast<uint8_t>(receivedData[0]);

#ifdef DEBUG_SERIAL
      Serial.print("[BLE] Red color value received: ");
      Serial.println(parsedValue);
#endif
      bleUpdateLedValue(0, parsedValue, clientAddress);
    }
#ifdef DEBUG_SERIAL
    else
    {
      Serial.println("[BLE] Received malformed empty value!");
    }
#endif
  }
};

class BLE_GreenColorCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t param)
  {
#ifdef DEBUG_SERIAL
    Serial.println("[BLE] Green color callback fired!");
#endif
    // Read value
    std::string receivedData = pCharacteristic->getValue();
    std::string clientAddress = convertEspBLEAddressToString(param.write.bda, 6);
    if (receivedData.length() > 0)
    {
      // Parse data and update led
      uint8_t parsedValue = static_cast<uint8_t>(receivedData[0]);

#ifdef DEBUG_SERIAL
      Serial.print("[BLE] Green color value received: ");
      Serial.println(parsedValue);
#endif
      bleUpdateLedValue(1, parsedValue, clientAddress);
    }
#ifdef DEBUG_SERIAL
    else
    {
      Serial.println("[BLE] Received malformed empty value!");
    }
#endif
  }
};

class BLE_BlueColorCharacteristicCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic, esp_ble_gatts_cb_param_t param)
  {
#ifdef DEBUG_SERIAL
    Serial.println("[BLE] Blue color callback fired!");
#endif
    // Read value
    std::string receivedData = pCharacteristic->getValue();
    std::string clientAddress = convertEspBLEAddressToString(param.write.bda, 6);
    if (receivedData.length() > 0)
    {
      // Parse data and update led
      uint8_t parsedValue = static_cast<uint8_t>(receivedData[0]);

#ifdef DEBUG_SERIAL
      Serial.print("[BLE] Blue color value received: ");
      Serial.println(parsedValue);
#endif
      bleUpdateLedValue(2, parsedValue, clientAddress);
    }
#ifdef DEBUG_SERIAL
    else
    {
      Serial.println("[BLE] Received malformed empty value!");
    }
#endif
  }
};

void setupBLEServer()
{
#ifdef DEBUG_SERIAL
  Serial.println("[BLE] Init BLE server....");
#endif

  // Init BLE Server
  BLEDevice::init(BLE_SERVER_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BLE_ServerCallbacks());

#ifdef DEBUG_SERIAL
  Serial.println("[BLE] Defining BLE server characterstics and callbacks...");
#endif

  // Create the service and characteristics
  BLEService *service = bleServer->createService(BLE_IMU_SERVICE_UUID);
  bleRedColorCharacteristic = service->createCharacteristic(
      BLE_R_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  bleGreenColorCharacteristic = service->createCharacteristic(
      BLE_G_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  bleBlueColorCharacteristic = service->createCharacteristic(
      BLE_B_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE);
  bleRedColorCharacteristic->setCallbacks(new BLE_RedColorCharacteristicCallbacks());
  bleGreenColorCharacteristic->setCallbacks(new BLE_GreenColorCharacteristicCallbacks());
  bleBlueColorCharacteristic->setCallbacks(new BLE_BlueColorCharacteristicCallbacks());

#ifdef DEBUG_SERIAL
  Serial.println("[BLE] Starting BLE advertising...");
#endif

  // Start the service and advertise
  service->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_IMU_SERVICE_UUID);
  bleServer->getAdvertising()->start();

#ifdef DEBUG_SERIAL
  Serial.println("[BLE] BLE Server init done!");
#endif
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

void setup()
{
#ifdef DEBUG_SERIAL
  Serial.begin(115200);
  delay(100);
#endif

  // Setup everything
  setupLed();
  setupBLEServer();
}

void loop()
{
}