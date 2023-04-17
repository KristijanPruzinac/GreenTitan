#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//Bluetooth message
String MessageBLE = "";

//Connection
BLEClient*  pClient = nullptr;
BLEAdvertising *pAdvertising = nullptr;
bool        isConnected = false;
BLECharacteristic* pCharacteristic = nullptr;

class MyCallbacks: public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    MessageBLE = value.c_str();
  }
};

class MyClientCallbacks: public BLEClientCallbacks {
    void onConnect(BLEClient* pClient) {
        isConnected = true;
        Serial.println("Bluetooth Connected");
    }

    void onDisconnect(BLEClient* pClient) {
        isConnected = false;
        if (pAdvertising)pAdvertising->start();
        Serial.println("Bluetooth Disconnected");
    }
};

void InitBluetooth(){
  BLEDevice::init("GreenTitan");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();

  // Create a new BLE client object and set its callbacks
  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallbacks());
}

bool BluetoothConnection() {
  /*
  if (!isConnected) {
    Serial.println("Trying to reconnect Bluetooth...");
    pClient->connect(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>("GreenTitan")));
    return false;
  }*/

  return true;
}

String BluetoothRead(){
  if (!BluetoothConnection()){}

  int len = MessageBLE.length();

  String oldStr = "";
  String newStr = "";
  bool foundEndline = false;
  for (int i = 0; i < len; i++){
    if (MessageBLE[i] == '/'){
      foundEndline = true;
      continue;
    }

    if (foundEndline){
      newStr += MessageBLE[i];
    }
    else {
      oldStr += MessageBLE[i];
    }
  }

  if (!foundEndline) {
    return "";
  }

  MessageBLE = newStr;

  return oldStr;
}

void BluetoothWrite(String message) {
  if (!BluetoothConnection()){}

  if (!pCharacteristic) {
    Serial.println("Error: Write characteristic not initialized");
    return;
  }

  pCharacteristic->setValue(message.c_str());
  pCharacteristic->notify();
}
