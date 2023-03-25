#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

//Bluetooth message
String MessageBLE = "";

class MyCallbacks: public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      for (int i = 0; i < value.length(); i++){
        MessageBLE += value[i];
      }
    }
  }
};

void InitBluetooth(){
  BLEDevice::init("GreenTitan");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

String BluetoothRead(){
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

  MessageBLE = newStr;

  return oldStr;
}