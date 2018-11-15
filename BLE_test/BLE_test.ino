/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    Extended by Thejesh GN
*/
 
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
 
 
#define SERVICE_UUID        "c0de0001-feed-f00d-c0ff-eeb3d05ebeef"
#define CHARACTERISTIC_UUID_TX "c0de0002-feed-f00d-c0ff-eeb3d05ebeef"
#define CHARACTERISTIC_UUID_RX "c0de0003-feed-f00d-c0ff-eeb3d05ebeef"

#define DEBUG_LOG(...) println(__VA_ARGS__)
 
bool deviceConnected = false;
bool messageReceivedComplete = false;
 
BLECharacteristic *pCharacteristicRX;
BLECharacteristic *pCharacteristicTX;
int echoNumber = 0;
std::string message;
int ledPin = 5;
 
class EchoServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      //stop advt?
      Serial.println("Connected!");
    };
 
    void onDisconnect(BLEServer* pServer) {
      //restart advt?
      deviceConnected = false;
      Serial.println("Disconnected!");
    }
};
 
 
 
void respond(std::string  send_message){
  Serial.println("inside send_message");
  pCharacteristicTX->setValue(send_message);
  pCharacteristicTX->notify();
  Serial.println("sent send_message");
 
}
 
 
 
//This call back happens when the android client writes 
class ServerReadCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string rxValue = pCharacteristic->getValue();
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++) {
          Serial.print(rxValue[i]);          
        }
        Serial.println();
 
        //add to message (as of now just one packet)        
        message = rxValue;
 
        //once you think all packets are received. As of now one packet
        messageReceivedComplete = true;
 
    }
};
 
void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
 
  BLEDevice::init("ThejEcho");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new EchoServerCallbacks());
 
  BLEService *pService = pServer->createService(SERVICE_UUID);
   
  pCharacteristicRX = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,                                         
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
 
  pCharacteristicRX->addDescriptor(new BLE2902());
 
  pCharacteristicRX->setCallbacks(new ServerReadCallbacks());
 
  pCharacteristicTX = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_TX,                                         
                                         BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
                                       );
 
  pCharacteristicTX->addDescriptor(new BLE2902());
   
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("Characteristic defined! Now you can read it in your phone!");

  pinMode(ledPin, OUTPUT);
}

size_t println(){
  return 0;
}

 template<typename T, typename ... TailType>
size_t println(T&& head, TailType&& ...tail){
  size_t r = 0;
  r+=Serial.println(head);
  r+=println((tail)...);
  return r;
}


void loop() { 
  // some delay before running repeatedly
  delay(1000);
 
 
  // if device is connected and all packets are received, then respond
  if(deviceConnected && messageReceivedComplete){
        messageReceivedComplete = false;        
       if(String("on").c_str() == message){
          //respond(String("world").c_str());
          //Serial.println("sent world");
          digitalWrite(ledPin, HIGH);
       }else if(String("off").c_str() == message){
         
          digitalWrite(ledPin, LOW);
       }

             
      respond(message);
      DEBUG_LOG("echo", 2, "hello", 3.3f);
      Serial.println("echoed");
      Serial.println(message.c_str());
       
  }

  

}
