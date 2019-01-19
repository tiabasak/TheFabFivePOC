#pragma once

#include <string>
#include <atomic>
#include <map>
#include <functional>

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "utils.h"
#include "motor_controller.h"
#include "motor.h"
#include "pid.h"

#define SET_BIT(val, pos) val |= BIT(pos)
#define STRINGIFY(STR) _STRINGIFY(STR)
#define _STRINGIFY(STR) #STR
#define PACKED __attribute__((packed))
#define CLEAR_BIT(val, pos) val &= ~(BIT((pos)));

#define MAX_NETWORK_NAME_SIZE 32
#define MAX_PASSWORD_SIZE 32

enum MessageID {
  SetLed,
  SetSteer,
  SetDrive,
  SetPid,
  SetNetworkName,
  SetPassword,
  ConnectToNetwork,
  GetPidLog,
  NumMessageIDs,
};


class iRoboCarBLECharacteristicUUIDs {
  public:
    static const std::string& GetUUID(MessageID message_id) {
      return CHARACTERISTIC_UUIDS[static_cast<int>(message_id)];
    }

  private:
    static const std::array<std::string, MessageID::NumMessageIDs> CHARACTERISTIC_UUIDS;
};
const std::array<std::string, NumMessageIDs> iRoboCarBLECharacteristicUUIDs::CHARACTERISTIC_UUIDS{{
    "c0de0001-feed-f00d-c0ff-eeb3d05ebeef", // SetLed
    "c0de0002-feed-f00d-c0ff-eeb3d05ebeef", // SetSteer
    "c0de0003-feed-f00d-c0ff-eeb3d05ebeef", // SetDrive
    "c0de0004-feed-f00d-c0ff-eeb3d05ebeef", // SetPid
    "c0de0005-feed-f00d-c0ff-eeb3d05ebeef", // SetNetworkName
    "c0de0006-feed-f00d-c0ff-eeb3d05ebeef", // SetPassword
    "c0de0007-feed-f00d-c0ff-eeb3d05ebeef", // ConnectToNetwork
    "c0de0008-feed-f00d-c0ff-eeb3d05ebeef", // GetPidLog
  }};


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename MessageData>
class IBLEMessage : public BLECharacteristicCallbacks {
  public:
    IBLEMessage(MessageID message_id) {
      message_id_ = message_id;
    }

    const MessageData& GetMessageData(uint8_t* characteristic_data) const {
      return *reinterpret_cast<MessageData*>(characteristic_data);
    }

    static MessageID message_id() {
      return message_id_;
    }

    static const std::string& uuid() {
      return iRoboCarBLECharacteristicUUIDs::GetUUID(message_id_);
    }

  private:
    static MessageID message_id_;
};
template<typename MessageData>
MessageID IBLEMessage<MessageData>::message_id_;


struct PACKED SetLedMessageData {
  bool state;
};
class SetLedMessage : public IBLEMessage<SetLedMessageData> {
  public:
    SetLedMessage() : IBLEMessage(SetLed) {}

    void onWrite(BLECharacteristic *characteristic) final {
      const auto& payload = GetMessageData(characteristic->getData());
      digitalWrite(LED_PIN, payload.state);
    }

  private:
    const int LED_PIN = 5;
};


struct PACKED SetSteerMessageData {
  float steer_angle;
};
class SetSteerMessage : public IBLEMessage<SetSteerMessageData> {
  public:
    SetSteerMessage() : IBLEMessage(SetSteer) {}

    void onWrite(BLECharacteristic *characteristic) final {
      const auto& payload = GetMessageData(characteristic->getData());
      DEBUG_LOG("Received SetSteer message, steer_angle = ", payload.steer_angle);
    }
};


struct PACKED SetDriveMessageData {
  float drive_speed;
};
class SetDriveMessage : public IBLEMessage<SetDriveMessageData> {
  public:
    SetDriveMessage() : IBLEMessage(SetDrive) {}

    void onWrite(BLECharacteristic *characteristic) final {
      const auto& payload = GetMessageData(characteristic->getData());
      DEBUG_LOG("Received SetDrive message, drive_speed = ", payload.drive_speed);
    }
};


struct PACKED SetPidMessageData {
  float p_gain;
  float i_gain;
  float d_gain;
  float i_limit;
};
class SetPidMessage : public IBLEMessage<SetPidMessageData> {
  public:
    SetPidMessage() : IBLEMessage(SetPid) {}

    void onWrite(BLECharacteristic *characteristic) final {
      const auto& payload = GetMessageData(characteristic->getData());
      DEBUG_LOG("Received SetPid message, p_gain = ", payload.p_gain, ", i_gain = ", payload.i_gain,
      ", d_gain = ", payload.d_gain, ", i_limit = ", payload.i_limit);
    }
};


struct PACKED SetNetworkNameMessageData {
  static constexpr int NETWORK_NAME_MESSAGE_SIZE = 32;
  char network_name[NETWORK_NAME_MESSAGE_SIZE];
};
class SetNetworkNameMessage : public IBLEMessage<SetNetworkNameMessageData> {
  public:
    SetNetworkNameMessage() : IBLEMessage(SetNetworkName) {}

    void onWrite(BLECharacteristic *characteristic) final {
      const auto& payload = GetMessageData(characteristic->getData());
      DEBUG_LOG("Received SetNetworkName message, network_name = ", payload.network_name);
    }
};


struct PACKED SetPasswordMessageData {
  static constexpr int PASSWORD_MESSAGE_SIZE = 32;
  char password[PASSWORD_MESSAGE_SIZE];
};
class SetPasswordMessage : public IBLEMessage<SetPasswordMessageData> {
  public:
    SetPasswordMessage() : IBLEMessage(SetPassword) {}

    void onWrite(BLECharacteristic *characteristic) final {
      const auto& payload = GetMessageData(characteristic->getData());
      DEBUG_LOG("Received SetPassword message, password = ", payload.password);
    }
};


struct PACKED ConnectToNetworkMessageData {};
class ConnectToNetworkMessage : public IBLEMessage<ConnectToNetworkMessageData> {
  public:
    ConnectToNetworkMessage() : IBLEMessage(ConnectToNetwork) {}

    void onWrite(BLECharacteristic *characteristic) final {
      const auto& payload = GetMessageData(characteristic->getData());
      DEBUG_LOG("Received ConnectToNetwork message");
    }
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class iRoboCarBLEServer : public BLEServerCallbacks {
  public:
    void Init() {
      BLEDevice::init("iRoboCar");
      BLEServer* server = BLEDevice::createServer();
      server->setCallbacks(this);
      BLEService* service = server->createService(SERVICE_UUID);

      RegisterMessage<SetLedMessage>(service);
      RegisterMessage<SetSteerMessage>(service);
      RegisterMessage<SetDriveMessage>(service);
      RegisterMessage<SetPidMessage>(service);
      RegisterMessage<SetNetworkNameMessage>(service);
      RegisterMessage<SetPasswordMessage>(service);
      RegisterMessage<ConnectToNetworkMessage>(service);

      service->start();
      BLEAdvertising* advertising = server->getAdvertising();
      advertising->start();
    }

    void onConnect(BLEServer* server) {
      device_connected_ = true;
      DEBUG_LOG("Connected");
    }

    void onDisconnect(BLEServer* server) {
      device_connected_ = false;
      DEBUG_LOG("Disconnected");
    }

    bool IsConnected() const {
      return device_connected_;
    }

  private:
    static const std::string SERVICE_UUID;

    template<typename Message>
    void RegisterMessage(BLEService* service) {
      Message* message = new Message;
      BLECharacteristic* characteristic = service->createCharacteristic(message->uuid().c_str(),
                                          BLECharacteristic::PROPERTY_WRITE);
      characteristic->addDescriptor(new BLE2902());
      characteristic->setCallbacks(message);
    }

    bool device_connected_ = false;
};
const std::string iRoboCarBLEServer::SERVICE_UUID = "c0de0000-feed-f00d-c0ff-eeb3d05ebeef";
