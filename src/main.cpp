#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <FastAccelStepper.h>

namespace {

constexpr char DEVICE_NAME[] = "XIAO BLE Motor";
constexpr char SERVICE_UUID[] = "7b7f0001-9b6d-4f8b-8c5d-9bb6f6f68c01";
constexpr char COMMAND_UUID[] = "7b7f0002-9b6d-4f8b-8c5d-9bb6f6f68c01";
constexpr char STATUS_UUID[] = "7b7f0003-9b6d-4f8b-8c5d-9bb6f6f68c01";

constexpr int DIR = D8;
constexpr int STEP = D9;
constexpr int MS1 = D0;
constexpr int MS2 = D1;
constexpr int MS3 = D2;
constexpr int EN = D3;

constexpr uint32_t STATUS_INTERVAL_MS = 100;
constexpr int32_t MIN_SPEED_HZ = 1;
constexpr int32_t MAX_SPEED_HZ = 5000;
constexpr int32_t MIN_ACCEL = 1;
constexpr int32_t MAX_ACCEL = 50000;

int d = 3000;
int stepCount = 3200;
int microstepMode = 8;
int acceleration = 400;
bool motorEnabled = true;
int32_t jogSpeedHz = 0;

bool bleConnected = false;
uint32_t lastStatusMs = 0;

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;
BLECharacteristic* statusCharacteristic = nullptr;

int32_t clampInt32(int32_t value, int32_t minValue, int32_t maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

uint32_t calcSpeedHz() {
  uint32_t periodUs = static_cast<uint32_t>(d) * 2UL;
  if (periodUs < 200UL) {
    periodUs = 200UL;
  }

  uint32_t hz = 1000000UL / periodUs;
  if (hz < 1UL) {
    hz = 1UL;
  }
  if (hz > static_cast<uint32_t>(MAX_SPEED_HZ)) {
    hz = MAX_SPEED_HZ;
  }
  return hz;
}

void applyMotionParams(uint32_t speedHz = 0) {
  if (stepper == nullptr) {
    return;
  }

  const uint32_t resolvedSpeed = speedHz > 0 ? speedHz : calcSpeedHz();
  stepper->setSpeedInHz(resolvedSpeed);
  stepper->setAcceleration(static_cast<uint32_t>(acceleration));
}

void setMicrostep(int mode) {
  microstepMode = mode;

  switch (mode) {
    case 1:
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, LOW);
      digitalWrite(MS3, LOW);
      break;
    case 2:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, LOW);
      digitalWrite(MS3, LOW);
      break;
    case 4:
      digitalWrite(MS1, LOW);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, LOW);
      break;
    case 8:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, LOW);
      break;
    case 16:
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, HIGH);
      break;
    default:
      microstepMode = 8;
      digitalWrite(MS1, HIGH);
      digitalWrite(MS2, HIGH);
      digitalWrite(MS3, LOW);
      break;
  }
}

void startJog(int32_t signedSpeedHz) {
  if (stepper == nullptr || !motorEnabled) {
    return;
  }

  if (signedSpeedHz == 0) {
    jogSpeedHz = 0;
    stepper->stopMove();
    return;
  }

  const bool wasRunning = stepper->isRunning();
  const bool signChanged = (jogSpeedHz > 0 && signedSpeedHz < 0) || (jogSpeedHz < 0 && signedSpeedHz > 0);
  const uint32_t speed = static_cast<uint32_t>(abs(signedSpeedHz));

  jogSpeedHz = signedSpeedHz;
  applyMotionParams(speed);

  if (wasRunning && signChanged) {
    stepper->forceStop();
    delay(5);
  }

  if (signedSpeedHz > 0) {
    stepper->runForward();
  } else {
    stepper->runBackward();
  }
}

void moveSteps(int32_t signedSteps) {
  if (stepper == nullptr || !motorEnabled || signedSteps == 0) {
    return;
  }

  jogSpeedHz = 0;
  applyMotionParams();
  stepper->move(signedSteps);
}

int readCsvInt(const String& command, int index, int defaultValue) {
  int start = 0;
  int part = 0;

  while (part <= index) {
    const int comma = command.indexOf(',', start);
    const int end = comma >= 0 ? comma : command.length();

    if (part == index) {
      return command.substring(start, end).toInt();
    }

    if (comma < 0) {
      break;
    }
    start = comma + 1;
    part++;
  }

  return defaultValue;
}

void handleCommand(String command) {
  command.trim();
  if (command.length() == 0 || stepper == nullptr) {
    return;
  }

  const char op = command.charAt(0);

  switch (op) {
    case 'C': {
      if (stepper->isRunning()) {
        stepper->forceStop();
        delay(5);
      }

      setMicrostep(readCsvInt(command, 1, microstepMode));
      d = clampInt32(readCsvInt(command, 2, d), 100, 1000000);
      stepCount = clampInt32(readCsvInt(command, 3, stepCount), 1, INT32_MAX);
      acceleration = clampInt32(readCsvInt(command, 4, acceleration), MIN_ACCEL, MAX_ACCEL);
      applyMotionParams();
      break;
    }
    case 'V': {
      const int32_t speed = clampInt32(readCsvInt(command, 1, 0), -MAX_SPEED_HZ, MAX_SPEED_HZ);
      acceleration = clampInt32(readCsvInt(command, 2, acceleration), MIN_ACCEL, MAX_ACCEL);
      startJog(speed);
      break;
    }
    case 'M': {
      moveSteps(readCsvInt(command, 1, stepCount));
      break;
    }
    case 'S': {
      jogSpeedHz = 0;
      stepper->forceStop();
      break;
    }
    case 'E': {
      motorEnabled = readCsvInt(command, 1, 1) != 0;
      if (motorEnabled) {
        stepper->enableOutputs();
      } else {
        jogSpeedHz = 0;
        stepper->forceStop();
        delay(5);
        stepper->disableOutputs();
      }
      break;
    }
    case 'R': {
      jogSpeedHz = 0;
      if (stepper->isRunning()) {
        stepper->forceStopAndNewPosition(0);
      } else {
        stepper->setCurrentPosition(0);
      }
      break;
    }
    default:
      Serial.print("[BLE] Unknown command: ");
      Serial.println(command);
      break;
  }
}

String buildStatusJson() {
  String json = "{";
  json += "\"position\":" + String(stepper != nullptr ? stepper->getCurrentPosition() : 0) + ",";
  json += "\"running\":" + String(stepper != nullptr && stepper->isRunning() ? "true" : "false") + ",";
  json += "\"direction\":\"" + String(jogSpeedHz < 0 ? "CCW" : "CW") + "\",";
  json += "\"jogSpeedHz\":" + String(jogSpeedHz) + ",";
  json += "\"speedHz\":" + String(abs(jogSpeedHz) > 0 ? abs(jogSpeedHz) : static_cast<int32_t>(calcSpeedHz())) + ",";
  json += "\"d\":" + String(d) + ",";
  json += "\"stepCount\":" + String(stepCount) + ",";
  json += "\"microstep\":" + String(microstepMode) + ",";
  json += "\"acceleration\":" + String(acceleration) + ",";
  json += "\"enabled\":" + String(motorEnabled ? "true" : "false");
  json += "}";
  return json;
}

void notifyStatus(bool force = false) {
  if (!bleConnected || statusCharacteristic == nullptr) {
    return;
  }

  const uint32_t now = millis();
  if (!force && now - lastStatusMs < STATUS_INTERVAL_MS) {
    return;
  }

  lastStatusMs = now;
  const String status = buildStatusJson();
  statusCharacteristic->setValue(status.c_str());
  statusCharacteristic->notify();
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    notifyStatus(true);
  }

  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    startJog(0);
    server->startAdvertising();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    const std::string value = characteristic->getValue();
    handleCommand(String(value.c_str()));
    notifyStatus(true);
  }
};

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  BLECharacteristic* commandCharacteristic = service->createCharacteristic(
      COMMAND_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  commandCharacteristic->setCallbacks(new CommandCallbacks());

  statusCharacteristic = service->createCharacteristic(
      STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  statusCharacteristic->addDescriptor(new BLE2902());
  statusCharacteristic->setValue(buildStatusJson().c_str());

  service->start();

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  pinMode(MS3, OUTPUT);

  setMicrostep(microstepMode);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP);

  if (stepper == nullptr) {
    Serial.println("[ERROR] stepperConnectToPin failed");
  } else {
    stepper->setDirectionPin(DIR, true);
    stepper->setEnablePin(EN, true);
    stepper->setAutoEnable(false);
    stepper->enableOutputs();
    applyMotionParams();
  }

  setupBle();
  Serial.println("[SYSTEM] BLE motor controller started");
}

void loop() {
  notifyStatus();
}
