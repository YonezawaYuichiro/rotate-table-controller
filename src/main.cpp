#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <FastAccelStepper.h>
#include <math.h>

#include "HidShutter.h"

namespace {

#ifndef MOTOR_PIN_DIR
#error "MOTOR_PIN_DIR is not defined. Set board-specific motor pins in platformio.ini."
#endif
#ifndef MOTOR_PIN_STEP
#error "MOTOR_PIN_STEP is not defined. Set board-specific motor pins in platformio.ini."
#endif
#ifndef MOTOR_PIN_MS1
#error "MOTOR_PIN_MS1 is not defined. Set board-specific motor pins in platformio.ini."
#endif
#ifndef MOTOR_PIN_MS2
#error "MOTOR_PIN_MS2 is not defined. Set board-specific motor pins in platformio.ini."
#endif
#ifndef MOTOR_PIN_MS3
#error "MOTOR_PIN_MS3 is not defined. Set board-specific motor pins in platformio.ini."
#endif
#ifndef MOTOR_PIN_EN
#error "MOTOR_PIN_EN is not defined. Set board-specific motor pins in platformio.ini."
#endif

// iPhone の Bluetooth 設定に「キーボード」として現れる名前。
// 制御は USB シリアルで行い、BLE は iPhone へのシャッター送信(HID)専用。
#if defined(MOTOR_BOARD_XIAO_ESP32S3)
constexpr char DEVICE_NAME[] = "XIAO Turntable";
#elif defined(MOTOR_BOARD_ESP32DEV)
constexpr char DEVICE_NAME[] = "ESP32 Turntable";
#else
constexpr char DEVICE_NAME[] = "Turntable Shutter";
#endif

constexpr int DIR = MOTOR_PIN_DIR;
constexpr int STEP = MOTOR_PIN_STEP;
constexpr int MS1 = MOTOR_PIN_MS1;
constexpr int MS2 = MOTOR_PIN_MS2;
constexpr int MS3 = MOTOR_PIN_MS3;
constexpr int EN = MOTOR_PIN_EN;

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

uint32_t lastStatusMs = 0;

FastAccelStepperEngine engine;
FastAccelStepper* stepper = nullptr;
HidShutter shutter;

// ---- 自動撮影ステートマシン --------------------------------------------
// 1周を autoFrames 分割し、各角度で「止まる→揺れが収まるまで待つ→
// シャッター→撮影完了待ち→次角度」を BLE を止めずに（非ブロッキングで）回す。
enum AutoPhase {
  AP_IDLE,    // 停止中
  AP_SETTLE,  // 停止位置で振動が収まるのを待つ
  AP_HOLD,    // シャッター押下を保持
  AP_POST,    // 撮影・保存の完了を待つ
  AP_MOVE,    // 次角度へ移動中
};

constexpr uint32_t SHUTTER_HOLD_MS = 60;      // シャッター押下の保持時間
constexpr uint32_t AUTO_MIN_MOVE_MS = 20;     // 移動完了判定前の最小待ち
constexpr uint32_t AUTO_MAX_DELAY_MS = 20000;  // settle/post の上限

AutoPhase autoPhase = AP_IDLE;
int autoFrames = 0;            // 総撮影枚数（1周の分割数）
int autoIndex = 0;            // 撮影済み枚数
uint32_t autoSettleMs = 500;  // 停止後の整定待ち
uint32_t autoPostMs = 1500;   // シャッター後の撮影・保存待ち
uint32_t autoPhaseStart = 0;  // 現フェーズの開始時刻
long autoTakenSteps = 0;      // 開始位置からの累積移動ステップ（ドリフト補正用）

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

void notifyStatus(bool force);

void stopAutomation() {
  autoPhase = AP_IDLE;
  shutter.release();
}

void startAutomation(int frames, uint32_t settleMs, uint32_t postMs) {
  if (stepper == nullptr || frames < 1) {
    return;
  }

  if (!motorEnabled) {
    motorEnabled = true;
    stepper->enableOutputs();
  }
  if (stepper->isRunning()) {
    stepper->forceStop();
    delay(5);
  }

  jogSpeedHz = 0;
  autoFrames = frames;
  autoIndex = 0;
  autoTakenSteps = 0;
  autoSettleMs = clampInt32(settleMs, 0, AUTO_MAX_DELAY_MS);
  autoPostMs = clampInt32(postMs, 0, AUTO_MAX_DELAY_MS);
  applyMotionParams();

  // 開始位置でまず整定してから1枚目を撮る
  autoPhase = AP_SETTLE;
  autoPhaseStart = millis();
}

void updateAutomation() {
  if (autoPhase == AP_IDLE || stepper == nullptr) {
    return;
  }

  const uint32_t now = millis();

  switch (autoPhase) {
    case AP_SETTLE:
      if (now - autoPhaseStart >= autoSettleMs) {
        shutter.pressVolumeUp();
        autoPhaseStart = now;
        autoPhase = AP_HOLD;
      }
      break;

    case AP_HOLD:
      if (now - autoPhaseStart >= SHUTTER_HOLD_MS) {
        shutter.release();
        autoIndex++;
        autoPhaseStart = now;
        autoPhase = AP_POST;
      }
      break;

    case AP_POST:
      if (now - autoPhaseStart >= autoPostMs) {
        if (autoIndex >= autoFrames) {
          autoPhase = AP_IDLE;  // 1周完了
          notifyStatus(true);
        } else {
          // 1周（200 * microstep ステップ）を frames 等分。丸め誤差が
          // 累積しないよう、毎回「累積目標との差分」を移動する。
          const long stepsPerRev = 200L * static_cast<long>(microstepMode);
          const long targetNow =
              lround(static_cast<double>(stepsPerRev) * autoIndex / autoFrames);
          const long delta = targetNow - autoTakenSteps;
          autoTakenSteps = targetNow;
          if (delta != 0) {
            applyMotionParams();
            stepper->move(delta);
          }
          autoPhaseStart = now;
          autoPhase = AP_MOVE;
        }
      }
      break;

    case AP_MOVE:
      if (now - autoPhaseStart >= AUTO_MIN_MOVE_MS && !stepper->isRunning()) {
        autoPhaseStart = now;
        autoPhase = AP_SETTLE;
      }
      break;

    default:
      break;
  }
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
    case 'A': {
      // A,frames,settleMs,postMs … 1周を frames 分割して自動撮影開始
      const int frames = readCsvInt(command, 1, 0);
      const uint32_t settleMs = static_cast<uint32_t>(readCsvInt(command, 2, autoSettleMs));
      const uint32_t postMs = static_cast<uint32_t>(readCsvInt(command, 3, autoPostMs));
      startAutomation(frames, settleMs, postMs);
      break;
    }
    case 'S': {
      stopAutomation();
      jogSpeedHz = 0;
      stepper->forceStop();
      break;
    }
    case 'P': {
      // 単発シャッターテスト：音量+を押して離す（切り分け用）
      shutter.pressVolumeUp();
      delay(SHUTTER_HOLD_MS);
      shutter.release();
      break;
    }
    case 'E': {
      motorEnabled = readCsvInt(command, 1, 1) != 0;
      if (motorEnabled) {
        stepper->enableOutputs();
      } else {
        stopAutomation();
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
      Serial.print("[CMD] Unknown command: ");
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
  json += "\"enabled\":" + String(motorEnabled ? "true" : "false") + ",";
  json += "\"hidReady\":" + String(shutter.isConnected() ? "true" : "false") + ",";
  json += "\"auto\":" + String(autoPhase != AP_IDLE ? "true" : "false") + ",";
  json += "\"frame\":" + String(autoIndex) + ",";
  json += "\"frames\":" + String(autoFrames);
  json += "}";
  return json;
}

// ステータスは USB シリアルに1行JSONで出す。PC側UI(Web Serial)がこれを読む。
void notifyStatus(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - lastStatusMs < STATUS_INTERVAL_MS) {
    return;
  }

  lastStatusMs = now;
  Serial.println(buildStatusJson());
}

// BLE は iPhone への HID シャッター専用。接続してくるセントラルは iPhone だけ
// なので、onConnect/onDisconnect はそのまま「シャッター相手の状態」を表す。
class HidServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    shutter.handleConnect();
    notifyStatus(true);
  }

  void onDisconnect(BLEServer* server) override {
    shutter.handleDisconnect();
    server->startAdvertising();  // 再接続を受け付ける
    // 制御は USB シリアル側なので、iPhone が切れても自動撮影は止めない
    // （モーターは回り続け、シャッターだけ空振りになる）。
    notifyStatus(true);
  }
};

// USB シリアルから1行ずつコマンドを受け取る（改行区切り）。
String serialLine;

void pollSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        handleCommand(serialLine);
        serialLine = "";
        notifyStatus(true);
      }
    } else {
      serialLine += c;
      if (serialLine.length() > 128) {  // 暴走・化け対策
        serialLine = "";
      }
    }
  }
}

void setupHid() {
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);

  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new HidServerCallbacks());

  // HID サービス＋アドバタイズ設定は shutter.begin が行う
  shutter.begin(server);

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
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

  setupHid();
  Serial.println("[SYSTEM] Turntable controller started (USB serial + BLE HID shutter)");
}

void loop() {
  pollSerial();
  updateAutomation();
  notifyStatus();
}
