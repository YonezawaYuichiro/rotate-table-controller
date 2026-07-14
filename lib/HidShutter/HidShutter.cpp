#include "HidShutter.h"

#include <BLE2902.h>
#include <BLEAdvertising.h>
#include <BLESecurity.h>
#include <BLEUUID.h>
#include <HIDTypes.h>

namespace {

// レポートIDと音量+ のビット位置。
// 標準的なメディアキー配列に合わせてあり、Volume Increment は
// 1バイト目の bit5（0x20）に対応する。
constexpr uint8_t MEDIA_KEYS_ID = 0x01;

// Consumer Control：16個の1ビットusageを持つ入力レポート。
// 中身は一般的なメディアキー配列と同一（ビット位置を標準に揃えるため）。
const uint8_t HID_REPORT_MAP[] = {
    USAGE_PAGE(1),      0x0C,           // Consumer
    USAGE(1),           0x01,           // Consumer Control
    COLLECTION(1),      0x01,           // Application
    REPORT_ID(1),       MEDIA_KEYS_ID,  //
    USAGE_PAGE(1),      0x0C,           //
    LOGICAL_MINIMUM(1), 0x00,           //
    LOGICAL_MAXIMUM(1), 0x01,           //
    REPORT_SIZE(1),     0x01,           //
    REPORT_COUNT(1),    0x10,           // 16bit
    USAGE(1),           0xB5,           // bit0  Scan Next Track
    USAGE(1),           0xB6,           // bit1  Scan Previous Track
    USAGE(1),           0xB7,           // bit2  Stop
    USAGE(1),           0xCD,           // bit3  Play/Pause
    USAGE(1),           0xE2,           // bit4  Mute
    USAGE(1),           0xE9,           // bit5  Volume Increment  <-- シャッター
    USAGE(1),           0xEA,           // bit6  Volume Decrement
    USAGE(2),           0x23, 0x02,     // bit7  WWW Home
    USAGE(2),           0x94, 0x01,     // bit8  My Computer
    USAGE(2),           0x92, 0x01,     // bit9  Calculator
    USAGE(2),           0x2A, 0x02,     // bit10 WWW fav
    USAGE(2),           0x21, 0x02,     // bit11 WWW search
    USAGE(2),           0x26, 0x02,     // bit12 WWW stop
    USAGE(2),           0x24, 0x02,     // bit13 WWW back
    USAGE(2),           0x83, 0x01,     // bit14 Media select
    USAGE(2),           0x8A, 0x01,     // bit15 Mail
    HIDINPUT(1),        0x02,           // Data,Var,Abs
    END_COLLECTION(0)};

void setCccdNotifications(BLECharacteristic* input, bool enable) {
  if (input == nullptr) {
    return;
  }
  BLE2902* cccd =
      static_cast<BLE2902*>(input->getDescriptorByUUID(BLEUUID(static_cast<uint16_t>(0x2902))));
  if (cccd != nullptr) {
    cccd->setNotifications(enable);
  }
}

}  // namespace

void HidShutter::begin(BLEServer* server) {
  m_hid = new BLEHIDDevice(server);
  m_input = m_hid->inputReport(MEDIA_KEYS_ID);

  m_hid->manufacturer("Hutzper");
  // iOS に受け入れられやすいよう Apple 系の VID/PID を名乗る（BleKeyboard と同値）
  m_hid->pnp(0x02, 0x05ac, 0x820a, 0x0210);
  m_hid->hidInfo(0x00, 0x01);

  // HID over BLE は iOS ではボンディング必須
  BLESecurity* security = new BLESecurity();
  security->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_MITM_BOND);

  m_hid->reportMap(const_cast<uint8_t*>(HID_REPORT_MAP), sizeof(HID_REPORT_MAP));
  m_hid->startServices();

  // アドバタイズには HID を追記するだけ。開始は呼び出し側（main）が
  // モーター用サービスUUIDを足したうえで1回だけ行う。
  BLEAdvertising* advertising = server->getAdvertising();
  advertising->setAppearance(HID_KEYBOARD);
  advertising->addServiceUUID(m_hid->hidService()->getUUID());

  m_hid->setBatteryLevel(100);
}

void HidShutter::handleConnect() {
  m_connected = true;
  setCccdNotifications(m_input, true);
}

void HidShutter::handleDisconnect() {
  m_connected = false;
  setCccdNotifications(m_input, false);
}

void HidShutter::pressVolumeUp() {
  if (!m_connected || m_input == nullptr) {
    return;
  }
  uint8_t report[2] = {0x20, 0x00};  // bit5 = Volume Increment
  m_input->setValue(report, sizeof(report));
  m_input->notify();
}

void HidShutter::release() {
  if (!m_connected || m_input == nullptr) {
    return;
  }
  uint8_t report[2] = {0x00, 0x00};
  m_input->setValue(report, sizeof(report));
  m_input->notify();
}
