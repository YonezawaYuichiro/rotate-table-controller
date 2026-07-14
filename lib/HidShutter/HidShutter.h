#ifndef HID_SHUTTER_H
#define HID_SHUTTER_H

#include <BLEServer.h>
#include <BLEHIDDevice.h>
#include <BLECharacteristic.h>

// 既存の BLEServer にぶら下がる最小の BLE HID デバイス。
// 「音量+（Volume Increment）」だけを送れる Consumer Control で、
// iPhone のカメラでは音量ボタン＝シャッターの仕様を使って撮影する。
//
// モーター制御用の独自 GATT サービスと同じサーバー上に同居させるため、
// 自前で BLEDevice::init / createServer は呼ばない（呼ぶとサーバーが
// 二重生成され、GATT イベントが片方にしか配送されず壊れる）。
class HidShutter {
 public:
  void begin(BLEServer* server);

  // サーバーの onConnect / onDisconnect から呼ぶ
  void handleConnect();
  void handleDisconnect();
  bool isConnected() const { return m_connected; }

  void pressVolumeUp();  // 音量+ ビットを立てて notify（＝シャッター押下）
  void release();        // 全ビットを落として notify（＝離す）

 private:
  BLEHIDDevice* m_hid = nullptr;
  BLECharacteristic* m_input = nullptr;
  bool m_connected = false;
};

#endif  // HID_SHUTTER_H
