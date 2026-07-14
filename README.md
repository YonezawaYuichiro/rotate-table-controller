# XIAO ESP32S3 Turntable Controller (USB Serial + BLE HID Shutter)

回転台フォトグラメトリ用のコントローラです。**制御はPCからUSBシリアル**で行い、
**シャッターはESP32からBLE HID（音量+キー）でiPhoneへ**送ります。
「回す→止まる→整定待ち→シャッター→撮影待ち」を自動で1周ぶん繰り返します。

- PC ──USB(シリアル)──> ESP32 ──BLE HID(音量+)──> iPhone(カメラ)

BLEはiPhoneへのシャッター送信専用（HID 1役）です。カスタムGATTサービスは廃止し、
モーター制御コマンドはUSBシリアルの1行テキストで受け取ります。二役BLEを避けることで、
`BLEDevice::createServer()` の二重生成でGATTサーバーが壊れる問題（Arduino ESP32 2.x）を回避しています。

## 構成

- `src/main.cpp`: ファームウェア（USBシリアル受信＋自動撮影ステートマシン）
- `lib/HidShutter/`: 既存BLEサーバーに同居する最小のBLE HID（音量+のみ）
- `web/index.html`: Web Serialで動くPC用コントロールUI（Chrome / Edge デスクトップ版）
- `index.html`: GitHub Pagesのルートから`web/`へ移動するための入口

## iPhone側の準備

1. 「設定 > Bluetooth」で `XIAO Turntable`（DevKitは `ESP32 Turntable`）をキーボードとしてペアリング
2. 撮影時は標準カメラ（またはKIRI）を前面に出しておく（音量+でシャッターが切れる）

## コマンド仕様（USBシリアル / 115200bps / 改行区切りのASCII）

```text
C,<microstep>,<d_us>,<steps>,<accel>   設定変更
V,<signed_speed_hz>,<accel>            ジョグ（正=CW / 負=CCW / 0=減速停止）
M,<signed_steps>                       指定ステップ移動
A,<frames>,<settle_ms>,<post_ms>       自動撮影開始（1周を frames 等分）
S                                      停止（自動撮影も中断）
E,<0|1>                                励磁 OFF/ON
R                                      現在位置を0にリセット
```

自動撮影の1周は `200 × microstep` ステップを `frames` で等分します（丸め誤差は累積しないよう補正）。
ステータスはESP32から1行JSONでシリアルに流れ、UIがそれを読んで表示します。

## Web Serial UI

Web SerialはHTTPSまたはlocalhost上のChrome / Edge（デスクトップ版）で動作します。
GitHub Pagesに公開したURLへアクセスし、「USB接続」でESP32のシリアルポートを選んでください。
速度入力は `steps/s` に統一（内部 `d_us = 500000 / steps_per_second` で自動換算）。

## ビルド

```powershell
platformio run -e seeed_xiao_esp32s3
platformio run -e esp32dev
```

書き込み:

```powershell
platformio run -e seeed_xiao_esp32s3 --target upload
platformio run -e esp32dev --target upload
```

## ピン割り当て

| Signal | XIAO ESP32S3 | ESP32 DevKit |
| --- | --- | --- |
| DIR | D8 | GPIO26 |
| STEP | D9 | GPIO25 |
| MS1 | D0 | GPIO14 |
| MS2 | D1 | GPIO27 |
| MS3 | D2 | GPIO33 |
| EN | D3 | GPIO32 |

ESP32 DevKitで別のGPIOを使う場合は、`platformio.ini` の `[env:esp32dev]` の `MOTOR_PIN_*` を変更してください。
