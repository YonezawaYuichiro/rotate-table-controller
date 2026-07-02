# XIAO ESP32S3 BLE Stepper Controller

ESP32をBLEペリフェラルにして、Web Bluetooth対応ブラウザからA4988経由のステップモータを制御するサンプルです。

## 構成

- `src/main.cpp`: XIAO ESP32S3向けPlatformIOファームウェア
- `web/index.html`: GitHub PagesなどHTTPS上で動くWeb Bluetoothアプリ
- `index.html`: GitHub Pagesのルートから`web/`へ移動するための入口

## BLE仕様

Device name: `XIAO BLE Motor`

Service UUID: `7b7f0001-9b6d-4f8b-8c5d-9bb6f6f68c01`

Characteristics:

- Command Write: `7b7f0002-9b6d-4f8b-8c5d-9bb6f6f68c01`
- Status Notify: `7b7f0003-9b6d-4f8b-8c5d-9bb6f6f68c01`

コマンドはASCIIテキストです。

```text
C,<microstep>,<d_us>,<steps>,<accel>
V,<signed_speed_hz>,<accel>
M,<signed_steps>
S
E,<0|1>
R
```

`V`はジョグ/速度指令です。正値がCW、負値がCCW、0が減速停止です。

## Web Bluetooth

Web BluetoothはHTTPSまたはlocalhost上でのみ動作します。GitHub Pagesに公開したURLへChromeまたはEdgeでアクセスしてください。iOS SafariはWeb Bluetoothに対応していません。

## ビルド

```powershell
platformio run
```

書き込み:

```powershell
platformio run --target upload
```
