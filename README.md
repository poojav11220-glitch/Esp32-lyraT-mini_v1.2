# ESP32 LyraT-Mini v1.2 — Project Examples

A collection of hands-on ESP-IDF v6.0.2 projects built and tested on the **ESP32 LyraT-Mini v1.2** development board.

---

## Board Overview

| Feature | Detail |
|---|---|
| SoC | ESP32 (dual-core Xtensa LX6, 240 MHz) |
| Flash | 8 MB |
| Audio DAC | ES8311 → headphone jack |
| Audio ADC | ES7243 → onboard microphone |
| I2S0 | MCLK=GPIO5, BCLK=GPIO25, WS=GPIO26 |
| I2S1 | MCLK=GPIO0, BCK=GPIO32, WS=GPIO33, DIN=GPIO36 |
| I2C | SDA=GPIO18, SCL=GPIO23 |
| Green LED | GPIO22 |
| Blue LED | GPIO27 |
| PA Enable | GPIO21 |
| Buttons | PLAY=GPIO34, SET=GPIO36, VOL+=GPIO38, VOL-=GPIO37, REC=GPIO35 |
| BOOT button | GPIO0 |
| JTAG header | 3V3, GND, TX0, RX0, TMS, TDI, TCLK, TDO |

---

## Projects

| # | Project | Description |
|---|---|---|
| 1 | [`Led_blink`](#1-led_blink) | Basic GPIO LED blink |
| 2 | [`button_led_toggle`](#2-button_led_toggle) | Button toggles LED via interrupt |
| 3 | [`bt_speaker`](#3-bt_speaker) | Bluetooth A2DP Sink — phone audio to headphones |
| 4 | [`ble_uart`](#4-ble_uart) | BLE UART terminal (Nordic UART Service) |
| 5 | [`deep_sleep`](#5-deep_sleep) | Deep sleep + wake on button + NVS wake counter |
| 6 | [`mic_test`](#6-mic_test) | Microphone loopback echo + PCM music playback |
| 7 | [`oled_lyra`](#7-oled_lyra) | OLED display with bit-bang I2C |
| 8 | [`Real_time`](#8-real_time) | DS1302 RTC — read date and time |
| 9 | [`Button_Music`](#9-button_music) | Button-triggered audio via ES8311 DAC |
| 10 | [`ledc_example`](#10-ledc_example) | Hardware PWM LED fade (LEDC peripheral) |
| 11 | [`uart_custom_echo`](#11-uart_custom_echo) | UART echo with custom protocol |
| 12 | [`wifi_station`](#12-wifi_station) | Connect to WiFi access point |

---

## Build & Flash

```bash
cd <project_folder>
idf.py build
idf.py -p COM3 flash monitor
```

**Requirements:**
- ESP-IDF v6.0.2
- CMake 3.16+
- Serial port: COM3 (adjust as needed)
- Baud rate: 115200

---

## Project Details

### 1. `Led_blink`
Blinks an LED on GPIO2 using `vTaskDelay`.  
**Concepts:** GPIO output, FreeRTOS delay  
**Expected output:** LED blinks every 1 second

---

### 2. `button_led_toggle`
Press **BOOT button (GPIO0)** → Green LED (GPIO22) toggles ON/OFF on every press.  
**Concepts:** GPIO interrupt (NEGEDGE), ISR, FreeRTOS queue, 50ms debounce  
**Expected serial output:**
```
Ready — press Boot button to toggle LED
Button pressed! LED = ON
Button pressed! LED = OFF
```

---

### 3. `bt_speaker`
ESP32 acts as a **Bluetooth A2DP Sink** — pair your phone and stream music to headphones.  
**Concepts:** Classic Bluetooth, A2DP, AVRC volume, ES8311 DAC, I2S, custom partition table  
**Hardware:** Plug headphones into the audio jack  
**Expected:** Phone pairs as "LyraT BT Speaker", music plays through headphones  
> Requires `partitions.csv` (3MB factory app) for large BT binary

---

### 4. `ble_uart`
BLE GATT server implementing **Nordic UART Service (NUS)**.  
Phone connects via **nRF Connect** app and sends/receives text commands.  
**Concepts:** BLE GATT, NUS UUIDs, notifications, CCCD, FreeRTOS  
**Commands available:**

| Command | Response |
|---|---|
| `hello` | Hello from LyraT Mini! |
| `ping` | pong |
| `uptime` | Time since boot |
| `mem` | Free heap memory |
| `chip` | Chip model, revision, cores |
| `status` | Board / IDF info |
| `echo <text>` | Echoes text back |
| `repeat <n> <text>` | Sends text n times (max 20) |
| `reset` | Restarts the board |
| `help` | Command list |

---

### 5. `deep_sleep`
Board enters **deep sleep** (~10µA), wakes when BOOT button pressed.  
Wake count saved in **NVS** — persists through power cuts.  
**LED behaviour:** Green ON (awake) → Blue blinks 3× (warning) → Both OFF (sleeping)  
**Concepts:** `esp_deep_sleep_start()`, ext0 wakeup, NVS read/write  
**Expected serial output:**
```
Boot #1  Wake reason: Power-on / RST  — Going to sleep in 3s...
Boot #2  Wake reason: BOOT button (GPIO0) — Going to sleep in 3s...
```

---

### 6. `mic_test`
Two modes selectable in `app_main()`:  
- **play_task** — plays embedded `canon.pcm` through headphones on loop  
- **echo_task** — captures mic (ES7243) and plays back through DAC (ES8311) with 2× gain  

**Concepts:** I2S duplex, ES8311 DAC, ES7243 ADC, `esp_codec_dev`, DMA audio pipeline  
**Hardware:** Plug headphones into audio jack  
**VU meter serial output (echo mode):**
```
MIC |########.................| level=8432
MIC |###......................| level=2100
```

---

### 7. `oled_lyra`
Displays text and a live counter on an **SH1106/SSD1306 OLED** display.  
Written entirely from scratch — no external library, pure bit-bang I2C.  
**Concepts:** Bit-bang I2C (GPIO13=SDA, GPIO14=SCL), OLED init sequence, 5×7 font  
**Hardware:** External OLED module wired to GPIO13 (SDA) and GPIO14 (SCL)  
**Display output:**
```
Hello World!
ESP32 LyraT
Counter:
42
```

---

### 8. `Real_time`
Reads date and time from a **DS1302 RTC module** every second.  
Auto-detects first boot using the CH (Clock Halt) bit — sets time from compile timestamp.  
Battery backup keeps time running when ESP32 is off.  
**Concepts:** DS1302 3-wire serial bit-bang, BCD encoding, RTC CH bit  
**Wiring:**

| DS1302 | LyraT-Mini JTAG |
|---|---|
| VCC | 3V3 |
| GND | GND |
| CLK | TCLK (GPIO13) |
| DAT | TDO (GPIO15) |
| RST | TMS (GPIO14) |

**Expected serial output:**
```
I (270) RTC: 2026-07-04   10:22:35
I (1270) RTC: 2026-07-04   10:22:36
```

---

### 9. `Button_Music`
Plays audio tones/music through the ES8311 DAC triggered by button presses.  
**Concepts:** GPIO buttons, I2S audio output, ES8311 codec, tone generation  
**Hardware:** Headphones on audio jack

---

### 10. `ledc_example`
Smoothly fades an LED using the ESP32 **LEDC hardware PWM** peripheral.  
**Concepts:** LEDC timer, channel config, duty cycle, fade interrupt  
**Expected:** LED fades in and out smoothly

---

### 11. `uart_custom_echo`
UART echo — receives data on UART0 and sends it back.  
**Concepts:** UART driver, ring buffer, event queue, custom framing  
**Expected serial output:** Whatever you type is echoed back

---

### 12. `wifi_station`
Connects ESP32 to a WiFi access point as a station.  
**Concepts:** WiFi station mode, ESP event loop, IP assignment, reconnection  
**Expected serial output:**
```
Connected to AP, IP: 192.168.1.xxx
```

---

## Folder Structure

Each project follows standard ESP-IDF layout:

```
<project>/
├── CMakeLists.txt          # Project build config
├── sdkconfig.defaults      # Default Kconfig settings
├── partitions.csv          # Custom partition table (if needed)
├── main/
│   ├── CMakeLists.txt      # Component registration
│   ├── idf_component.yml   # Managed component dependencies
│   └── <main>.c            # Application source
└── README.md               # Project-specific notes
```

---

## Author

**Pooja** — ESP32 LyraT-Mini v1.2 learning projects  
Board: ESP32 LyraT-Mini v1.2 | IDF: ESP-IDF v6.0.2
