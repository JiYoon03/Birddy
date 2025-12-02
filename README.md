# ESP32 Sensor System

This project reads data from a MAX30105 pulse oximeter sensor and a GSR (Galvanic Skin Response) sensor using an ESP32. The sensor data is transmitted over UART in JSON format for further processing or logging.

## Hardware Required

* ESP32 DevKit
* MAX30105 Pulse Oximeter & Heart Rate Sensor
* GSR Sensor (connected to analog pin 34)
* Jumper wires

### Pin Connections

| Component         | ESP32 Pin |
| ----------------- | --------- |
| MAX30105 SDA      | 21        |
| MAX30105 SCL      | 22        |
| GSR Sensor Output | 34 (ADC)  |
| UART TX (ESP32)   | 4         |
| UART RX (ESP32)   | 5         |

> Note: The MAX30105 uses I2C communication and the GSR sensor uses analog input.

## Software Requirements

* Arduino IDE or PlatformIO
* ESP32 board package installed
* Libraries:

  * MAX30105
  * `Wire` (built-in)
  * `spo2_algorithm.h` (for SPO2 calculations)

## Installation

1. Clone this repository or copy the code.
2. Open in Arduino IDE.
3. Select the correct ESP32 board and port.
4. Install required libraries (`MAX30105` and `spo2_algorithm`).
5. Upload the code to your ESP32.

## Code Overview

* **Initialization**

  * Serial communication at 115200 baud.
  * I2C bus initialized on pins 21 (SDA) and 22 (SCL).
  * MAX30105 sensor initialized and pulse amplitudes set for red and IR LEDs.
* **Loop**

  1. Check if new data is available in the MAX30105 FIFO buffer.
  2. Read IR and red light values from MAX30105.
  3. Read analog GSR value.
  4. Calculate approximate heart rate (HR) and SPO2 using `map()` as a placeholder.
  5. Send all sensor data over UART in JSON format:

```json
{
  "ir": 34567,
  "red": 45678,
  "hr": 78,
  "spo2": 96,
  "gsr": 512
}
```

* **Delay**: 100 ms between readings.

## Usage

1. Connect ESP32 to your computer or microcontroller.
2. Open Serial Monitor (115200 baud) or connect another MCU to receive the UART JSON output.
3. Observe real-time readings for IR, red, HR, SPO2, and GSR.

## Notes

* The `map()` function for HR and SPO2 is a simple linear mapping for demonstration. For accurate results, implement proper signal processing using `spo2_algorithm.h`.
* Ensure sufficient power to the MAX30105 sensor (3.3V recommended).
* JSON format is easy to parse for logging, visualization, or transmission over wireless interfaces.

# Birddy — ESP32 Audio Companion Firmware

Birddy is an interactive ESP32-based companion system that monitors heart rate (HR), SpO₂, and GSR signals from sensors and responds with audio feedback. It supports evening music, emergency alerts, and button-based interactions. Audio files are stored on an SD card and playback is handled via I2S.

## Features

* Plays WAV audio from SD card using `AudioGeneratorWAV`.
* Connects to WiFi and fetches time via NTP for evening music scheduling.
* Monitors sensor data (HR, SpO₂, GSR) over UART.
* Detects stress, fatigue, and abnormal conditions.
* Emergency prompt:

  * Single button click → "Family" alert
  * Double button click → Cancel emergency
* Plays siren and repeats emergency audio if unacknowledged.
* Non-blocking audio playback, allowing simultaneous sensor monitoring and button handling.
* Supports evening relaxation music (Gospel, Jazz, RnB) based on time and stress/fatigue.

---

## Hardware Requirements

* ESP32 DevKit
* MAX30105 or MAX30102 sensor (HR + SpO₂)
* GSR Sensor
* Push button
* SD card module (microSD)
* I2S audio DAC / amplifier (e.g., MAX98357A)
* Jumper wires

### Pin Connections

| Component             | ESP32 Pin |
| --------------------- | --------- |
| SD CS                 | 13        |
| I2S BCLK (SCK)        | 26        |
| I2S LRC (WS)          | 25        |
| I2S DOUT (DIN)        | 22        |
| Button (INPUT_PULLUP) | 14        |
| Sensor UART RX        | 4         |
| Sensor UART TX        | 5         |
| MAX30105 SDA          | 21        |
| MAX30105 SCL          | 22        |
| GSR analog input      | 34        |

> Notes: Adjust pins as needed for your hardware setup.

---

## Software Requirements

* Arduino IDE or PlatformIO
* ESP32 board package installed
* Libraries:

  * Audio by Earle F. Philhower (`AudioGeneratorWAV`, `AudioFileSourceSD`, `AudioOutputI2S`)
  * `WiFi` (built-in)
  * `WiFiUdp` (built-in)
  * `NTPClient`
  * `FS`, `SD`, `SPI` (built-in)

---

## SD Card File Structure

All WAV files should be in `/audio` folder on the SD card:

```
/audio/
  prompt.wav       // Emergency confirmation prompt
  family.wav       // Single click alert
  check.wav        // Cancel emergency
  siren.wav        // Siren audio
  emergency.wav    // Repeating emergency message
  stress.wav       // Stress alert
  tired.wav        // Low SpO2 alert
  calm.wav         // Normal/relaxed audio
  normal.wav       // Default normal audio
  gospel.wav       // Evening music
  jazz.wav         // Evening music
  rnb.wav          // Evening music
```

---

## Firmware Overview

### System Modes

* **MODE_NORMAL**: Regular monitoring and normal audio playback.
* **MODE_EMERGENCY_AWAIT**: Triggered on abnormal sensor readings. Plays prompt, waits for button confirmation.
* **MODE_EMERGENCY_BROADCAST**: Plays siren and repeats emergency message until canceled.

### Button Interaction

* **Single click** → Play `family.wav`.
* **Double click** → Cancel emergency and play `check.wav`.

### Sensor Monitoring

* Receives JSON data over UART from sensor ESP32:

```json
{"ir":34567,"red":45678,"hr":78,"spo2":96,"gsr":512}
```

* Triggers audio based on thresholds:

  * Stress: HR > 100 & GSR > 2500
  * Fatigue: SpO₂ < 92
* Evening music (Gospel/Jazz/RnB) plays if time is 18:00–23:00 and stress/fatigue detected.

### Audio Handling

* Non-blocking WAV playback via I2S.
* Automatically stops and frees resources after each track.
* Emergency audio loops indefinitely until canceled.

### NTP and Time Handling

* WiFi connected → fetches time from `pool.ntp.org`.
* Used to determine evening music conditions.
* Falls back to last known time if WiFi is unavailable.

---

## Usage

1. Insert SD card with `/audio` WAV files.
2. Connect ESP32 and sensors as per pin table.
3. Upload firmware using Arduino IDE or PlatformIO.
4. Ensure WiFi credentials are correct for NTP time fetch.
5. Open Serial Monitor for debugging.
6. System automatically monitors sensors, plays audio, and handles emergencies.

---

## Customization

* Adjust sensor thresholds: `STRESS_HR_THRESHOLD`, `STRESS_GSR_THRESHOLD`, `SPO2_THRESHOLD`.
* Change button debounce and click timing: `DEBOUNCE_MS`, `CLICK_WINDOW_MS`.
* Update SD card paths to match your audio filenames.
* Adjust emergency timeout: `EMERGENCY_TIMEOUT_MS`.

---

## Debugging

* Serial Monitor prints sensor readings and button presses.
* Audio status messages indicate playback start/stop.
* Emergency events are logged to Serial Monitor.
