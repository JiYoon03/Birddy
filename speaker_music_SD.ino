// Birddy — Full Firmware (AudioGeneratorWAV + SD + NTP evening music + Button-based emergency)
// Libraries required:
//   - Audio by Earle F. Philhower (AudioGeneratorWAV, AudioFileSourceSD, AudioOutputI2S)
//   - NTPClient
//   - WiFi

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include <AudioGeneratorWAV.h>
#include <AudioFileSourceSD.h>
#include <AudioOutputI2S.h>

// ---------------- Configuration ----------------
#define SD_CS        13     // SD CS pin (change if needed)
#define I2S_BCLK     26     // I2S BCLK (SCK)
#define I2S_LRC      25     // I2S LRC (WS)
#define I2S_DOUT     22     // I2S Data out (DIN)
#define BTN_PIN      14     // Button pin (INPUT_PULLUP)
#define UART_RX      4      // Sensor UART RX (to ESP32 RX)
#define UART_TX      5      // Sensor UART TX (to ESP32 TX)

HardwareSerial SensorSerial(2);

// WiFi / NTP
const char* WIFI_SSID = "yourSSID";
const char* WIFI_PASS = "yourPW";
WiFiUDP ntpUDP;
// Use local offset seconds, example -5*3600 for EST. Set to your timezone.
NTPClient timeClient(ntpUDP, "pool.ntp.org", -5 * 3600, 60000);

// Audio objects (dynamic to support non-blocking)
AudioGeneratorWAV *wav = nullptr;
AudioFileSourceSD *file = nullptr;
AudioOutputI2S *out = nullptr;
bool isPlaying = false;
const uint32_t AUDIO_LOOP_DELAY_MS = 1; // audio loop frequency (handled in main loop)

// ---------------- Filenames on SD (/audio/...) ----------------
const char* PATH_PROMPT    = "/audio/prompt.wav";     // "Are you okay? Press once for Family. Press twice to cancel."
const char* PATH_FAMILY    = "/audio/family.wav";
const char* PATH_CHECK     = "/audio/check.wav";      // cancel emergency
const char* PATH_SIREN     = "/audio/siren.wav";
const char* PATH_EMERGENCY = "/audio/emergency.wav";  // repeating emergency 
const char* PATH_STRESS    = "/audio/stress.wav";
const char* PATH_TIRED     = "/audio/tired.wav";
const char* PATH_CALM      = "/audio/calm.wav";
const char* PATH_NORMAL    = "/audio/normal.wav";
const char* PATH_GOSPEL    = "/audio/gospel.wav";
const char* PATH_JAZZ      = "/audio/jazz.wav";
const char* PATH_RNB       = "/audio/rnb.wav";

// ---------------- System  ----------------
enum SystemMode { MODE_NORMAL=0, MODE_EMERGENCY_AWAIT, MODE_EMERGENCY_BROADCAST };
SystemMode systemMode = MODE_NORMAL;

unsigned long emergencyStart = 0;
const unsigned long EMERGENCY_TIMEOUT_MS = 60000UL; // 60 s

bool sirenPlayed = false;
bool emergencyLooping = false;

// ---------------- Sensor thresholds ----------------
const int STRESS_HR_THRESHOLD = 100;
const int STRESS_GSR_THRESHOLD = 2500;
const int SPO2_THRESHOLD = 92;

// ---------------- Button handling ----------------
static int lastBtnState = HIGH;
static unsigned long lastBounce = 0;
const unsigned long DEBOUNCE_MS = 50;
static unsigned long lastPressMillis = 0;
static int clickCount = 0;
const unsigned long CLICK_WINDOW_MS = 300; // time to group clicks

// ---------------- Sensor parsing helper ----------------
int extractValue(String &src, const char *key) {
  int start = src.indexOf(key);
  if (start == -1) return 0;
  start += strlen(key);
  int end = src.indexOf(",", start);
  if (end == -1) end = src.indexOf("}", start);
  if (end == -1) return 0;
  String token = src.substring(start, end);
  token.trim();
  return token.toInt();
}

// ---------------- Audio helpers ----------------
void stopAudioIfAny() {
  if (wav) {
    wav->stop();
    delete wav;
    wav = nullptr;
  }
  if (file) {
    delete file;
    file = nullptr;
  }
  if (out) {
    delete out;
    out = nullptr;
  }
  isPlaying = false;
}

void playWavNonBlocking(const char *path) {
  // If already playing same file, ignore; else stop and start new
  if (isPlaying) {
    // Option: allow interrupting important messages (family/check)
    // Here we just ignore new requests unless you force stopAudioIfAny() before calling
    Serial.printf("Audio busy. Ignoring request: %s\n", path);
    return;
  }

  file = new AudioFileSourceSD(path);
  if (!file->isOpen()) {
    Serial.printf("Failed to open WAV: %s\n", path);
    delete file;
    file = nullptr;
    return;
  }

  out = new AudioOutputI2S();
  out->SetPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);

  wav = new AudioGeneratorWAV();
  if (!wav->begin(file, out)) {
    Serial.println("wav->begin failed");
    stopAudioIfAny();
    return;
  }

  isPlaying = true;
  Serial.printf("Started play: %s\n", path);
}

// Call frequently in loop
void audioService() {
  if (isPlaying && wav) {
    if (!wav->loop()) {
      // finished
      stopAudioIfAny();
      Serial.println("Playback finished");
    }
  }
}

// ---------------- Button read (debounced) ----------------
void pollButton() {
  int now = digitalRead(BTN_PIN);
  unsigned long t = millis();
  if (now != lastBtnState) {
    lastBounce = t;
  }
  if ((t - lastBounce) > DEBOUNCE_MS) {
    if (lastBtnState == HIGH && now == LOW) {
      // pressed
      clickCount++;
      lastPressMillis = t;
      Serial.printf("Button press detected. clickCount=%d\n", clickCount);
    }
  }
  lastBtnState = now;

  // If clicks collected and time window passed, evaluate
  if (clickCount > 0 && (millis() - lastPressMillis) > CLICK_WINDOW_MS) {
    if (systemMode == MODE_EMERGENCY_AWAIT || systemMode == MODE_EMERGENCY_BROADCAST) {
      if (clickCount == 1) {
        // Single click -> Family alert
        Serial.println("Single click -> FAMILY ALERT");
        // Interrupt current audio and play family
        stopAudioIfAny();
        playWavNonBlocking(PATH_FAMILY);
        // optionally keep emergency active; do not cancel
      } else if (clickCount == 2) {
        // Double click -> Cancel emergency
        Serial.println("Double click -> EMERGENCY CANCELED");
        stopAudioIfAny();
        playWavNonBlocking(PATH_CHECK);
        // reset emergency state
        systemMode = MODE_NORMAL;
        sirenPlayed = false;
        emergencyLooping = false;
      } else {
        Serial.printf("Unhandled clicks: %d\n", clickCount);
      }
    } else {
      Serial.printf("Button clicked in NORMAL mode (%d) — ignored\n", clickCount);
    }
    clickCount = 0;
  }
}

// ---------------- Sensor fusion & normal audio logic ----------------
void processNormal(int hr, int spo2, int gsr) {
  bool stress = (hr > STRESS_HR_THRESHOLD) && (gsr > STRESS_GSR_THRESHOLD);
  bool fatigue = (spo2 > 0 && spo2 < SPO2_THRESHOLD);

  // Evening music condition
  int hour = timeClient.getHours(); // requires timeClient.update() in loop
  bool isEvening = (hour >= 18 && hour <= 23);

  if (stress || fatigue) {
    // If evening and stress/tired -> play gospel/jazz/rnb randomly
    if (isEvening && (stress || fatigue)) {
      // choose random track
      int pick = random(0,3);
      const char* tracks[3] = { PATH_GOSPEL, PATH_JAZZ, PATH_RNB };
      Serial.printf("Evening relaxation → playing: %s\n", tracks[pick]);
      stopAudioIfAny();
      playWavNonBlocking(tracks[pick]);
      // Even after playing evening music, we still enter emergencyAwait (per design)
      // If you want music to prevent emergency, remove the next lines.
    }

    // Enter emergency await mode
    if (systemMode == MODE_NORMAL) {
      Serial.println("Abnormal state detected -> Entering Emergency Await Mode");
      systemMode = MODE_EMERGENCY_AWAIT;
      emergencyStart = millis();
      sirenPlayed = false;
      emergencyLooping = false;
      // Play prompt to confirm
      stopAudioIfAny();
      playWavNonBlocking(PATH_PROMPT);
    }
    return;
  }

  // Otherwise normal non-emergency announcements
  if (gsr > STRESS_GSR_THRESHOLD) {
    playWavNonBlocking(PATH_STRESS);
  }
  else if (spo2 > 0 && spo2 < SPO2_THRESHOLD) {
    playWavNonBlocking(PATH_TIRED);
  }
  else if (hr >= 65 && hr <= 80) {
    playWavNonBlocking(PATH_CALM);
  }
  else {
    playWavNonBlocking(PATH_NORMAL);
  }
}

// ---------------- Emergency handling ----------------
void handleEmergencyAwait() {
  // If timeout passed and siren not yet played => play siren then emergency repeating
  if ((millis() - emergencyStart) > EMERGENCY_TIMEOUT_MS && !sirenPlayed) {
    Serial.println("Emergency timeout -> start siren");
    stopAudioIfAny();
    playWavNonBlocking(PATH_SIREN);
    sirenPlayed = true;
    emergencyLooping = false;
  }

  // If siren played and finished, start repeating emergency message
  if (sirenPlayed && !isPlaying && !emergencyLooping) {
    Serial.println("Siren finished -> start repeating emergency message");
    stopAudioIfAny();
    playWavNonBlocking(PATH_EMERGENCY);
    emergencyLooping = true;
    systemMode = MODE_EMERGENCY_BROADCAST;
  }
}

void handleEmergencyBroadcast() {
  // When emergency.wav finishes, restart it to loop indefinitely
  if (!isPlaying && emergencyLooping) {
    playWavNonBlocking(PATH_EMERGENCY);
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  delay(50);

  // seed RNG
  randomSeed(esp_random());

  // Button
  pinMode(BTN_PIN, INPUT_PULLUP);
  lastBtnState = digitalRead(BTN_PIN);

  // Sensor UART
  SensorSerial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

  // SD init
  SPI.begin(); // default pins; library will use SD_CS
  if (!SD.begin(SD_CS)) {
    Serial.println("SD card init FAIL!");
    // If no SD
    while (1) {
      Serial.println("Insert SD and reset.");
      delay(2000);
    }
  }
  Serial.println("SD OK");

  // WiFi + NTP
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    Serial.print(".");
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    timeClient.begin();
    timeClient.update();
  } else {
    Serial.println("\nWiFi failed or timed out — NTP disabled (evening detection will use last known time if any)");
  }

  systemMode = MODE_NORMAL;
  emergencyStart = 0;
  sirenPlayed = false;
  emergencyLooping = false;

  Serial.println("System ready");
}

// ---------------- Main loop ----------------
void loop() {
  // keep audio processing frequent
  audioService();

  // poll button every cycle
  pollButton();

  // update NTP time if WiFi connected
  if (WiFi.status() == WL_CONNECTED) {
    timeClient.update();
  }

  // Mode handling
  if (systemMode == MODE_EMERGENCY_AWAIT) {
    handleEmergencyAwait();
    // still allow button input & audioService to operate
    // return to avoid treating sensor inputs while awaiting
    delay(10);
    return;
  }
  else if (systemMode == MODE_EMERGENCY_BROADCAST) {
    handleEmergencyBroadcast();
    delay(10);
    return;
  }

  // Normal mode: parse sensor serial JSON when available
  if (SensorSerial.available()) {
    String data = SensorSerial.readStringUntil('}');
    data += "}";
    if (data.length() > 4) {
      int hr   = extractValue(data, "\"hr\":");
      int spo2 = extractValue(data, "\"spo2\":");
      int gsr  = extractValue(data, "\"gsr\":");

      Serial.printf("Sensor -> HR:%d  SpO2:%d  GSR:%d\n", hr, spo2, gsr);

      processNormal(hr, spo2, gsr);
    }
  }

  delay(50);
}
