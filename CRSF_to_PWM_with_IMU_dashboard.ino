/*
 * ESP32-C3 SuperMini  —  CRSF (ELRS) to PWM converter + IMU field dashboard
 * --------------------------------------------------------------------------
 * Builds on your working CRSF->PWM sketch. Adds:
 *   1. MPU6050 IMU reading over I2C (raw registers, complementary filter
 *      for roll/pitch) so you can visually confirm the gyro is alive.
 *   2. A WiFi Access Point + tiny web server hosting a live dashboard
 *      (artificial horizon + channel readout) — connect your phone/laptop
 *      to the ESP32's own WiFi network in the field, no router needed.
 *
 * ============ IMPORTANT — adjust to YOUR wiring ============
 * You said your IMU is already wired and working — I don't know which
 * pins you used, so this sketch assumes:
 *   MPU6050 SDA  ----------> GPIO8
 *   MPU6050 SCL  ----------> GPIO9
 *   MPU6050 VCC  ----------> 3V3
 *   MPU6050 GND  ----------> GND
 * If your IMU is on different pins, or a different chip (BNO055, MPU9250,
 * etc.), only mpuInit()/mpuRead() and the Wire.begin() call need to change
 * — everything else (CRSF parsing, PWM, web dashboard) stays the same.
 * =============================================================
 *
 * Library required (same as before):
 *   "ESP32Servo" by Kevin Harrington / Dlloydev  (install the latest 3.x)
 * WiFi.h and WebServer.h ship with the ESP32 Arduino core — no extra
 * install needed.
 *
 * Usage:
 *   1. Flash this to the ESP32-C3.
 *   2. On your phone/laptop, connect to WiFi network "RCPlane-FC"
 *      (password "12345678").
 *   3. Open http://192.168.4.1 in a browser.
 *   4. Tilt the plane / wing — the horizon graphic and roll/pitch numbers
 *      should respond in real time, confirming the gyro + FC path works.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>

// ---------------- CRSF UART ----------------
#define CRSF_RX_PIN     20     // ESP32-C3 RX  <- receiver TX
#define CRSF_TX_PIN     21     // ESP32-C3 TX  -> receiver RX (telemetry, optional)
#define CRSF_BAUDRATE   420000
HardwareSerial CrsfSerial(1); // use UART1

// ---------------- Output pins ----------------
#define AILERON_PIN   2
#define ELEVATOR_PIN  3
#define THROTTLE_PIN  4
#define RUDDER_PIN    5
#define FLAPS_PIN     6
#define AUX_PIN       7

// ---------------- IMU I2C pins (ADJUST TO YOUR WIRING) ----------------
#define IMU_SDA_PIN 0
#define IMU_SCL_PIN 1

// ---------------- CRSF channel indices (0-based, AETR1234 default) ----------------
#define CH_AILERON   0
#define CH_ELEVATOR  1
#define CH_THROTTLE  2
#define CH_RUDDER    3
#define CH_FLAPS     4
#define CH_AUX       5
#define CRSF_NUM_CHANNELS 16

// ---------------- Failsafe ----------------
#define FAILSAFE_TIMEOUT_MS   300   // no valid frame for this long -> failsafe
#define FAILSAFE_THROTTLE_US  988   // min throttle (motor off / idle, per your ESC)
#define FAILSAFE_SURFACE_US   1500  // center for aileron/elevator/rudder/flaps/aux

// ---------------- Servo objects ----------------
Servo servoAileron;
Servo servoElevator;
Servo servoThrottle;
Servo servoRudder;
Servo servoFlaps;
Servo servoAux;

// ---------------- CRSF protocol constants ----------------
#define CRSF_SYNC_BYTE            0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16
#define CRSF_MAX_FRAME_LEN         64   // full frame incl. addr+len bytes

// Parser state machine
enum CrsfParseState { CRSF_WAIT_SYNC, CRSF_WAIT_LEN, CRSF_WAIT_DATA };
static CrsfParseState parseState = CRSF_WAIT_SYNC;
static uint8_t  crsfBuf[CRSF_MAX_FRAME_LEN];
static uint8_t  crsfIdx = 0;
static uint8_t  crsfFrameLen = 0; // value of the "length" byte (type+payload+crc)

// Decoded channel values, in microseconds (988-2012 typical)
static uint16_t rcChannels[CRSF_NUM_CHANNELS];
static bool     rcChannelsValid = false;
static uint32_t lastValidFrameMillis = 0;
static bool     failsafeActive = false;

// ---------------- IMU state ----------------
#define MPU_ADDR 0x68
static float roll = 0.0f;
static float pitch = 0.0f;
static int16_t raw_ax = 0, raw_ay = 0, raw_az = 0;
static int16_t raw_gx = 0, raw_gy = 0, raw_gz = 0;
static uint32_t lastImuMicros = 0;
static uint32_t lastImuReadMillis = 0;
#define IMU_READ_INTERVAL_MS 20   // ~50Hz

// ---------------- Flight Logger Config & State ----------------
#define LOG_FILE_PATH "/flight.log"
#define LOG_INTERVAL_MS 50  // 20Hz logging rate

struct __attribute__((__packed__)) LogRecord {
  uint32_t timestamp;  // Milliseconds since logging started
  int16_t roll;        // Roll angle (deg * 10)
  int16_t pitch;       // Pitch angle (deg * 10)
  int16_t ax, ay, az;  // Raw Accelerometer readings
  int16_t gx, gy, gz;  // Raw Gyroscope readings
  uint16_t aileron;    // us
  uint16_t elevator;   // us
  uint16_t throttle;   // us
  uint16_t rudder;     // us
  uint16_t flaps;      // us
  uint16_t aux;        // us
  uint8_t flags;       // Bit 0: Failsafe Active
};

static bool loggingActive = false;
static uint32_t lastLogWriteMillis = 0;
static File currentLogFile;
static uint32_t logRecordsCount = 0;
static uint32_t logStartTimeMillis = 0;

#define LOG_BUFFER_SIZE 10 // write in chunks of 10 records (330 bytes)
static LogRecord logBuffer[LOG_BUFFER_SIZE];
static uint8_t logBufferIndex = 0;

// ---------------- Web server ----------------
const char* AP_SSID = "RCPlane-FC";
const char* AP_PASS = "12345678";
WebServer server(80);

// ---------------- CRC-8/DVB-S2 (poly 0xD5), same as TBS CRSF ----------------
static uint8_t crsf_crc8(uint8_t crc, uint8_t a) {
  crc ^= a;
  for (uint8_t i = 0; i < 8; i++) {
    crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t crsf_crc8_buf(const uint8_t *buf, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) crc = crsf_crc8(crc, buf[i]);
  return crc;
}

// Map raw 11-bit CRSF channel value [172, 1811] -> microseconds [988, 2012]
static uint16_t crsf_channel_to_us(uint16_t raw) {
  static const float scale  = (2012.0f - 988.0f) / (1811.0f - 172.0f);
  static const float offset = 988.0f - 172.0f * scale;
  float us = raw * scale + offset;
  if (us < 988.0f)  us = 988.0f;
  if (us > 2012.0f) us = 2012.0f;
  return (uint16_t)(us + 0.5f);
}

// Unpack the 22-byte RC-channels payload into 16 x 11-bit values
static void crsf_unpack_channels(const uint8_t *payload, uint16_t *out) {
  uint32_t bitBuffer = 0;
  uint8_t  bitCount = 0;
  uint8_t  byteIdx = 0;

  for (uint8_t ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
    while (bitCount < 11) {
      bitBuffer |= (uint32_t)payload[byteIdx++] << bitCount;
      bitCount += 8;
    }
    out[ch] = bitBuffer & 0x07FF;
    bitBuffer >>= 11;
    bitCount -= 11;
  }
}

static void crsf_process_frame() {
  uint8_t len = crsfBuf[1];
  uint8_t type = crsfBuf[2];
  uint8_t receivedCrc = crsfBuf[2 + len - 1];

  uint8_t calcCrc = crsf_crc8_buf(&crsfBuf[2], len - 1);
  if (calcCrc != receivedCrc) {
    return; // corrupt frame, drop it
  }

  if (type == CRSF_FRAMETYPE_RC_CHANNELS && len == 24) {
    uint16_t raw[CRSF_NUM_CHANNELS];
    crsf_unpack_channels(&crsfBuf[3], raw);
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++) {
      rcChannels[i] = crsf_channel_to_us(raw[i]);
    }
    rcChannelsValid = true;
    lastValidFrameMillis = millis();
  }
}

static void crsf_parse_byte(uint8_t b) {
  switch (parseState) {
    case CRSF_WAIT_SYNC:
      if (b == CRSF_SYNC_BYTE) {
        crsfBuf[0] = b;
        crsfIdx = 1;
        parseState = CRSF_WAIT_LEN;
      }
      break;

    case CRSF_WAIT_LEN:
      if (b < 2 || b > (CRSF_MAX_FRAME_LEN - 2)) {
        parseState = CRSF_WAIT_SYNC;
        break;
      }
      crsfBuf[1] = b;
      crsfFrameLen = b;
      crsfIdx = 2;
      parseState = CRSF_WAIT_DATA;
      break;

    case CRSF_WAIT_DATA:
      crsfBuf[crsfIdx++] = b;
      if (crsfIdx >= (uint8_t)(2 + crsfFrameLen)) {
        crsf_process_frame();
        parseState = CRSF_WAIT_SYNC;
      }
      break;
  }
}

// ---------------- Servo helpers ----------------
static void applyFailsafe() {
  servoAileron.writeMicroseconds(FAILSAFE_SURFACE_US);
  servoElevator.writeMicroseconds(FAILSAFE_SURFACE_US);
  servoThrottle.writeMicroseconds(FAILSAFE_THROTTLE_US);
  servoRudder.writeMicroseconds(FAILSAFE_SURFACE_US);
  servoFlaps.writeMicroseconds(FAILSAFE_SURFACE_US);
  servoAux.writeMicroseconds(FAILSAFE_SURFACE_US);
}

static void applyChannels() {
  servoAileron.writeMicroseconds(rcChannels[CH_AILERON]);
  servoElevator.writeMicroseconds(rcChannels[CH_ELEVATOR]);
  servoThrottle.writeMicroseconds(rcChannels[CH_THROTTLE]);
  servoRudder.writeMicroseconds(rcChannels[CH_RUDDER]);
  servoFlaps.writeMicroseconds(rcChannels[CH_FLAPS]);
  servoAux.writeMicroseconds(rcChannels[CH_AUX]);
}

// ---------------- MPU6050 raw I2C helpers ----------------
// If your IMU is a different chip / library, replace mpuInit()/mpuRead()
// only — the rest of the sketch (filter, web server) doesn't care.
static void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // wake the sensor up
  Wire.endTransmission(true);
}

static bool mpuRead(int16_t &ax, int16_t &ay, int16_t &az,
                     int16_t &gx, int16_t &gy, int16_t &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14, (int)true) != 14) return false;

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperature, skipped
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  return true;
}

// Complementary filter: combines accelerometer (stable, noisy) with
// gyro (smooth, drifts) to get a usable roll/pitch estimate.
static void updateAttitude() {
  if (!mpuRead(raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz)) return;

  float accelRoll  = atan2((float)raw_ay, (float)raw_az) * 180.0f / PI;
  float accelPitch = atan2(-(float)raw_ax, sqrtf((float)raw_ay * raw_ay + (float)raw_az * raw_az)) * 180.0f / PI;

  // 131 LSB/(deg/s) is the MPU6050's default +-250dps sensitivity
  float gyroRollRate  = raw_gx / 131.0f;
  float gyroPitchRate = raw_gy / 131.0f;

  uint32_t now = micros();
  float dt = (lastImuMicros == 0) ? 0.0f : (now - lastImuMicros) / 1000000.0f;
  lastImuMicros = now;

  if (dt > 0.0f && dt < 1.0f) {
    roll  = 0.98f * (roll  + gyroRollRate  * dt) + 0.02f * accelRoll;
    pitch = 0.98f * (pitch + gyroPitchRate * dt) + 0.02f * accelPitch;
  } else {
    // first sample: snap straight to the accelerometer estimate
    roll = accelRoll;
    pitch = accelPitch;
  }
}

// ---------------- Flight Logger Implementations ----------------
static void flushLogBuffer() {
  if (!currentLogFile || logBufferIndex == 0) return;
  
  size_t bytesToWrite = logBufferIndex * sizeof(LogRecord);
  size_t bytesWritten = currentLogFile.write((uint8_t*)logBuffer, bytesToWrite);
  if (bytesWritten != bytesToWrite) {
    Serial.println("[Logger] SPI Flash write error!");
  } else {
    currentLogFile.flush();
  }
  logBufferIndex = 0;
}

static void startLogging() {
  if (loggingActive) return;
  
  // Create or overwrite log file
  currentLogFile = LittleFS.open(LOG_FILE_PATH, "w");
  if (currentLogFile) {
    loggingActive = true;
    logRecordsCount = 0;
    logStartTimeMillis = millis();
    logBufferIndex = 0;
    Serial.println("[Logger] Flight logging started! Writing to " LOG_FILE_PATH);
  } else {
    Serial.println("[Logger] Failed to open log file for writing!");
  }
}

static void stopLogging() {
  if (!loggingActive) return;
  
  flushLogBuffer();
  if (currentLogFile) {
    currentLogFile.close();
  }
  loggingActive = false;
  Serial.printf("[Logger] Flight logging stopped. Total records: %u\n", logRecordsCount);
}

static void writeLogRecord() {
  if (!loggingActive || !currentLogFile) return;

  LogRecord record;
  record.timestamp = millis() - logStartTimeMillis;
  record.roll = (int16_t)(roll * 10.0f);
  record.pitch = (int16_t)(pitch * 10.0f);
  record.ax = raw_ax;
  record.ay = raw_ay;
  record.az = raw_az;
  record.gx = raw_gx;
  record.gy = raw_gy;
  record.gz = raw_gz;
  record.aileron = rcChannels[CH_AILERON];
  record.elevator = rcChannels[CH_ELEVATOR];
  record.throttle = rcChannels[CH_THROTTLE];
  record.rudder = rcChannels[CH_RUDDER];
  record.flaps = rcChannels[CH_FLAPS];
  record.aux = rcChannels[CH_AUX];
  record.flags = failsafeActive ? 1 : 0;

  logBuffer[logBufferIndex++] = record;
  logRecordsCount++;

  if (logBufferIndex >= LOG_BUFFER_SIZE) {
    flushLogBuffer();
  }
}

static void tickLogger() {
  static uint32_t lastActiveFlightMillis = 0;
  
  // Active flight defined as receiver active and either throttle raised OR aux switch high
  bool flightActive = !failsafeActive && (rcChannels[CH_THROTTLE] > 1050 || rcChannels[CH_AUX] > 1500);
  
  if (flightActive) {
    lastActiveFlightMillis = millis();
    if (!loggingActive) {
      startLogging();
    }
  }

  if (loggingActive) {
    // Auto-stop logging if inactive/failsafe for > 5 seconds
    if (millis() - lastActiveFlightMillis > 5000) {
      stopLogging();
    } else {
      if (millis() - lastLogWriteMillis >= LOG_INTERVAL_MS) {
        lastLogWriteMillis = millis();
        writeLogRecord();
      }
    }
  }
}

// ---------------- Web dashboard ----------------
// Single-page dashboard: SVG artificial horizon + live channel/failsafe
// readout. Polls /data every ~150ms. No external assets, no internet
// needed — works fully offline against the ESP32's own AP.
const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RC Plane FC Dashboard</title>
<style>
  body { background:#0b0d10; color:#e8e8e8; font-family:sans-serif; text-align:center; margin:0; padding:16px; }
  h2 { margin:4px 0 12px; font-weight:600; }
  #horizonWrap { width:260px; height:260px; margin:0 auto; border-radius:50%; overflow:hidden;
                 border:4px solid #444; background:#000; }
  #status { margin-top:10px; font-size:15px; }
  .ok   { color:#4caf50; font-weight:bold; }
  .fail { color:#f44336; font-weight:bold; }
  table { margin:16px auto; border-collapse:collapse; }
  td { padding:4px 12px; font-size:14px; }
  td.label { color:#9aa0a6; text-align:right; }
  td.val   { color:#e8e8e8; text-align:left; font-variant-numeric:tabular-nums; }
  .card { background:#16191f; border-radius:8px; border:1px solid #2d3139; max-width:300px; margin:20px auto; padding:16px; text-align:left; box-shadow: 0 4px 12px rgba(0,0,0,0.3); }
  .card h3 { margin:0 0 12px; font-size:15px; color:#0088cc; font-weight:600; border-bottom: 1px solid #2d3139; padding-bottom:6px; }
  .btn { display:inline-block; background:#0088cc; color:#fff; padding:6px 12px; border:none; border-radius:4px; font-size:12px; text-decoration:none; cursor:pointer; margin-right:4px; margin-bottom:4px; font-weight:bold; transition: background 0.2s; }
  .btn:hover { background:#00a2f3; }
  .btn.danger { background:#c9302c; }
  .btn.danger:hover { background:#ec5854; }
  .info-row { display:flex; justify-content:space-between; font-size:13px; margin:6px 0; }
  .info-row span.val { font-weight:bold; color:#fff; font-variant-numeric:tabular-nums; }
  #analysisModal { display:none; position:fixed; z-index:100; left:0; top:0; width:100%; height:100%; overflow-y:auto; background-color:rgba(10,13,16,0.96); backdrop-filter:blur(6px); }
  .modal-content { background:#0f121a; border:1px solid #2d3139; border-radius:12px; max-width:850px; margin:20px auto; padding:20px; box-shadow:0 10px 30px rgba(0,0,0,0.5); position:relative; text-align:left; }
  .close-modal { position:absolute; right:20px; top:20px; font-size:26px; cursor:pointer; color:#7a889b; font-weight:bold; }
  .close-modal:hover { color:#fff; }
  .stats-grid { display:grid; grid-template-columns:repeat(auto-fit, minmax(110px, 1fr)); gap:10px; margin:16px 0; }
  .stats-card { background:#161922; border:1px solid #252a35; border-radius:6px; padding:10px; text-align:center; }
  .stats-card .title { font-size:10px; color:#7a889b; text-transform:uppercase; font-weight:bold; }
  .stats-card .value { font-size:16px; color:#fff; font-weight:bold; margin-top:4px; font-family:monospace; }
  .stats-card.highlight .value { color:#0088cc; }
  .chart-container { margin-bottom:16px; background:#11141c; padding:12px; border-radius:8px; border:1px solid #222630; }
  .chart-container h4 { margin:0 0 8px 0; font-size:13px; color:#a0aec0; font-weight:600; }
  .chart-container svg { width:100%; height:auto; display:block; }
</style>
</head>
<body>
  <h2>RC Plane FC Dashboard</h2>
  <div id="horizonWrap">
    <svg id="horizon" viewBox="-100 -100 200 200" width="260" height="260">
      <g id="horizonGroup">
        <rect x="-200" y="-200" width="400" height="200" fill="#3b7dd8"/>
        <rect x="-200" y="0"    width="400" height="200" fill="#6b4a2f"/>
        <line x1="-200" y1="0" x2="200" y2="0" stroke="#fff" stroke-width="2"/>
      </g>
      <line x1="-40" y1="0" x2="-12" y2="0" stroke="#ffcc00" stroke-width="4"/>
      <line x1="12" y1="0" x2="40" y2="0" stroke="#ffcc00" stroke-width="4"/>
      <circle cx="0" cy="0" r="3" fill="#ffcc00"/>
    </svg>
  </div>
  <div id="status">Connecting...</div>
  <table>
    <tr><td class="label">Roll</td><td class="val" id="rollTxt">--</td></tr>
    <tr><td class="label">Pitch</td><td class="val" id="pitchTxt">--</td></tr>
    <tr><td class="label">Aileron</td><td class="val" id="ail">--</td></tr>
    <tr><td class="label">Elevator</td><td class="val" id="ele">--</td></tr>
    <tr><td class="label">Throttle</td><td class="val" id="thr">--</td></tr>
    <tr><td class="label">Rudder</td><td class="val" id="rud">--</td></tr>
    <tr><td class="label">Flaps</td><td class="val" id="flp">--</td></tr>
    <tr><td class="label">Aux</td><td class="val" id="aux">--</td></tr>
  </table>
  
  <div class="card">
    <h3>Post-Flight Data Logger</h3>
    <div class="info-row"><span>Log Status:</span><span id="logStatus" class="val">--</span></div>
    <div class="info-row"><span>Log Size:</span><span id="logSize" class="val">--</span></div>
    <div class="info-row"><span>Duration:</span><span id="logDuration" class="val">--</span></div>
    <div style="margin-top:14px; display:flex; flex-wrap:wrap;">
      <a href="/log" class="btn" id="dlBtn" download="flight.log">Download</a>
      <button class="btn" onclick="analyzeFlightLog()" id="analyzeBtn">Analyze</button>
      <button class="btn danger" onclick="clearLog()">Clear</button>
    </div>
  </div>

  <!-- Telemetry Analysis Modal -->
  <div id="analysisModal">
    <div class="modal-content">
      <span class="close-modal" onclick="closeAnalysis()">&times;</span>
      <h2 style="color:#fff; margin:0 0 16px 0; font-size:20px; border-bottom:1px solid #2d3139; padding-bottom:8px;">Flight Telemetry Analysis</h2>
      
      <div class="stats-grid">
        <div class="stats-card highlight"><div class="title">Duration</div><div class="value" id="anDuration">--</div></div>
        <div class="stats-card"><div class="title">Peak G-Force</div><div class="value" id="anPeakG">--</div></div>
        <div class="stats-card"><div class="title">Max Roll</div><div class="value" id="anMaxRoll">--</div></div>
        <div class="stats-card"><div class="title">Max Pitch</div><div class="value" id="anMaxPitch">--</div></div>
        <div class="stats-card"><div class="title">Vibration</div><div class="value" id="anVibe">--</div></div>
        <div class="stats-card"><div class="title">Failsafes</div><div class="value" id="anFailsafes">--</div></div>
      </div>
      
      <div class="chart-container">
        <h4>Attitude Dynamics (Roll/Pitch)</h4>
        <svg id="attitudeSvg" viewBox="0 0 1000 220"></svg>
      </div>
      
      <div class="chart-container">
        <h4>Control Inputs (Throttle/Aileron/Elevator)</h4>
        <svg id="servosSvg" viewBox="0 0 1000 220"></svg>
      </div>
      
      <div class="chart-container">
        <h4>G-Force & Connectivity</h4>
        <svg id="forcesSvg" viewBox="0 0 1000 220"></svg>
      </div>
    </div>
  </div>

<script>
let logPollCounter = 0;
let hasSavedLog = false;

async function updateLogStatus() {
  try {
    const r = await fetch('/log_status');
    const d = await r.json();
    const statusEl = document.getElementById('logStatus');
    if (d.active) {
      statusEl.innerHTML = '<span class="ok" style="color:#4caf50;">Recording...</span>';
    } else {
      statusEl.textContent = d.size > 0 ? 'Log Saved' : 'Empty';
    }
    document.getElementById('logSize').textContent = (d.size / 1024).toFixed(2) + ' KB (' + d.records + ' pts)';
    const secs = Math.floor(d.duration_ms / 1000);
    const mins = Math.floor(secs / 60);
    document.getElementById('logDuration').textContent = mins + 'm ' + (secs % 60) + 's';
    
    const dlBtn = document.getElementById('dlBtn');
    const anBtn = document.getElementById('analyzeBtn');
    hasSavedLog = d.size > 0;
    
    if (!hasSavedLog || d.active) {
      dlBtn.style.pointerEvents = 'none';
      dlBtn.style.opacity = '0.4';
      anBtn.disabled = true;
      anBtn.style.opacity = '0.4';
    } else {
      dlBtn.style.pointerEvents = 'auto';
      dlBtn.style.opacity = '1';
      anBtn.disabled = false;
      anBtn.style.opacity = '1';
    }
  } catch (e) {}
}

async function clearLog() {
  if (confirm("Are you sure you want to clear the flight log?")) {
    try {
      await fetch('/log_clear');
      updateLogStatus();
    } catch(e) {}
  }
}

function downsample(arr, maxPoints = 800) {
  if (arr.length <= maxPoints) return arr;
  const step = Math.floor(arr.length / maxPoints);
  const res = [];
  for (let i = 0; i < arr.length; i += step) {
    res.push(arr[i]);
  }
  return res;
}

function drawChart(svgId, data, seriesList, minVal, maxVal, title, units, drawFailsafeBackground = false) {
  const svg = document.getElementById(svgId);
  if (!svg) return;
  svg.innerHTML = "";
  
  const width = 1000;
  const height = 220;
  const paddingLeft = 50;
  const paddingRight = 20;
  const paddingTop = 20;
  const paddingBottom = 30;
  
  const chartW = width - paddingLeft - paddingRight;
  const chartH = height - paddingTop - paddingBottom;
  
  let min = minVal;
  let max = maxVal;
  if (min === null || max === null) {
    let allVals = [];
    data.forEach(d => {
      seriesList.forEach(s => allVals.push(d[s.key]));
    });
    if (min === null) min = Math.min(...allVals) - 0.1;
    if (max === null) max = Math.max(...allVals) + 0.1;
  }
  
  if (max === min) { max += 1; min -= 1; }
  const range = max - min;
  
  let gridLines = "";
  const numGridLines = 4;
  for (let i = 0; i <= numGridLines; i++) {
    const val = min + (range / numGridLines) * i;
    const y = paddingTop + chartH - (i * (chartH / numGridLines));
    gridLines += `<line x1="${paddingLeft}" y1="${y}" x2="${width - paddingRight}" y2="${y}" stroke="#222630" stroke-width="1"/>`;
    gridLines += `<text x="${paddingLeft - 8}" y="${y + 3}" fill="#7a889b" font-size="10" text-anchor="end">${val.toFixed(1)}</text>`;
  }
  
  let timeLabels = "";
  const numTimeLabels = 5;
  const maxTime = data[data.length - 1].time / 1000.0;
  for (let i = 0; i <= numTimeLabels; i++) {
    const t = (maxTime / numTimeLabels) * i;
    const x = paddingLeft + (i * (chartW / numTimeLabels));
    gridLines += `<line x1="${x}" y1="${paddingTop}" x2="${x}" y2="${paddingTop + chartH}" stroke="#222630" stroke-dasharray="2,2"/>`;
    timeLabels += `<text x="${x}" y="${height - 8}" fill="#7a889b" font-size="10" text-anchor="middle">${t.toFixed(0)}s</text>`;
  }

  let failsafeOverlay = "";
  if (drawFailsafeBackground) {
    let fsStart = null;
    for (let i = 0; i < data.length; i++) {
      const d = data[i];
      const x = paddingLeft + (i / (data.length - 1)) * chartW;
      if (d.failsafe && fsStart === null) {
        fsStart = x;
      } else if (!d.failsafe && fsStart !== null) {
        failsafeOverlay += `<rect x="${fsStart}" y="${paddingTop}" width="${x - fsStart}" height="${chartH}" fill="rgba(239, 68, 68, 0.15)"/>`;
        fsStart = null;
      }
    }
    if (fsStart !== null) {
      failsafeOverlay += `<rect x="${fsStart}" y="${paddingTop}" width="${width - paddingRight - fsStart}" height="${chartH}" fill="rgba(239, 68, 68, 0.15)"/>`;
    }
  }
  
  let paths = "";
  seriesList.forEach(series => {
    let pointsStr = "";
    data.forEach((d, idx) => {
      const x = paddingLeft + (idx / (data.length - 1)) * chartW;
      const val = d[series.key];
      const y = paddingTop + chartH - ((val - min) / range) * chartH;
      if (idx === 0) pointsStr += `M ${x} ${y}`;
      else pointsStr += ` L ${x} ${y}`;
    });
    paths += `<path d="${pointsStr}" fill="none" stroke="${series.color}" stroke-width="${series.width || 2}" />`;
  });
  
  const hoverGroup = `
    <g id="${svgId}_hover" opacity="0">
      <line id="${svgId}_hoverLine" x1="0" y1="${paddingTop}" x2="0" y2="${paddingTop + chartH}" stroke="#00a2f3" stroke-width="1.5" stroke-dasharray="3,3"/>
      <rect id="${svgId}_hoverBox" x="0" y="0" width="130" height="70" rx="4" fill="#1b202c" stroke="#2d3548" stroke-width="1"/>
      <text id="${svgId}_hoverText" x="0" y="0" fill="#fff" font-size="10"></text>
    </g>
  `;
  
  svg.innerHTML = `
    <rect width="${width}" height="${height}" fill="#11141c" rx="6"/>
    ${failsafeOverlay}
    ${gridLines}
    ${paths}
    ${timeLabels}
    ${hoverGroup}
    <rect id="${svgId}_overlay" x="${paddingLeft}" y="${paddingTop}" width="${chartW}" height="${chartH}" fill="transparent" style="cursor:crosshair;"/>
  `;
  
  const overlay = document.getElementById(`${svgId}_overlay`);
  const hover = document.getElementById(`${svgId}_hover`);
  const hoverLine = document.getElementById(`${svgId}_hoverLine`);
  const hoverBox = document.getElementById(`${svgId}_hoverBox`);
  const hoverText = document.getElementById(`${svgId}_hoverText`);
  
  overlay.addEventListener("mousemove", (e) => {
    const rect = svg.getBoundingClientRect();
    const clientX = e.clientX - rect.left;
    const svgX = (clientX / rect.width) * width;
    
    const pct = (svgX - paddingLeft) / chartW;
    const idx = Math.max(0, Math.min(data.length - 1, Math.round(pct * (data.length - 1))));
    const d = data[idx];
    const snapX = paddingLeft + (idx / (data.length - 1)) * chartW;
    
    hoverLine.setAttribute("x1", snapX);
    hoverLine.setAttribute("x2", snapX);
    
    let txt = `<tspan x="6" dy="12" font-weight="bold" fill="#7a889b">Time: ${(d.time/1000).toFixed(2)}s</tspan>`;
    seriesList.forEach((series) => {
      const val = d[series.key];
      txt += `<tspan x="6" dy="12" fill="${series.color}">${series.name}: ${val.toFixed(1)}${units}</tspan>`;
    });
    if (d.failsafe) {
      txt += `<tspan x="6" dy="12" fill="#ef4444" font-weight="bold">FAILSAFE</tspan>`;
    }
    hoverText.innerHTML = txt;
    
    let boxX = snapX + 10;
    if (boxX + 130 > width - paddingRight) {
      boxX = snapX - 140;
    }
    hoverBox.setAttribute("x", boxX);
    hoverBox.setAttribute("y", paddingTop + 10);
    hoverText.setAttribute("x", boxX + 6);
    hoverText.setAttribute("y", paddingTop + 10);
    
    hover.setAttribute("opacity", "1");
  });
  
  overlay.addEventListener("mouseleave", () => {
    hover.setAttribute("opacity", "0");
  });
}

async function analyzeFlightLog() {
  const btn = document.getElementById("analyzeBtn");
  const originalText = btn.textContent;
  btn.textContent = "Loading...";
  btn.disabled = true;
  
  try {
    const r = await fetch('/log');
    if (r.status === 404) {
      alert("Log file not found.");
      return;
    }
    const arrayBuffer = await r.arrayBuffer();
    btn.textContent = "Analyzing...";
    
    const view = new DataView(arrayBuffer);
    const records = [];
    const recordSize = 33;
    const numRecords = Math.floor(arrayBuffer.byteLength / recordSize);
    
    if (numRecords === 0) {
      alert("Flight log is empty.");
      return;
    }
    
    let maxRoll = 0;
    let maxPitch = 0;
    let maxG = 0;
    let failsafesCount = 0;
    let inFailsafe = false;
    let deltaGSum = 0;
    let lastG = null;
    
    for (let i = 0; i < numRecords; i++) {
      const o = i * recordSize;
      const time = view.getUint32(o, true);
      const roll = view.getInt16(o + 4, true) / 10;
      const pitch = view.getInt16(o + 6, true) / 10;
      const ax = view.getInt16(o + 8, true);
      const ay = view.getInt16(o + 10, true);
      const az = view.getInt16(o + 12, true);
      const gx = view.getInt16(o + 14, true);
      const gy = view.getInt16(o + 16, true);
      const gz = view.getInt16(o + 18, true);
      const ail = view.getUint16(o + 20, true);
      const ele = view.getUint16(o + 22, true);
      const thr = view.getUint16(o + 24, true);
      const rud = view.getUint16(o + 26, true);
      const flp = view.getUint16(o + 28, true);
      const aux = view.getUint16(o + 30, true);
      const failsafe = view.getUint8(o + 32) === 1;
      
      const ax_g = ax / 16384.0;
      const ay_g = ay / 16384.0;
      const az_g = az / 16384.0;
      const g_force = Math.sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
      
      maxRoll = Math.max(maxRoll, Math.abs(roll));
      maxPitch = Math.max(maxPitch, Math.abs(pitch));
      maxG = Math.max(maxG, g_force);
      
      const thrPct = Math.max(0, Math.min(100, ((thr - 988) / (2012 - 988)) * 100));
      
      if (failsafe && !inFailsafe) {
        failsafesCount++;
        inFailsafe = true;
      } else if (!failsafe && inFailsafe) {
        inFailsafe = false;
      }
      
      if (lastG !== null) {
        deltaGSum += Math.abs(g_force - lastG);
      }
      lastG = g_force;
      
      records.push({
        time, roll, pitch, g_force, ail, ele, thr: thrPct, rud, flp, aux, failsafe
      });
    }
    
    const durationSec = (records[records.length - 1].time - records[0].time) / 1000;
    const durM = Math.floor(durationSec / 60);
    const durS = Math.floor(durationSec % 60);
    
    document.getElementById("anDuration").textContent = durM + "m " + durS + "s";
    document.getElementById("anPeakG").textContent = maxG.toFixed(2) + " G";
    document.getElementById("anMaxRoll").textContent = maxRoll.toFixed(1) + "°";
    document.getElementById("anMaxPitch").textContent = maxPitch.toFixed(1) + "°";
    document.getElementById("anFailsafes").textContent = failsafesCount;
    
    const avgDeltaG = deltaGSum / (records.length - 1);
    let vibeText = "Smooth";
    if (avgDeltaG > 0.25) vibeText = "Danger";
    else if (avgDeltaG > 0.12) vibeText = "High";
    else if (avgDeltaG > 0.04) vibeText = "Good";
    document.getElementById("anVibe").textContent = vibeText + " (" + avgDeltaG.toFixed(3) + ")";
    
    const plotData = downsample(records, 800);
    
    drawChart("attitudeSvg", plotData, [
      { key: "roll", name: "Roll", color: "#a855f7" },
      { key: "pitch", name: "Pitch", color: "#ec4899" }
    ], -180, 180, "Attitude Dynamics", "°");
    
    drawChart("servosSvg", plotData, [
      { key: "thr", name: "Throttle %", color: "#38bdf8" },
      { key: "ail", name: "Aileron (us)", color: "#22c55e", width: 1 },
      { key: "ele", name: "Elevator (us)", color: "#f59e0b", width: 1 }
    ], 0, 2100, "Control Inputs", "");
    
    drawChart("forcesSvg", plotData, [
      { key: "g_force", name: "G-Force", color: "#f97316" }
    ], 0, null, "G-Forces & Failsafes", " G", true);
    
    document.getElementById("analysisModal").style.display = "block";
  } catch (e) {
    alert("Error parsing log: " + e.message);
  } finally {
    btn.textContent = originalText;
  }
}

function closeAnalysis() {
  document.getElementById("analysisModal").style.display = "none";
}

async function poll() {
  try {
    const r = await fetch('/data');
    const d = await r.json();
    document.getElementById('horizonGroup').setAttribute(
      'transform', 'rotate(' + (-d.roll).toFixed(1) + ') translate(0,' + (d.pitch * 2.2).toFixed(1) + ')'
    );
    document.getElementById('rollTxt').textContent  = d.roll.toFixed(1) + ' deg';
    document.getElementById('pitchTxt').textContent = d.pitch.toFixed(1) + ' deg';
    document.getElementById('ail').textContent = d.ail + ' us';
    document.getElementById('ele').textContent = d.ele + ' us';
    document.getElementById('thr').textContent = d.thr + ' us';
    document.getElementById('rud').textContent = d.rud + ' us';
    document.getElementById('flp').textContent = d.flp + ' us';
    document.getElementById('aux').textContent = d.aux + ' us';
    const s = document.getElementById('status');
    if (d.failsafe) { s.innerHTML = '<span class="fail">FAILSAFE - no CRSF data</span>'; }
    else { s.innerHTML = '<span class="ok">Gyro + FC responding</span>'; }
  } catch (e) {
    document.getElementById('status').innerHTML = '<span class="fail">Connection lost</span>';
  }
  
  if (logPollCounter++ % 10 === 0) {
    updateLogStatus();
  }
  
  setTimeout(poll, 150);
}
updateLogStatus();
poll();
</script>
</body>
</html>
)HTMLPAGE";

static void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

static void handleData() {
  String json = "{";
  json += "\"roll\":"  + String(roll, 1) + ",";
  json += "\"pitch\":" + String(pitch, 1) + ",";
  json += "\"ail\":" + String(rcChannels[CH_AILERON]) + ",";
  json += "\"ele\":" + String(rcChannels[CH_ELEVATOR]) + ",";
  json += "\"thr\":" + String(rcChannels[CH_THROTTLE]) + ",";
  json += "\"rud\":" + String(rcChannels[CH_RUDDER]) + ",";
  json += "\"flp\":" + String(rcChannels[CH_FLAPS]) + ",";
  json += "\"aux\":" + String(rcChannels[CH_AUX]) + ",";
  json += "\"failsafe\":" + String(failsafeActive ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

static void handleLog() {
  if (LittleFS.exists(LOG_FILE_PATH)) {
    File f = LittleFS.open(LOG_FILE_PATH, "r");
    if (f) {
      server.streamFile(f, "application/octet-stream");
      f.close();
      return;
    }
  }
  server.send(404, "text/plain", "Log file not found");
}

static void handleLogClear() {
  stopLogging();
  if (LittleFS.exists(LOG_FILE_PATH)) {
    if (LittleFS.remove(LOG_FILE_PATH)) {
      server.send(200, "application/json", "{\"status\":\"cleared\"}");
      Serial.println("[Logger] Log cleared via Web request.");
      return;
    }
  }
  server.send(200, "application/json", "{\"status\":\"not_found_or_error\"}");
}

static void handleLogStatus() {
  size_t fileSize = 0;
  if (LittleFS.exists(LOG_FILE_PATH)) {
    File f = LittleFS.open(LOG_FILE_PATH, "r");
    if (f) {
      fileSize = f.size();
      f.close();
    }
  }
  
  String json = "{";
  json += "\"active\":" + String(loggingActive ? "true" : "false") + ",";
  json += "\"size\":" + String(fileSize) + ",";
  json += "\"records\":" + String(logRecordsCount) + ",";
  json += "\"duration_ms\":" + String(loggingActive ? (millis() - logStartTimeMillis) : 0);
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("ESP32-C3 CRSF->PWM + IMU dashboard starting...");

  // CRSF UART: 420000 baud, 8N1, uninverted
  CrsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

  // IMU I2C
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);
  mpuInit();

  // LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("[System] LittleFS mount failed! Formatting...");
  } else {
    Serial.println("[System] LittleFS mounted successfully.");
    if (LittleFS.exists(LOG_FILE_PATH)) {
      File f = LittleFS.open(LOG_FILE_PATH, "r");
      if (f) {
        logRecordsCount = f.size() / sizeof(LogRecord);
        f.close();
        Serial.printf("[System] Found existing flight log, records: %u\n", logRecordsCount);
      }
    }
  }

  // ESP32Servo timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoAileron.setPeriodHertz(50);
  servoElevator.setPeriodHertz(50);
  servoThrottle.setPeriodHertz(50);
  servoRudder.setPeriodHertz(50);
  servoFlaps.setPeriodHertz(50);
  servoAux.setPeriodHertz(50);

  servoAileron.attach(AILERON_PIN, 988, 2012);
  servoElevator.attach(ELEVATOR_PIN, 988, 2012);
  servoThrottle.attach(THROTTLE_PIN, 988, 2012);
  servoRudder.attach(RUDDER_PIN, 988, 2012);
  servoFlaps.attach(FLAPS_PIN, 988, 2012);
  servoAux.attach(AUX_PIN, 988, 2012);

  applyFailsafe();
  lastValidFrameMillis = millis();

  // WiFi AP + dashboard
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Dashboard AP IP: ");
  Serial.println(WiFi.softAPIP()); // normally 192.168.4.1

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/log", handleLog);
  server.on("/log_clear", handleLogClear);
  server.on("/log_status", handleLogStatus);
  server.begin();
}

void loop() {
  // Drain all bytes currently in the UART RX buffer
  while (CrsfSerial.available()) {
    crsf_parse_byte((uint8_t)CrsfSerial.read());
  }

  failsafeActive = (millis() - lastValidFrameMillis) > FAILSAFE_TIMEOUT_MS;

  if (failsafeActive) {
    applyFailsafe();
  } else if (rcChannelsValid) {
    applyChannels();
  }

  // IMU update, ~50Hz, non-blocking
  if (millis() - lastImuReadMillis >= IMU_READ_INTERVAL_MS) {
    lastImuReadMillis = millis();
    updateAttitude();
  }

  // Logger tick
  tickLogger();

  // Serve the dashboard
  server.handleClient();

  // Lightweight debug print, ~10Hz
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    if (failsafeActive) {
      Serial.println("FAILSAFE - no CRSF data");
    } else {
      Serial.printf("AIL:%4u  ELE:%4u  THR:%4u  RUD:%4u  FLP:%4u  AUX:%4u  ROLL:%6.1f  PITCH:%6.1f\n",
                     rcChannels[CH_AILERON], rcChannels[CH_ELEVATOR],
                     rcChannels[CH_THROTTLE], rcChannels[CH_RUDDER],
                     rcChannels[CH_FLAPS], rcChannels[CH_AUX], roll, pitch);
    }
  }
}
