// =====================================================
//  ESP32 CAR: DATA LOGGER + WI-FI REMOTE CONTROL
// =====================================================
//  What it does
//   - Reads the MPU6500 (accel + gyro) and the GPS at 50 Hz.
//   - Sends every sample as a CSV row over Wi-Fi to wifi_logger.py (laptop).
//   - Serves a control page (phone / laptop browser) to drive the car and
//     change the event label.
//
//  How it is organised
//   - loop()   (core 1): sampling only. Never waits for the network.
//   - netTask  (core 0): Wi-Fi, TCP to laptop, web page, motors.
//   - A queue between them buffers ~10 s of samples if Wi-Fi drops.
//
//  Baud rates
//   - Serial Monitor: 115200
//   - GPS module (UART2, pins 16/17): 9600
//   - Wi-Fi / TCP has no baud rate.
//
//  Wiring (L298N): see the pin table below. Common GND is required.
//  Remove the ENA / ENB jumpers on the L298N so PWM speed control works.
// =====================================================

struct Sample;  // Forward declaration for Arduino's generated prototypes

#include <Wire.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <WebServer.h>

// =====================================================
// 1. SETTINGS YOU MAY NEED TO EDIT
// =====================================================

const char* WIFI_SSID     = "Check";        // hotspot name (case-sensitive)
const char* WIFI_PASSWORD = "12345678";     // hotspot password

// Laptop IPv4 address. wifi_logger.py prints it when it starts.
const char* SERVER_IP = "10.154.218.226";
const uint16_t SERVER_PORT = 5000;

// =====================================================
// 2. PINS
// =====================================================

// IMU (I2C) and GPS (UART2)
#define SDA_PIN     21
#define SCL_PIN     22
#define GPS_RX_PIN  16   // ESP32 RX <- GPS TX
#define GPS_TX_PIN  17   // ESP32 TX -> GPS RX
#define MPU_ADDR    0x68

// L298N motor driver
const int L_IN1 = 25, L_IN2 = 26, L_EN = 27;   // left motors  (IN1, IN2, ENA)
const int R_IN3 = 32, R_IN4 = 33, R_EN = 13;   // right motors (IN3, IN4, ENB)

// =====================================================
// 3. CONSTANTS
// =====================================================

// MPU6500 registers
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B

// Accelerometer +/-8 g, gyroscope +/-500 deg/s
const float ACCEL_SENSITIVITY = 4096.0f;
const float GYRO_SENSITIVITY  = 65.5f;

// Sampling
const uint32_t SAMPLE_INTERVAL_US = 20000;     // 50 Hz
const float    IMPACT_THRESHOLD_G = 3.0f;      // simple threshold flag, not a crash classifier
const uint32_t IMPACT_COOLDOWN_MS = 1500;

// Networking
const uint32_t WIFI_RETRY_INTERVAL_MS  = 10000;
const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;
const uint32_t TCP_RETRY_INTERVAL_MS   = 3000;
const uint32_t SEND_INTERVAL_MS        = 100;    // rows are sent in batches
const uint32_t STATS_INTERVAL_MS       = 10000;

// Car
const uint32_t FAILSAFE_MS = 600;   // stop if no drive command for this long

// Queue between the sampler and the network task (500 rows = 10 s at 50 Hz)
const int QUEUE_LEN = 500;
const size_t MAX_ROW_BYTES = 192;

const char CSV_HEADER[] =
    "timestamp_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
    "roll_deg,pitch_deg,tilt_deg,accel_mag_g,"
    "gps_fix,satellites,latitude,longitude,"
    "speed_kmph,course_deg,impact_flag,event_label\n";

// =====================================================
// 4. EVENT LABELS
// =====================================================

const char* const LABELS[] = {
    "unlabeled", "normal", "bump", "hard_braking", "turn", "accident", "other"
};
const uint8_t LABEL_COUNT = sizeof(LABELS) / sizeof(LABELS[0]);

volatile uint8_t labelIndex = 0;   // written by the network task, read by the sampler

bool setLabel(const char* name) {
    for (uint8_t i = 0; i < LABEL_COUNT; i++) {
        if (strcmp(name, LABELS[i]) == 0) {
            labelIndex = i;
            Serial.print("# Label changed to: ");
            Serial.println(LABELS[i]);
            return true;
        }
    }
    return false;
}

// =====================================================
// 5. SHARED DATA TYPES
// =====================================================

struct Sample {
    uint32_t t_ms;
    float ax, ay, az, gx, gy, gz;
    float roll, pitch, tilt, mag;
    double lat, lon;
    float speed, course;
    uint8_t sats;
    uint8_t label;
    bool fix, hasSats, hasSpeed, hasCourse, impact;
};

QueueHandle_t sampleQueue = nullptr;

// Counters written only by the sampler, read by the network task for reporting.
volatile uint32_t statSamples = 0;
volatile uint32_t statDropped = 0;
volatile uint32_t statImuErrors = 0;

// =====================================================
// 6. GLOBAL OBJECTS
// =====================================================

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

WiFiClient tcpClient;
WebServer server(80);
bool webServerStarted = false;

// Network state (used only by the network task)
uint32_t lastWiFiAttemptMs = 0;
uint32_t wifiConnectStartedMs = 0;
uint32_t lastTCPAttemptMs = 0;
bool wifiConnecting = false;

char commandBuffer[40];
uint8_t commandLength = 0;

// Sensor values (used only by the sampler in loop())
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float roll = 0, pitch = 0, tilt = 0, accelMagnitude = 0;

uint32_t nextSampleUs = 0;
uint32_t lastImpactMs = 0;
bool impactCooldownStarted = false;
bool imuReady = false;

// =====================================================
// 7. MOTOR CONTROL (L298N), used only by the network task
// =====================================================

int carSpeed = 200;          // 0-255, set by the page slider
bool motorsActive = false;
uint32_t lastCmdMs = 0;

void pwmSetup() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(L_EN, 1000, 8);
    ledcAttach(R_EN, 1000, 8);
#else
    ledcSetup(0, 1000, 8); ledcAttachPin(L_EN, 0);
    ledcSetup(1, 1000, 8); ledcAttachPin(R_EN, 1);
#endif
}

void pwmWrite(bool left, int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(left ? L_EN : R_EN, duty);
#else
    ledcWrite(left ? 0 : 1, duty);
#endif
}

// dir: +1 forward, -1 backward, 0 stop
void driveSide(bool left, int dir, int speed) {
    int a = left ? L_IN1 : R_IN3;
    int b = left ? L_IN2 : R_IN4;

    if (dir > 0)      { digitalWrite(a, HIGH); digitalWrite(b, LOW);  }
    else if (dir < 0) { digitalWrite(a, LOW);  digitalWrite(b, HIGH); }
    else              { digitalWrite(a, LOW);  digitalWrite(b, LOW);  }

    pwmWrite(left, dir == 0 ? 0 : speed);
}

void stopCar()   { driveSide(true, 0, 0);          driveSide(false, 0, 0);          motorsActive = false; }
void forward()   { driveSide(true, +1, carSpeed);  driveSide(false, +1, carSpeed);  motorsActive = true;  }
void backward()  { driveSide(true, -1, carSpeed);  driveSide(false, -1, carSpeed);  motorsActive = true;  }
void turnLeft()  { driveSide(true, -1, carSpeed);  driveSide(false, +1, carSpeed);  motorsActive = true;  }
void turnRight() { driveSide(true, +1, carSpeed);  driveSide(false, -1, carSpeed);  motorsActive = true;  }

// Stop the car if commands stop arriving (Wi-Fi lost, phone locked, page closed).
void motorFailsafe() {
    if (motorsActive && (uint32_t)(millis() - lastCmdMs) > FAILSAFE_MS) {
        stopCar();
    }
}

// =====================================================
// 8. WEB CONTROL PAGE
// =====================================================

const char PAGE[] PROGMEM = R"rawliteral(
<!doctype html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Car Control</title>
<style>
  body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:16px;
       text-align:center;user-select:none;-webkit-user-select:none;touch-action:none}
  .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:360px;margin:12px auto}
  button{font-size:28px;padding:22px 0;border:0;border-radius:14px;background:#2d3748;color:#fff;
         touch-action:none}
  button:active,button.on{background:#3182ce}
  .stop{background:#c53030}
  .labels{display:flex;flex-wrap:wrap;gap:8px;justify-content:center;max-width:420px;margin:8px auto}
  .labels button{font-size:15px;padding:12px 14px}
  .labels button.cur{background:#2f855a}
  input[type=range]{width:80%;max-width:340px}
  small{color:#999}
</style></head><body>
<h2>Car Control</h2>
<div class="pad">
  <span></span><button data-c="F">&#9650;</button><span></span>
  <button data-c="L">&#9664;</button><button class="stop" data-c="S">&#9632;</button><button data-c="R">&#9654;</button>
  <span></span><button data-c="B">&#9660;</button><span></span>
</div>
<div>Speed: <span id="sv">200</span><br>
<input id="spd" type="range" min="100" max="255" value="200"></div>
<h3>Label: <span id="lab">...</span></h3>
<div class="labels" id="labels"></div>
<p><small>Hold a button to drive. Release to stop. Laptop: W A S D or arrow keys.</small></p>
<script>
let spd=200, timer=null;
function send(c){fetch('/cmd?c='+c+'&s='+spd).catch(()=>{});}
function press(c,el){
  clearInterval(timer);
  send(c);
  if(c!=='S'){timer=setInterval(()=>send(c),200);}
  if(el)el.classList.add('on');
}
function release(el){
  clearInterval(timer);timer=null;send('S');
  if(el)el.classList.remove('on');
}
document.querySelectorAll('.pad button').forEach(b=>{
  const c=b.dataset.c;
  b.addEventListener('pointerdown',e=>{e.preventDefault();press(c,b);});
  ['pointerup','pointerleave','pointercancel'].forEach(ev=>
    b.addEventListener(ev,()=>{if(c!=='S')release(b);}));
});
document.getElementById('spd').addEventListener('input',e=>{
  spd=+e.target.value;document.getElementById('sv').textContent=spd;});
const keys={w:'F',arrowup:'F',s:'B',arrowdown:'B',a:'L',arrowleft:'L',d:'R',arrowright:'R',' ':'S'};
let held=null;
addEventListener('keydown',e=>{
  const c=keys[e.key.toLowerCase()];
  if(!c||e.repeat)return;
  e.preventDefault();held=c;press(c);
});
addEventListener('keyup',e=>{
  const c=keys[e.key.toLowerCase()];
  if(c&&c===held){held=null;release();}
});
const L=['normal','bump','hard_braking','turn','other','unlabeled'];
const box=document.getElementById('labels');
function showLabel(t){
  document.getElementById('lab').textContent=t;
  box.querySelectorAll('button').forEach(b=>b.classList.toggle('cur',b.textContent===t));
}
L.forEach(n=>{
  const b=document.createElement('button');b.textContent=n;
  b.onclick=()=>fetch('/label?l='+n).then(r=>r.text()).then(showLabel).catch(()=>{});
  box.appendChild(b);
});
fetch('/label').then(r=>r.text()).then(showLabel).catch(()=>{});
</script></body></html>
)rawliteral";

void handleRoot() {
    server.send_P(200, "text/html", PAGE);
}

void handleCmd() {
    String c = server.arg("c");
    if (server.hasArg("s")) {
        carSpeed = constrain(server.arg("s").toInt(), 100, 255);
    }

    if (c == "F")      forward();
    else if (c == "B") backward();
    else if (c == "L") turnLeft();
    else if (c == "R") turnRight();
    else               stopCar();

    lastCmdMs = millis();
    server.send(200, "text/plain", "ok");
}

void handleLabel() {
    if (server.hasArg("l")) {
        String l = server.arg("l");
        setLabel(l.c_str());
    }
    server.send(200, "text/plain", LABELS[labelIndex]);
}

// =====================================================
// 9. WI-FI AND TCP (network task)
// =====================================================

void startWiFiConnection() {
    Serial.print("# Connecting to hotspot: ");
    Serial.println(WIFI_SSID);

    WiFi.disconnect(false, false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    lastWiFiAttemptMs = millis();
    wifiConnectStartedMs = lastWiFiAttemptMs;
    wifiConnecting = true;
}

void maintainWiFi() {
    if (WiFi.status() == WL_CONNECTED) {

    if (wifiConnecting) {
        Serial.println("# Wi-Fi connected");
        Serial.print("# ESP32 IP: ");
        Serial.println(WiFi.localIP());

        Serial.print("# Control page: http://");
        Serial.println(WiFi.localIP());

        wifiConnecting = false;
    }

    // Start HTTP server only once, after Wi-Fi connects
    if (!webServerStarted) {
        Serial.println("# Starting HTTP WebServer...");

        server.begin();

        webServerStarted = true;

        Serial.println("# HTTP WebServer started!");
    }

    return;
}

    uint32_t now = millis();

    if (wifiConnecting) {
        if (now - wifiConnectStartedMs < WIFI_CONNECT_TIMEOUT_MS) {
            return;
        }
        Serial.println("# Wi-Fi connection timed out");
        wifiConnecting = false;
    }

    if (now - lastWiFiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
        startWiFiConnection();
    }
}

void maintainTCP() {
    if (WiFi.status() != WL_CONNECTED) {
        if (tcpClient.connected()) {
            tcpClient.stop();
        }
        return;
    }

    if (tcpClient.connected()) {
        return;
    }

    uint32_t now = millis();

    if (now - lastTCPAttemptMs < TCP_RETRY_INTERVAL_MS) {
        return;
    }

    lastTCPAttemptMs = now;
    tcpClient.stop();

    Serial.print("# Connecting to laptop TCP server at ");
    Serial.print(SERVER_IP);
    Serial.print(":");
    Serial.println(SERVER_PORT);

    if (tcpClient.connect(SERVER_IP, SERVER_PORT)) {
        Serial.println("# TCP connected");
        tcpClient.setNoDelay(true);

        tcpClient.print(CSV_HEADER);

        // Tell the laptop where the control page is.
        tcpClient.print("# Control page: http://");
        tcpClient.println(WiFi.localIP());
    } else {
        Serial.println("# TCP connection failed; will retry");
    }
}

// Labels sent from the Python script ("LABEL=bump\n").
void processTCPCommands() {
    while (tcpClient.connected() && tcpClient.available() > 0) {
        char c = (char)tcpClient.read();

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            commandBuffer[commandLength] = '\0';

            if (strncmp(commandBuffer, "LABEL=", 6) == 0) {
                setLabel(commandBuffer + 6);
            }

            commandLength = 0;
        } else if (commandLength < sizeof(commandBuffer) - 1) {
            commandBuffer[commandLength++] = c;
        } else {
            commandLength = 0;
        }
    }
}

// Turn one Sample into one CSV line. Returns the length, or 0 if it did not fit.
size_t formatRow(char* out, size_t cap, const Sample& s) {
    char satS[8] = "";
    char latS[16] = "";
    char lonS[16] = "";
    char spdS[12] = "";
    char crsS[12] = "";

    if (s.hasSats)   snprintf(satS, sizeof(satS), "%u", (unsigned)s.sats);
    if (s.fix) {
        snprintf(latS, sizeof(latS), "%.6f", s.lat);
        snprintf(lonS, sizeof(lonS), "%.6f", s.lon);
    }
    if (s.hasSpeed)  snprintf(spdS, sizeof(spdS), "%.2f", s.speed);
    if (s.hasCourse) snprintf(crsS, sizeof(crsS), "%.2f", s.course);

    int n = snprintf(
        out, cap,
        "%lu,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.3f,%d,%s,%s,%s,%s,%s,%d,%s\n",
        (unsigned long)s.t_ms,
        s.ax, s.ay, s.az,
        s.gx, s.gy, s.gz,
        s.roll, s.pitch, s.tilt, s.mag,
        s.fix ? 1 : 0,
        satS, latS, lonS, spdS, crsS,
        s.impact ? 1 : 0,
        LABELS[s.label]
    );

    if (n <= 0 || (size_t)n >= cap) {
        return 0;
    }
    return (size_t)n;
}

// Send queued samples to the laptop in batches.
void drainQueue() {
    if (!tcpClient.connected()) {
        return;   // samples stay in the queue until the link is back
    }

    static uint32_t lastSendMs = 0;
    uint32_t now = millis();
    if (now - lastSendMs < SEND_INTERVAL_MS) {
        return;
    }
    lastSendMs = now;

    static char batch[1400];
    Sample s;

    // Up to 20 batches per call, so a backlog after a Wi-Fi gap clears quickly.
    for (int b = 0; b < 20; b++) {
        size_t len = 0;

        while (len + MAX_ROW_BYTES <= sizeof(batch) &&
               xQueueReceive(sampleQueue, &s, 0) == pdTRUE) {
            len += formatRow(batch + len, sizeof(batch) - len, s);
        }

        if (len == 0) {
            return;
        }

        // A partial write would leave a half row in the stream: drop the
        // connection instead; it reconnects and sends a fresh header.
        if (tcpClient.write((const uint8_t*)batch, len) != len) {
            tcpClient.stop();
            return;
        }
    }
}

// Print the real sampling rate every few seconds (Serial and the laptop window).
void reportStats() {
    static uint32_t lastMs = 0;
    static uint32_t lastCount = 0;

    uint32_t now = millis();
    if (now - lastMs < STATS_INTERVAL_MS) {
        return;
    }

    uint32_t count = statSamples;
    float hz = (count - lastCount) * 1000.0f / (float)(now - lastMs);
    lastMs = now;
    lastCount = count;

    char msg[110];
    snprintf(msg, sizeof(msg),
             "# stats: %.1f Hz, queued=%u, dropped=%lu, imu_errors=%lu",
             hz,
             (unsigned)uxQueueMessagesWaiting(sampleQueue),
             (unsigned long)statDropped,
             (unsigned long)statImuErrors);

    Serial.println(msg);
    if (tcpClient.connected()) {
        tcpClient.println(msg);
    }
}

void netTask(void* arg) {
    (void)arg;
    startWiFiConnection();

    for (;;) {
        maintainWiFi();
        maintainTCP();
        processTCPCommands();
        if (webServerStarted) {
            server.handleClient();
        }
        motorFailsafe();
        drainQueue();
        reportStats();
        vTaskDelay(1);
    }
}

// =====================================================
// 10. MPU6500
// =====================================================

bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool configureIMU() {
    if (!writeRegister(REG_PWR_MGMT_1, 0x00)) {
        return false;
    }
    delay(100);

    bool ok = true;
    ok &= writeRegister(REG_CONFIG, 0x03);
    ok &= writeRegister(REG_ACCEL_CONFIG2, 0x03);
    ok &= writeRegister(REG_GYRO_CONFIG, 0x08);    // +/-500 deg/s
    ok &= writeRegister(REG_ACCEL_CONFIG, 0x10);   // +/-8 g
    return ok;
}

bool readIMU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(REG_ACCEL_XOUT_H);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) {
        while (Wire.available()) {
            Wire.read();
        }
        return false;
    }

    uint8_t data[14];
    for (int i = 0; i < 14; i++) {
        data[i] = Wire.read();
    }

    int16_t rawAX = (int16_t)((data[0] << 8) | data[1]);
    int16_t rawAY = (int16_t)((data[2] << 8) | data[3]);
    int16_t rawAZ = (int16_t)((data[4] << 8) | data[5]);

    int16_t rawGX = (int16_t)((data[8] << 8) | data[9]);
    int16_t rawGY = (int16_t)((data[10] << 8) | data[11]);
    int16_t rawGZ = (int16_t)((data[12] << 8) | data[13]);

    ax = rawAX / ACCEL_SENSITIVITY;
    ay = rawAY / ACCEL_SENSITIVITY;
    az = rawAZ / ACCEL_SENSITIVITY;

    gx = rawGX / GYRO_SENSITIVITY;
    gy = rawGY / GYRO_SENSITIVITY;
    gz = rawGZ / GYRO_SENSITIVITY;

    return true;
}

void calculateMotion() {
    // Assumption: X forward, Y left, Z upward.
    roll  = atan2(ay, az) * 180.0f / PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / PI;
    tilt  = atan2(sqrt(ax * ax + ay * ay), az) * 180.0f / PI;
    accelMagnitude = sqrt(ax * ax + ay * ay + az * az);
}

// =====================================================
// 11. GPS
// =====================================================

void processGPS() {
    while (GPSSerial.available() > 0) {
        gps.encode((char)GPSSerial.read());
    }
}

bool gpsHasFreshFix()   { return gps.location.isValid() && gps.location.age() < 3000; }
bool gpsSpeedIsFresh()  { return gps.speed.isValid()    && gps.speed.age() < 3000; }
bool gpsCourseIsFresh() { return gps.course.isValid()   && gps.course.age() < 3000; }

// =====================================================
// 12. SETUP
// =====================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n# CHECKPOINT 1: Setup started");

    // Motor GPIO
    pinMode(L_IN1, OUTPUT);
    pinMode(L_IN2, OUTPUT);
    pinMode(R_IN3, OUTPUT);
    pinMode(R_IN4, OUTPUT);

    Serial.println("# CHECKPOINT 2: Motor GPIO initialized");

    pwmSetup();
    Serial.println("# CHECKPOINT 3: PWM initialized");

    stopCar();
    Serial.println("# CHECKPOINT 4: Motors stopped");

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    Serial.println("# CHECKPOINT 5: I2C initialized");

    // GPS
    GPSSerial.setRxBufferSize(1024);
    GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    Serial.println("# CHECKPOINT 6: GPS initialized");

    delay(100);

    imuReady = configureIMU();

    Serial.println("# CHECKPOINT 7: IMU configuration attempted");

    sampleQueue = xQueueCreate(QUEUE_LEN, sizeof(Sample));

    if (sampleQueue == nullptr) {
        Serial.println("# ERROR: Queue allocation failed");
        while (true) delay(1000);
    }

    Serial.println("# CHECKPOINT 8: Queue created");

    Serial.println("# CHECKPOINT 8A: Before server.on root");
    server.on("/", handleRoot);

    Serial.println("# CHECKPOINT 8B: Before server.on cmd");
    server.on("/cmd", handleCmd);

    Serial.println("# CHECKPOINT 8C: Before server.on label");
    server.on("/label", handleLabel);

    Serial.println("# TEST: WebServer begin skipped");

    Serial.println("# CHECKPOINT 9: Continuing after WebServer");

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    Serial.println("# CHECKPOINT 10: Wi-Fi initialized");

    nextSampleUs = micros() + SAMPLE_INTERVAL_US;

    BaseType_t taskResult = xTaskCreatePinnedToCore(
        netTask,
        "net",
        8192,
        nullptr,
        1,
        nullptr,
        0
    );

    if (taskResult != pdPASS) {
        Serial.println("# ERROR: Network task creation failed");
    } else {
        Serial.println("# CHECKPOINT 11: Network task created");
    }

    Serial.println("# SETUP COMPLETED");
}

// =====================================================
// 13. MAIN LOOP: SAMPLING ONLY
// =====================================================

void loop() {
    processGPS();

    // If the IMU was not found, retry every 2 s. The car stays drivable.
    if (!imuReady) {
        static uint32_t lastTryMs = 0;
        if (millis() - lastTryMs >= 2000) {
            lastTryMs = millis();
            imuReady = configureIMU();
            if (imuReady) {
                Serial.println("# MPU6500 found");
                nextSampleUs = micros() + SAMPLE_INTERVAL_US;
            }
        }
        delay(10);
        return;
    }

    // Wait for the next 20 ms slot.
    int32_t remaining = (int32_t)(nextSampleUs - micros());
    if (remaining > 0) {
        if (remaining > 2000) {
            delay(1);
        }
        return;
    }

    nextSampleUs += SAMPLE_INTERVAL_US;
    // If we fell far behind, skip ahead instead of firing a burst.
    if ((int32_t)(micros() - nextSampleUs) > 100000) {
        nextSampleUs = micros() + SAMPLE_INTERVAL_US;
    }

    if (!readIMU()) {
        statImuErrors = statImuErrors + 1;
        return;
    }

    calculateMotion();

    uint32_t nowMs = millis();

    // Simple high-acceleration flag with a cooldown.
    bool impact = false;
    if (accelMagnitude >= IMPACT_THRESHOLD_G &&
        (!impactCooldownStarted ||
         (uint32_t)(nowMs - lastImpactMs) >= IMPACT_COOLDOWN_MS)) {
        impact = true;
        lastImpactMs = nowMs;
        impactCooldownStarted = true;
    }

    Sample s;
    s.t_ms = nowMs;
    s.ax = ax; s.ay = ay; s.az = az;
    s.gx = gx; s.gy = gy; s.gz = gz;
    s.roll = roll; s.pitch = pitch; s.tilt = tilt; s.mag = accelMagnitude;

    s.fix = gpsHasFreshFix();
    s.lat = s.fix ? gps.location.lat() : 0.0;
    s.lon = s.fix ? gps.location.lng() : 0.0;

    s.hasSats = gps.satellites.isValid();
    s.sats = s.hasSats ? (uint8_t)gps.satellites.value() : 0;

    s.hasSpeed = gpsSpeedIsFresh();
    s.speed = s.hasSpeed ? (float)gps.speed.kmph() : 0.0f;

    s.hasCourse = gpsCourseIsFresh();
    s.course = s.hasCourse ? (float)gps.course.deg() : 0.0f;

    s.impact = impact;
    s.label = labelIndex;

    // Never block: if the queue is full, drop the oldest row to make room.
    if (xQueueSend(sampleQueue, &s, 0) != pdTRUE) {
        Sample oldest;
        xQueueReceive(sampleQueue, &oldest, 0);
        xQueueSend(sampleQueue, &s, 0);
        statDropped = statDropped + 1;
    }

    statSamples = statSamples + 1;
}
