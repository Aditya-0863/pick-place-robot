/*
  ================================================================
  2WD Robot — ESP32 Firmware  (TB6612FNG + 2× Servo)
  ================================================================
  Receives UDP commands from the Python vision controller:
    "M <left> <right>"  — motor speeds, each -255..255
    "B <angle>"         — base servo  0-180°
    "G <angle>"         — grip servo  0° open / 90° closed
    "STOP"              — immediate active brake

  Pin config (TB6612FNG):
    PWMA  32    AIN1  18    AIN2  19
    STBY   4
    PWMB   5    BIN1  21    BIN2  22

  Servo pins:
    BASE  26    GRIP  25
  ================================================================
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ── WiFi credentials ──────────────────────────────────────────
const char* WIFI_SSID = "Adi";
const char* WIFI_PASS = "bleh@7812";
const int   UDP_PORT  = 4210;

// ── Pin definitions ───────────────────────────────────────────
#define PIN_PWMA  32
#define PIN_AIN1  18
#define PIN_AIN2  19
#define PIN_STBY   4

#define PIN_PWMB   5
#define PIN_BIN1  21
#define PIN_BIN2  22

#define PIN_BASE  26
#define PIN_GRIP  25

// ── LEDC PWM ──────────────────────────────────────────────────
#define PWM_FREQ  5000
#define PWM_BITS  8

// ── Motor ramping ─────────────────────────────────────────────
#define MOTOR_RAMP_STEP  8
#define LOOP_MS          10

// ── Servo interpolation ───────────────────────────────────────
#define SERVO_STEP_DEG  1.5f
#define BASE_MIN    0
#define BASE_MAX  180
#define GRIP_MIN    0
#define GRIP_MAX   90

// ── Watchdog ──────────────────────────────────────────────────
#define WATCHDOG_MS  1000





// ── Objects ───────────────────────────────────────────────────
WiFiUDP udp;
Servo   servoBase;
Servo   servoGrip;

// ── Motor state ───────────────────────────────────────────────
int motorA_cur = 0, motorA_tgt = 0;
int motorB_cur = 0, motorB_tgt = 0;


// ── Servo state ───────────────────────────────────────────────
float baseCur = 50.0f, baseTgt = 50.0f;
float gripCur = 105.0f, gripTgt = 105.0f;

// ── Timing ────────────────────────────────────────────────────
unsigned long lastLoopMs   = 0;
unsigned long lastPacketMs = 0;
bool          watchdogFired = false;

// ── Forward declarations ──────────────────────────────────────
void setupMotors();
void setupServos();
void connectWiFi();
void updateMotors();
void updateServos();
void parseCommand(const char* cmd);
void brakeMotors();
void driveMotorA(int spd);
void driveMotorB(int spd);
int   rampInt(int cur, int tgt, int step);
float rampFloat(float cur, float tgt, float step);

// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Robot ESP32 starting…");

  setupMotors();
  setupServos();
  connectWiFi();

  udp.begin(UDP_PORT);
  Serial.printf("[NET]  UDP listening on port %d\n", UDP_PORT);

  lastPacketMs = millis();
  Serial.println("[BOOT] Ready — waiting for commands.");
}

// =============================================================
void loop() {
  unsigned long now = millis();

  // ── Receive UDP packets ──────────────────────────────────────
  int pktSize = udp.parsePacket();
  if (pktSize > 0) {
    char buf[64];
    int  len = udp.read(buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      parseCommand(buf);
      lastPacketMs  = now;
      watchdogFired = false;
    }
  }

  // ── Watchdog ─────────────────────────────────────────────────
  if (!watchdogFired && (now - lastPacketMs) > WATCHDOG_MS) {
    Serial.println("[WDG]  No packet for 1 s — braking!");
    brakeMotors();
    watchdogFired = true;
  }

  // ── Fixed-rate control loop ──────────────────────────────────
  if (now - lastLoopMs >= LOOP_MS) {
    lastLoopMs = now;
    updateMotors();
    updateServos();
  }
}

// =============================================================
//  ACTIVE BRAKE
//  TB6612FNG: AIN1=HIGH, AIN2=HIGH → short-brake mode.
//  Also resets ramp state so updateMotors() doesn't re-drive.
// =============================================================
void brakeMotors() {
  // Kill ramp immediately so it doesn't fight the brake
  motorA_tgt = 0;  motorA_cur = 0;
  motorB_tgt = 0;  motorB_cur = 0;


  // Short-brake both channels
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  digitalWrite(PIN_BIN1, HIGH);
  digitalWrite(PIN_BIN2, HIGH);
  ledcWrite(PIN_PWMA, 255);
  ledcWrite(PIN_PWMB, 255);
  delay(80);          // hold 80 ms — reduce to 60 if it jerks

  // Release to idle (no current draw)
  ledcWrite(PIN_PWMA, 0);
  ledcWrite(PIN_PWMB, 0);
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  digitalWrite(PIN_BIN1, LOW);
  digitalWrite(PIN_BIN2, LOW);

  Serial.println("[MOT]  Brake applied");
}

// =============================================================
//  COMMAND PARSER
// =============================================================
void parseCommand(const char* cmd) {
  Serial.printf("[RX]   \"%s\"\n", cmd);

  if (strncmp(cmd, "M ", 2) == 0) {
    int l = 0, r = 0;
    if (sscanf(cmd + 2, "%d %d", &l, &r) == 2) {
      if (l == 0 && r == 0) {
        // M 0 0 → hard brake, same as STOP
        brakeMotors();
      } else {
        motorA_tgt = constrain(l, -255, 255);
        motorB_tgt = constrain(r, -255, 255);
        Serial.printf("[MOT]  Target L=%d R=%d\n", motorA_tgt, motorB_tgt);
      }
    }
  }
  else if (strncmp(cmd, "B ", 2) == 0) {
    int ang = 0;
    if (sscanf(cmd + 2, "%d", &ang) == 1) {
      baseTgt = constrain((float)ang, BASE_MIN, BASE_MAX);
      Serial.printf("[SRV]  Base target=%.0f°\n", baseTgt);
    }
  }
  else if (strncmp(cmd, "G ", 2) == 0) {
    int ang = 0;
    if (sscanf(cmd + 2, "%d", &ang) == 1) {
      gripTgt = constrain((float)ang, GRIP_MIN, GRIP_MAX);
      Serial.printf("[SRV]  Grip target=%.0f°\n", gripTgt);
    }
  }
  else if (strcmp(cmd, "STOP") == 0) {
    brakeMotors();
  }
  else {
    Serial.printf("[WARN] Unknown command: \"%s\"\n", cmd);
  }
}

// =============================================================
//  MOTOR UPDATE — ramp toward target each tick
// =============================================================
void updateMotors() {
  motorA_cur = rampInt(motorA_cur, motorA_tgt, MOTOR_RAMP_STEP);
  motorB_cur = rampInt(motorB_cur, motorB_tgt, MOTOR_RAMP_STEP);
  driveMotorA(motorA_cur);
  driveMotorB(motorB_cur);
}
// Motor A — Left wheel
void driveMotorA(int spd) {
  ledcWrite(PIN_PWMA, (unsigned int)abs(spd));
  if (spd > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else if (spd < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  } else {
    // Brake when idle (HIGH,HIGH = short-brake on TB6612FNG)
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, HIGH);
  }
}

// Motor B — Right wheel
void driveMotorB(int spd) {

  ledcWrite(PIN_PWMB, (unsigned int)abs(spd));
  if (spd > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
  } else if (spd < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
  } else {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, HIGH);
  }
}


void updateServos() {
  baseCur = rampFloat(baseCur, baseTgt, SERVO_STEP_DEG);
  gripCur = rampFloat(gripCur, gripTgt, SERVO_STEP_DEG);
  servoBase.write((int)baseCur);
  servoGrip.write((int)gripCur);
}

float rampFloat(float cur, float tgt, float step) {
  float diff = tgt - cur;
  if (fabsf(diff) <= step) return tgt;
  return cur + (diff > 0 ? step : -step);
}

int rampInt(int cur, int tgt, int step) {
  if (cur < tgt) return min(cur + step, tgt);
  if (cur > tgt) return max(cur - step, tgt);
  return cur;
}

void setupMotors() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);

  digitalWrite(PIN_STBY, HIGH);   // enable driver

  ledcAttach(PIN_PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PIN_PWMB, PWM_FREQ, PWM_BITS);

  driveMotorA(0);
  driveMotorB(0);
  Serial.println("[INIT] Motors OK");
}

void setupServos() {
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servoBase.setPeriodHertz(50);
  servoGrip.setPeriodHertz(50);

  servoBase.attach(PIN_BASE, 500, 2400);
  servoGrip.attach(PIN_GRIP, 500, 2400);

  servoBase.write((int)baseCur);
  servoGrip.write((int)gripCur);
  Serial.println("[INIT] Servos OK");
}

void connectWiFi() {
  Serial.printf("[NET]  Connecting to \"%s\"", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - t0 > 15000) {
      Serial.println("\n[NET]  WiFi timeout! Rebooting…");
      delay(3000);
      ESP.restart();
    }
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\n[NET]  Connected! IP: %s\n", WiFi.localIP().toString().c_str());
}
