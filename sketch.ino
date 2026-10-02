/* =====================================================================
   Dual-Core PID Line Follower + BT Tuning Console (ESP32)
   ---------------------------------------------------------------------
   CORE 1 : sensorTask -> sensors, error, PID (sole motor-cmd publisher)
   CORE 0 : motorTask  -> PWM output + staleness watchdog
   CORE 0 : btTask     -> "terminator" BT console: kp/ki/kd/speed editing

   Ownership:
     digitalRead : sensorTask         Serial (USB) : sensorTask, setup
     analogWrite : motorTask, setup   SerialBT     : btTask, setup
     g_cmd  : written ONLY by sensorTask, read by motorTask   (g_cmdMux)
     g_ctrl : params/pause from btTask, started flag from sensorTask (g_ctrlMux)
   ===================================================================== */

#include <Arduino.h>
#include <stdarg.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ---------------- Pins ----------------
#define SENSOR_LEFTMOST    22
#define SENSOR_LEFT        16
#define SENSOR_MIDDLE      21
#define SENSOR_RIGHT       17
#define SENSOR_RIGHTMOST   23

#define M1_FWD  5     // LEFT motor
#define M1_REV  18
#define M2_FWD  15    // RIGHT motor
#define M2_REV  2
#define STBY    4

#define LINE_ACTIVE_LEVEL HIGH

// ---------------- Fixed settings ----------------
#define MAX_SPEED             255
#define LOOP_PERIOD_MS        10
#define MOTOR_TASK_PERIOD_MS  10
#define BT_TASK_PERIOD_MS     10
#define CMD_TIMEOUT_MS        250     // motor watchdog
#define STOP_ON_FULL_LINE     false
#define STARTUP_GRACE_MS      1500

// ------------- PID state (private to sensorTask) -------------
static float error = 0, previousError = 0, integral = 0, derivative = 0;
static float lastKnownError = 0;

// ------------- motor mailbox (sensorTask -> motorTask) -------------
typedef struct {
  int32_t  left;
  int32_t  right;
  uint32_t stamp;
} MotorCmd;

static MotorCmd     g_cmd = { 0, 0, 0 };
static portMUX_TYPE g_cmdMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t g_cmdTimeouts = 0;

// ------------- control block (btTask <-> sensorTask) -------------
typedef struct {
  float   kp, ki, kd;
  int32_t base;       // "speed"
  bool    paused;     // set by btTask, honoured by sensorTask
  bool    started;    // set by sensorTask, announced by btTask
} Ctrl;

static Ctrl         g_ctrl = { 30.0f, 0.0f, 25.0f, 150, false, false };
static portMUX_TYPE g_ctrlMux = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t sensorTaskHandle, motorTaskHandle, btTaskHandle;

// ---------------- thread-safe helpers ----------------
static void publishCmd(int left, int right) {
  portENTER_CRITICAL(&g_cmdMux);
  g_cmd.left  = left;
  g_cmd.right = right;
  g_cmd.stamp = millis();
  portEXIT_CRITICAL(&g_cmdMux);
}

static MotorCmd readCmd() {
  MotorCmd c;
  portENTER_CRITICAL(&g_cmdMux);
  c = g_cmd;
  portEXIT_CRITICAL(&g_cmdMux);
  return c;
}

static Ctrl getCtrl() {
  Ctrl c;
  portENTER_CRITICAL(&g_ctrlMux);
  c = g_ctrl;
  portEXIT_CRITICAL(&g_ctrlMux);
  return c;
}

static void setCtrlParam(int which, float v) {
  portENTER_CRITICAL(&g_ctrlMux);
  switch (which) {
    case 0: g_ctrl.kp   = v;             break;
    case 1: g_ctrl.ki   = v;             break;
    case 2: g_ctrl.kd   = v;             break;
    case 3: g_ctrl.base = (int32_t)v;    break;
  }
  portEXIT_CRITICAL(&g_ctrlMux);
}

static void setPaused(bool p) {
  portENTER_CRITICAL(&g_ctrlMux);
  g_ctrl.paused = p;
  portEXIT_CRITICAL(&g_ctrlMux);
}

// ---------------- raw motor output (motorTask + setup only) ----------------
static void rawMotor(int fwdPin, int revPin, int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed >= 0) { analogWrite(fwdPin, speed); analogWrite(revPin, 0); }
  else            { analogWrite(fwdPin, 0);     analogWrite(revPin, -speed); }
}

static void rawMotors(int left, int right) {
  rawMotor(M1_FWD, M1_REV, left);    // M1 = left
  rawMotor(M2_FWD, M2_REV, right);   // M2 = right
}

/* =====================================================================
   BT console (btTask only)
   ===================================================================== */

// BluetoothSerial lacks printf() on some core versions -> safe wrapper
static void btPrintf(const char *fmt, ...) {
  char buf[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  SerialBT.println(buf);
}

static const char *PARAM_NAMES[4] = { "kp",   "ki",    "kd",    "speed" };
static const float PARAM_LO[4]    = { 0.0f,   0.0f,    0.0f,    0.0f    };
static const float PARAM_HI[4]    = { 500.0f, 100.0f,  500.0f,  255.0f  };

static bool parseFloatStrict(const String &s, float &out) {
  if (s.length() == 0) return false;
  const char *c = s.c_str();
  char *end = nullptr;
  float v = strtof(c, &end);
  if (end == c || *end != '\0') return false;   // not a clean number
  out = v;
  return true;
}

// returns: 1 = valid value, 0 = cancelled, -1 = connection dropped
static int readBTValue(float lo, float hi, float &out) {
  String line;
  for (;;) {
    while (SerialBT.available()) {
      char ch = (char)SerialBT.read();
      if (ch == '\n' || ch == '\r') {
        if (line.length() == 0) continue;
        line.trim();
        if (line.equalsIgnoreCase("x")) return 0;
        float v;
        if (parseFloatStrict(line, v) && v >= lo && v <= hi) {
          out = v;
          return 1;
        }
        btPrintf("invalid — enter %.1f to %.1f, or x to cancel", lo, hi);
        line = "";
      } else if (line.length() < 15) {
        line += ch;
      }
    }
    if (!SerialBT.connected()) return -1;
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void handleBtLine(String cmd) {
  cmd.trim();
  cmd.toLowerCase();

  int which = -1;
  if      (cmd == "kp")    which = 0;
  else if (cmd == "ki")    which = 1;
  else if (cmd == "kd")    which = 2;
  else if (cmd == "speed") which = 3;

  if (which >= 0) {
    Ctrl c = getCtrl();
    float cur = (which == 0) ? c.kp : (which == 1) ? c.ki
             : (which == 2) ? c.kd : (float)c.base;

    setPaused(true);                     // robot brakes within ~20 ms
    btPrintf("STOPPED — %s = %.2f", PARAM_NAMES[which], cur);
    btPrintf("enter new %s (%.1f to %.1f), or x to cancel",
             PARAM_NAMES[which], PARAM_LO[which], PARAM_HI[which]);

    float v = 0;
    int r = readBTValue(PARAM_LO[which], PARAM_HI[which], v);

    if (r == 1) {
      setCtrlParam(which, v);
      btPrintf("%s = %.2f — resuming in 1 s", PARAM_NAMES[which], v);
      vTaskDelay(pdMS_TO_TICKS(1000));
      setPaused(false);
    } else if (r == 0) {
      SerialBT.println("cancelled — resuming in 1 s");
      vTaskDelay(pdMS_TO_TICKS(1000));
      setPaused(false);
    } else {
      // phone dropped mid-prompt: stay STOPPED.
      // Reconnect (greeting shows state), then send "start".
    }
  }
  else if (cmd == "stop") { setPaused(true);  SerialBT.println("STOPPED — send 'start' to resume"); }
  else if (cmd == "start" || cmd == "go") { setPaused(false); SerialBT.println("RUNNING"); }
  else if (cmd == "status") {
    Ctrl c = getCtrl();
    btPrintf("kp=%.2f  ki=%.2f  kd=%.2f  speed=%d  %s",
             c.kp, c.ki, c.kd, (int)c.base, c.paused ? "[STOPPED]" : "[RUNNING]");
  }
  else {
    SerialBT.println("cmds: kp | ki | kd | speed | status | stop | start");
  }
}

static void btTask(void *arg) {
  bool   wasConnected = false;
  bool   wasStarted   = false;
  String line;

  for (;;) {
    bool conn = SerialBT.connected();
    Ctrl c = getCtrl();

    if (conn && !wasConnected) {
      wasConnected = true;
      SerialBT.println();
      SerialBT.println("=== TERMINATOR ONLINE ===");
      btPrintf("state: %s", c.paused ? "STOPPED" : "RUNNING");
      SerialBT.println("cmds: kp | ki | kd | speed | status | stop | start");
    } else if (!conn && wasConnected) {
      wasConnected = false;
      line = "";                          // drop half-typed input
    }

    if (c.started && !wasStarted) {       // "start message" when motion begins
      wasStarted = true;
      SerialBT.println("TERMINATOR: line following ENGAGED");
    }

    while (SerialBT.available()) {
      char ch = (char)SerialBT.read();
      if (ch == '\n' || ch == '\r') {
        if (line.length() > 0) {
          handleBtLine(line);
          line = "";
        }
      } else if (line.length() < 31) {    // cap line length
        line += ch;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(BT_TASK_PERIOD_MS));
  }
}

/* =====================================================================
   CORE 1 — sensors + PID
   ===================================================================== */
static void sensorTask(void *arg) {
  Serial.printf("[PID] running on core %d\n", xPortGetCoreID());

  bool wasPaused   = false;
  bool firstSample = true;                // suppress derivative kick on (re)start

  vTaskDelay(pdMS_TO_TICKS(STARTUP_GRACE_MS));

  portENTER_CRITICAL(&g_ctrlMux);         // tell BT console we're moving
  g_ctrl.started = true;
  portEXIT_CRITICAL(&g_ctrlMux);

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(LOOP_PERIOD_MS);

  for (;;) {
    Ctrl c = getCtrl();

    // ---- paused: hold motors at 0, keep watchdog fed ----
    if (c.paused) {
      if (!wasPaused) {                    // brake once, wipe PID memory
        error = previousError = integral = lastKnownError = 0;
        wasPaused = true;
        firstSample = true;
      }
      publishCmd(0, 0);                    // fresh stamp, watchdog stays quiet
      vTaskDelayUntil(&lastWake, period);
      continue;
    }
    if (wasPaused) {                       // just resumed
      wasPaused = false;
      firstSample = true;
    }

    // ---- 1. Read sensors ----
    int lm = (digitalRead(SENSOR_LEFTMOST)  == LINE_ACTIVE_LEVEL) ? 1 : 0;
    int l  = (digitalRead(SENSOR_LEFT)      == LINE_ACTIVE_LEVEL) ? 1 : 0;
    int m  = (digitalRead(SENSOR_MIDDLE)    == LINE_ACTIVE_LEVEL) ? 1 : 0;
    int r  = (digitalRead(SENSOR_RIGHT)     == LINE_ACTIVE_LEVEL) ? 1 : 0;
    int rm = (digitalRead(SENSOR_RIGHTMOST) == LINE_ACTIVE_LEVEL) ? 1 : 0;

    int activeCount = lm + l + m + r + rm;

    // ---- 2. Position -> error ----
    if (activeCount == 5 && STOP_ON_FULL_LINE) {
      publishCmd(0, 0);
      vTaskSuspend(NULL);                  // finished: halt this task
    }

    if (activeCount > 0) {
      error = (float)(lm * -4 + l * -2 + m * 0 + r * 2 + rm * 4) / activeCount;
      lastKnownError = error;
    } else {
      error = (lastKnownError >= 0) ? 6.0f : -6.0f;   // line lost: hunt
      integral = 0;
    }

    // ---- 3. PID ----
    if (firstSample) {                     // no D-spike after pause/resume
      previousError = error;
      integral = 0;
      firstSample = false;
    }

    integral += error;
    integral = constrain(integral, -50.0f, 50.0f);    // anti-windup
    if (error == 0) integral = 0;

    derivative = error - previousError;
    float correction = c.kp * error + c.ki * integral + c.kd * derivative;
    previousError = error;

    // ---- 4. Publish (SWAP +/- if robot steers wrong way) ----
    publishCmd(c.base + (int)correction, c.base - (int)correction);

    // ---- 5. Debug: USB only, never sent over BT ----
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 250) {
      lastPrint = millis();
      Serial.printf("e:%5.2f P:%6.1f D:%6.1f | kp:%.1f ki:%.1f kd:%.1f spd:%d | wd:%lu\n",
                    error, c.kp * error, c.kd * derivative,
                    c.kp, c.ki, c.kd, (int)c.base,
                    (unsigned long)g_cmdTimeouts);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

/* =====================================================================
   CORE 0 — motor output + watchdog
   ===================================================================== */
static void motorTask(void *arg) {
  int lastL = 0, lastR = 0;

  for (;;) {
    MotorCmd c = readCmd();

    if (millis() - c.stamp > CMD_TIMEOUT_MS) {
      if (lastL != 0 || lastR != 0) {      // PID side silent -> stop
        rawMotors(0, 0);
        lastL = lastR = 0;
        g_cmdTimeouts++;
      }
    } else if (c.left != lastL || c.right != lastR) {
      rawMotors(c.left, c.right);          // apply only on change
      lastL = c.left;
      lastR = c.right;
    }

    vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
  }
}

/* ===================================================================== */
void setup() {
  Serial.begin(115200);
  SerialBT.begin("terminator");            // device name "terminator"
  Serial.println("terminator booted — pair over Bluetooth");

  pinMode(SENSOR_LEFTMOST,  INPUT);
  pinMode(SENSOR_LEFT,      INPUT);
  pinMode(SENSOR_MIDDLE,    INPUT);
  pinMode(SENSOR_RIGHT,     INPUT);
  pinMode(SENSOR_RIGHTMOST, INPUT);

  pinMode(M1_FWD, OUTPUT);
  pinMode(M1_REV, OUTPUT);
  pinMode(M2_FWD, OUTPUT);
  pinMode(M2_REV, OUTPUT);
  pinMode(STBY,   OUTPUT);
  digitalWrite(STBY, HIGH);

  rawMotors(0, 0);   // safe state while still single-threaded

  //                     function     name         stack  arg  prio  handle              core
  xTaskCreatePinnedToCore(sensorTask, "sensor_pid",  4096, NULL, 3, &sensorTaskHandle,   1);
  xTaskCreatePinnedToCore(motorTask,  "motor_out",   4096, NULL, 2, &motorTaskHandle,    0);
  xTaskCreatePinnedToCore(btTask,     "bt_console",  8192, NULL, 1, &btTaskHandle,       0);
}

void loop() {
  vTaskDelay(portMAX_DELAY);   // loopTask idles forever
}
