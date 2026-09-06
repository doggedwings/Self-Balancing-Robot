/*
  BalanceBot — ESP32 self-balancing robot

  Pin map:
    GPIO 32  Driver 1 STEP        GPIO 33  Driver 1 DIR
    GPIO 26  Driver 2 STEP        GPIO 27  Driver 2 DIR
    GPIO 25  Both EN (active low, 10k pull-up to 3.3V)
    GPIO 19  MPU6050 SDA          GPIO 18  MPU6050 SCL

  1/8 microstepping: MS1 high, MS2 high, MS3 low = 1600 steps/rev.
  WiFi AP "BalanceBot" / pass "balancebot" -> http://192.168.4.1

  Gains persist across reboots (saved to flash on every Apply).
  Gains can also be set from the address bar, e.g.
    http://192.168.4.1/gains?kpa=40&kda=1.6&trim=3

  Adjust number on line 382 to increase or decrease max speed.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Preferences.h>
#include "soc/gpio_reg.h"

#define M1_STEP 32
#define M1_DIR  33
#define M2_STEP 26
#define M2_DIR  27
#define MOT_EN  25
#define I2C_SDA 19
#define I2C_SCL 18

#define M1_INVERT  false
#define M2_INVERT  true

#define STEPS_PER_REV   1600.0f
#define MAX_SPEED       6000.0f
#define MAX_ACCEL       96000.0f
#define FALL_ANGLE      40.0f
#define LOOP_HZ         100.0f
#define SPEED_LOOP_DIV  4
#define ISR_HZ          20000
#define CMD_TIMEOUT_MS  1000

volatile float Kp_angle = 40.0f;
volatile float Ki_angle = 0.5f;
volatile float Kd_angle = 2.4f;
volatile float Kp_speed = 0.002f;
volatile float Ki_speed = 0.004f;
volatile float angleTrim = 2.8f;

volatile float  motor1Speed = 0, motor2Speed = 0;
volatile bool   motorsEnabled = false;
volatile uint32_t lastCmdMs = 0;

float pitch = 0, gyroRate = 0;
float gyroBiasY = 0;
float angleIntegral = 0, speedIntegral = 0;
float targetAngle = 0;
float cmdSpeed = 0, cmdTurn = 0;
float cmdSpeedSmooth = 0, cmdTurnSmooth = 0;
bool  fallen = true;
uint32_t lastLoopUs = 0;
uint8_t speedLoopCounter = 0;

Preferences prefs;

hw_timer_t *stepTimer = NULL;
portMUX_TYPE stepMux = portMUX_INITIALIZER_UNLOCKED;
volatile int32_t acc1 = 0, acc2 = 0;
volatile int32_t isrSpeed1 = 0, isrSpeed2 = 0;

// IRAM-safe GPIO write. Handles both banks (pins above 31 use OUT1 registers).
static inline void IRAM_ATTR fastWrite(uint8_t pin, bool level) {
  if (pin < 32) {
    REG_WRITE(level ? GPIO_OUT_W1TS_REG : GPIO_OUT_W1TC_REG, 1UL << pin);
  } else {
    REG_WRITE(level ? GPIO_OUT1_W1TS_REG : GPIO_OUT1_W1TC_REG, 1UL << (pin - 32));
  }
}

void IRAM_ATTR onStepTimer() {
  fastWrite(M1_STEP, false);
  fastWrite(M2_STEP, false);

  int32_t s1 = isrSpeed1;
  if (s1 != 0) {
    fastWrite(M1_DIR, ((s1 > 0) != M1_INVERT));
    acc1 += (s1 > 0 ? s1 : -s1);
    if (acc1 >= ISR_HZ) { acc1 -= ISR_HZ; fastWrite(M1_STEP, true); }
  }

  int32_t s2 = isrSpeed2;
  if (s2 != 0) {
    fastWrite(M2_DIR, ((s2 > 0) != M2_INVERT));
    acc2 += (s2 > 0 ? s2 : -s2);
    if (acc2 >= ISR_HZ) { acc2 -= ISR_HZ; fastWrite(M2_STEP, true); }
  }
}

void setMotorSpeeds(float s1, float s2) {
  portENTER_CRITICAL(&stepMux);
  isrSpeed1 = (int32_t)s1;
  isrSpeed2 = (int32_t)s2;
  portEXIT_CRITICAL(&stepMux);
}

void loadGains() {
  prefs.begin("bot", true);
  Kp_angle  = prefs.getFloat("kpa", 40.0f);
  Ki_angle  = prefs.getFloat("kia", 0.0f);
  Kd_angle  = prefs.getFloat("kda", 1.6f);
  Kp_speed  = prefs.getFloat("kps", 0.0f);
  Ki_speed  = prefs.getFloat("kis", 0.0f);
  angleTrim = prefs.getFloat("trim", 3.0f);
  prefs.end();
}

void saveGains() {
  prefs.begin("bot", false);
  prefs.putFloat("kpa", Kp_angle);
  prefs.putFloat("kia", Ki_angle);
  prefs.putFloat("kda", Kd_angle);
  prefs.putFloat("kps", Kp_speed);
  prefs.putFloat("kis", Ki_speed);
  prefs.putFloat("trim", angleTrim);
  prefs.end();
}

#define MPU_ADDR 0x68

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuInit() {
  Wire.begin(I2C_SDA, I2C_SCL, 100000);
  Wire.setTimeOut(50);
  delay(50);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpuWrite(0x6B, 0x80); delay(100);
  mpuWrite(0x6B, 0x01); delay(50);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x00);
  mpuWrite(0x1C, 0x00);
  delay(50);
  return true;
}

void mpuRead(float &ax, float &ay, float &az, float &gy) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  if (Wire.available() < 14) { ax = ay = az = gy = 0; return; }
  int16_t rax = (Wire.read() << 8) | Wire.read();
  int16_t ray = (Wire.read() << 8) | Wire.read();
  int16_t raz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  Wire.read(); Wire.read();
  int16_t rgy = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  ax = rax / 16384.0f;
  ay = ray / 16384.0f;
  az = raz / 16384.0f;
  gy = rgy / 131.0f;
}

void calibrateGyro() {
  float sum = 0;
  int good = 0;
  for (int i = 0; i < 500; i++) {
    float ax, ay, az, gy;
    mpuRead(ax, ay, az, gy);
    if (!(ax == 0 && ay == 0 && az == 0)) { sum += gy; good++; }
    delay(3);
  }
  Serial.printf("  %d/500 reads ok\n", good);
  gyroBiasY = good ? sum / good : 0;
}

float accelPitch(float ax, float ay, float az) {
  return atan2f(-ax, az) * 57.2957795f;
}

WebServer server(80);

const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>BalanceBot</title><style>
body{font-family:system-ui,sans-serif;background:#15171a;color:#e6e6e6;margin:0;padding:16px}
h2{font-weight:500;margin:0 0 12px}
.row{display:flex;gap:8px;align-items:center;margin:6px 0}
.row label{width:110px;font-size:13px;color:#9aa0a6}
input[type=number]{width:80px;background:#22262b;border:1px solid #3a3f45;color:#e6e6e6;padding:5px;border-radius:5px}
button{background:#2d6cdf;border:0;color:#fff;padding:9px 16px;border-radius:6px;font-size:14px;cursor:pointer}
button.stop{background:#c0392b}
#tel{font-family:ui-monospace,monospace;font-size:13px;background:#1b1e22;padding:10px;border-radius:6px;line-height:1.7;white-space:pre}
#st{font-size:12px;color:#7d8590;margin:4px 0 0}
.keys{display:grid;grid-template-columns:repeat(3,54px);gap:6px;margin:14px 0}
.k{height:46px;background:#22262b;border:1px solid #3a3f45;border-radius:6px;display:flex;align-items:center;justify-content:center;font-size:15px;user-select:none;touch-action:none}
.k.on{background:#2d6cdf;border-color:#2d6cdf}
.sp{visibility:hidden}
small{color:#7d8590}
</style></head><body>
<h2>BalanceBot</h2>
<div id=tel>connecting...</div>
<div class=keys>
<div class="k sp"></div><div class=k id=kW>W</div><div class="k sp"></div>
<div class=k id=kA>A</div><div class=k id=kS>S</div><div class=k id=kD>D</div>
</div>
<div class=row><button id=en>Enable</button><button class=stop id=stop>STOP (space)</button></div>
<div class=row><label>Kp angle</label><input type=number step=0.5 id=kpa></div>
<div class=row><label>Ki angle</label><input type=number step=0.1 id=kia></div>
<div class=row><label>Kd angle</label><input type=number step=0.05 id=kda></div>
<div class=row><label>Kp speed</label><input type=number step=0.005 id=kps></div>
<div class=row><label>Ki speed</label><input type=number step=0.005 id=kis></div>
<div class=row><label>Angle trim</label><input type=number step=0.2 id=trim></div>
<div class=row><button id=apply>Apply and save</button></div>
<div id=st>fields sync with the robot until you edit one</div>
<small>Hold keys to drive. Gains are saved to flash on Apply.</small>
<script>
let held={w:0,a:0,s:0,d:0},dirty=false,touched=false;
const ids=['kpa','kia','kda','kps','kis','trim'];
const map={KeyW:'w',KeyA:'a',KeyS:'s',KeyD:'d',ArrowUp:'w',ArrowLeft:'a',ArrowDown:'s',ArrowRight:'d'};
function paint(){for(const k of['w','a','s','d'])document.getElementById('k'+k.toUpperCase()).classList.toggle('on',!!held[k]);}
for(const id of ids)document.getElementById(id).addEventListener('input',()=>{
 touched=true;document.getElementById('st').textContent='edited - press Apply to send';});
addEventListener('keydown',e=>{if(e.target.tagName==='INPUT')return;
 if(e.code==='Space'){estop();e.preventDefault();return;}
 const k=map[e.code];if(k&&!held[k]){held[k]=1;dirty=true;paint();e.preventDefault();}});
addEventListener('keyup',e=>{if(e.target.tagName==='INPUT')return;
 const k=map[e.code];if(k){held[k]=0;dirty=true;paint();e.preventDefault();}});
addEventListener('blur',()=>{held={w:0,a:0,s:0,d:0};dirty=true;paint();});
for(const k of ['w','a','s','d']){const el=document.getElementById('k'+k.toUpperCase());
 const on=e=>{held[k]=1;dirty=true;paint();e.preventDefault();};
 const off=e=>{held[k]=0;dirty=true;paint();e.preventDefault();};
 el.addEventListener('pointerdown',on);el.addEventListener('pointerup',off);el.addEventListener('pointerleave',off);}
function estop(){fetch('/stop');held={w:0,a:0,s:0,d:0};paint();}
document.getElementById('stop').onclick=estop;
document.getElementById('en').onclick=()=>fetch('/enable');
document.getElementById('apply').onclick=()=>{
 const q=ids.map(i=>i+'='+document.getElementById(i).value).join('&');
 document.getElementById('st').textContent='sending...';
 fetch('/gains?'+q).then(()=>{touched=false;
  document.getElementById('st').textContent='applied and saved';})
 .catch(()=>{document.getElementById('st').textContent='send FAILED - retry';});};
setInterval(()=>{fetch(`/c?f=${held.w-held.s}&t=${held.d-held.a}`);},150);
setInterval(async()=>{try{const r=await fetch('/t');const d=await r.json();
 document.getElementById('tel').textContent=
  `angle  ${d.a.toFixed(2)} deg\ntarget ${d.g.toFixed(2)} deg\nspeed  ${d.s.toFixed(0)} steps/s\nstate  ${d.e?(d.f?'FALLEN':'RUNNING'):'DISABLED'}`;
 if(!touched){kpa.value=d.p1;kia.value=d.i1;kda.value=d.d1;kps.value=d.p2;kis.value=d.i2;trim.value=d.tr;}
 }catch(e){}},200);
</script></body></html>)HTML";

void handleRoot()  { server.send_P(200, "text/html", PAGE); }

void handleCmd() {
  cmdSpeed = server.arg("f").toFloat();
  cmdTurn  = server.arg("t").toFloat();
  lastCmdMs = millis();
  server.send(200, "text/plain", "ok");
}

void handleStop() {
  motorsEnabled = false;
  cmdSpeed = cmdTurn = 0;
  server.send(200, "text/plain", "stopped");
}

void handleEnable() {
  angleIntegral = speedIntegral = 0;
  lastCmdMs = millis();
  motorsEnabled = true;
  server.send(200, "text/plain", "enabled");
}

void handleGains() {
  if (server.hasArg("kpa"))  Kp_angle  = server.arg("kpa").toFloat();
  if (server.hasArg("kia"))  Ki_angle  = server.arg("kia").toFloat();
  if (server.hasArg("kda"))  Kd_angle  = server.arg("kda").toFloat();
  if (server.hasArg("kps"))  Kp_speed  = server.arg("kps").toFloat();
  if (server.hasArg("kis"))  Ki_speed  = server.arg("kis").toFloat();
  if (server.hasArg("trim")) angleTrim = server.arg("trim").toFloat();
  angleIntegral = speedIntegral = 0;
  saveGains();
  Serial.printf("gains: Kp %.2f  Ki %.2f  Kd %.2f  kps %.3f  kis %.3f  trim %.1f\n",
                Kp_angle, Ki_angle, Kd_angle, Kp_speed, Ki_speed, angleTrim);
  server.send(200, "text/plain", "ok");
}

void handleTelemetry() {
  lastCmdMs = millis();
  char buf[220];
  snprintf(buf, sizeof(buf),
    "{\"a\":%.2f,\"g\":%.2f,\"s\":%.0f,\"e\":%d,\"f\":%d,"
    "\"p1\":%.2f,\"i1\":%.2f,\"d1\":%.2f,\"p2\":%.3f,\"i2\":%.3f,\"tr\":%.1f}",
    pitch, targetAngle, motor1Speed, motorsEnabled ? 1 : 0, fallen ? 1 : 0,
    Kp_angle, Ki_angle, Kd_angle, Kp_speed, Ki_speed, angleTrim);
  server.send(200, "application/json", buf);
}

void balanceTask(void *param) {
  lastLoopUs = micros();
  for (;;) {
    uint32_t now = micros();
    float dt = (now - lastLoopUs) / 1000000.0f;
    if (dt < 1.0f / LOOP_HZ) { vTaskDelay(1); continue; }
    lastLoopUs = now;
    if (dt > 0.05f) dt = 0.05f;

    float ax, ay, az, gy;
    mpuRead(ax, ay, az, gy);
    gyroRate = gy - gyroBiasY;

    float accAngle = accelPitch(ax, ay, az);
    pitch = 0.98f * (pitch + gyroRate * dt) + 0.02f * accAngle;

    if (fabsf(pitch - angleTrim) > FALL_ANGLE) {
      if (!fallen) { angleIntegral = speedIntegral = 0; }
      fallen = true;
      motor1Speed = motor2Speed = 0;
      setMotorSpeeds(0, 0);
      digitalWrite(MOT_EN, HIGH);
      vTaskDelay(1);
      continue;
    }
    if (fallen && fabsf(pitch - angleTrim) < 3.0f) fallen = false;

    if (!motorsEnabled || fallen) {
      motor1Speed = motor2Speed = 0;
      setMotorSpeeds(0, 0);
      digitalWrite(MOT_EN, HIGH);
      vTaskDelay(1);
      continue;
    }
    digitalWrite(MOT_EN, LOW);

    // If the browser goes away, stop driving but keep balancing.
    if (millis() - lastCmdMs > CMD_TIMEOUT_MS) { cmdSpeed = 0; cmdTurn = 0; }

    cmdSpeedSmooth += (cmdSpeed - cmdSpeedSmooth) * 0.05f;
    cmdTurnSmooth  += (cmdTurn  - cmdTurnSmooth)  * 0.05f;

    if (++speedLoopCounter >= SPEED_LOOP_DIV) {
      speedLoopCounter = 0;
      float actualSpeed = (motor1Speed + motor2Speed) * 0.5f;
      float wantSpeed   = cmdSpeedSmooth * MAX_SPEED * 0.20f;
      float sErr = wantSpeed - actualSpeed;
      if (Ki_speed == 0.0f) {
        speedIntegral = 0;
      } else {
        speedIntegral += sErr * (dt * SPEED_LOOP_DIV);
        speedIntegral = constrain(speedIntegral, -3000.0f, 3000.0f);
      }
      targetAngle = Kp_speed * sErr + Ki_speed * speedIntegral;
      targetAngle = constrain(targetAngle, -12.0f, 12.0f);
      targetAngle = -targetAngle + angleTrim;
    }

    float aErr = targetAngle - pitch;
    if (Ki_angle == 0.0f) {
      angleIntegral = 0;
    } else {
      angleIntegral += aErr * dt;
      angleIntegral = constrain(angleIntegral, -200.0f, 200.0f);
    }

    float dTerm = -gyroRate;
    float accel = Kp_angle * aErr + Ki_angle * angleIntegral + Kd_angle * dTerm;
    accel = constrain(accel, -MAX_ACCEL * dt, MAX_ACCEL * dt);

    float base = (motor1Speed + motor2Speed) * 0.5f + accel;
    base = constrain(base, -MAX_SPEED, MAX_SPEED);

    float turn = cmdTurnSmooth * MAX_SPEED * 0.15f;
    motor1Speed = constrain(base + turn, -MAX_SPEED, MAX_SPEED);
    motor2Speed = constrain(base - turn, -MAX_SPEED, MAX_SPEED);

    setMotorSpeeds(motor1Speed, motor2Speed);
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(M1_STEP, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_STEP, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(MOT_EN,  OUTPUT);
  digitalWrite(MOT_EN, HIGH);
  digitalWrite(M1_STEP, LOW); digitalWrite(M2_STEP, LOW);

  Serial.println("\nBalanceBot starting");

  loadGains();
  Serial.printf("gains: Kp %.2f  Ki %.2f  Kd %.2f  kps %.3f  kis %.3f  trim %.1f\n",
                Kp_angle, Ki_angle, Kd_angle, Kp_speed, Ki_speed, angleTrim);

  if (!mpuInit()) {
    Serial.println("MPU6050 NOT FOUND on GPIO19/18 - check wiring, halting");
    while (1) { delay(1000); }
  }
  Serial.println("MPU6050 ok. Hold the robot STILL and upright...");
  delay(1500);
  calibrateGyro();
  Serial.printf("gyro bias Y = %.3f dps\n", gyroBiasY);

  { float ax, ay, az, gy; mpuRead(ax, ay, az, gy); pitch = accelPitch(ax, ay, az); }
  Serial.printf("starting pitch = %.2f deg\n", pitch);

  Serial.printf("Arduino core %d.%d.%d\n",
                ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR,
                ESP_ARDUINO_VERSION_PATCH);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  stepTimer = timerBegin(1000000);
  if (!stepTimer) { Serial.println("timerBegin FAILED"); while (1) delay(1000); }
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 1000000 / ISR_HZ, true, 0);
#else
  stepTimer = timerBegin(0, 80, true);
  if (!stepTimer) { Serial.println("timerBegin FAILED"); while (1) delay(1000); }
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, 1000000 / ISR_HZ, true);
  timerAlarmEnable(stepTimer);
#endif

  Serial.println("timer ok");

  WiFi.mode(WIFI_AP);
  WiFi.softAP("BalanceBot", "balancebot");
  Serial.print("AP up at http://");
  Serial.println(WiFi.softAPIP());

  server.on("/",       handleRoot);
  server.on("/c",      handleCmd);
  server.on("/t",      handleTelemetry);
  server.on("/stop",   handleStop);
  server.on("/enable", handleEnable);
  server.on("/gains",  handleGains);
  server.begin();

  lastCmdMs = millis();
  xTaskCreatePinnedToCore(balanceTask, "balance", 8192, NULL, 5, NULL, 1);

  Serial.println("ready - open the page and press Enable");
}

void loop() {
  server.handleClient();
  delay(1);
}