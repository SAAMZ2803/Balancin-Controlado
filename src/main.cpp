#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <WebServer.h>

TFT_eSPI tft = TFT_eSPI();

// --- ADC y calibracion ---
// IMPORTANTE: usar pin del ADC1 (32,33,34,35,36,39) porque WiFi bloquea ADC2.
// GPIO15 estaba en ADC2 -> conflicto con softAP. GPIO36 es input-only, ideal.
constexpr uint8_t  kAdcPin         = 36;
constexpr float    kAdcMax         = 4095.0f;
constexpr float    kVref           = 3.3f;
constexpr int      kAdcMin         = 3791;
constexpr int      kAdcMaxCal      = 2520;
constexpr float    kAngleMaxDeg    = 64.0f;
constexpr int      kAvgSamples     = 10;
constexpr float    kZeroThresholdDeg = 1.0f;

// --- PWM motor ---
constexpr uint8_t  kMotorPwmPin      = 25;
constexpr uint8_t  kPwmChannel       = 0;
constexpr uint32_t kPwmFrequencyHz   = 200000;
constexpr uint8_t  kPwmResolutionBits = 8;
constexpr uint16_t kPwmMaxDuty       = (1U << kPwmResolutionBits) - 1U;
constexpr unsigned long kRampDurationMs = 5000;

// --- Controlador PI con anti-windup. Ganancias para u en [0,255]. ---
constexpr float kPidKp = 0.0163f;
constexpr float kPidKi = 0.0698f;
constexpr float kPidKd = 0.0f;

constexpr float kPidUMax = 255.0f;
constexpr float kPidUMin = 0.0f;

// Mapeo de zona muerta del motor: u in [0,255] -> PWM in [kPwmDeadbandMin, 255]
constexpr float kPwmDeadbandMin = 120.0f;

// --- WiFi Access Point ---
constexpr char     kApSsid[]        = "Balancin";
constexpr char     kApPassword[]    = "12345678";
constexpr uint16_t kHttpPort        = 80;

// --- Scheduler no bloqueante ---
constexpr unsigned long kControlPeriodMs   = 20;   // 50 Hz lazo de control
constexpr unsigned long kTelemetryPeriodMs = 100;  // 10 Hz telemetria serial

// ─────────────────────────────────────────────────────────────────────────────

enum class MotorMode {
  Idle,
  RampToFull,
  FullOn,
  PiControl,
};

MotorMode    motorMode    = MotorMode::Idle;
unsigned long modeStartMs = 0;
bool          motorEnabled = false;

unsigned long startTimeMs  = 0;
bool          timingActive = false;

// Estado del PID
float piSetpoint        = 0.0f;
float pidErrorSum       = 0.0f;
float pidErrorPrev      = 0.0f;
unsigned long pidPrevMs = 0;
unsigned long pidLastReadMs = 0;

// Ultima lectura disponible para el endpoint /state
volatile float gAngleDeg      = 0.0f;
volatile float gMotorDutyPct  = 0.0f;

// Buffer serial
String serialBuffer = "";
unsigned long serialBufferLastCharMs = 0;
constexpr unsigned long kSerialBufferIdleMs = 200;

// Schedulers
unsigned long lastControlMs   = 0;
unsigned long lastTelemetryMs = 0;

WebServer server(kHttpPort);

// ─────────────────────────────────────────────────────────────────────────────
// HTML embebido en flash
// ─────────────────────────────────────────────────────────────────────────────

const char kHtmlPage[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Balancin Control</title>
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  body {
    margin: 0; padding: 16px;
    font-family: -apple-system, system-ui, sans-serif;
    background: #0f1419; color: #e6edf3;
    min-height: 100vh;
  }
  h1 {
    margin: 0 0 16px; font-size: 1.4rem; text-align: center;
    color: #4ade80; letter-spacing: 1px;
  }
  .card {
    background: #1c2128; border-radius: 12px;
    padding: 16px; margin-bottom: 12px;
    border: 1px solid #30363d;
  }
  .row { display: flex; justify-content: space-between; align-items: center; margin: 8px 0; }
  .label { color: #8b949e; font-size: 0.85rem; }
  .value { font-family: 'SF Mono', monospace; font-weight: 600; font-size: 1.1rem; }
  .value.big { font-size: 1.6rem; color: #4ade80; }
  .bar-bg { background: #30363d; border-radius: 6px; height: 10px; overflow: hidden; }
  .bar-fill { background: linear-gradient(90deg,#4ade80,#22c55e); height: 100%; transition: width .15s; }
  .btn-row { display: grid; grid-template-columns: repeat(2,1fr); gap: 8px; }
  button {
    background: #21262d; color: #e6edf3; border: 1px solid #30363d;
    padding: 14px; border-radius: 8px; font-size: 1rem;
    cursor: pointer; transition: background .15s; font-weight: 500;
  }
  button:active { background: #4ade80; color: #0f1419; }
  button.danger { color: #f87171; border-color: #5b2727; }
  button.danger:active { background: #f87171; color: #0f1419; }
  button.primary { color: #4ade80; border-color: #2d5b3a; }
  input[type=range] { width: 100%; }
  input[type=number] {
    background: #0f1419; color: #e6edf3; border: 1px solid #30363d;
    padding: 8px 12px; border-radius: 6px; width: 80px; font-size: 1rem;
    text-align: center;
  }
  .sp-row { display: flex; gap: 8px; align-items: center; margin-top: 8px; }
  .badge {
    display: inline-block; padding: 3px 10px; border-radius: 12px;
    font-size: 0.75rem; background: #30363d; color: #8b949e;
  }
  .badge.active { background: #2d5b3a; color: #4ade80; }
  .status-dot {
    width: 8px; height: 8px; border-radius: 50%;
    background: #f87171; display: inline-block; margin-right: 6px;
  }
  .status-dot.ok { background: #4ade80; }
</style>
</head>
<body>
<h1>BALANCIN CONTROL</h1>

<div class="card">
  <div class="row">
    <span class="label">Angulo actual</span>
    <span class="value big" id="angle">--.--&deg;</span>
  </div>
  <div class="row">
    <span class="label">Setpoint</span>
    <span class="value" id="setpoint">--.--&deg;</span>
  </div>
  <div class="row">
    <span class="label">PWM</span>
    <span class="value" id="pwm">--%</span>
  </div>
  <div class="bar-bg"><div class="bar-fill" id="pwmbar" style="width:0%"></div></div>
  <div class="row" style="margin-top:12px">
    <span class="label">Modo</span>
    <span class="badge" id="modebadge">--</span>
  </div>
  <div class="row">
    <span class="label">Error</span>
    <span class="value" id="error">--</span>
  </div>
  <div class="row">
    <span class="label"><span class="status-dot" id="dot"></span>Conexion</span>
    <span class="value" id="conn" style="font-size:.9rem">...</span>
  </div>
</div>

<div class="card">
  <div class="label" style="margin-bottom:10px">Modos rapidos</div>
  <div class="btn-row">
    <button class="danger" onclick="cmd('mode','0')">Apagar</button>
    <button onclick="cmd('mode','1')">Rampa 5s</button>
    <button onclick="cmd('mode','2')">100%</button>
    <button onclick="cmd('mode','3')">Toggle</button>
  </div>
</div>

<div class="card">
  <div class="label" style="margin-bottom:10px">Control PID (Modo 4)</div>
  <input type="range" id="slider" min="0" max="64" step="0.5" value="30"
         oninput="document.getElementById('spIn').value=this.value">
  <div class="sp-row">
    <input type="number" id="spIn" min="0" max="64" step="0.5" value="30"
           oninput="document.getElementById('slider').value=this.value">
    <span class="label">grados</span>
    <button class="primary" style="flex:1" onclick="applyPid()">Aplicar PID</button>
  </div>
</div>

<script>
let okCount = 0;

async function cmd(key, val) {
  try {
    const body = new URLSearchParams({[key]: val});
    const r = await fetch('/cmd', {method:'POST', body});
    if (!r.ok) throw 0;
  } catch(e) { console.error(e); }
}

function applyPid() {
  const sp = document.getElementById('spIn').value;
  cmd('mode','4').then(()=>cmd('setpoint', sp));
}

const MODES = ['Idle','Rampa','Full ON','PID'];

async function poll() {
  try {
    const r = await fetch('/state');
    const s = await r.json();
    document.getElementById('angle').textContent = s.angle.toFixed(2)+'°';
    document.getElementById('setpoint').textContent = s.setpoint.toFixed(2)+'°';
    document.getElementById('pwm').textContent = s.pwm.toFixed(1)+'%';
    document.getElementById('pwmbar').style.width = s.pwm.toFixed(1)+'%';
    document.getElementById('error').textContent = (s.setpoint - s.angle).toFixed(2);
    const badge = document.getElementById('modebadge');
    badge.textContent = MODES[s.mode] || '?';
    badge.className = 'badge' + (s.enabled ? ' active' : '');
    document.getElementById('dot').className = 'status-dot ok';
    document.getElementById('conn').textContent = 'OK';
    okCount++;
  } catch(e) {
    document.getElementById('dot').className = 'status-dot';
    document.getElementById('conn').textContent = 'sin conexion';
  }
}

setInterval(poll, 200);
poll();
</script>
</body>
</html>
)HTML";

// ─────────────────────────────────────────────────────────────────────────────

void printMenu() {
  Serial.println();
  Serial.println("=== Menu PWM motor ===");
  Serial.println("1 -> Rampa 0% a 100% en 5 s");
  Serial.println("2 -> 100% inmediato");
  Serial.println("0 -> Apagar motor");
  Serial.println("3 -> Encender / apagar motor");
  Serial.println("4 -> Control PID (setpoint por serial)");
  Serial.println();
  Serial.println("Modo 4: envia el angulo deseado, ej: 30.0");
}

void setMotorDutyPercent(float dutyPercent) {
  dutyPercent = constrain(dutyPercent, 0.0f, 100.0f);
  const uint16_t duty = static_cast<uint16_t>((dutyPercent / 100.0f) * kPwmMaxDuty);
  ledcWrite(kPwmChannel, duty);
}

float runPidController(float angleDeg) {
  const unsigned long now = millis();
  const float dt_ms = (pidPrevMs == 0) ? 40.0f : static_cast<float>(now - pidPrevMs);

  if ((now - pidLastReadMs) > 1000UL) {
    pidErrorSum = 0.0f;
  }
  pidLastReadMs = now;

  const float dt_s = dt_ms / 1000.0f;
  const float error = piSetpoint - angleDeg;
  pidErrorSum += error * dt_s;
  const float errorDeriv = (dt_s > 0.0f) ? (error - pidErrorPrev) / dt_s : 0.0f;

  float u = kPidKp * error + kPidKi * pidErrorSum + kPidKd * errorDeriv;
  if (u > kPidUMax) u = kPidUMax;
  if (u < kPidUMin) u = kPidUMin;

  float pwmMapped = 0.0f;
  if (u < 5.0f) {
    pwmMapped = 0.0f;
  } else {
    pwmMapped = kPwmDeadbandMin + (u / kPidUMax) * (kPidUMax - kPwmDeadbandMin);
  }
  const float dutyPercent = (pwmMapped / kPidUMax) * 100.0f;

  setMotorDutyPercent(dutyPercent);

  pidErrorPrev = error;
  pidPrevMs    = now;
  return dutyPercent;
}

float getMotorDutyPercent() {
  if (!motorEnabled) {
    setMotorDutyPercent(0.0f);
    return 0.0f;
  }
  switch (motorMode) {
    case MotorMode::RampToFull: {
      const unsigned long elapsedMs = millis() - modeStartMs;
      if (elapsedMs >= kRampDurationMs) {
        motorMode = MotorMode::FullOn;
        setMotorDutyPercent(100.0f);
        return 100.0f;
      }
      const float duty = (elapsedMs * 100.0f) / static_cast<float>(kRampDurationMs);
      setMotorDutyPercent(duty);
      return duty;
    }
    case MotorMode::FullOn:
      setMotorDutyPercent(100.0f);
      return 100.0f;
    default:
      setMotorDutyPercent(0.0f);
      return 0.0f;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Comandos: tanto serial como HTTP usan estas funciones para mantener una
// sola fuente de verdad sobre las transiciones de modo.
// ─────────────────────────────────────────────────────────────────────────────

void applyModeCommand(char cmd) {
  switch (cmd) {
    case '1':
      motorEnabled = true;
      motorMode    = MotorMode::RampToFull;
      modeStartMs  = millis();
      setMotorDutyPercent(0.0f);
      Serial.println("Modo 1: rampa 0->100% en 5 s");
      break;
    case '2':
      motorEnabled = true;
      motorMode    = MotorMode::FullOn;
      setMotorDutyPercent(100.0f);
      Serial.println("Modo 2: 100% inmediato");
      break;
    case '0':
      motorEnabled = false;
      motorMode    = MotorMode::Idle;
      pidErrorSum  = 0.0f;
      pidErrorPrev = 0.0f;
      pidPrevMs    = 0;
      setMotorDutyPercent(0.0f);
      Serial.println("Motor apagado");
      break;
    case '3':
      motorEnabled = !motorEnabled;
      if (motorEnabled) {
        motorMode = MotorMode::FullOn;
        setMotorDutyPercent(100.0f);
        Serial.println("Motor encendido");
      } else {
        motorMode    = MotorMode::Idle;
        pidErrorSum  = 0.0f;
        pidErrorPrev = 0.0f;
        pidPrevMs    = 0;
        setMotorDutyPercent(0.0f);
        Serial.println("Motor apagado");
      }
      break;
    case '4':
      if (motorMode == MotorMode::PiControl && motorEnabled) {
        // Ya estamos en PID: no resetear el integrador, solo confirmar.
        Serial.printf("Modo 4 PID ya activo. Setpoint actual: %.2f\n", piSetpoint);
      } else {
        motorEnabled  = true;
        motorMode     = MotorMode::PiControl;
        pidErrorSum   = 0.0f;
        pidErrorPrev  = 0.0f;
        pidPrevMs     = 0;
        pidLastReadMs = millis();
        Serial.printf("Modo 4 PID activo. Setpoint actual: %.2f\n", piSetpoint);
      }
      break;
  }
}

bool applySetpoint(float sp) {
  if (sp < 0.0f || sp > kAngleMaxDeg) return false;
  piSetpoint    = sp;
  // No reseteamos el integrador: mantiene la compensacion de gravedad y la
  // transicion al nuevo setpoint es continua (sin apagar el motor).
  // El reset solo ocurre al entrar a PID o tras inactividad >1s en runPidController.
  pidLastReadMs = millis();
  Serial.printf("Nuevo setpoint: %.2f grados\n", piSetpoint);
  return true;
}

void processPiSerialBuffer() {
  if (serialBuffer.length() == 0) return;

  if (serialBuffer.length() == 1 &&
      serialBuffer[0] >= '0' && serialBuffer[0] <= '4') {
    const char cmd = serialBuffer[0];
    serialBuffer = "";
    if (cmd == '4') {
      Serial.printf("Ya estas en Modo 4 PID. Setpoint actual: %.2f\n", piSetpoint);
    } else {
      applyModeCommand(cmd);
    }
    return;
  }

  const String original = serialBuffer;
  const float sp = serialBuffer.toFloat();
  serialBuffer = "";
  if (!applySetpoint(sp)) {
    Serial.printf("Setpoint invalido '%s' o fuera de rango [0, %.0f]\n",
                  original.c_str(), kAngleMaxDeg);
  }
}

void handleSerialMenu() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    if (motorMode == MotorMode::PiControl) {
      if (c == '\n' || c == '\r') {
        processPiSerialBuffer();
        continue;
      }
      if (c >= ' ') {
        serialBuffer          += c;
        serialBufferLastCharMs = millis();
      }
      continue;
    }

    serialBuffer = "";
    if (c >= '0' && c <= '4') {
      applyModeCommand(c);
    }
  }

  if (motorMode == MotorMode::PiControl &&
      serialBuffer.length() > 0 &&
      (millis() - serialBufferLastCharMs) > kSerialBufferIdleMs) {
    processPiSerialBuffer();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP handlers
// ─────────────────────────────────────────────────────────────────────────────

void handleRoot() {
  server.send_P(200, "text/html", kHtmlPage);
}

void handleState() {
  char buf[192];
  snprintf(buf, sizeof(buf),
    "{\"angle\":%.2f,\"setpoint\":%.2f,\"pwm\":%.2f,\"mode\":%d,\"enabled\":%s}",
    gAngleDeg, piSetpoint, gMotorDutyPct,
    static_cast<int>(motorMode),
    motorEnabled ? "true" : "false");
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", buf);
}

void handleCmd() {
  bool ok = false;
  String msg = "ok";

  if (server.hasArg("mode")) {
    const String m = server.arg("mode");
    if (m.length() == 1 && m[0] >= '0' && m[0] <= '4') {
      applyModeCommand(m[0]);
      ok = true;
    } else {
      msg = "modo invalido";
    }
  }
  if (server.hasArg("setpoint")) {
    const float sp = server.arg("setpoint").toFloat();
    if (applySetpoint(sp)) {
      ok = true;
    } else {
      msg = "setpoint fuera de rango";
    }
  }

  server.sendHeader("Access-Control-Allow-Origin", "*");
  if (ok) {
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    String r = "{\"ok\":false,\"msg\":\"" + msg + "\"}";
    server.send(400, "application/json", r);
  }
}

void handleNotFound() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(404, "text/plain", "Not Found");
}

// ─────────────────────────────────────────────────────────────────────────────
// Lazo de control (extraido para llamarlo desde el scheduler)
// ─────────────────────────────────────────────────────────────────────────────

void runControlStep() {
  long sum = 0;
  for (int i = 0; i < kAvgSamples; i++) {
    sum += analogRead(kAdcPin);
  }
  const float adcAvg  = sum / static_cast<float>(kAvgSamples);
  float angleDeg = 0.0f;
  if (kAdcMaxCal != kAdcMin) {
    angleDeg = (adcAvg - kAdcMin) * (kAngleMaxDeg / static_cast<float>(kAdcMaxCal - kAdcMin));
  }
  angleDeg = constrain(angleDeg, 0.0f, kAngleMaxDeg);

  float motorDutyPercent = 0.0f;
  if (motorMode == MotorMode::PiControl && motorEnabled) {
    motorDutyPercent = runPidController(angleDeg);
  } else {
    motorDutyPercent = getMotorDutyPercent();
  }

  gAngleDeg     = angleDeg;
  gMotorDutyPct = motorDutyPercent;

  if (!timingActive && angleDeg <= kZeroThresholdDeg) {
    startTimeMs  = millis();
    timingActive = true;
  }
  if (timingActive && angleDeg >= kAngleMaxDeg) {
    Serial.printf("t_0_to_max_ms:%lu\n", millis() - startTimeMs);
    timingActive = false;
  }
}

// ─────────────────────────────────────────────────────────────────────────────

void setup() {
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);

  Serial.begin(115200);
  delay(100);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  ledcSetup(kPwmChannel, kPwmFrequencyHz, kPwmResolutionBits);
  ledcAttachPin(kMotorPwmPin, kPwmChannel);
  setMotorDutyPercent(0.0f);
  motorEnabled = false;

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Balancin WiFi");

  // --- WiFi AP ---
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPassword);
  const IPAddress ip = WiFi.softAPIP();

  Serial.printf("AP SSID: %s\n", kApSsid);
  Serial.printf("AP Pass: %s\n", kApPassword);
  Serial.printf("AP IP:   %s\n", ip.toString().c_str());

  tft.setTextSize(1);
  tft.setCursor(10, 45);
  tft.printf("SSID: %s", kApSsid);
  tft.setCursor(10, 60);
  tft.printf("Pass: %s", kApPassword);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(10, 80);
  tft.println(ip.toString());
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(10, 110);
  tft.println("Menu: 0 1 2 3 4");

  // --- HTTP routes ---
  server.on("/", HTTP_GET, handleRoot);
  server.on("/state", HTTP_GET, handleState);
  server.on("/cmd", HTTP_POST, handleCmd);
  server.on("/cmd", HTTP_OPTIONS, [](){
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.sendHeader("Access-Control-Allow-Methods", "POST, GET, OPTIONS");
    server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
    server.send(204);
  });
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server iniciado en puerto 80");

  printMenu();
}

void loop() {
  // 1) Atender peticiones HTTP siempre que sea posible
  server.handleClient();

  // 2) Procesar entrada serial sin bloquear
  handleSerialMenu();

  // 3) Lazo de control a frecuencia fija
  const unsigned long now = millis();
  if (now - lastControlMs >= kControlPeriodMs) {
    lastControlMs = now;
    runControlStep();
  }

  // 4) Telemetria serial a 10 Hz para no saturar el monitor
  if (now - lastTelemetryMs >= kTelemetryPeriodMs) {
    lastTelemetryMs = now;
    Serial.printf("Angulo:%.2f | Setpoint:%.2f | PWM:%.2f%% | Modo:%d | Error:%.2f | Isum:%.2f | Eprev:%.2f\n",
                  gAngleDeg,
                  piSetpoint,
                  gMotorDutyPct,
                  static_cast<int>(motorMode),
                  piSetpoint - gAngleDeg,
                  pidErrorSum,
                  pidErrorPrev);
  }
}
