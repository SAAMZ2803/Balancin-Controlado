#include <Arduino.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

// --- ADC y calibracion ---
constexpr uint8_t  kAdcPin         = 15;
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
// Kp dominante para respuesta rapida; Ki conservador (la planta es
// unidireccional: el motor solo empuja y la gravedad recupera, asi que
// el integrador solo compensa el sesgo gravitacional).
constexpr float kPidKp = 0.0163f;
constexpr float kPidKi = 0.0698f;
constexpr float kPidKd = 0.0f;

// Saturacion del esfuerzo de control (igual que original: 0..255)
constexpr float kPidUMax = 255.0f;
constexpr float kPidUMin = 0.0f;

// Mapeo de zona muerta del motor: u in [0,255] -> PWM in [kPwmDeadbandMin, 255]
constexpr float kPwmDeadbandMin = 120.0f;

// ─────────────────────────────────────────────────────────────────────────────

enum class MotorMode {
  Idle,
  RampToFull,
  FullOn,
  PiControl,   // Modo 4: lazo cerrado PID con setpoint desde serial
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

// Buffer para leer setpoint flotante desde serial
String serialBuffer = "";
unsigned long serialBufferLastCharMs = 0;
constexpr unsigned long kSerialBufferIdleMs = 200;  // si no llega nada en 200 ms, procesar buffer

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

// Calcula la salida del PID y actualiza el motor. Retorna duty [0-100].
// Implementacion portada desde codigo_control_PID.txt:
//   error = ref - cm
//   errorsuma     += error * dt/1000
//   errorderivativo = (error - errorantiguo) / (dt/1000)
//   u = kp*error + ki*errorsuma + kd*errorderivativo
//   u saturado en [0,255]
//   pwm = map(u, 0,255, 120,255)
float runPidController(float angleDeg) {
  const unsigned long now = millis();
  const float dt_ms = (pidPrevMs == 0) ? 40.0f : static_cast<float>(now - pidPrevMs);

  // Reset del integrador si no hubo actualizacion en >1s (igual que original)
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

  // Mapeo zona muerta: u[0..255] -> pwm[120..255]
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

// Calcula duty para modos sin PI. Retorna duty [0-100].
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

// Procesa el contenido actual de serialBuffer estando en modo PID.
// Si es un solo digito '0'..'4' lo trata como comando de modo,
// de lo contrario lo parsea como setpoint flotante.
void processPiSerialBuffer() {
  if (serialBuffer.length() == 0) return;

  if (serialBuffer.length() == 1 &&
      (serialBuffer[0] == '0' || serialBuffer[0] == '1' ||
       serialBuffer[0] == '2' || serialBuffer[0] == '3' ||
       serialBuffer[0] == '4')) {
    const char cmd = serialBuffer[0];
    serialBuffer = "";
    if (cmd == '1') {
      motorEnabled = true;
      motorMode    = MotorMode::RampToFull;
      modeStartMs  = millis();
      setMotorDutyPercent(0.0f);
      Serial.println("Modo 1: rampa 0->100% en 5 s");
    } else if (cmd == '2') {
      motorEnabled = true;
      motorMode    = MotorMode::FullOn;
      setMotorDutyPercent(100.0f);
      Serial.println("Modo 2: 100% inmediato");
    } else if (cmd == '0') {
      motorEnabled = false;
      motorMode    = MotorMode::Idle;
      pidErrorSum  = 0.0f;
      pidErrorPrev = 0.0f;
      pidPrevMs    = 0;
      setMotorDutyPercent(0.0f);
      Serial.println("Motor apagado");
    } else if (cmd == '3') {
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
    } else if (cmd == '4') {
      Serial.printf("Ya estas en Modo 4 PID. Setpoint actual: %.2f\n", piSetpoint);
    }
    return;
  }

  const String original = serialBuffer;
  const float sp = serialBuffer.toFloat();
  serialBuffer = "";
  if (sp >= 0.0f && sp <= kAngleMaxDeg) {
    piSetpoint    = sp;
    pidErrorSum   = 0.0f;
    pidErrorPrev  = 0.0f;
    pidPrevMs     = 0;
    pidLastReadMs = millis();
    Serial.printf("Nuevo setpoint: %.2f grados\n", piSetpoint);
  } else {
    Serial.printf("Setpoint invalido '%s' o fuera de rango [0, %.0f]\n",
                  original.c_str(), kAngleMaxDeg);
  }
}

void handleSerialMenu() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());

    // ── Modo PID activo: acumular hasta '\n'/'\r' o timeout de inactividad ───
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

    // ── Comandos de modo (fuera de PI) ───────────────────────────────────────
    serialBuffer = "";

    if (c == '1') {
      motorEnabled = true;
      motorMode    = MotorMode::RampToFull;
      modeStartMs  = millis();
      setMotorDutyPercent(0.0f);
      Serial.println("Modo 1: rampa 0->100% en 5 s");
    } else if (c == '2') {
      motorEnabled = true;
      motorMode    = MotorMode::FullOn;
      setMotorDutyPercent(100.0f);
      Serial.println("Modo 2: 100% inmediato");
    } else if (c == '0') {
      motorEnabled = false;
      motorMode    = MotorMode::Idle;
      pidErrorSum  = 0.0f;
      pidErrorPrev = 0.0f;
      pidPrevMs    = 0;
      setMotorDutyPercent(0.0f);
      Serial.println("Motor apagado");
    } else if (c == '3') {
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
    } else if (c == '4') {
      motorEnabled  = true;
      motorMode     = MotorMode::PiControl;
      pidErrorSum   = 0.0f;
      pidErrorPrev  = 0.0f;
      pidPrevMs     = 0;
      pidLastReadMs = millis();
      Serial.printf("Modo 4 PID activo. Setpoint actual: %.2f\n", piSetpoint);
      Serial.println("Envia el angulo deseado (ej: 30.0) y presiona Enter.");
    }
  }

  // Timeout: si en modo PID el buffer tiene contenido y no llegan caracteres
  // nuevos en kSerialBufferIdleMs, procesarlo (compatibilidad con monitores
  // que envian sin terminador de linea).
  if (motorMode == MotorMode::PiControl &&
      serialBuffer.length() > 0 &&
      (millis() - serialBufferLastCharMs) > kSerialBufferIdleMs) {
    processPiSerialBuffer();
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
  tft.setCursor(10, 20);
  tft.println("Menu PWM motor");
  tft.setCursor(10, 50);
  tft.println("0 1 2 3 4");

  printMenu();
}

void loop() {
  handleSerialMenu();

  // --- Leer angulo ---
  long sum = 0;
  for (int i = 0; i < kAvgSamples; i++) {
    sum += analogRead(kAdcPin);
    delay(2);
  }
  const float adcAvg  = sum / static_cast<float>(kAvgSamples);
  float angleDeg = 0.0f;
  if (kAdcMaxCal != kAdcMin) {
    angleDeg = (adcAvg - kAdcMin) * (kAngleMaxDeg / static_cast<float>(kAdcMaxCal - kAdcMin));
  }
  angleDeg = constrain(angleDeg, 0.0f, kAngleMaxDeg);

  // --- Controlar motor ---
  float motorDutyPercent = 0.0f;
  if (motorMode == MotorMode::PiControl && motorEnabled) {
    motorDutyPercent = runPidController(angleDeg);
  } else {
    motorDutyPercent = getMotorDutyPercent();
  }

  // --- Deteccion de tiempo 0->max ---
  if (!timingActive && angleDeg <= kZeroThresholdDeg) {
    startTimeMs  = millis();
    timingActive = true;
  }
  if (timingActive && angleDeg >= kAngleMaxDeg) {
    Serial.printf("t_0_to_max_ms:%lu\n", millis() - startTimeMs);
    timingActive = false;
  }

  // --- Telemetria en una sola linea ---
  Serial.printf("Angulo:%.2f | Setpoint:%.2f | PWM:%.2f%% | Modo:%d | Error:%.2f | Isum:%.2f | Eprev:%.2f\n",
                angleDeg,
                piSetpoint,
                motorDutyPercent,
                static_cast<int>(motorMode),
                piSetpoint - angleDeg,
                pidErrorSum,
                pidErrorPrev);

  delay(20);
}
