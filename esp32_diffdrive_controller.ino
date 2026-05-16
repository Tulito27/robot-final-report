#include <Arduino.h>
#include <string.h>
#include <strings.h>

// ============================================
// ESP32 + 2 BTS7960 + 2 encoders
// Firmware base para robot diferencial
// Listo para usar con Raspberry por Serial
// ============================================

// ---------- PWM ----------
const uint32_t PWM_FREQ = 20000;   // 20 kHz
const uint8_t  PWM_RES  = 8;       // duty 0..255

// ---------- Mapeo físico real ----------
// Motor físico IZQUIERDO   
const uint8_t RPWM_L = 23;
const uint8_t LPWM_L = 25;
const uint8_t REN_L  = 26;
const uint8_t LEN_L  = 27;

// Motor físico DERECHO
const uint8_t RPWM_R = 18;
const uint8_t LPWM_R = 19;
const uint8_t REN_R  = 21;
const uint8_t LEN_R  = 22;

// Encoder físico IZQUIERDO
const uint8_t ENC_L_A = 16;
const uint8_t ENC_L_B = 17;

// Encoder físico DERECHO
const uint8_t ENC_R_A = 32;
const uint8_t ENC_R_B = 33;

// ---------- Ajustes ya calibrados ----------
// Dirección de motores
const bool INVERT_LEFT_MOTOR  = true;
const bool INVERT_RIGHT_MOTOR = false;

// Signo de encoder para que:
// adelante -> ticks positivos
// atrás    -> ticks negativos
const int LEFT_ENCODER_SIGN  =  1;
const int RIGHT_ENCODER_SIGN = -1;

// Ganancias base
const float LEFT_GAIN  = 1.00f;
const float RIGHT_GAIN = 1.66f;   // puedes afinar luego entre 1.66 y 1.70

// ---------- Seguridad / comunicación ----------
const unsigned long COMMAND_TIMEOUT_MS = 400;   // si no llegan comandos, stop
const unsigned long STREAM_PERIOD_MS   = 100;   // envío periódico de ticks

// ---------- Estado encoder ----------
volatile long ticksLeft = 0;
volatile long ticksRight = 0;
portMUX_TYPE tickMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- Estado motor ----------
int commandedLeft = 0;
int commandedRight = 0;
unsigned long lastCommandMs = 0;
bool streamEnabled = true;

// ---------- Serial ----------
char lineBuffer[96];
size_t lineIndex = 0;
unsigned long lastStreamMs = 0;

// ============================================
// ISR encoders
// ============================================
void IRAM_ATTR isrLeftEncoderA() {
  bool a = digitalRead(ENC_L_A);
  bool b = digitalRead(ENC_L_B);

  portENTER_CRITICAL_ISR(&tickMux);
  if (a == b) {
    ticksLeft += 1 * LEFT_ENCODER_SIGN;
  } else {
    ticksLeft += -1 * LEFT_ENCODER_SIGN;
  }
  portEXIT_CRITICAL_ISR(&tickMux);
}

void IRAM_ATTR isrRightEncoderA() {
  bool a = digitalRead(ENC_R_A);
  bool b = digitalRead(ENC_R_B);

  portENTER_CRITICAL_ISR(&tickMux);
  if (a == b) {
    ticksRight += 1 * RIGHT_ENCODER_SIGN;
  } else {
    ticksRight += -1 * RIGHT_ENCODER_SIGN;
  }
  portEXIT_CRITICAL_ISR(&tickMux);
}

// ============================================
// Utilidades
// ============================================
int applyGainAndClamp(int pwm, float gain) {
  int out = (int)(pwm * gain);
  if (out > 255) out = 255;
  if (out < -255) out = -255;
  return out;
}

void setMotorRaw(uint8_t pinForward, uint8_t pinReverse, int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    ledcWrite(pinForward, pwm);
    ledcWrite(pinReverse, 0);
  } else if (pwm < 0) {
    ledcWrite(pinForward, 0);
    ledcWrite(pinReverse, -pwm);
  } else {
    ledcWrite(pinForward, 0);
    ledcWrite(pinReverse, 0);
  }
}

void setMotorLeft(int pwm) {
  pwm = applyGainAndClamp(pwm, LEFT_GAIN);
  if (INVERT_LEFT_MOTOR) pwm = -pwm;
  setMotorRaw(RPWM_L, LPWM_L, pwm);
}

void setMotorRight(int pwm) {
  pwm = applyGainAndClamp(pwm, RIGHT_GAIN);
  if (INVERT_RIGHT_MOTOR) pwm = -pwm;
  setMotorRaw(RPWM_R, LPWM_R, pwm);
}

void setBothMotors(int leftPwm, int rightPwm) {
  commandedLeft = constrain(leftPwm, -255, 255);
  commandedRight = constrain(rightPwm, -255, 255);

  setMotorLeft(commandedLeft);
  setMotorRight(commandedRight);
}

void stopAllMotors() {
  commandedLeft = 0;
  commandedRight = 0;
  setMotorLeft(0);
  setMotorRight(0);
}

void enableDrivers() {
  digitalWrite(REN_L, HIGH);
  digitalWrite(LEN_L, HIGH);
  digitalWrite(REN_R, HIGH);
  digitalWrite(LEN_R, HIGH);
}

void resetTicks() {
  portENTER_CRITICAL(&tickMux);
  ticksLeft = 0;
  ticksRight = 0;
  portEXIT_CRITICAL(&tickMux);
}

void readTicks(long &l, long &r) {
  portENTER_CRITICAL(&tickMux);
  l = ticksLeft;
  r = ticksRight;
  portEXIT_CRITICAL(&tickMux);
}

void sendTicks() {
  long l, r;
  readTicks(l, r);
  Serial.print("ENC,");
  Serial.print(l);
  Serial.print(",");
  Serial.println(r);
}

void sendConfig() {
  Serial.print("OK,CFG,LEFT_GAIN,");
  Serial.print(LEFT_GAIN, 3);
  Serial.print(",RIGHT_GAIN,");
  Serial.print(RIGHT_GAIN, 3);
  Serial.print(",INV_L_MOTOR,");
  Serial.print(INVERT_LEFT_MOTOR ? 1 : 0);
  Serial.print(",INV_R_MOTOR,");
  Serial.print(INVERT_RIGHT_MOTOR ? 1 : 0);
  Serial.print(",ENC_SIGN_L,");
  Serial.print(LEFT_ENCODER_SIGN);
  Serial.print(",ENC_SIGN_R,");
  Serial.println(RIGHT_ENCODER_SIGN);
}

void sendStatusOk(const char* msg) {
  Serial.print("OK,");
  Serial.println(msg);
}

void sendError(const char* msg) {
  Serial.print("ERR,");
  Serial.println(msg);
}

// ============================================
// Parser de comandos
// ============================================
void executeCommand(char *line) {
  char *cmd = strtok(line, ", \t\r\n");
  if (cmd == nullptr) return;

  if (strcasecmp(cmd, "PING") == 0) {
    sendStatusOk("PONG");
    return;
  }

  if (strcasecmp(cmd, "S") == 0 || strcasecmp(cmd, "STOP") == 0) {
    stopAllMotors();
    lastCommandMs = millis();
    sendStatusOk("STOP");
    return;
  }

  if (strcasecmp(cmd, "E?") == 0 || strcasecmp(cmd, "ENC?") == 0) {
    sendTicks();
    return;
  }

  if (strcasecmp(cmd, "Z") == 0 || strcasecmp(cmd, "RST") == 0) {
    resetTicks();
    sendStatusOk("TICKS_RESET");
    return;
  }

  if (strcasecmp(cmd, "CFG?") == 0) {
    sendConfig();
    return;
  }

  if (strcasecmp(cmd, "STREAM") == 0) {
    char *arg = strtok(nullptr, ", \t\r\n");
    if (!arg) {
      sendError("USE_STREAM_0_OR_1");
      return;
    }

    int v = atoi(arg);
    streamEnabled = (v != 0);
    sendStatusOk(streamEnabled ? "STREAM_ON" : "STREAM_OFF");
    return;
  }

  if (strcasecmp(cmd, "M") == 0) {
    char *arg1 = strtok(nullptr, ", \t\r\n");
    char *arg2 = strtok(nullptr, ", \t\r\n");

    if (!arg1 || !arg2) {
      sendError("USE_M_LEFT_RIGHT");
      return;
    }

    int leftPwm = atoi(arg1);
    int rightPwm = atoi(arg2);

    setBothMotors(leftPwm, rightPwm);
    lastCommandMs = millis();

    Serial.print("OK,M,");
    Serial.print(commandedLeft);
    Serial.print(",");
    Serial.println(commandedRight);
    return;
  }

  sendError("UNKNOWN_COMMAND");
}

// ============================================
// Setup
// ============================================
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(REN_L, OUTPUT);
  pinMode(LEN_L, OUTPUT);
  pinMode(REN_R, OUTPUT);
  pinMode(LEN_R, OUTPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  bool ok = true;
  ok &= ledcAttach(RPWM_L, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(LPWM_L, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(RPWM_R, PWM_FREQ, PWM_RES);
  ok &= ledcAttach(LPWM_R, PWM_FREQ, PWM_RES);

  if (!ok) {
    Serial.println("ERR,LEDC_CONFIG");
    while (true) delay(1000);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightEncoderA, CHANGE);

  enableDrivers();
  stopAllMotors();
  resetTicks();

  lastCommandMs = millis();
  lastStreamMs = millis();

  Serial.println("OK,READY");
  sendConfig();
}

// ============================================
// Loop
// ============================================
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (lineIndex > 0) {
        lineBuffer[lineIndex] = '\0';
        executeCommand(lineBuffer);
        lineIndex = 0;
      }
    } else {
      if (lineIndex < sizeof(lineBuffer) - 1) {
        lineBuffer[lineIndex++] = c;
      }
    }
  }

  // Timeout de seguridad
  if ((millis() - lastCommandMs) > COMMAND_TIMEOUT_MS) {
    if (commandedLeft != 0 || commandedRight != 0) {
      stopAllMotors();
      Serial.println("OK,TIMEOUT_STOP");
    }
    lastCommandMs = millis();
  }

  // Streaming periódico de ticks
  if (streamEnabled && (millis() - lastStreamMs >= STREAM_PERIOD_MS)) {
    lastStreamMs = millis();
    sendTicks();
  }
}