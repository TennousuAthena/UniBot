#include <Servo.h>
#include "MPU6050_getdata.h"

/*
  Fast obstacle-race sketch for a 2-wheel (front drive) + rear caster smart car.

  Architecture: non-blocking continuous sense-think-act loop.
  - Servo sweeps continuously updating a 3-direction distance map.
  - Rate-dominant yaw control: uses raw gyro angular rate as the primary
    damping signal, with a weak angle term for drift correction only.
    This avoids the noise from numerical differentiation and handles
    the caster-wheel transport delay much better than position-based PD.
  - Wall repulsion: instead of centering (which fails with HC-SR04 at
    oblique angles), applies a repulsive correction only when a side
    reading drops below a danger threshold.
  - State machine: STARTUP -> CRUISE <-> SLOW_APPROACH <-> EMERGENCY -> REVERSE

  Tune in this order:
  1. RIGHT_FORWARD_LEVEL / LEFT_FORWARD_LEVEL if the car drives backward.
  2. YAW_SENSE — flip to -1 if the car oscillates worse instead of stabilizing.
  3. RIGHT_SPEED_SCALE / LEFT_SPEED_SCALE for coarse left-right trim.
  4. YAW_KR (rate gain) for damping; YAW_KP (angle gain) for drift.
  5. WALL_REPULSE_* for corridor wall avoidance.
  6. DIST_* thresholds for your track.
*/

// ========================== PIN DEFINITIONS ==========================
static const uint8_t PIN_MOTOR_PWMA = 5;   // Right motor PWM
static const uint8_t PIN_MOTOR_PWMB = 6;   // Left motor PWM
static const uint8_t PIN_MOTOR_AIN1 = 7;   // Right motor direction
static const uint8_t PIN_MOTOR_BIN1 = 8;   // Left motor direction
static const uint8_t PIN_MOTOR_STBY = 3;   // TB6612 standby
static const uint8_t PIN_SERVO     = 10;
static const uint8_t PIN_ULTRA_TRIG = 13;
static const uint8_t PIN_ULTRA_ECHO = 12;

// ======================== MOTOR DIRECTION ============================
static const bool  RIGHT_FORWARD_LEVEL = HIGH;
static const bool  LEFT_FORWARD_LEVEL  = HIGH;
static const float RIGHT_SPEED_SCALE   = 1.00f;
static const float LEFT_SPEED_SCALE    = 1.00f;

// ================ YAW CONTROLLER (rate-dominant) =====================
// Uses gyro angular rate directly (not noisy numerical derivative).
// KR * gyroRate  = strong damping (the main stabilizer, resists rotation)
// KP * angleError = weak drift correction (slowly brings heading back)
//
// For a caster-wheel car, KR >> KP prevents overshoot because the rate
// signal is instantaneous while the angle response has caster delay.
static const bool   ENABLE_YAW_HOLD    = true;
static const int8_t YAW_SENSE          = 1;    // Flip to -1 if correction is backwards
static const float  YAW_KR             = 1.8f; // Rate gain: PWM per deg/s of rotation
static const float  YAW_KP             = 0.3f; // Angle gain: PWM per degree of error (weak!)
static const uint8_t YAW_MAX_CORRECTION = 45;
static const uint8_t YAW_MIN_PWM       = 35;
static const uint8_t YAW_UPDATE_PERIOD_MS = 10;
static const uint8_t YAW_CORRECTION_DEADBAND = 2;

// ================ WALL REPULSION (replaces corridor centering) =======
// HC-SR04 at 50/130 degrees gives unreliable side distances due to
// specular reflection off flat walls. Instead of centering, we use a
// repulsive field: when a side reading drops below WALL_DANGER_CM,
// apply a correction pushing away from that wall.
// Only the CLOSER wall contributes — no attempt at centering.
static const uint16_t WALL_DANGER_CM     = 25;  // Activate repulsion below this
static const float    WALL_REPULSE_GAIN  = 1.5f; // PWM per cm below danger threshold
static const int16_t  WALL_REPULSE_MAX   = 35;   // Max repulsion PWM correction

// ===================== SERVO SWEEP ===================================
static const uint8_t SWEEP_ANGLES[]    = {50, 90, 130, 90}; // R -> C -> L -> C
static const uint8_t SWEEP_COUNT       = 4;
static const uint8_t SWEEP_SETTLE_MS   = 60;

// =================== DISTANCE THRESHOLDS (cm) ========================
static const uint16_t DIST_MAX         = 150;
static const uint16_t ENTER_SLOW_CM    = 35;  // CRUISE -> SLOW_APPROACH
static const uint16_t EXIT_SLOW_CM     = 45;  // SLOW_APPROACH -> CRUISE
static const uint16_t ENTER_EMERGENCY_CM = 17; // -> EMERGENCY
static const uint16_t STOP_CM          = 22;   // Speed = 0 below this
static const uint16_t DIST_FULL_SPEED  = 90;

// =================== SPEED PRESETS (0-255) ============================
static const uint8_t SPEED_MAX         = 200;
static const uint8_t SPEED_MIN         = 100;
static const uint8_t SPEED_NARROW_CAP  = 160;
static const uint8_t SPEED_REVERSE     = 120;
static const uint8_t SPEED_PIVOT       = 140;

// =================== TIMING (ms) =====================================
static const uint16_t STARTUP_DELAY_MS      = 1800;
static const uint16_t EMERGENCY_REVERSE_MS  = 250;
static const uint16_t PIVOT_MS              = 200;
static const uint16_t SPEED_RAMP_PERIOD_MS  = 20;

// =================== SLOW APPROACH STEERING ==========================
static const uint16_t SLOW_SIDE_MARGIN = 15;
static const int16_t  SLOW_MAX_BIAS   = 30;

static const bool ENABLE_SERIAL_DEBUG  = true;

// ========================== ENUMS ====================================
enum CarState
{
  STATE_STARTUP,
  STATE_CRUISE,
  STATE_SLOW_APPROACH,
  STATE_EMERGENCY,
  STATE_REVERSE
};

// ======================== DISTANCE MAP ===============================
struct DistanceMap
{
  uint16_t left;    // from 130 degrees (forward-left oblique)
  uint16_t center;  // from 90 degrees (straight ahead)
  uint16_t right;   // from 50 degrees (forward-right oblique)
};

// ======================== GLOBAL STATE ===============================
Servo scanServo;

// State machine
CarState carState = STATE_STARTUP;
unsigned long stateEntryMs = 0;
unsigned long startupReleaseMs = 0;

// Distance map
DistanceMap distMap = {DIST_MAX, DIST_MAX, DIST_MAX};
uint16_t filteredFrontDistanceCm = DIST_MAX;

// Servo sweep
uint8_t sweepIndex = 0;
unsigned long sweepStartMs = 0;

// IMU / yaw hold
bool imuReady = false;
bool yawHoldActive = false;
float yawNow = 0.0f;
float targetYaw = 0.0f;
unsigned long lastYawUpdateMs = 0;

// Speed ramp
uint8_t currentSpeed = 0;
unsigned long lastSpeedRampMs = 0;

// Debug
unsigned long lastDebugMs = 0;

// =====================================================================
//                          MOTOR HELPERS
// =====================================================================

uint8_t clampPwm(int value)
{
  if (value < 0) value = 0;
  else if (value > 255) value = 255;
  return static_cast<uint8_t>(value);
}

uint8_t applyScale(int pwm, float scale)
{
  return clampPwm(static_cast<int>(pwm * scale));
}

void setMotorChannel(uint8_t pwmPin, uint8_t dirPin, int signedSpeed,
                     bool forwardLevel, float scale)
{
  uint8_t pwm = applyScale(abs(signedSpeed), scale);
  if (pwm == 0)
  {
    analogWrite(pwmPin, 0);
    return;
  }
  bool forward = signedSpeed > 0;
  digitalWrite(dirPin, forward ? forwardLevel : !forwardLevel);
  analogWrite(pwmPin, pwm);
}

void driveRaw(int rightSpeed, int leftSpeed)
{
  if (rightSpeed == 0 && leftSpeed == 0)
  {
    analogWrite(PIN_MOTOR_PWMA, 0);
    analogWrite(PIN_MOTOR_PWMB, 0);
    digitalWrite(PIN_MOTOR_STBY, LOW);
    return;
  }
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  setMotorChannel(PIN_MOTOR_PWMA, PIN_MOTOR_AIN1, rightSpeed,
                  RIGHT_FORWARD_LEVEL, RIGHT_SPEED_SCALE);
  setMotorChannel(PIN_MOTOR_PWMB, PIN_MOTOR_BIN1, leftSpeed,
                  LEFT_FORWARD_LEVEL, LEFT_SPEED_SCALE);
}

// =====================================================================
//                        YAW ESTIMATION
// =====================================================================

void resetYawHold()
{
  yawHoldActive = false;
}

bool updateYawEstimate()
{
  if (!imuReady) return false;

  unsigned long now = millis();
  if (lastYawUpdateMs != 0 && (now - lastYawUpdateMs) < YAW_UPDATE_PERIOD_MS)
    return true;

  MPU6050Getdata.MPU6050_dveGetEulerAngles(&yawNow);
  lastYawUpdateMs = now;
  return true;
}

void latchYawTarget()
{
  if (!updateYawEstimate()) return;
  targetYaw = yawNow;
  yawHoldActive = true;
}

// Rate-dominant yaw correction:
//   correction = KR * gyroRate + KP * angleError
// KR provides strong instantaneous damping (resists any rotation).
// KP provides weak long-term drift correction.
int computeYawCorrection(uint8_t speed)
{
  if (!ENABLE_YAW_HOLD || !imuReady) return 0;
  if (!yawHoldActive) latchYawTarget();
  if (!updateYawEstimate()) return 0;

  // Get angular rate directly from IMU (already filtered by deadband in MPU6050_getdata)
  float gyroRate = MPU6050Getdata.getLastRate(); // deg/s

  // Angle error (weak, for drift correction only)
  float yawError = yawNow - targetYaw;

  // Rate-dominant correction
  int correction = static_cast<int>(
      (gyroRate * YAW_KR + yawError * YAW_KP) * YAW_SENSE);
  correction = constrain(correction,
                         -static_cast<int>(YAW_MAX_CORRECTION),
                          static_cast<int>(YAW_MAX_CORRECTION));

  // Deadband for caster friction
  if (abs(correction) < YAW_CORRECTION_DEADBAND) correction = 0;

  return correction;
}

// =====================================================================
//                      ULTRASONIC SENSOR
// =====================================================================

uint16_t readDistanceOnce()
{
  digitalWrite(PIN_ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRA_TRIG, LOW);

  unsigned long duration = pulseIn(PIN_ULTRA_ECHO, HIGH, 25000UL);
  if (duration == 0) return DIST_MAX;

  uint16_t distance = static_cast<uint16_t>(duration / 58UL);
  if (distance == 0) return DIST_MAX;
  if (distance > DIST_MAX) distance = DIST_MAX;
  return distance;
}

// =====================================================================
//                    CONTINUOUS SERVO SWEEP
// =====================================================================

void initSweep()
{
  sweepIndex = 0;
  scanServo.write(SWEEP_ANGLES[0]);
  sweepStartMs = millis();
}

void updateSweep()
{
  unsigned long now = millis();
  if (now - sweepStartMs < SWEEP_SETTLE_MS) return;

  uint16_t dist = readDistanceOnce();
  uint8_t angle = SWEEP_ANGLES[sweepIndex];

  if (angle <= 60)
  {
    distMap.right = dist;
  }
  else if (angle >= 120)
  {
    distMap.left = dist;
  }
  else
  {
    distMap.center = dist;
    filteredFrontDistanceCm = static_cast<uint16_t>(
        (filteredFrontDistanceCm * 2UL + dist) / 3UL);
  }

  sweepIndex = (sweepIndex + 1) % SWEEP_COUNT;
  scanServo.write(SWEEP_ANGLES[sweepIndex]);
  sweepStartMs = now;
}

// =====================================================================
//                     SPEED MANAGEMENT
// =====================================================================

uint8_t computeTargetSpeed()
{
  uint16_t front = filteredFrontDistanceCm;

  uint8_t baseSpeed;
  if (front >= DIST_FULL_SPEED)
  {
    baseSpeed = SPEED_MAX;
  }
  else if (front <= STOP_CM)
  {
    baseSpeed = 0;
  }
  else
  {
    baseSpeed = static_cast<uint8_t>(
        map(front, STOP_CM, DIST_FULL_SPEED, SPEED_MIN, SPEED_MAX));
  }

  // Narrow corridor speed cap (both oblique readings are short)
  if (distMap.left < 40 && distMap.right < 40)
  {
    if (baseSpeed > SPEED_NARROW_CAP)
      baseSpeed = SPEED_NARROW_CAP;
  }

  return baseSpeed;
}

void updateSpeedRamp()
{
  unsigned long now = millis();
  if (now - lastSpeedRampMs < SPEED_RAMP_PERIOD_MS) return;
  lastSpeedRampMs = now;

  uint8_t target = computeTargetSpeed();

  if (target > currentSpeed)
  {
    uint8_t step = 3;
    currentSpeed = (currentSpeed + step > target) ? target : currentSpeed + step;
  }
  else if (target < currentSpeed)
  {
    uint8_t step = 5;
    currentSpeed = (currentSpeed < target + step) ? target : currentSpeed - step;
  }
}

// =====================================================================
//                     WALL REPULSION
// =====================================================================
// Instead of centering between walls (fails with oblique HC-SR04),
// apply a repulsive push away from any wall that is too close.
// Only the closer wall contributes.

int computeWallRepulsion()
{
  int repulsion = 0;

  // Right wall too close -> push left (negative correction)
  if (distMap.right < WALL_DANGER_CM)
  {
    int deficit = static_cast<int>(WALL_DANGER_CM) - static_cast<int>(distMap.right);
    repulsion -= static_cast<int>(deficit * WALL_REPULSE_GAIN);
  }

  // Left wall too close -> push right (positive correction)
  if (distMap.left < WALL_DANGER_CM)
  {
    int deficit = static_cast<int>(WALL_DANGER_CM) - static_cast<int>(distMap.left);
    repulsion += static_cast<int>(deficit * WALL_REPULSE_GAIN);
  }

  return constrain(repulsion,
                   -static_cast<int>(WALL_REPULSE_MAX),
                    static_cast<int>(WALL_REPULSE_MAX));
}

// =====================================================================
//                   CORRIDOR DRIVE (combined control)
// =====================================================================

void driveCorridor(uint8_t speed)
{
  if (speed == 0)
  {
    driveRaw(0, 0);
    return;
  }

  // 1. Yaw correction (rate-dominant: KR*rate + KP*error)
  int yawCorr = computeYawCorrection(speed);

  // 2. Wall repulsion (push away from close walls)
  int wallCorr = computeWallRepulsion();

  // 3. Combine: yaw + wall repulsion (additive, not blended)
  //    Wall repulsion is independent of yaw — it's a safety overlay.
  //    When wall repulsion is active, also re-latch yaw to prevent
  //    the angle term from fighting the wall avoidance.
  if (abs(wallCorr) > 5)
  {
    targetYaw = yawNow; // Don't let angle error fight wall repulsion
  }

  int totalCorr = constrain(yawCorr + wallCorr, -55, 55);

  int rightSpeed = constrain(static_cast<int>(speed) + totalCorr, YAW_MIN_PWM, 255);
  int leftSpeed  = constrain(static_cast<int>(speed) - totalCorr, YAW_MIN_PWM, 255);
  driveRaw(rightSpeed, leftSpeed);
}

// =====================================================================
//                       STATE MACHINE
// =====================================================================

void changeState(CarState newState)
{
  carState = newState;
  stateEntryMs = millis();
  if (newState == STATE_CRUISE)
  {
    latchYawTarget();
  }
  if (newState == STATE_EMERGENCY || newState == STATE_REVERSE)
  {
    resetYawHold();
  }
}

void stateCruise()
{
  if (filteredFrontDistanceCm < ENTER_EMERGENCY_CM)
  {
    changeState(STATE_EMERGENCY);
    return;
  }
  if (filteredFrontDistanceCm < ENTER_SLOW_CM)
  {
    changeState(STATE_SLOW_APPROACH);
    return;
  }

  updateSpeedRamp();
  driveCorridor(currentSpeed);
}

void stateSlowApproach()
{
  if (filteredFrontDistanceCm >= EXIT_SLOW_CM)
  {
    changeState(STATE_CRUISE);
    return;
  }
  if (filteredFrontDistanceCm < ENTER_EMERGENCY_CM)
  {
    changeState(STATE_EMERGENCY);
    return;
  }

  updateSpeedRamp();
  uint8_t speed = currentSpeed;
  if (speed == 0)
  {
    driveRaw(0, 0);
    return;
  }

  // Steer toward the more open side + apply wall repulsion
  resetYawHold();

  // Direction bias from oblique scan comparison
  int bias = 0;
  if (distMap.left > distMap.right + SLOW_SIDE_MARGIN)
  {
    int diff = static_cast<int>(distMap.left - distMap.right);
    bias = -min(diff, static_cast<int>(SLOW_MAX_BIAS));
  }
  else if (distMap.right > distMap.left + SLOW_SIDE_MARGIN)
  {
    int diff = static_cast<int>(distMap.right - distMap.left);
    bias = min(diff, static_cast<int>(SLOW_MAX_BIAS));
  }

  // Add wall repulsion
  int wallCorr = computeWallRepulsion();
  int totalBias = constrain(bias + wallCorr, -50, 50);

  int rightSpeed = constrain(static_cast<int>(speed) + totalBias, 60, 255);
  int leftSpeed  = constrain(static_cast<int>(speed) - totalBias, 60, 255);
  driveRaw(rightSpeed, leftSpeed);
}

void stateEmergency()
{
  unsigned long elapsed = millis() - stateEntryMs;

  if (elapsed < EMERGENCY_REVERSE_MS)
  {
    driveRaw(-SPEED_REVERSE, -SPEED_REVERSE);
  }
  else
  {
    // Pick direction and start pivot
    if (distMap.left > distMap.right)
    {
      driveRaw(SPEED_PIVOT, -SPEED_PIVOT);  // Pivot left
    }
    else
    {
      driveRaw(-SPEED_PIVOT, SPEED_PIVOT);  // Pivot right
    }
    changeState(STATE_REVERSE);
  }
}

void stateReverse()
{
  if (millis() - stateEntryMs >= PIVOT_MS)
  {
    changeState(STATE_CRUISE);
  }
}

// =====================================================================
//                          DEBUG OUTPUT
// =====================================================================

void printDebug()
{
  if (!ENABLE_SERIAL_DEBUG) return;

  unsigned long now = millis();
  if (now - lastDebugMs < 200) return;
  lastDebugMs = now;

  float gyroRate = MPU6050Getdata.getLastRate();
  float yawError = yawNow - targetYaw;
  int yawCorr = static_cast<int>(
      (gyroRate * YAW_KR + yawError * YAW_KP) * YAW_SENSE);

  Serial.print(F("st="));
  Serial.print(static_cast<int>(carState));
  Serial.print(F(" spd="));
  Serial.print(currentSpeed);
  Serial.print(F(" fc="));
  Serial.print(filteredFrontDistanceCm);
  Serial.print(F(" L="));
  Serial.print(distMap.left);
  Serial.print(F(" C="));
  Serial.print(distMap.center);
  Serial.print(F(" R="));
  Serial.print(distMap.right);
  Serial.print(F(" yaw="));
  Serial.print(yawNow, 1);
  Serial.print(F(" rate="));
  Serial.print(gyroRate, 1);
  Serial.print(F(" err="));
  Serial.print(yawError, 1);
  Serial.print(F(" yc="));
  Serial.print(yawCorr);
  Serial.print(F(" wc="));
  Serial.println(computeWallRepulsion());
}

// =====================================================================
//                        SETUP & LOOP
// =====================================================================

void setup()
{
  if (ENABLE_SERIAL_DEBUG)
  {
    Serial.begin(115200);
  }

  pinMode(PIN_MOTOR_PWMA, OUTPUT);
  pinMode(PIN_MOTOR_PWMB, OUTPUT);
  pinMode(PIN_MOTOR_AIN1, OUTPUT);
  pinMode(PIN_MOTOR_BIN1, OUTPUT);
  pinMode(PIN_MOTOR_STBY, OUTPUT);
  pinMode(PIN_ULTRA_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_ECHO, INPUT);

  driveRaw(0, 0);

  scanServo.attach(PIN_SERVO);
  scanServo.write(SWEEP_ANGLES[0]);
  delay(350);

  bool imuInitFailed = MPU6050Getdata.MPU6050_dveInit();
  if (!imuInitFailed)
  {
    delay(120);
    MPU6050Getdata.MPU6050_calibration();
    delay(50);
    imuReady = updateYawEstimate();
    targetYaw = yawNow;
  }
  else
  {
    imuReady = false;
  }

  filteredFrontDistanceCm = readDistanceOnce();
  distMap.center = filteredFrontDistanceCm;

  initSweep();

  startupReleaseMs = millis() + STARTUP_DELAY_MS;
  carState = STATE_STARTUP;
  stateEntryMs = millis();
}

void loop()
{
  // === SENSE (non-blocking) ===
  updateSweep();
  updateYawEstimate();

  // === STATE MACHINE ===
  switch (carState)
  {
  case STATE_STARTUP:
    driveRaw(0, 0);
    if (millis() >= startupReleaseMs)
      changeState(STATE_CRUISE);
    break;

  case STATE_CRUISE:
    stateCruise();
    break;

  case STATE_SLOW_APPROACH:
    stateSlowApproach();
    break;

  case STATE_EMERGENCY:
    stateEmergency();
    break;

  case STATE_REVERSE:
    stateReverse();
    break;
  }

  // === DEBUG ===
  printDebug();
}
