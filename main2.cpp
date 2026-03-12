#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>
#include <NewPing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TaskScheduler.h>
#include <ArduinoLog.h>
#include <Adafruit_NeoPixel.h>

// 引入引脚定义
#include "PinConfig.h"

// ==========================================
// ⚙️ 参数配置区域（改进版）
// ==========================================

// --- 任务周期 ---
#define SERVO_SENSOR_INTERVAL 40    // [毫秒] 舵机+传感器刷新
#define SERIAL_REPORT_INTERVAL 200  // [毫秒] 串口上报
#define LED_UPDATE_INTERVAL 100     // [毫秒] 灯光刷新频率
#define MOTOR_UPDATE_INTERVAL 20    // [毫秒] 电机控制刷新

// --- 距离阈值 ---
#define DIST_SAFE_THRESHOLD 75.0f      // [cm] 安全距离 (绿)
#define DIST_DANGER_THRESHOLD 10.0f    // [cm] 危险距离 (红)
#define OBSTACLE_SCAN_THRESHOLD 25.0f  // [cm] 进入避障扫描阈值

// --- 电机参数 ---
#define MOTOR_SPEED_MIN 90
#define MOTOR_SPEED_MAX 150
#define MOTOR_TURN_SPEED 130

// --- 左右轮基础配平 ---
// 如果车总偏左：增大左轮trim或减小右轮trim
// 如果车总偏右：增大右轮trim或减小左轮trim
#define MOTOR_LEFT_TRIM  0
#define MOTOR_RIGHT_TRIM 8

// --- 舵机扫描位 ---
#define SERVO_CENTER_ANGLE 90
#define SERVO_SCAN_RIGHT_ANGLE 30
#define SERVO_SCAN_LEFT_ANGLE 150
#define SERVO_SETTLE_MS 300

// --- 避障动作时长 ---
#define AVOID_REVERSE_DURATION_MS 350
#define AVOID_TURN_DURATION_MS 420
#define AVOID_TURN_AFTER_REVERSE_MS 520

// --- 超声波滤波 ---
#define SONAR_MEDIAN_SAMPLES 3
#define FRONT_DISTANCE_ALPHA 0.30f    // 前向距离低通滤波系数
#define SCAN_DISTANCE_ALPHA 0.60f     // 扫描方向更新权重
#define OBSTACLE_CONFIRM_COUNT 2      // 连续命中几次才进入避障

// --- MPU6050 偏航修正 ---
#define GYRO_CALIBRATION_SAMPLES 100
#define GYRO_CALIBRATION_DELAY_MS 5
#define GYRO_Z_DEADBAND_DPS 0.5f
#define HEADING_CORRECTION_KP 1.2f
#define HEADING_CORRECTION_MAX 25

enum MotionCommand
{
    MOTION_STOP,
    MOTION_FORWARD,
    MOTION_BACKWARD,
    MOTION_TURN_LEFT,
    MOTION_TURN_RIGHT,
};

enum AvoidancePhase
{
    AVOID_DRIVE_FORWARD,
    AVOID_SCAN_RIGHT,
    AVOID_SCAN_CENTER,
    AVOID_SCAN_LEFT,
    AVOID_DECIDE,
    AVOID_REVERSE,
    AVOID_TURN_LEFT,
    AVOID_TURN_RIGHT,
};

// ==========================================
// 全局对象与变量
// ==========================================

Scheduler runner;

Servo myServo;
NewPing sonar(PIN_SONAR_TRIG, PIN_SONAR_ECHO, 200);
Adafruit_MPU6050 mpu;
Adafruit_NeoPixel strip(WS2812_NUM_LEDS, PIN_WS2812, NEO_GRB + NEO_KHZ800);

struct RobotState
{
    // 舵机
    int servoAngle = SERVO_CENTER_ANGLE;
    int servoTargetAngle = SERVO_CENTER_ANGLE;
    unsigned long lastServoCommandMs = 0;

    // 超声波
    float distanceCm = 0.0f;      // 当前舵机朝向下的测距
    float frontDistanceCm = 200.0f;
    unsigned long pingTimeUs = 0;
    float scanRightDistanceCm = 0.0f;
    float scanCenterDistanceCm = 0.0f;
    float scanLeftDistanceCm = 0.0f;

    // 前向障碍确认计数
    uint8_t obstacleDetectCount = 0;

    // MPU6050
    float accelX = 0, accelY = 0, accelZ = 0;
    float gyroX = 0, gyroY = 0, gyroZ = 0; // [deg/s]
    float temp = 20.0f;
    float yawDeg = 0.0f;
    float headingTargetDeg = 0.0f;
    float gyroBiasZ = 0.0f; // [rad/s]
    unsigned long lastImuUpdateUs = 0;
    bool mpuReady = false;
    int headingCorrection = 0;

    // 电机输出
    uint8_t motorLeftSpeed = 0;
    uint8_t motorRightSpeed = 0;
    bool motorLeftForward = true;
    bool motorRightForward = true;
    MotionCommand motionCommand = MOTION_STOP;

    // 避障状态机
    AvoidancePhase avoidancePhase = AVOID_DRIVE_FORWARD;
    unsigned long phaseStartedMs = 0;
    unsigned long phaseDurationMs = 0;
} state;

// ==========================================
// 函数声明
// ==========================================
void servoSensorCallback();
void serialReportCallback();
void ledControlCallback();
void motorControlCallback();

void setMotorOutput(uint8_t enPin, uint8_t inPin1, uint8_t inPin2, bool forward, uint8_t speed, bool invert);
bool calibrateGyroBias();
void setServoTarget(int angle);
bool isServoSettled(unsigned long nowMs);

float normalizeDistanceCm(float distanceCm);
uint8_t computeCruiseSpeed(float frontDistanceCm);
void setAvoidancePhase(AvoidancePhase phase, unsigned long nowMs, unsigned long durationMs = 0);
void captureHeadingTarget();
void applyMotion(MotionCommand motion, uint8_t baseSpeed, bool useHeadingCorrection);

float readDistanceOnceCm();
float readDistanceMedianCm();
float angleLpf(float oldValue, float newValue, float alpha);

// ==========================================
// 任务定义
// ==========================================

Task tServoSensor(SERVO_SENSOR_INTERVAL, TASK_FOREVER, &servoSensorCallback);
Task tReport(SERIAL_REPORT_INTERVAL, TASK_FOREVER, &serialReportCallback);
Task tLed(LED_UPDATE_INTERVAL, TASK_FOREVER, &ledControlCallback);
Task tMotor(MOTOR_UPDATE_INTERVAL, TASK_FOREVER, &motorControlCallback);

// ==========================================
// Setup & Loop
// ==========================================

void setup()
{
    Serial.begin(9600);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.notice(F("--- WALL-E Robot System Starting ---" CR));

    myServo.attach(PIN_SERVO_1);
    myServo.write(state.servoAngle);
    state.lastServoCommandMs = millis();

    strip.begin();
    strip.setBrightness(50);
    strip.show();
    Log.notice(F("WS2812 LED initialized on pin %d" CR), PIN_WS2812);

    if (!mpu.begin())
    {
        state.mpuReady = false;
        Log.error(F("Failed to find MPU6050 chip! Heading correction disabled." CR));
    }
    else
    {
        state.mpuReady = true;
        Log.notice(F("MPU6050 Found!" CR));
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        calibrateGyroBias();
    }

    pinMode(PIN_MOTOR_L_EN, OUTPUT);
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);
    pinMode(PIN_MOTOR_R_EN, OUTPUT);
    pinMode(PIN_MOTOR_R_IN3, OUTPUT);
    pinMode(PIN_MOTOR_R_IN4, OUTPUT);

    digitalWrite(PIN_MOTOR_L_IN1, LOW);
    digitalWrite(PIN_MOTOR_L_IN2, LOW);
    digitalWrite(PIN_MOTOR_R_IN3, LOW);
    digitalWrite(PIN_MOTOR_R_IN4, LOW);
    analogWrite(PIN_MOTOR_L_EN, 0);
    analogWrite(PIN_MOTOR_R_EN, 0);

    runner.init();
    runner.addTask(tServoSensor);
    runner.addTask(tReport);
    runner.addTask(tLed);
    runner.addTask(tMotor);

    tServoSensor.enable();
    tReport.enable();
    tLed.enable();
    tMotor.enable();

    Log.notice(F("System Ready." CR));
}

void loop()
{
    runner.execute();
}

// ==========================================
// 辅助函数
// ==========================================

bool calibrateGyroBias()
{
    if (!state.mpuReady)
    {
        return false;
    }

    float sumZ = 0.0f;
    sensors_event_t a, g, temp;
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; ++i)
    {
        mpu.getEvent(&a, &g, &temp);
        sumZ += g.gyro.z;
        delay(GYRO_CALIBRATION_DELAY_MS);
    }

    state.gyroBiasZ = sumZ / GYRO_CALIBRATION_SAMPLES;
    state.lastImuUpdateUs = micros();
    state.yawDeg = 0.0f;
    state.headingTargetDeg = 0.0f;

    Log.notice(F("MPU6050 gyro bias calibrated: %d mdps" CR), (int)(state.gyroBiasZ * 57295.0f));
    return true;
}

void setServoTarget(int angle)
{
    state.servoTargetAngle = constrain(angle, 0, 180);
}

bool isServoSettled(unsigned long nowMs)
{
    return state.servoAngle == state.servoTargetAngle &&
           (nowMs - state.lastServoCommandMs) >= SERVO_SETTLE_MS;
}

float normalizeDistanceCm(float distanceCm)
{
    return distanceCm > 0.0f ? distanceCm : 999.0f;
}

float angleLpf(float oldValue, float newValue, float alpha)
{
    return oldValue * (1.0f - alpha) + newValue * alpha;
}

uint8_t computeCruiseSpeed(float frontDistanceCm)
{
    float dist = normalizeDistanceCm(frontDistanceCm);

    if (dist >= DIST_SAFE_THRESHOLD)
    {
        return MOTOR_SPEED_MAX;
    }

    long speed = map((long)(dist * 10.0f),
                     (long)(OBSTACLE_SCAN_THRESHOLD * 10.0f),
                     (long)(DIST_SAFE_THRESHOLD * 10.0f),
                     MOTOR_SPEED_MIN,
                     MOTOR_SPEED_MAX);

    return (uint8_t)constrain(speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX);
}

void setAvoidancePhase(AvoidancePhase phase, unsigned long nowMs, unsigned long durationMs)
{
    state.avoidancePhase = phase;
    state.phaseStartedMs = nowMs;
    state.phaseDurationMs = durationMs;
}

void captureHeadingTarget()
{
    state.headingTargetDeg = state.yawDeg;
}

float readDistanceOnceCm()
{
    state.pingTimeUs = sonar.ping();

    if (state.pingTimeUs > 0)
    {
        float speedOfSound = (331.3f + 0.606f * state.temp) / 10000.0f;
        return (state.pingTimeUs / 2.0f) * speedOfSound;
    }

    return 0.0f;
}

float readDistanceMedianCm()
{
    float vals[SONAR_MEDIAN_SAMPLES];

    for (int i = 0; i < SONAR_MEDIAN_SAMPLES; ++i)
    {
        delay(8);
        vals[i] = normalizeDistanceCm(readDistanceOnceCm());
    }

    for (int i = 0; i < SONAR_MEDIAN_SAMPLES - 1; ++i)
    {
        for (int j = i + 1; j < SONAR_MEDIAN_SAMPLES; ++j)
        {
            if (vals[j] < vals[i])
            {
                float tmp = vals[i];
                vals[i] = vals[j];
                vals[j] = tmp;
            }
        }
    }

    return vals[SONAR_MEDIAN_SAMPLES / 2];
}

void setMotorOutput(uint8_t enPin, uint8_t inPin1, uint8_t inPin2, bool forward, uint8_t speed, bool invert)
{
    bool dir = invert ? !forward : forward;

    if (speed == 0)
    {
        // 这里仍保留滑行停止，如需更急停可改成主动刹车
        digitalWrite(inPin1, LOW);
        digitalWrite(inPin2, LOW);
        analogWrite(enPin, 0);
        return;
    }

    digitalWrite(inPin1, dir ? HIGH : LOW);
    digitalWrite(inPin2, dir ? LOW : HIGH);
    analogWrite(enPin, speed);
}

void applyMotion(MotionCommand motion, uint8_t baseSpeed, bool useHeadingCorrection)
{
    int leftSpeed = 0;
    int rightSpeed = 0;
    bool leftForward = true;
    bool rightForward = true;
    int correction = 0;

    if (motion != state.motionCommand &&
        (motion == MOTION_FORWARD || motion == MOTION_BACKWARD))
    {
        captureHeadingTarget();
    }

    if (useHeadingCorrection && state.mpuReady &&
        (motion == MOTION_FORWARD || motion == MOTION_BACKWARD))
    {
        // 目标 - 当前，更容易理解和调参
        float yawError = state.headingTargetDeg - state.yawDeg;
        correction = constrain((int)(yawError * HEADING_CORRECTION_KP),
                               -HEADING_CORRECTION_MAX,
                               HEADING_CORRECTION_MAX);
    }

    switch (motion)
    {
    case MOTION_STOP:
        leftSpeed = 0;
        rightSpeed = 0;
        break;

    case MOTION_FORWARD:
        leftForward = true;
        rightForward = true;
        leftSpeed = baseSpeed + correction + MOTOR_LEFT_TRIM;
        rightSpeed = baseSpeed - correction + MOTOR_RIGHT_TRIM;
        break;

    case MOTION_BACKWARD:
        leftForward = false;
        rightForward = false;
        leftSpeed = baseSpeed - correction + MOTOR_LEFT_TRIM;
        rightSpeed = baseSpeed + correction + MOTOR_RIGHT_TRIM;
        break;

    case MOTION_TURN_LEFT:
        leftForward = false;
        rightForward = true;
        leftSpeed = baseSpeed + MOTOR_LEFT_TRIM;
        rightSpeed = baseSpeed + MOTOR_RIGHT_TRIM;
        break;

    case MOTION_TURN_RIGHT:
        leftForward = true;
        rightForward = false;
        leftSpeed = baseSpeed + MOTOR_LEFT_TRIM;
        rightSpeed = baseSpeed + MOTOR_RIGHT_TRIM;
        break;
    }

    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    state.motionCommand = motion;
    state.headingCorrection = correction;
    state.motorLeftForward = leftForward;
    state.motorRightForward = rightForward;
    state.motorLeftSpeed = (uint8_t)leftSpeed;
    state.motorRightSpeed = (uint8_t)rightSpeed;

    setMotorOutput(PIN_MOTOR_L_EN, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2,
                   leftForward, state.motorLeftSpeed, MOTOR_LEFT_INVERT);

    setMotorOutput(PIN_MOTOR_R_EN, PIN_MOTOR_R_IN3, PIN_MOTOR_R_IN4,
                   rightForward, state.motorRightSpeed, MOTOR_RIGHT_INVERT);
}

// ==========================================
// 任务具体实现
// ==========================================

void servoSensorCallback()
{
    if (state.servoAngle != state.servoTargetAngle)
    {
        state.servoAngle = state.servoTargetAngle;
        myServo.write(state.servoAngle);
        state.lastServoCommandMs = millis();
    }

    // --- MPU6050 更新 ---
    if (state.mpuReady)
    {
        sensors_event_t a, g, temp;
        mpu.getEvent(&a, &g, &temp);

        state.accelX = a.acceleration.x;
        state.accelY = a.acceleration.y;
        state.accelZ = a.acceleration.z;
        state.gyroX = g.gyro.x * 57.2958f;
        state.gyroY = g.gyro.y * 57.2958f;
        state.temp = temp.temperature;

        float correctedGyroZDeg = (g.gyro.z - state.gyroBiasZ) * 57.2958f;
        if (fabsf(correctedGyroZDeg) < GYRO_Z_DEADBAND_DPS)
        {
            correctedGyroZDeg = 0.0f;
        }
        state.gyroZ = correctedGyroZDeg;

        unsigned long nowUs = micros();
        if (state.lastImuUpdateUs != 0)
        {
            float dt = (nowUs - state.lastImuUpdateUs) / 1000000.0f;
            state.yawDeg += correctedGyroZDeg * dt;

            while (state.yawDeg > 180.0f)
                state.yawDeg -= 360.0f;
            while (state.yawDeg < -180.0f)
                state.yawDeg += 360.0f;
        }
        state.lastImuUpdateUs = nowUs;
    }

    // --- 超声波更新：改成中值滤波 ---
    state.distanceCm = readDistanceMedianCm();

    // 中位位置时更新前向距离，并做低通滤波，避免误触发避障
    if (abs(state.servoAngle - SERVO_CENTER_ANGLE) <= 5)
    {
        float d = normalizeDistanceCm(state.distanceCm);
        state.frontDistanceCm = angleLpf(state.frontDistanceCm, d, FRONT_DISTANCE_ALPHA);
    }
}

void motorControlCallback()
{
    unsigned long nowMs = millis();
    float frontDistance = normalizeDistanceCm(state.frontDistanceCm);

    switch (state.avoidancePhase)
    {
    case AVOID_DRIVE_FORWARD:
        setServoTarget(SERVO_CENTER_ANGLE);

        if (frontDistance <= OBSTACLE_SCAN_THRESHOLD)
        {
            if (state.obstacleDetectCount < 255)
            {
                state.obstacleDetectCount++;
            }
        }
        else
        {
            state.obstacleDetectCount = 0;
        }

        if (state.obstacleDetectCount >= OBSTACLE_CONFIRM_COUNT)
        {
            applyMotion(MOTION_STOP, 0, false);

            state.scanRightDistanceCm = 0.0f;
            state.scanCenterDistanceCm = 0.0f;
            state.scanLeftDistanceCm = 0.0f;
            state.obstacleDetectCount = 0;

            setAvoidancePhase(AVOID_SCAN_RIGHT, nowMs);
            setServoTarget(SERVO_SCAN_RIGHT_ANGLE);
            return;
        }

        applyMotion(MOTION_FORWARD, computeCruiseSpeed(frontDistance), true);
        return;

    case AVOID_SCAN_RIGHT:
        applyMotion(MOTION_STOP, 0, false);
        if (isServoSettled(nowMs))
        {
            float d = normalizeDistanceCm(state.distanceCm);
            state.scanRightDistanceCm = angleLpf(state.scanRightDistanceCm > 0 ? state.scanRightDistanceCm : d,
                                                 d,
                                                 SCAN_DISTANCE_ALPHA);

            setAvoidancePhase(AVOID_SCAN_CENTER, nowMs);
            setServoTarget(SERVO_CENTER_ANGLE);
        }
        return;

    case AVOID_SCAN_CENTER:
        applyMotion(MOTION_STOP, 0, false);
        if (isServoSettled(nowMs))
        {
            float d = normalizeDistanceCm(state.distanceCm);
            state.scanCenterDistanceCm = angleLpf(state.scanCenterDistanceCm > 0 ? state.scanCenterDistanceCm : d,
                                                  d,
                                                  SCAN_DISTANCE_ALPHA);
            state.frontDistanceCm = state.scanCenterDistanceCm;

            setAvoidancePhase(AVOID_SCAN_LEFT, nowMs);
            setServoTarget(SERVO_SCAN_LEFT_ANGLE);
        }
        return;

    case AVOID_SCAN_LEFT:
        applyMotion(MOTION_STOP, 0, false);
        if (isServoSettled(nowMs))
        {
            float d = normalizeDistanceCm(state.distanceCm);
            state.scanLeftDistanceCm = angleLpf(state.scanLeftDistanceCm > 0 ? state.scanLeftDistanceCm : d,
                                                d,
                                                SCAN_DISTANCE_ALPHA);

            setAvoidancePhase(AVOID_DECIDE, nowMs);
            setServoTarget(SERVO_CENTER_ANGLE);
        }
        return;

    case AVOID_DECIDE:
        applyMotion(MOTION_STOP, 0, false);

        if (!isServoSettled(nowMs))
        {
            return;
        }

        if (state.scanRightDistanceCm > OBSTACLE_SCAN_THRESHOLD)
        {
            setAvoidancePhase(AVOID_TURN_RIGHT, nowMs, AVOID_TURN_DURATION_MS);
            return;
        }

        if (state.scanCenterDistanceCm > OBSTACLE_SCAN_THRESHOLD)
        {
            captureHeadingTarget();
            setAvoidancePhase(AVOID_DRIVE_FORWARD, nowMs);
            return;
        }

        if (state.scanLeftDistanceCm > OBSTACLE_SCAN_THRESHOLD)
        {
            setAvoidancePhase(AVOID_TURN_LEFT, nowMs, AVOID_TURN_DURATION_MS);
            return;
        }

        setAvoidancePhase(AVOID_REVERSE, nowMs, AVOID_REVERSE_DURATION_MS);
        return;

    case AVOID_REVERSE:
        applyMotion(MOTION_BACKWARD, MOTOR_TURN_SPEED, true);
        if ((nowMs - state.phaseStartedMs) >= state.phaseDurationMs)
        {
            setAvoidancePhase(AVOID_TURN_RIGHT, nowMs, AVOID_TURN_AFTER_REVERSE_MS);
        }
        return;

    case AVOID_TURN_LEFT:
        applyMotion(MOTION_TURN_LEFT, MOTOR_TURN_SPEED, false);
        if ((nowMs - state.phaseStartedMs) >= state.phaseDurationMs)
        {
            captureHeadingTarget();
            setAvoidancePhase(AVOID_DRIVE_FORWARD, nowMs);
        }
        return;

    case AVOID_TURN_RIGHT:
        applyMotion(MOTION_TURN_RIGHT, MOTOR_TURN_SPEED, false);
        if ((nowMs - state.phaseStartedMs) >= state.phaseDurationMs)
        {
            captureHeadingTarget();
            setAvoidancePhase(AVOID_DRIVE_FORWARD, nowMs);
        }
        return;
    }
}

void ledControlCallback()
{
    float dist = normalizeDistanceCm(state.frontDistanceCm);
    uint32_t color;

    if (dist >= DIST_SAFE_THRESHOLD)
    {
        color = strip.Color(0, 255, 0);
    }
    else if (dist <= DIST_DANGER_THRESHOLD)
    {
        color = strip.Color(255, 0, 0);
    }
    else
    {
        long hue = map((long)(dist * 10.0f),
                       (long)(DIST_DANGER_THRESHOLD * 10.0f),
                       (long)(DIST_SAFE_THRESHOLD * 10.0f),
                       0,
                       21845);
        color = strip.ColorHSV((uint16_t)hue, 255, 255);
    }

    for (int i = 0; i < strip.numPixels(); i++)
    {
        strip.setPixelColor(i, color);
    }
    strip.show();
}

void serialReportCallback()
{
    Log.notice(F("F:%d C:%d Y:%d HC:%d P:%d SR:%d SC:%d SL:%d ML:%d MR:%d FL:%d FR:%d" CR),
               (int)normalizeDistanceCm(state.frontDistanceCm),
               (int)normalizeDistanceCm(state.distanceCm),
               (int)state.yawDeg,
               state.headingCorrection,
               (int)state.avoidancePhase,
               (int)normalizeDistanceCm(state.scanRightDistanceCm),
               (int)normalizeDistanceCm(state.scanCenterDistanceCm),
               (int)normalizeDistanceCm(state.scanLeftDistanceCm),
               (int)state.motorLeftSpeed,
               (int)state.motorRightSpeed,
               (int)state.motorLeftForward,
               (int)state.motorRightForward);
}
