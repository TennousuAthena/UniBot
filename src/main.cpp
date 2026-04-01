#include <Arduino.h>
#include <Servo.h>
#include "PinConfig.h"

// =========================================================
// 状态与全局对象
// =========================================================
enum State
{
    STATE_NORMAL,
    STATE_OBSTACLE
};

State currentState = STATE_NORMAL;
Servo myServo;

// =========================================================
// 参数配置
// =========================================================

// 运动参数
const int BASE_SPEED = 120; // 基础速度 (0-255)
const int MAX_SPEED = 200;  // 限制最大速度
const int MIN_SPEED = 40;   // 限制最小速度

// 舵机参数
const int SERVO_CENTER = 80;
const int SERVO_SCAN_AMP = 50;  // 正常扫描幅度：50度到110度
const int SERVO_SCAN_SPEED = 5; // 扫描角度步长 (增加步长加快扫描)
const int SERVO_SCAN_DELAY = 5; // 扫描延时（毫秒），越小扫描越快

// 超声波及避障参数
const float OBSTACLE_DIST_THRES = 20.0; // 触发避障的最短距离（厘米）
const float MAX_TRACK_WIDTH = 50.0;     // 用于PID的赛道最大宽度限制（厘米）

// PID 控制参数
float Kp = 2;
float Ki = 0.15;
float Kd = 0.45;

float error = 0.0;
float last_error = 0.0;
float integral = 0.0;

// 距离与扫描变量
float dist_left = MAX_TRACK_WIDTH;
float dist_right = MAX_TRACK_WIDTH;
float dist_center = MAX_TRACK_WIDTH;

int currentServoAngle = SERVO_CENTER;
int servoDirection = 1; // 1: 角度增加, -1: 角度减少
unsigned long lastServoMoveTime = 0;

// =========================================================
// 函数声明
// =========================================================
void setupPins();
float readSonar();
void setMotorSpeed(int speedL, int speedR);
void handleNormalState();
void handleObstacleState();
void updateServoScan();

// =========================================================
// 初始化
// =========================================================
void setup()
{
    Serial.begin(115200);
    setupPins();

    myServo.attach(PIN_SERVO_1);
    myServo.write(SERVO_CENTER);
    delay(500); // 等待舵机归位
}

void setupPins()
{
    // 超声波
    pinMode(PIN_SONAR_TRIG, OUTPUT);
    pinMode(PIN_SONAR_ECHO, INPUT);

    // 左电机
    pinMode(PIN_MOTOR_L_EN, OUTPUT);
    pinMode(PIN_MOTOR_L_IN1, OUTPUT);
    pinMode(PIN_MOTOR_L_IN2, OUTPUT);

    // 右电机
    pinMode(PIN_MOTOR_R_EN, OUTPUT);
    pinMode(PIN_MOTOR_R_IN3, OUTPUT);
    pinMode(PIN_MOTOR_R_IN4, OUTPUT);

    // 待机引脚
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    digitalWrite(PIN_MOTOR_STBY, HIGH); // 启用TB6612
}

// =========================================================
// 主循环
// =========================================================
void loop()
{
    if (currentState == STATE_NORMAL)
    {
        handleNormalState();
    }
    else if (currentState == STATE_OBSTACLE)
    {
        handleObstacleState();
    }
}

// =========================================================
// 传感器与执行器基础函数
// =========================================================

// 读取超声波距离（单位：厘米）
float readSonar()
{
    digitalWrite(PIN_SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONAR_TRIG, LOW);

    // timeout设置为30000微秒（约5米左右的往返时间），防止程序卡死
    long duration = pulseIn(PIN_SONAR_ECHO, HIGH, 30000);
    if (duration == 0)
        return MAX_TRACK_WIDTH; // 如果超时，认为距离很远

    float dist = duration * 0.034 / 2.0;
    return dist;
}

// 设置电机速度（正数前进，负数后退）
void setMotorSpeed(int speedL, int speedR)
{
    // 处理配置中的反转标志
    if (MOTOR_LEFT_INVERT)
        speedL = -speedL;
    if (MOTOR_RIGHT_INVERT)
        speedR = -speedR;

    // 控制左电机
    if (speedL >= 0)
    {
        digitalWrite(PIN_MOTOR_L_IN1, HIGH);
        digitalWrite(PIN_MOTOR_L_IN2, LOW);
        analogWrite(PIN_MOTOR_L_EN, constrain(speedL, 0, 255));
    }
    else
    {
        digitalWrite(PIN_MOTOR_L_IN1, LOW);
        digitalWrite(PIN_MOTOR_L_IN2, HIGH);
        analogWrite(PIN_MOTOR_L_EN, constrain(-speedL, 0, 255));
    }

    // 控制右电机
    if (speedR >= 0)
    {
        digitalWrite(PIN_MOTOR_R_IN3, HIGH);
        digitalWrite(PIN_MOTOR_R_IN4, LOW);
        analogWrite(PIN_MOTOR_R_EN, constrain(speedR, 0, 255));
    }
    else
    {
        digitalWrite(PIN_MOTOR_R_IN3, LOW);
        digitalWrite(PIN_MOTOR_R_IN4, HIGH);
        analogWrite(PIN_MOTOR_R_EN, constrain(-speedR, 0, 255));
    }
}

// =========================================================
// 状态处理逻辑
// =========================================================

// 无阻塞的舵机小角度连续扫描
void updateServoScan()
{
    unsigned long currentMillis = millis();
    if (currentMillis - lastServoMoveTime >= SERVO_SCAN_DELAY)
    {
        lastServoMoveTime = currentMillis;

        currentServoAngle += servoDirection * SERVO_SCAN_SPEED;

        if (currentServoAngle >= SERVO_CENTER + SERVO_SCAN_AMP)
        {
            currentServoAngle = SERVO_CENTER + SERVO_SCAN_AMP;
            servoDirection = -1; // 触及右边界，反向
        }
        else if (currentServoAngle <= SERVO_CENTER - SERVO_SCAN_AMP)
        {
            currentServoAngle = SERVO_CENTER - SERVO_SCAN_AMP;
            servoDirection = 1; // 触及左边界，反向
        }

        myServo.write(currentServoAngle);
    }
}

// 正常行进状态：小角度扫描 + PID 循迹
void handleNormalState()
{
    // 1. 更新舵机位置
    updateServoScan();

    // 2. 读取当前超声波距离
    float dist = readSonar();

    // 根据舵机的当前角度，将距离划分为左、中、右
    // 舵机角度：大于80是左侧，小于80是右侧
    if (currentServoAngle > SERVO_CENTER + 15)
    {
        dist_left = dist;
    }
    else if (currentServoAngle < SERVO_CENTER - 15)
    {
        dist_right = dist;
    }
    else
    {
        dist_center = dist;
        // 如果中间方向距离过近，触发避障状态
        if (dist_center > 0 && dist_center < OBSTACLE_DIST_THRES)
        {
            setMotorSpeed(0, 0); // 紧急停车
            currentState = STATE_OBSTACLE;
            return;
        }
    }

    // 3. 运行 PID 算法
    // 对左右距离进行限幅，避免在空旷区域单侧距离过大导致误差飙升
    float cap_left = min(dist_left, MAX_TRACK_WIDTH);
    float cap_right = min(dist_right, MAX_TRACK_WIDTH);

    // 误差：左边距离减去右边距离。
    // 如果 cap_left > cap_right，误差 > 0，说明左侧更宽阔，我们希望车向左偏。
    // 在这套逻辑下，向左偏需要右轮加速，左轮减速。
    error = cap_left - cap_right;

    integral += error;
    integral = constrain(integral, -100, 100); // 积分限幅防饱和

    float derivative = error - last_error;

    // PID 输出的是修正速度
    float turn_speed = Kp * error + Ki * integral + Kd * derivative;
    last_error = error;

    // 4. 将修正速度应用到马达
    // turn_speed > 0 时，speedR增加，speedL减小 -> 左转
    int speedL = BASE_SPEED - turn_speed;
    int speedR = BASE_SPEED + turn_speed;

    // 限幅并执行
    speedL = constrain(speedL, MIN_SPEED, MAX_SPEED);
    speedR = constrain(speedR, MIN_SPEED, MAX_SPEED);

    setMotorSpeed(speedL, speedR);

    delay(15); // 给超声波模块一点喘息时间
}

// 避障状态：大角度全景扫描，寻找最远距离并转向
void handleObstacleState()
{
    Serial.println("Obstacle detected! Scanning max angles...");

    int best_angle = SERVO_CENTER;
    float max_dist = 0;

    // 从右到左进行一次大角度扫描：0度到160度
    for (int angle = 0; angle <= 160; angle += 20)
    {
        myServo.write(angle);
        delay(100); // 减少等待舵机转动到位的时间 (原为200)

        float d = readSonar();
        Serial.print("Scan Angle: ");
        Serial.print(angle);
        Serial.print(" Dist: ");
        Serial.println(d);

        // 寻找最开阔的方向
        if (d > max_dist)
        {
            max_dist = d;
            best_angle = angle;
        }
    }

    Serial.print("Best safe angle: ");
    Serial.println(best_angle);

    // 扫描结束后，舵机归中
    myServo.write(SERVO_CENTER);
    currentServoAngle = SERVO_CENTER;
    delay(150); // 减少归中等待时间 (原为300)

    // 计算最开阔方向与正前方的角度差
    int angle_diff = best_angle - SERVO_CENTER;

    // 转向逻辑
    if (max_dist < OBSTACLE_DIST_THRES)
    {
        // 如果四周都被堵死（连最远距离都很近），选择后退并随机旋转
        Serial.println("Trapped! Reversing...");
        setMotorSpeed(-BASE_SPEED, -BASE_SPEED);
        delay(600);
        setMotorSpeed(BASE_SPEED, -BASE_SPEED); // 原地右转
        delay(800);
    }
    else
    {
        // 根据角度差估算需要的原地转向时间（时间常数 8 需根据实际车体打滑情况调整）
        int turn_time = abs(angle_diff) * 8;

        if (angle_diff > 10)
        {
            // 需要左转：左轮后退，右轮前进
            setMotorSpeed(-BASE_SPEED, BASE_SPEED);
            delay(turn_time);
        }
        else if (angle_diff < -10)
        {
            // 需要右转：左轮前进，右轮后退
            setMotorSpeed(BASE_SPEED, -BASE_SPEED);
            delay(turn_time);
        }
    }

    // 转向完毕，刹车
    setMotorSpeed(0, 0);
    delay(200);

    // 准备恢复正常状态，重置 PID 的状态变量和历史距离
    error = 0;
    last_error = 0;
    integral = 0;
    dist_left = MAX_TRACK_WIDTH;
    dist_right = MAX_TRACK_WIDTH;
    dist_center = MAX_TRACK_WIDTH;

    Serial.println("Returning to NORMAL state.");
    currentState = STATE_NORMAL;
}
