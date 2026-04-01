#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

// ==========================================
// 引脚定义配置文件
// ==========================================

// --- HC-SR04 超声波传感器 ---
#define PIN_SONAR_TRIG 13
#define PIN_SONAR_ECHO 12

// --- SG90 舵机 ---
#define PIN_SERVO_1 10

// --- WS2812 RGB 灯 ---
#define PIN_WS2812 4
#define WS2812_NUM_LEDS 1

// --- MPU6050 (I2C) ---
// SDA -> A4
// SCL -> A5
#define PIN_I2C_SDA A4
#define PIN_I2C_SCL A5

// --- TB6612 马达驱动 ---
// 左电机
#define PIN_MOTOR_L_EN 6 // PWMB
#define PIN_MOTOR_L_IN1 8
#define PIN_MOTOR_L_IN2 9 // 修复：原为 8，现改为 9，以支持反转

// 右电机
#define PIN_MOTOR_R_EN 5 // PWMA
#define PIN_MOTOR_R_IN3 7
#define PIN_MOTOR_R_IN4 2 // 修复：原为 7，现改为 2，以支持反转
#define PIN_MOTOR_STBY 3

// 电机方向反转开关：0 正常，1 反转（仅影响方向，不影响速度）
#define MOTOR_LEFT_INVERT 0
#define MOTOR_RIGHT_INVERT 0

#endif // PIN_CONFIG_H
