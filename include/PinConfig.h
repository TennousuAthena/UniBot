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

// --- L298N 马达驱动 (预留) ---
// 左电机
#define PIN_MOTOR_L_EN 5 // PWM
#define PIN_MOTOR_L_IN1 6
#define PIN_MOTOR_L_IN2 7

// 右电机
// 避免与舵机(定时器1)和WS2812冲突：EN=3，IN4=8
#define PIN_MOTOR_R_EN 3 // PWM
#define PIN_MOTOR_R_IN3 2
#define PIN_MOTOR_R_IN4 8

// 电机方向反转开关：0 正常，1 反转（仅影响方向，不影响速度）
#define MOTOR_LEFT_INVERT 1
#define MOTOR_RIGHT_INVERT 1

#endif // PIN_CONFIG_H
