#include <Arduino.h>
#include <ArduinoLog.h>

// ==========================================
// ESP32-CAM 主程序
// ==========================================

void setup() {
    Serial.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial);
    Log.notice(F("ESP32-CAM Started" CR));
}

void loop() {
    // 暂时空置，等待后续实现摄像头逻辑
    delay(1000);
}
