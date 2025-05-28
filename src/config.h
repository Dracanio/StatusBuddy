//
// Created by Andre on 28.05.2025.
//

#ifndef CONFIG_H
#define CONFIG_H

//WiFiManager Config
const char WiFiName[] = "ESP32_HighFive1";

//Pin Config
const int LED_PIN = 33;
const int BUZZER_PIN = 14;
const int TOUCH_PIN = 26;
const int VIBRATION_PIN = 18;

//MQTT Config
const char MQTT_SERVER[] = "6c900d6407744144abdc9e2fa58d9cdd.s1.eu.hivemq.cloud";
const int MQTT_PORT = 8883;
const char MQTT_USERNAME[] = "kito1";   //HiveMQ Username
const char MQTT_PASS[] = "Iot123456";   //HiveMQ PW
const char MQTT_CLIENT_ID[] = "Device2";    //muss für Devices angepasst werden
const char MQTT_TOPIC_Publish[] = "statusBuddy/Device1";
const char MQTT_TOPIC_Subscribe[] = "statusBuddy/Device2";

//Touch Struct
struct touch_struct {
    bool touched;
    unsigned long timestamp;
};

#endif //CONFIG_H
