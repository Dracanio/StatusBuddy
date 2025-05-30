#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <WiFiManager.h>
#include "rgb_lcd.h"
#include "TouchStates/TouchStates.h"
#include "config.h"

#define MQTT_TOPIC_CONST "statusBuddy/Device1"
#define MQTT_TOPIC2_CONST "statusBuddy/Device2"

WiFiClientSecure secureClient;
PubSubClient mqtt_client(secureClient);
rgb_lcd lcd;
TouchStates touch_state;
SemaphoreHandle_t lcd_semaphore;

touch_struct our_touch;
touch_struct opponent_touch;

void setupWiFi() {
    // WiFiManager
    WiFiManager wm;
    wm.setDebugOutput(true);
    bool res = wm.autoConnect(WiFiName, "12345678");
    if (!res) {
        Serial.println("Keine Verbindung – Neustart");
        ESP.restart();
    }
    Serial.println("WLAN verbunden!");
}

void setupPins() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBRATION_PIN, OUTPUT);
    pinMode(TOUCH_PIN, INPUT);
}

void touch_detection(void *parameters) {
    (void) parameters;
    for (;;) {
        while (digitalRead(TOUCH_PIN) == HIGH) {
            Serial.println("Touch detected, sending");
            mqtt_client.publish(MQTT_TOPIC_Publish, "highfive");
            our_touch.touched = true;
            our_touch.timestamp = millis();
            vTaskDelay(200 / portTICK_RATE_MS);
        }
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}

TouchStates get_touch_state() {
    if (our_touch.touched == true && opponent_touch.touched == true) {
        return BOTH;
    }
    if (our_touch.touched == true || opponent_touch.touched == true) {
        return ONE;
    }
    return NONE;
}

void lcd_writer(void *parameters) {
    (void) parameters;
    for (;;) {
        if (xSemaphoreTake(lcd_semaphore, portMAX_DELAY)) {
            switch (get_touch_state()) {
                case NONE:
                    lcd.setColor(RED);
                    digitalWrite(BUZZER_PIN, LOW);
                    digitalWrite(VIBRATION_PIN, LOW);
                    break;
                case ONE:
                    lcd.setColor(BLUE);
                    digitalWrite(BUZZER_PIN, LOW);
                    digitalWrite(VIBRATION_PIN, LOW);
                    break;
                case BOTH:
                    lcd.setColor(GREEN);
                    digitalWrite(BUZZER_PIN, HIGH);
                    digitalWrite(VIBRATION_PIN, HIGH);
                    break;
                default:
                    break;
            }
            lcd.clear();
            lcd.setCursor(0, 0);
            xSemaphoreGive(lcd_semaphore);
        }
        vTaskDelay(500 / portTICK_RATE_MS);
    }
}

void keep_connection_alive(void *parameters) {
    (void) parameters;
    for (;;) {
        if (!mqtt_client.connected()) {
            while (!mqtt_client.connected()) {
                if (mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASS)) {
                    mqtt_client.subscribe(MQTT_TOPIC_Subscribe);
                }else {
                    vTaskDelay(500 / portTICK_RATE_MS);
                }
            }
        }
        mqtt_client.loop();
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

void touch_reset(void *parameters) {
    (void) parameters;
    for (;;) {
        if (our_touch.touched && millis() - our_touch.timestamp > 2000) {
            our_touch.touched = false;
        }
        if (opponent_touch.touched && millis() - our_touch.timestamp > 2000) {
            opponent_touch.touched = false;
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    if (String(topic) == MQTT_TOPIC_Subscribe && msg == "highfive") {
        opponent_touch.touched = true;
        opponent_touch.timestamp = millis();
        Serial.println("MQTT: High Five vom anderen Gerät empfangen!");
    }
}

void setup() {
    Serial.begin(9600);
    setupWiFi();
    secureClient.setInsecure();
    setupPins();
    mqtt_client.setServer(MQTT_SERVER,MQTT_PORT);
    mqtt_client.setCallback(callback);

    lcd_semaphore = xSemaphoreCreateMutex();
    lcd.begin(16,2);
    our_touch.touched = false;
    xTaskCreate(touch_detection, "Touch detection Task", 2048, NULL, 1, NULL);
    xTaskCreate(lcd_writer, "LCD Writer Task", 2048, NULL, 1, NULL);
    xTaskCreate(touch_reset, "Reset Touch Task", 2048, NULL, 1, NULL);
    xTaskCreatePinnedToCore(keep_connection_alive, "Keep Connection Alive Task", 4096, NULL,1,NULL, ARDUINO_RUNNING_CORE);
}

void loop() {
}