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

bool touchSent = false;
bool receivedTouch = false;
unsigned long touchTime = 0;
unsigned long recivedTime = 0;

struct touch_struct {
    bool touched;
    unsigned long timestamp;
};
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

void touch_detection(void* parameters) {
    (void) parameters;
    for (;;) {
        while (digitalRead(TOUCH_PIN) == HIGH) {
            mqtt_client.publish(MQTT_TOPIC_Publish, "highfive");
            our_touch.touched = true;
            our_touch.timestamp = millis();
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
                    break;
                case ONE:
                    lcd.setColor(BLUE);
                    break;
                case BOTH:
                    lcd.setColor(GREEN);
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
                    mqtt_client.subscribe(MQTT_TOPIC_CONST); //use MQTT_TOPIC_CONST or MQTT_TOPIC2_CONST
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
        receivedTouch = true;
        recivedTime = millis();
        //opponent_touch.touched = true;
        //opponent_touch.timestamp = millis();
        Serial.println("MQTT: High Five vom anderen Gerät empfangen!");
    }
}

void reconnect() {
    while (!mqtt_client.connected()) {
        if (mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASS)) {
            mqtt_client.subscribe(MQTT_TOPIC_Subscribe);
        } else {
            delay(1000);
            Serial.println("MQTT: Reconnecting to server");
        }
    }
}

void setup() {
    Serial.begin(9600);
    setupWiFi();
    secureClient.setInsecure();
    setupPins();
    mqtt_client.setServer(MQTT_SERVER,MQTT_PORT);
    mqtt_client.setCallback(callback);


    //WIP
    lcd_semaphore = xSemaphoreCreateMutex();
    our_touch.touched = false;
   /* xTaskCreate(touch_detection, "Touch detection Task", 2048, NULL, 1, NULL);
    xTaskCreate(lcd_writer, "LCD Writer Task", 2048, NULL, 1, NULL);
    xTaskCreate(touch_reset, "Reset Touch Task", 2048, NULL, 1, NULL);
    xTaskCreatePinnedToCore(keep_connection_alive, "Keep Connection Alive Task", 2048, NULL,1,NULL, ARDUINO_RUNNING_CORE);*/
}

void loop() {
    if (!mqtt_client.connected()) reconnect();
    mqtt_client.loop();

    //MQTT Message senden bzw Highfive senden
    bool touchState = digitalRead(TOUCH_PIN);
    if (touchState && !touchSent) {
        Serial.println("Touch erkannt → sende High Five");
        mqtt_client.publish(MQTT_TOPIC_Publish, "highfive");
        touchSent = true;
        touchTime = millis();
    }
    // Reset touch wenn länger nicht gedrückt
    if (touchSent && millis() - touchTime > 2000) {
        touchSent = false;
    }
    // Reset recived touch wenn länger nicht gedrückt
    if (receivedTouch && millis() - recivedTime > 2000) {
        receivedTouch = false;
    }

    // Nach Highfive prüfen
    if (touchSent && receivedTouch) {
        Serial.println("🎉 HIGH FIVE erkannt!");
        digitalWrite(LED_PIN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        digitalWrite(VIBRATION_PIN, HIGH);
        delay(3000); // LED 3s an
        digitalWrite(LED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(VIBRATION_PIN, LOW);
        receivedTouch = false;
    }
}