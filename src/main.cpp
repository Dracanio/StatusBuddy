#include <Arduino.h>
#include <WiFi.h>
const int LED_PIN = 33;
const int BUZZER_PIN = 34;
const int TOUCH_PIN = 26;
const int VIBRATION_PIN = 18;
const int BUTTON_PIN = 14;
const char WIFI_SSID[] = "CoCoLabor3_IoT";
const char WIFI_PASS[] = "cocolabor12345";
WiFiClient client;
WiFiServer server(80);


void setup() {
    Serial.begin(9600);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("connecting...");
    }
    server.begin();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void loop() {
// write your code here
}