#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

const int LED_PIN = 33;
const int BUZZER_PIN = 34;
const int TOUCH_PIN = 26;
const int VIBRATION_PIN = 18;
const int BUTTON_PIN = 14;
const char WIFI_SSID[] = "CoCoLabor3_IoT";
const char WIFI_PASS[] = "cocolabor12345";


//MQTT Setup
const char MQTT_SERVER[] = "6c900d6407744144abdc9e2fa58d9cdd.s1.eu.hivemq.cloud";
const char MQTT_USERNAME[] = "kito1";
const char MQTT_PASS[] = "Iot123456";
const int MQTT_PORT = 8883;
const char MQTT_CLIENT_ID[] = "Device1";    //muss für Devices angepasst werden
const char MQTT_TOPIC[] = "statusBuddy/Device1";
const char MQTT_TOPIC2[] = "statusBuddy/Device2";

WiFiClientSecure secureClient;
PubSubClient mqtt_client(secureClient);
WiFiServer server(80);

bool touchSent = false;
bool receivedTouch = false;
unsigned long touchTime = 0;
unsigned long recivedTime = 0;

void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print("connecting...");
    }
    server.begin();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void setupPins() {
    pinMode(LED_PIN, OUTPUT);
    //pinMode(BUZZER_PIN, OUTPUT);
    //pinMode(VIBRATION_PIN, OUTPUT);
    //pinMode(BUTTON_PIN, INPUT);
    pinMode(TOUCH_PIN, INPUT);
}

void callback(char* topic, byte* payload, unsigned int length) {
    String msg;
    for (unsigned int i = 0; i < length; i++) {
        msg += (char)payload[i];
    }

    if (String(topic) == MQTT_TOPIC2 && msg == "highfive") { //für device2 MQTT_TOPIC2 = MQTT_TOPIC
        receivedTouch = true;
        recivedTime = millis();
        Serial.println("MQTT: High Five vom anderen Gerät empfangen!");
    }
}

void reconnect() {
    while (!mqtt_client.connected()) {
        if (mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASS)) {
            mqtt_client.subscribe(MQTT_TOPIC2); //für device2 MQTT_TOPIC2 = MQTT_TOPIC
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

}

void loop() {
    if (!mqtt_client.connected()) reconnect();
    mqtt_client.loop();

    //MQTT Message senden bzw Highfive senden
    bool touchState = digitalRead(TOUCH_PIN);
    if (touchState && !touchSent) {
        Serial.println("Touch erkannt → sende High Five");
        mqtt_client.publish(MQTT_TOPIC, "highfive"); //für device2 MQTT_TOPIC = MQTT_TOPIC2
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
        delay(3000); // LED 3s an
        digitalWrite(LED_PIN, LOW);
        receivedTouch = false;
    }
}