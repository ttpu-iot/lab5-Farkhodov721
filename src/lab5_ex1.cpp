// lab5_ex1.cpp

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Arduino.h>
#include <ArduinoJson.h>


const int RED_PIN = 26;
const int GREEN_PIN = 27;
const int BLUE_PIN = 14;
const int YELLOW_PIN = 12;
const int BUZZER_PIN = 32;
const int SERVO_PIN = 5;
const int BUTTON_PIN = 25;
const int LIGHT_SENSOR_PIN = 33; 


const int SERVO_LEDC_CHANNEL = 1;
const int SERVO_FREQUENCY = 50; 


void setServoAngle(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  int pulse_us = map(angle, 0, 180, 1000, 2000);
  const uint32_t dutyMax = (1UL << 16) - 1;
  uint32_t duty = (uint32_t) pulse_us * SERVO_FREQUENCY * dutyMax / 1000000UL;
  ledcWrite(SERVO_LEDC_CHANNEL, duty);
}

hd44780_I2Cexp lcd;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// WiFi
char ssid[] = "Wokwi-GUEST";
char password[] = "";

// MQTT Broker settings
const char* mqtt_broker = "mqtt.iotserver.uz"; 
const int mqtt_port = 1883;
const char* mqtt_username = "userTTPU"; 
const char* mqtt_password = "";  

const char* mqtt_topic_pub = "ttpu/lab5/test/out";  
const char* mqtt_topic_sub = "ttpu/lab5/test/in";    

WiFiClient espClient;
PubSubClient mqtt_client(espClient);


volatile int servoSpeedMs = 10;
volatile int buzzerFreq = 1000; 
String mqttMessage = "";      

void connectWiFi();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void connectMQTT();

// FreeRTOS tasks
void servoTask(void* pvParameters);
void buzzerTask(void* pvParameters);
void buttonTask(void* pvParameters);
void lightTask(void* pvParameters);
void lcdTask(void* pvParameters);

//----------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);


  int status = lcd.begin(LCD_COLS, LCD_ROWS);
  if (status) {
    Serial.println("LCD initialization failed!");
    hd44780::fatalError(status);
  }
  lcd.noBacklight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

 
  ledcSetup(SERVO_LEDC_CHANNEL, SERVO_FREQUENCY, 16);
  ledcAttachPin(SERVO_PIN, SERVO_LEDC_CHANNEL);
  setServoAngle(90); 


  ledcSetup(2, buzzerFreq, 16);
  ledcAttachPin(BUZZER_PIN, 2);


  connectWiFi();


  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setCallback(mqttCallback);


  xTaskCreatePinnedToCore(servoTask, "servoTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(buzzerTask, "buzzerTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(buttonTask, "buttonTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(lightTask, "lightTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(lcdTask, "lcdTask", 4096, NULL, 1, NULL, 1);

  Serial.println("Setup complete");
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  if (!mqtt_client.connected()) {
    connectMQTT();
  }

  mqtt_client.loop();
  delay(10);
}


void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("\nConnecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed - will retry in loop");
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  DynamicJsonDocument doc(256);
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    // not JSON? treat as plain string
    String message = "";
    for (unsigned int i = 0; i < length; i++) message += (char)payload[i];
    mqttMessage = message;
    Serial.print("Plain message: ");
    Serial.println(message);
    return;
  }

  if (doc.containsKey("servoSpeedMs")) {
    servoSpeedMs = doc["servoSpeedMs"];
    Serial.print("Set servoSpeedMs = "); Serial.println(servoSpeedMs);
  }
  if (doc.containsKey("buzzerFreq")) {
    buzzerFreq = doc["buzzerFreq"];
    Serial.print("Set buzzerFreq = "); Serial.println(buzzerFreq);
    ledcSetup(2, buzzerFreq, 16);
  }
  if (doc.containsKey("message")) {
    mqttMessage = String((const char*)doc["message"]);
    Serial.print("Set mqttMessage = "); Serial.println(mqttMessage);
  }
}



void connectMQTT() {
  while (!mqtt_client.connected()) {
    Serial.println("Connecting to MQTT broker...");
    String client_id = "esp32-client-" + String(WiFi.macAddress());

    if (mqtt_client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker!");
      mqtt_client.subscribe(mqtt_topic_sub);
      Serial.print("Subscribed to topic: ");
      Serial.println(mqtt_topic_sub);
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.println(mqtt_client.state());
      Serial.println("Retrying in 5 seconds...");
      delay(5000);
    }
  }
}


void servoTask(void* pvParameters) {
  int angle = 0;
  int direction = 1;
  for (;;) {

  setServoAngle(angle);
    angle += direction;
    if (angle >= 180) { angle = 180; direction = -1; }
    if (angle <= 0) { angle = 0; direction = 1; }

    vTaskDelay(pdMS_TO_TICKS(servoSpeedMs));
  }
}

void buzzerTask(void* pvParameters) {
  for (;;) {
    if (buzzerFreq > 0) {
      ledcWriteTone(2, buzzerFreq);
      vTaskDelay(pdMS_TO_TICKS(1000));
      ledcWriteTone(2, 0);
    } else {
      // if freq is 0, skip
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelay(pdMS_TO_TICKS(4000)); 
  }
}

void buttonTask(void* pvParameters) {
  int lastState = LOW;
  for (;;) {
    int state = digitalRead(BUTTON_PIN);
    if (state == HIGH && lastState == LOW) {
      // Button pressed
      Serial.println("Button pressed - publishing");
      if (mqtt_client.connected()) {
        mqtt_client.publish(mqtt_topic_pub, "button_pressed");
      }
    }
    lastState = state;
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

void lightTask(void* pvParameters) {
  for (;;) {
    int raw = analogRead(LIGHT_SENSOR_PIN);
    char buf[64];
    snprintf(buf, sizeof(buf), "{\"light\":%d}", raw);
    if (mqtt_client.connected()) {
      mqtt_client.publish(mqtt_topic_pub, buf);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void lcdTask(void* pvParameters) {
  for (;;) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Servo:" );
    lcd.print("-- ");
    lcd.print("L:");
    lcd.print(analogRead(LIGHT_SENSOR_PIN));
    lcd.setCursor(0, 1);
    lcd.print(mqttMessage);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}
