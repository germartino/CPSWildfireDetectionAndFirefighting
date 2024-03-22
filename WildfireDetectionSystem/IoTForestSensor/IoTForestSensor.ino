#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include "HardwareSerial.h"

#define DHTTYPE DHT11
#define DHTPIN 13
#define GAS_ANALOG 36
#define BUZ 21
#define LED 12
// Replace the next variables with your SSID/Password combination
const char* ssid = "*****";
const char* password = "*****";

// Add your MQTT Broker IP address, example:
const char* mqtt_server = "192.168.1.184";
const int mqtt_port = 1883;

DHT dht (DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal lcd(19, 23, 18, 22, 4, 15);
StaticJsonDocument<256> forest;
HardwareSerial GPSSerial(2); // change with 1 if not working

const int flameSensor = 5;
long lastMsg = 0;
char msg[50];
char out[128];
char lat_coordinate[10];
char lon_coordinate[10];
unsigned char buffer[256];
int count = 0;
static const int TXPin = 17, RXPin = 16;
static const uint32_t GPSBaud = 9600;
boolean cam_alert = false;

// The TinyGPS++ object
TinyGPSPlus gps;

void setup() {

  Serial.begin(115200);
  lcd.begin(16, 2);
  dht.begin();
  GPSSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin );
  pinMode(LED, OUTPUT);
  pinMode(BUZ, OUTPUT);
  pinMode(flameSensor, INPUT);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "forest/iot/status") {
    deserializeJson(forest, messageTemp);
    String sensor = forest["sensor"];
    if (sensor == "FIRE_OFF") {
      Serial.print("FIRE EXTINGUISHED!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("FIRE EXTINGUISHED!");
      digitalWrite(BUZ, LOW);
      digitalWrite(LED, LOW);
    }
  }
  else if (String(topic) == "esp32CAM/detection") {
    if (messageTemp == "fire") {
      cam_alert = true;
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("IoT-Forest-Sensors ")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("forest/iot/status");
      client.subscribe("esp32CAM/detection");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void GPSInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void loop() {
  int gasSensorAnalog = analogRead(GAS_ANALOG);
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // This sketch displays information every time a new sentence is correctly encoded.
  while (GPSSerial.available() > 0)
    if (gps.encode(GPSSerial.read()))
      GPSInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }

  if (isnan(humidity) || isnan(temperature)) { // is not a number
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  } else {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
  }

  Serial.print("Gas Sensor Analog: ");
  Serial.println(gasSensorAnalog);

  delay(1000);

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;

    // Read from flame sensor
    int flame = digitalRead(flameSensor);

    if (flame == 1 || gasSensorAnalog >= 1500 || cam_alert) {
      cam_alert = false;
      Serial.print("FIRE DETECTED!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ALERT: ");
      lcd.setCursor(0, 1);
      lcd.print("FIRE DETECTED!");

      digitalWrite(BUZ, HIGH);
      digitalWrite(LED, HIGH);

      forest["sensor"] = "FIRE_ON";
      forest["temperature"] = temperature;
      forest["humidity"] = humidity;
      forest["co2"] = gasSensorAnalog;

      JsonArray data = forest.createNestedArray("position");
      data.add("*****"); //gps.location.lat()  
      data.add("*****"); //gps.location.lng()
      // Generate the minified JSON and send it to the Serial port.
      serializeJson(forest, out);
      client.publish("forest/iot/fire", out);
    }
    else {
      forest["temperature"] = temperature;
      forest["humidity"] = humidity;
      forest["co2"] = gasSensorAnalog;
      serializeJson(forest, out);
      client.publish("forest/iot/sensors", out);
    }
    memset(out, 0, sizeof(out));
  }
}
