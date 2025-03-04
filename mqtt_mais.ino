#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

const char* ssid = "tp-link";
const char* password = "09270734452";
const char* mqtt_server = "157.245.204.46";

WiFiClient espClient;
PubSubClient client(espClient);

#define DHTTYPE DHT11
const int DHTPin = 0;
const int SoilMoisturePin = A0;
DHT dht(DHTPin, DHTTYPE);

#define TEMP_TOPIC "sensor/device1/temperature"
#define HUMIDITY_TOPIC "sensor/device1/humidity"

void setup() {
  Serial.begin(115200);
  dht.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  client.setServer(mqtt_server, 1883);
  reconnect();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  sendTemperatureData();
  // soilMoistureData();

  delay(5000);  // Publish every 5 seconds
}

void soilMoistureData() {
  float moisture_percentage = (100.00 - ((analogRead(SoilMoisturePin) / 1023.00) * 100.00));

  Serial.print("Soil Moisture(in Percentage) = ");
  Serial.print(moisture_percentage);
  Serial.println("%");
}

float lastTemperature = -1000.0;
float lastHumidity = -1000.0;

void sendTemperatureData() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("❌ Failed to read from DHT sensor!");
    return;
  }

  // Check if values changed before sending
  if (temperature == lastTemperature && humidity == lastHumidity) {
    Serial.println("🔄 No change in temperature or humidity, skipping MQTT publish.");
    return;
  }

  lastTemperature = temperature;
  lastHumidity = humidity;

  Serial.print("📡 Sending Data - Temperature: ");
  Serial.print(temperature);
  Serial.print("°C, Humidity: ");
  Serial.print(humidity);
  Serial.println("%");

  // Convert float to string before sending via MQTT
  char tempString[8], humString[8];
  dtostrf(temperature, 6, 2, tempString);
  dtostrf(humidity, 6, 2, humString);

  client.publish(TEMP_TOPIC, tempString);
  client.publish(HUMIDITY_TOPIC, humString);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("ESP8266_TempMonitor")) {
      Serial.println("✅ Connected!");
      client.subscribe(TEMP_TOPIC);
      client.subscribe(HUMIDITY_TOPIC);
    } else {
      Serial.print("❌ Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying...");
      delay(2000);
    }
  }
}
