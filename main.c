// Experiment 7
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "config.h"
#include <WiFiClientSecure.h>

// Define pins
#define LDR_PIN 32  // Analog pin for Light Sensor (LDR)
#define LED_PIN 27  // Digital pin for LED 

// Setup MQTT over TLS
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, IO_SERVER, IO_SERVERPORT, IO_USERNAME, IO_KEY);

// Feed for publishing light data
Adafruit_MQTT_Publish lightLevel = Adafruit_MQTT_Publish(&mqtt, IO_USERNAME "/feeds/light"); // Uplink feed
// Feed for subscribing to led control from adafruit io
Adafruit_MQTT_Subscribe ledControl = Adafruit_MQTT_Subscribe(&mqtt, IO_USERNAME "/feeds/led"); // Downlink feed

// Timing variables for sending sensor data
unsigned long previousMillis = 0;
const long interval = 5000;  // Send light data every 5 seconds

void setup() {
  // Init serial monitor
  Serial.begin(115200);
  delay(1000);
  Serial.println("Experiment 7");

  // Init pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED off initially
  pinMode(LDR_PIN, INPUT);     // LDR sensor input

  // Connect to WiFi
  connectToWiFi();

  // Configure TLS connection
  client.setTimeout(15000);    // Increase timeout to allow for a long setup
  client.setInsecure();     
  // client.setCertificate(adafruitio_root_ca);
  Serial.println("TLS security configured");

  // Setup subscription for LED control feed downlink)
  mqtt.subscribe(&ledControl);

  Serial.println("Setup complete");
}

void loop() {
  // ebsure connection to MQTT
  if (!mqtt.connected()) {
    MQTT_connect();
    delay(1000);
    return;
  }
  
  // Keep the connection alive with ping
  if (!mqtt.ping()) {
    Serial.println("Connection lost - reconnecting");
    mqtt.disconnect();
    return;
  }

   // UPLINK: Send light sensor reading to Adafruit IO
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read the light level from LDR sensor
    int lightValue = analogRead(LDR_PIN);
    Serial.print("Light level: ");
    Serial.println(lightValue);

    // Publish data to Adafruit IO cloud
    Serial.print("Uploading data to cloud... ");
    if (!lightLevel.publish((int32_t)lightValue)) {
      Serial.println("Failed!");
    } else {
      Serial.println("Success!");
    }
  }
  
  // DOWNLINK: Check for incoming messages to control LED
  Adafruit_MQTT_Subscribe *subscription;
  // allow for timeout on open connection
  while ((subscription = mqtt.readSubscription(50))) {
    if (subscription == &ledControl) {
      // Message received from adafruit to control LED
      String ledState = (char *)ledControl.lastread;
      Serial.print("Received command: ");
      Serial.println(ledState);

      // Turn LED on or off based on received message
      if (ledState.equalsIgnoreCase("ON")) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("LED turned ON");
      } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println("LED turned OFF");
      }
    }
  }
}

// Connect to WiFi network
void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.disconnect(true);
  delay(1000);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Wait for connection with timeout
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (++attempts > 20) {
      Serial.println("\nWiFi connection failed. Restarting...");
      ESP.restart();
    }
  }

  Serial.println("\nWiFi connected successfully!");
}

// Connect or reconnect to MQTT broker
void MQTT_connect() {
  if (mqtt.connected()) {
    return;  // already connected
  }

  Serial.println("Establishing MQTT connection");
  
  // ensure WiFi is still connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost, reconnecting first");
    connectToWiFi();
  }
  
  // Attempt MQTT connection
  Serial.print("Connecting to Adafruit IO... ");
  
  mqtt.disconnect();  // Ensure clean state
  delay(3000);
  
  int8_t ret = mqtt.connect();
  if (ret != 0) {
    Serial.println("Connection failed");
    Serial.printf("Error: %d - %s\n", ret, mqtt.connectErrorString(ret));
  } else {
    Serial.println("Connection successful!");
    // Resubscribe to receive LED control messages
    mqtt.subscribe(&ledControl);
  }
}