#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <DHT.h>
#include <TinyGPSPlus.h>

// Define sensor pins and types
#define DHT_PIN 23
#define DHT_TYPE DHT22
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define ANEMOMETER_PIN 19

// Wi-Fi credentials
#define WIFI_SSID "h"
#define WIFI_PASSWORD "123456789Satu"

// Firebase API key and database URL
#define API_KEY "AIzaSyBGnS0sO6KXRKL-k1Eb8WJJOAebLQYMLyw"
#define DATABASE_URL "https://sailwatch-48901-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Initialize sensors
DHT dht(DHT_PIN, DHT_TYPE);
TinyGPSPlus gps;

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Timer to control data sending frequency
unsigned long sendDataPrevMillis = 0;
unsigned long anemometerPrevMillis = 0;

// Variables to hold latitude and longitude
float latitude = 0.0;
float longitude = 0.0;

// Variables for wind speed calculation
volatile unsigned int windCounter = 0;
const float RADIUS_CM = 9.0; // Radius of the anemometer in centimeters
#define PI 3.1415926535897932384626433832795

// Function declarations
void connectToWiFi();
void initializeFirebase();
void sendDataToFirebase(float humidity, float temperature, float latitude, float longitude, float windSpeed);
void IRAM_ATTR handleAnemometerInterrupt();

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Initialize DHT sensor
  Serial.println("Initializing DHT sensor...");
  dht.begin();
  Serial.println("DHT sensor initialized.");

  // Connect to Wi-Fi
  connectToWiFi();

  // Initialize Firebase
  initializeFirebase();

  // Set up anemometer pin and interrupt
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), handleAnemometerInterrupt, FALLING);
}

void loop() {
  // Check if it's time to send data
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Read GPS data
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        if (gps.location.isValid()) {
          latitude = gps.location.lat();
          longitude = gps.location.lng();
        }
      }
    }

    // Read humidity and temperature
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // Check if any reads failed and exit early (to try again)
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    } else {
      Serial.print("Humidity: ");
      Serial.print(humidity);
      Serial.print(" %, Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");
    }

    // Calculate wind speed
    float windSpeed = 0.0;
    if (millis() - anemometerPrevMillis >= 1000) {
      detachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN));
      float revolutionsPerSecond = windCounter;
      windSpeed = (revolutionsPerSecond * 2 * PI * RADIUS_CM) / 100; // cm/s to m/s
      windCounter = 0;
      anemometerPrevMillis = millis();
      attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), handleAnemometerInterrupt, FALLING);
    }

    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s");

    sendDataToFirebase(humidity, temperature, latitude, longitude, windSpeed);
  }
}

// Connect to Wi-Fi
void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to Wi-Fi...");

  // Wait until connected
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// Initialize Firebase
void initializeFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback; // Optional but useful for debugging

  // Set authentication credentials
  auth.user.email = "sensor@gmail.com";
  auth.user.password = "sensor123456";

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// Send sensor data to Firebase
void sendDataToFirebase(float humidity, float temperature, float latitude, float longitude, float windSpeed) {
  // Send humidity data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/humidity", humidity)) {
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" % - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save humidity: ");
    Serial.println(fbdo.errorReason());
  }

  // Send temperature data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/temperature", temperature)) {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save temperature: ");
    Serial.println(fbdo.errorReason());
  }

  // Send latitude data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/latitude", latitude)) {
    Serial.print("Latitude: ");
    Serial.print(latitude);
    Serial.println(" - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save latitude: ");
    Serial.println(fbdo.errorReason());
  }

  // Send longitude data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/longitude", longitude)) {
    Serial.print("Longitude: ");
    Serial.print(longitude);
    Serial.println(" - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save longitude: ");
    Serial.println(fbdo.errorReason());
  }

  // Send wind speed data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/windSpeed", windSpeed)) {
    Serial.print("Wind Speed: ");
    Serial.print(windSpeed);
    Serial.println(" m/s - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save wind speed: ");
    Serial.println(fbdo.errorReason());
  }
}

// Interrupt service routine for anemometer
void IRAM_ATTR handleAnemometerInterrupt() {
  windCounter++;
}
