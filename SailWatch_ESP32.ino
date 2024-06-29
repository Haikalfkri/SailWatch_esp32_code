#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <DHT.h>
// #include <TinyGPSPlus.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>

// Define sensor pins and types
#define DHT_PIN 23
#define DHT_TYPE DHT11
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
// TinyGPSPlus gps;
Adafruit_BMP085 bmp;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Timer to control data sending frequency
unsigned long sendDataPrevMillis = 0;

// Variables to hold latitude and longitude
// float latitude = 0.0;
// float longitude = 0.0;
float windSpeedMph = 0.0;

// Variables for wind speed calculation
volatile unsigned long windPulseCount = 0;
unsigned long lastWindCheck = 0;
const unsigned long WIND_MEASUREMENT_INTERVAL = 5000;  // 5 seconds
const float ANEMOMETER_CALIBRATION_FACTOR = 2.23694;   // Adjust this based on your anemometer

// Debounce parameters
volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 15;  // Debounce delay in milliseconds

// Function declarations
void connectToWiFi();
void initializeFirebase();
void sendDataToFirebase(float humidity, float temperature, float windSpeedMph, float pressure, float altitude);
void IRAM_ATTR handleWindPulse();

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  // Initialize pressure sensor (bmp180)
  Serial.println("Initializing Pressure sensor...");
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  Serial.println("Pressure sensor initialized.");

  // Initialize DHT sensor
  Serial.println("Initializing DHT sensor...");
  dht.begin();
  Serial.println("DHT sensor initialized.");

  // Set anemometer pin as input and attach interrupt
  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), handleWindPulse, FALLING);

  // Connect to Wi-Fi
  connectToWiFi();

  // Initialize Firebase
  initializeFirebase();
}

void loop() {
  // Check if it's time to send data
  if (Firebase.ready() && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();

    // Read GPS data
    // while (Serial1.available() > 0) {
    //   if (gps.encode(Serial1.read())) {
    //     if (gps.location.isValid()) {
    //       latitude = gps.location.lat();
    //       longitude = gps.location.lng();
    //     }
    //   }
    // }

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
    unsigned long currentTime = millis();
    if (currentTime - lastWindCheck >= WIND_MEASUREMENT_INTERVAL) {
      // Calculate wind speed in mph
      float windSpeed = (windPulseCount / 2.0) * ANEMOMETER_CALIBRATION_FACTOR * (1.0 / (WIND_MEASUREMENT_INTERVAL / 1000.0));  // mph
      windSpeedMph = windSpeed;

      // Reset wind pulse count and last wind check time
      windPulseCount = 0;
      lastWindCheck = currentTime;

      Serial.print("Wind Speed: ");
      Serial.print(windSpeedMph, 1);
      Serial.println(" mph");
    }

    // Read pressure
    float pressure = bmp.readPressure();
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" Pa");

    // Read altitude
    float altitude = bmp.readAltitude();
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" Meters");

    sendDataToFirebase(humidity, temperature, windSpeedMph, pressure, altitude);
  }
}

// Interrupt Service Routine for wind speed pulse
void IRAM_ATTR handleWindPulse() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceDelay) {
    windPulseCount++;
    lastDebounceTime = currentTime;
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
  config.token_status_callback = tokenStatusCallback;  // Optional but useful for debugging

  // Set authentication credentials
  auth.user.email = "sensor1@gmail.com";
  auth.user.password = "sensor123456";

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

// Send sensor data to Firebase
void sendDataToFirebase(float humidity, float temperature, float windSpeedMph, float pressure, float altitude) {
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
  // if (Firebase.RTDB.setFloat(&fbdo, "Sensor/latitude", latitude)) {
  //   Serial.print("Latitude: ");
  //   Serial.print(latitude);
  //   Serial.println(" - Successfully saved to Firebase");
  // } else {
  //   Serial.print("Failed to save latitude: ");
  //   Serial.println(fbdo.errorReason());
  // }

  // // Send longitude data
  // if (Firebase.RTDB.setFloat(&fbdo, "Sensor/longitude", longitude)) {
  //   Serial.print("Longitude: ");
  //   Serial.print(longitude);
  //   Serial.println(" - Successfully saved to Firebase");
  // } else {
  //   Serial.print("Failed to save longitude: ");
  //   Serial.println(fbdo.errorReason());
  // }

  // Send wind speed data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/windSpeed", windSpeedMph)) {
    Serial.print("Wind Speed: ");
    Serial.print(windSpeedMph);
    Serial.println(" mph - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save wind speed: ");
    Serial.println(fbdo.errorReason());
  }

  // Send pressure data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/pressure", pressure)) {
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.println(" Pa - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save pressure: ");
    Serial.println(fbdo.errorReason());
  }

  // Send altitude data
  if (Firebase.RTDB.setFloat(&fbdo, "Sensor/altitude", altitude)) {
    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println(" Meters - Successfully saved to Firebase");
  } else {
    Serial.print("Failed to save altitude: ");
    Serial.println(fbdo.errorReason());
  }
}
