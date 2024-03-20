#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#include <DHT.h>
#define DHT_PIN 23
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

#define WIFI_SSID "vivo 1806"
#define WIFI_PASSWORD "123456789Satu"
#define API_KEY "AIzaSyBpJQsE-cSkOpIj3uJ0DGDQayOOp-5Jo8k"
#define DATABASE_URL "https://sailwatch-cdd16-default-rtdb.asia-southeast1.firebasedatabase.app/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

void setup() {
  Serial.begin(115200);
  pinMode(DHT_PIN, INPUT);
  dht.begin();

  // connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting...");
  while(WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    delay(500);
  }
  Serial.println();
  Serial.println("Connected");
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if(Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("signUp OK");
    signupOK = true;
  } else {
    Serial.println("Signup Error");
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  // Serial.print("Humidity: ");
  // Serial.print(h);
  // Serial.println(" %\t");

  // Serial.print("Temperature: ");
  // Serial.print(t);
  // Serial.println(" *c");

  if(Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
    sendDataPrevMillis = millis();
    
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/humidity", h)) {
      Serial.println();
      Serial.print(h);
      Serial.print(" - Successfully save to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    } else {
      Serial.println("FAILED: " + fbdo.errorReason());
    }

    if(Firebase.RTDB.setFloat(&fbdo, "Sensor/temperature", t)) {
      Serial.println();
      Serial.print(t);
      Serial.print(" - Successfully save to: " + fbdo.dataPath());
      Serial.println(" (" + fbdo.dataType() + ") ");
    } else {
      Serial.println("FAILED: " + fbdo.errorReason());
    }
  }
}
