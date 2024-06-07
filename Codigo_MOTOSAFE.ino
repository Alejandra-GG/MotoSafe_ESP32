#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Replace with your network credentials
#define WIFI_SSID "ARRIS-78ED"
#define WIFI_PASSWORD "9CC8FC6478ED"

// Firebase credentials
#define FIREBASE_HOST "https://motosafe-58aca-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "TF3EjGCdkix2hUa9f3nI2CU6vREKIsAX7b7CJqDa"

// Create instances for GPS, MPU6050, and Firebase
#define GPSSerial Serial2
Adafruit_GPS GPS(&GPSSerial);
Adafruit_MPU6050 mpu;
FirebaseData firebaseData;
FirebaseConfig config;
FirebaseAuth auth;

uint32_t timer = millis();

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600);

  // Connect to Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Firebase configuration
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  // Initialize Firebase
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  // GPS initialization
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  GPSSerial.println(PMTK_Q_RELEASE);

  // MPU6050 initialization
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {
      return;
    }
  }

  // Read data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    if (GPS.fix) {
      float latitude = GPS.latitudeDegrees;
      float longitude = GPS.longitudeDegrees;
      
      // Send GPS data to Firebase
      if (Firebase.setFloat(firebaseData, "/location/latitude", latitude)) {
        Serial.println("Latitude data sent successfully");
      } else {
        Serial.println("Failed to send latitude data");
        Serial.println(firebaseData.errorReason());
      }

      if (Firebase.setFloat(firebaseData, "/location/longitude", longitude)) {
        Serial.println("Longitude data sent successfully");
      } else {
        Serial.println("Failed to send longitude data");
        Serial.println(firebaseData.errorReason());
      }
    } else {
      Serial.println("No GPS fix available");
    }

    // Send MPU6050 data to Firebase
    if (Firebase.setFloat(firebaseData, "/acceleration/x", a.acceleration.x)) {
      Serial.println("Acceleration X data sent successfully");
    } else {
      Serial.println("Failed to send acceleration X data");
      Serial.println(firebaseData.errorReason());
    }

    if (Firebase.setFloat(firebaseData, "/acceleration/y", a.acceleration.y)) {
      Serial.println("Acceleration Y data sent successfully");
    } else {
      Serial.println("Failed to send acceleration Y data");
      Serial.println(firebaseData.errorReason());
    }

    if (Firebase.setFloat(firebaseData, "/acceleration/z", a.acceleration.z)) {
      Serial.println("Acceleration Z data sent successfully");
    } else {
      Serial.println("Failed to send acceleration Z data");
      Serial.println(firebaseData.errorReason());
    }
  }

  delay(1000);
}

