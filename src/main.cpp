#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Firebase_ESP_Client.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiManager.h>  // Include the WiFiManager library
#include <Wire.h>

#include "addons/RTDBHelper.h"
#include "addons/TokenHelper.h"
#include "sntp.h"
#include "time.h"

HardwareSerial sim900aSerial(2);
String phoneNumber = "+639669036425";  // "+639105510642";
String message = "ALERT: Pinyaxtract Extractor hit 70°C! Heater auto-shutoff activated to prevent damage.";

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
#define RPWM_PIN 5
#define PWM_FREQ 1000
#define PWM_CHANNEL_R 0
#define PWM_RESOLUTION 8

#define INCREASE_BUTTON_PIN 32
#define DECREASE_BUTTON_PIN 33

#define STEP_PIN 19
#define DIR_PIN 18
#define BTN_INC_PIN 14
#define BTN_DEC_PIN 27

#define DHTPIN 25
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);
bool smsSent = false;  // Flag to track if SMS was already sent
uint32_t delayMS;
// Define relay pins
const int RELAY_HEATER_LOW = 15;
const int RELAY_HEATER_HIGH = 13;
const int RELAY_FAN_PIN = 12;     // Pin for Fan relay (new pin)
const int RELAY_ENGINE_PIN = 23;  // Pin for Engine relay (new pin)

// Target temperature
const float targetTemp = 60.0;
const char* ssid = "PLDTHOMEFIBRbaa40";
const char* password = "PLDTWIFI9xq6z";

#define DATABASE_URL "https://pinyaxtract-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define API_KEY "AIzaSyBvgmzFBU4F3t-ytAEP9Pn8j3Z_YvLiiAw"

FirebaseData fbdo, fbdo_s1, fbdo_s2, fbdo_s3, fbdo_s4, fbdo_s5;
FirebaseAuth auth;
FirebaseConfig config;
int Steps_Per_Unit = 200;  // Set this to the appropriate value based on your motor's specifications

#define STEPS_PER_MM 1000
#define MAX_COMPRESSION 5
#define MIN_COMPRESSION 0
#define STEP_DELAY 500

float currentCompression = 0;
int motorSpeed = 100;
const int speedStep = 5;
String Heater = "";  // To hold the current state ("low", "high", "off")
String relayLowState = "";
String relayHighState = "";
unsigned long lastMillisTemp = 0;
const unsigned long tempInterval = 5000;  // Update every 2 seconds
unsigned long previousMillis = 0;         // Tracks the last time the weight was checked
const unsigned long interval = 5000;
unsigned long debounceMillis[4] = {0, 0, 0, 0};  // Debounce timers for buttons
const unsigned long debounceDelay = 300;
unsigned long sendDataPrevMillis = 0;
bool firebaseReady = false;
bool signupOK = false;
bool HeaterManual = true;

bool FanStatus = true;
bool EngineStatus = false;
// Initialize LCD
LiquidCrystal_I2C lcdTempWeight(0x26, 16, 2);        // LCD for Temperature and Weight
LiquidCrystal_I2C lcdCompressionSpeed(0x27, 16, 2);  // LCD for Compression and Motor Speed
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long gmtOffset_sec = 28000;
const int daylightOffset_sec = 28000;

// Functions Declaration
void sendSMS(String phoneNumber, String message);
void sendATCommand(String command);
void sendATCommand(String command);
void Temp();
void fetchHeaterStateFromFirebase();
void updateRelays();
void setHeater(String newState);
bool isButtonPressed(int pin, int index);

// Functions Implementation
void sendSMS(String phoneNumber, String message) {
    Serial.println("Sending SMS...");

    sendATCommand("AT+CMGF=1");

    String command = "AT+CMGS=\"" + phoneNumber + "\"";
    sendATCommand(command);

    sendATCommand(message);
    sim900aSerial.write(0x1A);
    delay(1000);

    Serial.println("SMS Sent!");
}
void sendATCommand(String command) {
    sim900aSerial.println(command);
    delay(1000);
    while (sim900aSerial.available()) {
        Serial.write(sim900aSerial.read());
    }
}

void Temp() {
    unsigned long currentMillis = millis();

    if (currentMillis - lastMillisTemp >= tempInterval) {
        lastMillisTemp = currentMillis;  // update the lastMillisTemp time

        delay(delayMS);
        sensors_event_t event;
        dht.temperature().getEvent(&event);

        if (!isnan(event.temperature)) {
            Serial.println(event.temperature);

            // Store the actual temperature instead of overwriting it with 0
            byte temperature = (byte)event.temperature;
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.println(" °C");

            lcdTempWeight.clear();
            lcdTempWeight.setCursor(0, 0);
            lcdTempWeight.print("Temp: ");
            lcdTempWeight.print(temperature);
            lcdTempWeight.print("C");
            if (temperature >= 70) {
                digitalWrite(RELAY_HEATER_LOW, HIGH);
                digitalWrite(RELAY_HEATER_HIGH, HIGH);
                Serial.println("Heater: OFF");
                sendSMS(phoneNumber, message);
            }
            if (!HeaterManual) {
                if (temperature < targetTemp) {
                    digitalWrite(RELAY_HEATER_LOW, LOW);
                    digitalWrite(RELAY_HEATER_HIGH, LOW);
                    Serial.println("Heater: LOW");
                } else if (temperature == targetTemp) {
                    digitalWrite(RELAY_HEATER_LOW, HIGH);
                    digitalWrite(RELAY_HEATER_HIGH, LOW);
                    Serial.println("Heater: HIGH");
                } else {
                    digitalWrite(RELAY_HEATER_LOW, HIGH);
                    digitalWrite(RELAY_HEATER_HIGH, HIGH);
                    Serial.println("Heater: OFF");
                }

            } else {
                Serial.println("Automatic Heater is Disabled!.");
                fetchHeaterStateFromFirebase();
                // Read relay states from Firebase
            }
            if (Firebase.RTDB.setInt(&fbdo, "/Temperature", temperature)) {
                Serial.println("Temperature sent to Firebase successfully!");
            } else {
                Serial.printf("Error sending temperature to Firebase: %s\n", fbdo.errorReason().c_str());
            }
        }
    }
}
void fetchHeaterStateFromFirebase() {
    if (Firebase.RTDB.getBool(&fbdo, "/HeaterManual")) {
        HeaterManual = fbdo.boolData();
        Serial.printf("Fetched HeaterManual: %s\n", HeaterManual ? "true" : "false");
    }

    if (firebaseReady && Firebase.RTDB.getString(&fbdo, "/Heater")) {
        Heater = fbdo.stringData();
        Serial.printf("Relay State: %s\n", Heater.c_str());
        updateRelays();
    }
}
void updateRelays() {
    if (Heater == "low") {
        digitalWrite(RELAY_HEATER_LOW, LOW);    // Activate LOW relay
        digitalWrite(RELAY_HEATER_HIGH, HIGH);  // Deactivate HIGH relay
        Serial.println("Relay Low is ON, Relay High is OFF");
    } else if (Heater == "high") {
        digitalWrite(RELAY_HEATER_LOW, LOW);   // Activate LOW relay
        digitalWrite(RELAY_HEATER_HIGH, LOW);  // Activate HIGH relay
        Serial.println("Relay High and Relay Low are ON");
    } else {                                    // "off"
        digitalWrite(RELAY_HEATER_LOW, HIGH);   // Deactivate LOW relay
        digitalWrite(RELAY_HEATER_HIGH, HIGH);  // Deactivate HIGH relay
        Serial.println("Both Relays are OFF");
    }
}
void setHeater(String newState) {
    Heater = newState;

    if (firebaseReady) {
        Firebase.RTDB.setString(&fbdo, "/Heater", Heater);
    }

    updateRelays();  // Update relays immediately
}
// Global weight variable
float weight = 0.0;

bool isButtonPressed(int pin, int index) {
    if (digitalRead(pin) == LOW && millis() - debounceMillis[index] > debounceDelay) {
        debounceMillis[index] = millis();
        return true;
    }
    return false;
}

void setup() {
    Serial.begin(115200);

    sim900aSerial.begin(9600, SERIAL_8N1, 17, 16);  // RX pin: 17, TX pin: 16
    delay(2000);
    Serial.println("Initializing SIM900A...");
    sendATCommand("AT");  // Check if SIM900A module is responding
    Serial.println("SIM900A initialized");

    // Initialize motor PWM
    ledcSetup(PWM_CHANNEL_R, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(RPWM_PIN, PWM_CHANNEL_R);

    dht.begin();
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    delayMS = sensor.min_delay / 1000;

    // Initialize relays
    pinMode(RELAY_HEATER_LOW, OUTPUT);
    pinMode(RELAY_HEATER_HIGH, OUTPUT);
    pinMode(RELAY_FAN_PIN, OUTPUT);
    pinMode(RELAY_ENGINE_PIN, OUTPUT);

    digitalWrite(RELAY_HEATER_LOW, HIGH);
    digitalWrite(RELAY_HEATER_HIGH, HIGH);
    digitalWrite(RELAY_FAN_PIN, HIGH);
    digitalWrite(RELAY_ENGINE_PIN, HIGH);

    // Initialize LCDs
    lcdTempWeight.begin(16, 2);
    lcdTempWeight.backlight();
    lcdTempWeight.setCursor(0, 0);
    lcdTempWeight.print("PinyaXTractor");

    lcdCompressionSpeed.begin(16, 2);
    lcdCompressionSpeed.backlight();
    lcdCompressionSpeed.setCursor(0, 0);
    lcdCompressionSpeed.print("PinyaXTractor");
    // Initialize buttons
    pinMode(INCREASE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(DECREASE_BUTTON_PIN, INPUT_PULLUP);

    // put your setup code here, to run once:
    // Create WiFiManager object

    WiFiManager wifiManager;

    // Uncomment to reset stored Wi-Fi settings
    // wifiManager.resetSettings();

    // Try to connect to Wi-Fi, if it fails, create an AP
    if (!wifiManager.autoConnect("PinyaXtractor")) {
        Serial.println("Failed to connect, restarting...");
    }

    WiFi.setAutoReconnect(true);  // Auto-reconnect kapag nawala ang Wi-Fi
    WiFi.persistent(true);        // I-save ang Wi-Fi credentials sa flash memory

    Serial.println("Connected to Wi-Fi!");

    Serial.println("\nWi-Fi connected!");
    lcdTempWeight.clear();
    lcdTempWeight.print("Wi-Fi Connected");

    lcdCompressionSpeed.clear();
    lcdCompressionSpeed.print("Wi-Fi Connected");

    // Firebase setup
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;

    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("Firebase sign-up successful!");
        signupOK = true;
    } else {
        Serial.printf("Firebase sign-up failed: %s\n", config.signer.signupError.message.c_str());
    }

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);

    if (Firebase.ready()) {
        firebaseReady = true;
        Serial.println("Firebase initialized successfully.");
    } else {
        Serial.println("Firebase initialization failed.");
    }

    if (!Firebase.RTDB.beginStream(&fbdo_s1, "/RollerSpeed")) {
        Serial.printf("Stream fail (RollerSpeed): %s\n", fbdo_s1.errorReason().c_str());
    }

    if (!Firebase.RTDB.beginStream(&fbdo_s2, "/ExtractionLevel")) {
        Serial.printf("Stream fail (ExtractionLevel): %s\n", fbdo_s2.errorReason().c_str());
    }

    if (!Firebase.RTDB.beginStream(&fbdo_s3, "/HeaterManual")) {
        Serial.printf("Stream fail (HeaterManual): %s\n", fbdo_s3.errorReason().c_str());
    }

    if (!Firebase.RTDB.beginStream(&fbdo_s4, "/Weight")) {
        Serial.printf("Stream fail (Machine): %s\n", fbdo_s4.errorReason().c_str());
    }

    if (!Firebase.RTDB.beginStream(&fbdo_s5, "/Fan")) {
        Serial.printf("Stream fail (Fan): %s\n", fbdo_s5.errorReason().c_str());
    }

    lcdCompressionSpeed.clear();
    lcdCompressionSpeed.print("Setup completed!");

    lcdTempWeight.clear();
    lcdTempWeight.print("Setup completed!");

    delay(1000);  // Allow serial monitor to initialize

    Serial.println("Setup completed!");
}
void loop() {
    lcdTempWeight.setCursor(0, 1);
    lcdTempWeight.print("Weight: ");
    lcdTempWeight.print(weight);  // Make sure this refers to the variable
    lcdTempWeight.print("kg");

    Temp();

    if (isButtonPressed(INCREASE_BUTTON_PIN, 0)) {
        motorSpeed = min(255, motorSpeed + speedStep);
        ledcWrite(PWM_CHANNEL_R, motorSpeed);
        Serial.printf("Motor speed increased to: %d\n", motorSpeed);

        lcdCompressionSpeed.clear();
        lcdCompressionSpeed.setCursor(0, 0);
        lcdCompressionSpeed.print("Speed: ");
        lcdCompressionSpeed.setCursor(0, 1);
        lcdCompressionSpeed.print(motorSpeed);
        lcdCompressionSpeed.print("RPM");
    }

    if (isButtonPressed(DECREASE_BUTTON_PIN, 1)) {
        motorSpeed = max(0, motorSpeed - speedStep);
        ledcWrite(PWM_CHANNEL_R, motorSpeed);
        Serial.printf("Motor speed decreased to: %d\n", motorSpeed);

        lcdCompressionSpeed.clear();
        lcdCompressionSpeed.setCursor(0, 0);
        lcdCompressionSpeed.print("Speed: ");
        lcdCompressionSpeed.setCursor(0, 1);
        lcdCompressionSpeed.print(motorSpeed);
        lcdCompressionSpeed.print("RPM");
    }
    // Firebase update every 5 seconds
    if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)) {
        sendDataPrevMillis = millis();

        lcdCompressionSpeed.clear();
        lcdCompressionSpeed.setCursor(0, 0);
        lcdCompressionSpeed.print("Speed: ");
        lcdCompressionSpeed.setCursor(0, 1);
        lcdCompressionSpeed.print(motorSpeed);
        lcdCompressionSpeed.print("RPM");

        // Update Firebase values
        if (Firebase.RTDB.setInt(&fbdo, "/RollerSpeed", motorSpeed)) {
            Serial.println("Motor speed updated in Firebase.");
        } else {
            Serial.printf("Failed to update motor speed: %s\n", fbdo.errorReason().c_str());
        }

        if (Firebase.RTDB.setBool(&fbdo, "/HeaterManual", HeaterManual)) {
            Serial.println("HeaterManual updated in Firebase.");
        } else {
            Serial.printf("Failed to update HeaterManual: %s\n", fbdo.errorReason().c_str());
        }
    }

    // Firebase read operations
    if (Firebase.ready() && signupOK) {
        // Read motor speed (int)
        if (!Firebase.RTDB.readStream(&fbdo_s1)) {
            Serial.printf("Stream 1 read Error: %s\n", fbdo_s1.errorReason().c_str());
        } else if (fbdo_s1.streamAvailable() && fbdo_s1.dataType() == "int") {
            motorSpeed = fbdo_s1.intData();
            Serial.printf("Motor Speed: %d\n", motorSpeed);
            ledcWrite(PWM_CHANNEL_R, motorSpeed);
        }

        // Read HeaterManual status (boolean)
        if (!Firebase.RTDB.readStream(&fbdo_s3)) {
            Serial.printf("Stream 3 read Error: %s\n", fbdo_s3.errorReason().c_str());
        } else if (fbdo_s3.streamAvailable()) {
            if (fbdo_s3.dataType() == "boolean") {
                HeaterManual = fbdo_s3.boolData();
                Serial.printf("Heater Manual: %s\n", HeaterManual ? "ON" : "OFF");
            } else {
                Serial.println("Stream 3 received invalid data type.");
            }
        }

        // Read weight (float)
        if (!Firebase.RTDB.readStream(&fbdo_s4)) {
            Serial.printf("Stream 4 read Error: %s\n", fbdo_s4.errorReason().c_str());
        } else if (fbdo_s4.streamAvailable()) {
            if (fbdo_s4.dataType() == "float") {
                weight = fbdo_s4.floatData();
                Serial.printf("Weight: %.2f\n", weight);
            } else if (fbdo_s4.dataType() == "int") {
                weight = (float)fbdo_s4.intData();  // Convert int to float
                Serial.printf("Weight (Converted from Int): %.2f\n", weight);
            } else {
                Serial.printf("Stream 4 received invalid data type: %s\n", fbdo_s4.dataType().c_str());
            }
        }

        // Read FanStatus (boolean)
        if (!Firebase.RTDB.readStream(&fbdo_s5)) {
            Serial.printf("Stream 5 read Error: %s\n", fbdo_s5.errorReason().c_str());
        } else if (fbdo_s5.streamAvailable() && fbdo_s5.dataType() == "boolean") {
            FanStatus = fbdo_s5.boolData();
            Serial.printf("Fan Status: %s\n", FanStatus ? "ON" : "OFF");
            digitalWrite(RELAY_FAN_PIN, !FanStatus);
        }
    }
}
