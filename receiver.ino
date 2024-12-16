#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Motor Control Message Structure
typedef struct MotorData {
    int forward;  // Forward speed (0-255)
    int backward; // Backward speed (0-255)
    int turn;     // -1 for left, 0 for stop, 1 for right
} MotorData;

// Create an instance of MotorData
MotorData motorData;

// Pin Definitions for L298N (4 Motors)
const int LED_PIN = 2;           // ESP32 onboard LED

// First L298N - Motor1 and Motor2
const int MOTOR1_PIN1 = 25;      // Motor1 Forward (1A)
const int MOTOR1_PIN2 = 33;      // Motor1 Backward (1B)
const int MOTOR2_PIN1 = 26;      // Motor2 Forward (2A)
const int MOTOR2_PIN2 = 27;      // Motor2 Backward (2B)

// Second L298N - Motor3 and Motor4
const int MOTOR3_PIN1 = 14;      // Motor3 Forward (1A)
const int MOTOR3_PIN2 = 12;      // Motor3 Backward (1B)
const int MOTOR4_PIN1 = 13;      // Motor4 Forward (2A)
const int MOTOR4_PIN2 = 15;      // Motor4 Backward (2B)

// Connection Status
bool isConnected = false;
unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT = 2000;  // 2 seconds

// Callback function for receiving data
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    memcpy(&motorData, incomingData, sizeof(motorData));

    // Debugging output
    Serial.println("Data Received:");
    Serial.printf("Forward Speed: %d\n", motorData.forward);
    Serial.printf("Backward Speed: %d\n", motorData.backward);
    Serial.printf("Turn Direction: %d\n", motorData.turn);

    lastReceiveTime = millis();
    isConnected = true;
}

void setup() {
    // Initialize Serial Monitor
    Serial.begin(115200);

    // Pin Setup
    pinMode(LED_PIN, OUTPUT);

    pinMode(MOTOR1_PIN1, OUTPUT);
    pinMode(MOTOR1_PIN2, OUTPUT);
    pinMode(MOTOR2_PIN1, OUTPUT);
    pinMode(MOTOR2_PIN2, OUTPUT);

    pinMode(MOTOR3_PIN1, OUTPUT);
    pinMode(MOTOR3_PIN2, OUTPUT);
    pinMode(MOTOR4_PIN1, OUTPUT);
    pinMode(MOTOR4_PIN2, OUTPUT);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the receive callback function
    esp_now_register_recv_cb(OnDataRecv);

    Serial.println("ESP-NOW Receiver Initialized");
}

void stopAllMotors() {
    // Stop all motors
    analogWrite(MOTOR1_PIN1, 0);
    analogWrite(MOTOR1_PIN2, 0);
    analogWrite(MOTOR2_PIN1, 0);
    analogWrite(MOTOR2_PIN2, 0);
    analogWrite(MOTOR3_PIN1, 0);
    analogWrite(MOTOR3_PIN2, 0);
    analogWrite(MOTOR4_PIN1, 0);
    analogWrite(MOTOR4_PIN2, 0);
}

void loop() {
    unsigned long currentTime = millis();

    // Check connection status
    if (isConnected && (currentTime - lastReceiveTime > TIMEOUT)) {
        isConnected = false;  // Connection lost
    }

    // LED and Motor Control Logic
    if (isConnected) {
        digitalWrite(LED_PIN, HIGH);  // Connected

        if (motorData.forward > 0) {
            // Forward Movement: All motors forward
            motorData.forward=150;
            analogWrite(MOTOR1_PIN1, motorData.forward);
            analogWrite(MOTOR1_PIN2, 0);

            analogWrite(MOTOR2_PIN1, motorData.forward);
            analogWrite(MOTOR2_PIN2, 0);

            analogWrite(MOTOR3_PIN1, motorData.forward);
            analogWrite(MOTOR3_PIN2, 0);

            analogWrite(MOTOR4_PIN1, motorData.forward);
            analogWrite(MOTOR4_PIN2, 0);
        } 
        else if (motorData.backward > 0) {
            // Backward Movement: All motors backward

            motorData.backward=150;
            analogWrite(MOTOR1_PIN1, 0);
            analogWrite(MOTOR1_PIN2, motorData.backward);

            analogWrite(MOTOR2_PIN1, 0);
            analogWrite(MOTOR2_PIN2, motorData.backward);

            analogWrite(MOTOR3_PIN1, 0);
            analogWrite(MOTOR3_PIN2, motorData.backward);

            analogWrite(MOTOR4_PIN1, 0);
            analogWrite(MOTOR4_PIN2, motorData.backward);
        } 
        else if (motorData.turn == -1) {
            // Left Turn: Motor1 and Motor3 Reverse, Motor2 and Motor4 Forward
            analogWrite(MOTOR1_PIN1, 0);
            analogWrite(MOTOR1_PIN2, 150);  // Reverse

            analogWrite(MOTOR2_PIN1, 0);  // Forward
            analogWrite(MOTOR2_PIN2, 150);

            analogWrite(MOTOR3_PIN1, 150);
            analogWrite(MOTOR3_PIN2, 0);  // Reverse

            analogWrite(MOTOR4_PIN1, 150);  // Forward
            analogWrite(MOTOR4_PIN2, 0);
        } 
        else if (motorData.turn == 1) {
            // Right Turn: Motor1 and Motor3 Forward, Motor2 and Motor4 Reverse
            analogWrite(MOTOR1_PIN1, 150);  // Forward
            analogWrite(MOTOR1_PIN2, 0);

            analogWrite(MOTOR2_PIN1, 150);
            analogWrite(MOTOR2_PIN2, 0);  // Reverse

            analogWrite(MOTOR3_PIN1, 0);  // Forward
            analogWrite(MOTOR3_PIN2, 150);

            analogWrite(MOTOR4_PIN1, 0);
            analogWrite(MOTOR4_PIN2, 150);  // Reverse
        } 
        else {
            // Stop Motors
            stopAllMotors();
        }
    } 
    else {
        // Blink LED to indicate disconnection
        digitalWrite(LED_PIN, millis() % 500 < 250 ? HIGH : LOW);

        // Stop all motors
        stopAllMotors();
    }

    delay(100);
}
