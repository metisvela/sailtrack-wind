#include "Arduino.h"
#include "utilities.h"

// #define RS485_RX_PIN 2 // Example pin number for RX
// #define RS485_TX_PIN 3 // Example pin number for TX
// #define RS485_CON_PIN 4 // Example pin number for RS485 direction control

//HardwareSerial Serial485(Serial485); // Use the third serial port on ESP32

// RS485 transmission control
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Wind sensor request packet
const byte windSensorRequest[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
byte windSensorResponse[9]; // Response buffer

void RS485_Mode(int Mode) {
    digitalWrite(RS485_CON_PIN, Mode);
}

void setup() {
    Serial.begin(9600); // Start the built-in serial port, for debugging
    Serial485.begin(4800, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN); // Start the RS485 communication port
    pinMode(RS485_CON_PIN, OUTPUT); // RS485 direction control pin
    RS485_Mode(RS485_RECEIVE); // Initialize in receive mode
}

void loop() {
    // Transmit request
    RS485_Mode(RS485_TRANSMIT);
    delay(10); // Wait a bit for the line to become stable

    Serial485.write(windSensorRequest, sizeof(windSensorRequest)); // Send the request

    RS485_Mode(RS485_RECEIVE);
    delay(10); // Delay to allow response

    // Wait for response
    unsigned long startTime = millis();
    while (Serial485.available() < sizeof(windSensorResponse) && millis() - startTime < 1000) {
        delay(1);
    }

    // Read the response if available
    if (Serial485.available() >= sizeof(windSensorResponse)) {
        for (int i = 0; i < sizeof(windSensorResponse); i++) {
            windSensorResponse[i] = Serial485.read();
        }

        // Process the wind speed and direction from the response
        int windSpeedInt = (windSensorResponse[3] << 8) | windSensorResponse[4];
        float windSpeedMps = windSpeedInt / 100.0;
        float windSpeedKph = windSpeedMps * 3.6;
        
        int windDirection = (windSensorResponse[5] << 8) | windSensorResponse[6];

        Serial.print("Wind Speed: ");
        Serial.print(windSpeedKph);
        Serial.println(" km/h");
        Serial.print("Wind Direction: ");
        Serial.print(windDirection);
        Serial.println(" degrees");
    } else {
        Serial.println("No response or incomplete response from the sensor.");
    }

    delay(2000); // Wait for a bit before the next request
}
