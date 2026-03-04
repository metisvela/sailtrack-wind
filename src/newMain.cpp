#include <Arduino.h>
#include <Wire.h>
#include <SailtrackModule.h>

//#define Wired_Com

#define MQTT_PUBLISH_FREQ_HZ    5
#define AHRS_UPDATE_FREQ_HZ     5

#define BATTERY_ADC_PIN         35
#define BATTERY_ADC_RESOLUTION  4095
#define BATTERY_ADC_REF_VOLTAGE 1.1
#define BATTERY_ESP32_REF_VOLTAGE 3.3
#define BATTERY_NUM_READINGS    32
#define BATTERY_READING_DELAY_MS 20

#define LOOP_TASK_INTERVAL_MS   5
#define MQTT_TASK_INTERVAL_MS   1000 / MQTT_PUBLISH_FREQ_HZ

#define HALL_PIN 27   // Hall effect sensor pin
#define PULSES_PER_REV 1  // Number of pulses per rotation

SailtrackModule stm;

class ModuleCallbacks: public SailtrackModuleCallbacks {
    void onStatusPublish(JsonObject status) {
        JsonObject battery = status.createNestedObject("battery");
        float avg = 0;
        for (int i = 0; i < BATTERY_NUM_READINGS; i++) {
            avg += analogRead(BATTERY_ADC_PIN) / BATTERY_NUM_READINGS;
            delay(BATTERY_READING_DELAY_MS);
        }
        battery["voltage"] = 2 * avg / BATTERY_ADC_RESOLUTION * BATTERY_ESP32_REF_VOLTAGE * BATTERY_ADC_REF_VOLTAGE;
    }
};

#define AS5600_ADDR 0x36
#define AS5600_RAW_ANGLE_HIGH 0x0C
#define AS5600_RAW_ANGLE_LOW 0x0D

uint16_t readRawAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_ANGLE_HIGH);
    Wire.endTransmission(false);

    Wire.requestFrom(AS5600_ADDR, 2);
    if (Wire.available() == 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte = Wire.read();
        return (highByte << 8) | lowByte;
    } else {
        Serial.println("Error: No data received from AS5600");
        return 0;
    }
}

int windDir() {
    uint16_t rawAngle = readRawAngle();
    int windDirN = map(rawAngle, 0, 4095, 0, 360);
    return windDirN;
}

volatile unsigned long pulseCount = 0;

void IRAM_ATTR hallISR() {
    pulseCount++;
}

float getRPM(unsigned int pulsesPerRevolution) {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - lastTime;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    lastTime = currentTime;

    float rpm = (float)pulses / pulsesPerRevolution * (60000.0 / dt);
    return rpm;
}

void mqttTask(void * pvArguments) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (true) {
        StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
        JsonObject wind = doc.createNestedObject("wind");

        int angle = windDir();
        float rpm = getRPM(PULSES_PER_REV);

        wind["Direction"] = angle;
        wind["RPM"] = rpm;

        stm.publish("sensor/wind", doc.as<JsonObjectConst>());
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(MQTT_TASK_INTERVAL_MS));
    }
}

void setup() {
    Wire.begin();
    Serial.begin(115200);
    while (!Serial);

    pinMode(HALL_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);
    #ifndef Wired_Com
        stm.begin("wind", IPAddress(192, 168, 42, 104), new ModuleCallbacks());
        xTaskCreate(mqttTask, "mqttTask", STM_TASK_MEDIUM_STACK_SIZE, NULL, STM_TASK_MEDIUM_PRIORITY, NULL);
    #endif
}

void loop() {


    uint16_t angle = windDir();
    float rpm = getRPM(PULSES_PER_REV);

    #ifdef Wired_Com
        Serial.print("Wind Angle: "); Serial.println(angle);
        Serial.print("RPM: "); Serial.println(rpm);
        Serial.println("________");
    #endif

    #ifndef Wired_Com
        TickType_t lastWakeTime = xTaskGetTickCount();
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
    #endif


    delay(100);
}