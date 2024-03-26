
#include "Arduino.h"
#include "SailtrackModule.h"


#define MQTT_PUBLISH_FREQ_HZ		5
#define AHRS_UPDATE_FREQ_HZ			100

#define BATTERY_ADC_PIN 			15
#define BATTERY_ADC_RESOLUTION 		4095
#define BATTERY_ADC_REF_VOLTAGE 	1.1
#define BATTERY_ESP32_REF_VOLTAGE	3.3
#define BATTERY_NUM_READINGS 		32
#define BATTERY_READING_DELAY_MS	20

#define I2C_SDA_PIN 				27
#define I2C_SCL_PIN 				25

#define LOOP_TASK_INTERVAL_MS		1000 / AHRS_UPDATE_FREQ_HZ
#define MQTT_TASK_INTERVAL_MS	 	1000 / MQTT_PUBLISH_FREQ_HZ


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

void onMqttMessage(const char * topic, JsonObjectConst message) {
    //TODO if necessary implememt this function
}

//beginWIND(){...}

void setup()
{   
    //TODO add wind.begin()
    stm.begin("windtest", IPAddress(192, 168, 42, 117), new ModuleCallbacks());

    Serial.begin(115200);
}


void loop() {
	
	TickType_t lastWakeTime = xTaskGetTickCount();
	if (true) { //TODo check valid data
		StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
		doc["test1"] = "15";		
		doc["test"] = "16";
		stm.publish("sensor/gps0", doc.as<JsonObjectConst>());
	}
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}
