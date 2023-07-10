// Sonic Anemometer
#include <SailtrackModule.h>
#include <math.h>
  
// -------------------------- Configuration -------------------------- //

#define MQTT_PUBLISH_FREQ_HZ		5
#define BATTERY_ADC_PIN 		    35
#define BATTERY_ADC_RESOLUTION 		4095
#define BATTERY_ADC_REF_VOLTAGE 	1.1
#define BATTERY_ESP32_REF_VOLTAGE	3.3
#define BATTERY_NUM_READINGS 	  	32
#define BATTERY_READING_DELAY_MS	20

// i pimn io generarno impulsi, i vout sono le tensioni ottenute
#define PIN_IO1 					13
#define PIN_VOUT1					3
#define PIN_IO2 					0
#define PIN_VOUT2					12
#define PIN_IO3 					2
#define PIN_VOUT3					38
#define PIN_IO4 					14
#define PIN_VOUT4					4

#define SIGNAL						0
#define LOOP_TASK_INTERVAL_MS  		1 		// Da ridefinire //TODO
// -------------------------------------------------------------------- //

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

void setup(){
    stm.begin("wind_phase", IPAddress(192, 168, 42, 106), new ModuleCallbacks());
    stm.subscribe("sensor/wind0");
}

void loop(){
	int distanza;
	int tempoandatax;
	int temporitornox;
	int tempoandatay;
	int temporitornoy;
	double vy = 0.5 * (distanza) * ((1/tempoandatay) - (1/temporitornoy));
	double vx = 0.5 * (distanza) * ((1/tempoandatax) - (1/temporitornox));
	double v = sqrt(vy*vy + vx*vx);
	double theta = atan2(vx , vy);
	//TODO sviluppare una relazione per la velocità del suono e la temperatura?


TickType_t lastWakeTime = xTaskGetTickCount();
StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;
// qui va creato il doc //TODO
stm.publish("sensor/wind0", doc.as<JsonObjectConst>());

vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}