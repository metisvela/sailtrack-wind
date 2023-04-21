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

#define W_CALM						529		//West sensor reading in still air
#define E_CALM 						535		//East sensor reading in still air
#define W_NORTH 					530		//West sensor reading in north wind
#define E_NORTH 					537		//East sensor reading in north wind
#define W_EAST						525		//West sensor reading in east wind
#define E_EAST						540		//East sensor reading in east wind
#define WINDSPEED					15.3	//Wind speed during calibration

#define WEST_PIN					25		//West sensor analog pin
#define EAST_PIN					27		//East sensor analog pin

#define LOOP_TASK_INTERVAL_MS  		50 		// Da ridefinire TODO

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
    stm.subscribe("sensor/wind0");	//TODO non capisco come funziona
}

void loop(){

int Vwest = analogRead(WEST_PIN);        // Measure west sensor voltage
Vwest = analogRead(WEST_PIN);        	 // Repeat for settling time //TODO non capisco perche venga fatta 2 volte
int Veast = analogRead(EAST_PIN);        //Measure east sensor voltage
Veast = analogRead(EAST_PIN);
double northwind = (Vwest + Veast - E_CALM - W_CALM) / (W_NORTH + E_NORTH - E_CALM - W_CALM) * WINDSPEED;
double eastwind = (Vwest - W_CALM - (Veast - E_CALM)) / (W_EAST - W_CALM - (E_EAST - E_CALM)) * WINDSPEED;
int wind = round(sqrt(northwind * northwind + eastwind * eastwind)); 	//Calculate wind speed
int heading = 270 - std::round(atan2((double) - northwind, (double) - eastwind) * 57.3);	//(northwind<0);


//TODO forse non serve grazie ad atan2 function line 59
if (heading > 359)
	heading = heading - 360; 	//Calculate wind direction 


TickType_t lastWakeTime = xTaskGetTickCount();
StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;	
doc["west_sensor_reading"]=Vwest;
doc["east_sensor_reading"]=Veast;
doc["wind_speed"]=wind; 		// in m/s
doc["wind_direction"]=heading;	// in degrees
stm.publish("sensor/wind0", doc.as<JsonObjectConst>());

vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(LOOP_TASK_INTERVAL_MS));
}