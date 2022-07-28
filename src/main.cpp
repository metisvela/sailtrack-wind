#include <Arduino.h>
#include <SailtrackModule.h>
#include <driver/mcpwm.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "esp_private/esp_clk.h"

#define ECHO_F GPIO_NUM_26
#define ECHO_L GPIO_NUM_33
#define ECHO_R GPIO_NUM_34

#define TRIGGER_F GPIO_NUM_18
#define TRIGGER_L GPIO_NUM_19
#define TRIGGER_R GPIO_NUM_23

#define LENGTH .475 //[m]
#define SPD(m1, m2) .5 * LENGTH *(1 / m1 - 1 / m2)

#define BATTERY_ADC_PIN 35
#define BATTERY_ADC_RESOLUTION 4095
#define BATTERY_ADC_REF_VOLTAGE 1.1
#define BATTERY_ESP32_REF_VOLTAGE 3.3
#define BATTERY_NUM_READINGS 32
#define BATTERY_READING_DELAY_MS 20

static bool echo_isr(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata, void *arg);

static QueueHandle_t measure_queue;

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

typedef struct pulse
{
    uint32_t pulse_count;
    mcpwm_capture_channel_id_t cap_id;
} pulse;

void setup()
{
    Serial.begin(115200);
    Serial.println("Hello I am alive!");
    // Get cpu frequency
    // cpu_freq = ESP.getCpuFreqMHz();
    // Serial.println(cpu_freq);

    // Setp PINs and interrupts
    pinMode(TRIGGER_F, OUTPUT);
    pinMode(TRIGGER_L, OUTPUT);
    pinMode(TRIGGER_R, OUTPUT);
    digitalWrite(TRIGGER_F, LOW);
    digitalWrite(TRIGGER_L, LOW);
    digitalWrite(TRIGGER_R, LOW);
    // pinMode(ECHO_F, INPUT_PULLDOWN);
    // pinMode(ECHO2, INPUT_PULLDOWN);
    // pinMode(ECHO3, INPUT_PULLDOWN);

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, ECHO_F));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, ECHO_L));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, ECHO_R));
    ESP_ERROR_CHECK(gpio_pulldown_en(ECHO_F));
    ESP_ERROR_CHECK(gpio_pulldown_en(ECHO_L));
    ESP_ERROR_CHECK(gpio_pulldown_en(ECHO_R));

    mcpwm_capture_config_t conf = {
        .cap_edge = MCPWM_BOTH_EDGE,
        .cap_prescale = 1,
        .capture_cb = echo_isr,
        .user_data = NULL};

    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, &conf));
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, &conf));
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, &conf));

    Serial.println("Pins set...");

    measure_queue = xQueueCreate(6, sizeof(pulse));
    if (measure_queue == NULL)
    {
        Serial.println("failed to alloc measure_queue");
        return;
    }
    Serial.println("Queue created...");

    stm.begin("wind0", IPAddress(192, 168, 42, 100), new ModuleCallbacks());

}

unsigned int trig1[] = {TRIGGER_F, TRIGGER_L, TRIGGER_R};
unsigned int trig2[] = {TRIGGER_L, TRIGGER_R, TRIGGER_F};
unsigned int idx = 0;

static uint32_t cap_val_begin_of_sample[] = {0, 0, 0};
static uint32_t cap_val_end_of_sample[] = {0, 0, 0};

void loop()
{
    double measure;
    double v_sound;
    double raw_pulses_us[6];

    char sensor;
    for (size_t idx = 0; idx < 3; idx++)
    {
        pulse p;
        uint32_t m[3] = {0, 0, 0};

        GPIO.out_w1ts = (1 << trig1[idx] | 1 << trig2[idx]);
        delayMicroseconds(20);
        GPIO.out_w1tc = (1 << trig1[idx] | 1 << trig2[idx]);

        while (xQueueReceive(measure_queue, &p, 200) == pdTRUE)
        {
            // Serial.printf("%d: %.3f us\n",p.cap_id, p.pulse_count*(1000000.0 / esp_clk_apb_freq()));
            m[p.cap_id] = p.pulse_count ;
        }

        // Serial.printf("%d\t%d\t%d\n", m[0], m[1], m[2]);
        // delay(100);
        switch (idx)
        {
        case 0:
            raw_pulses_us[0] = m[0]*(1000000.0 / esp_clk_apb_freq());
            raw_pulses_us[1] = m[1]*(1000000.0 / esp_clk_apb_freq());
            break;
        case 1:
            raw_pulses_us[2] = m[1]*(1000000.0 / esp_clk_apb_freq());
            raw_pulses_us[3] = m[2]*(1000000.0 / esp_clk_apb_freq());
            break;
        case 2:
            raw_pulses_us[4] = m[2]*(1000000.0 / esp_clk_apb_freq());
            raw_pulses_us[5] = m[0]*(1000000.0 / esp_clk_apb_freq());
            break;
        }
    }

    StaticJsonDocument<STM_JSON_DOCUMENT_MEDIUM_SIZE> doc;

    doc["raw_us_0"] = raw_pulses_us[0];
    doc["raw_us_1"] = raw_pulses_us[1];
    doc["raw_us_2"] = raw_pulses_us[2];
    doc["raw_us_3"] = raw_pulses_us[3];
    doc["raw_us_4"] = raw_pulses_us[4];
    doc["raw_us_5"] = raw_pulses_us[5];

    stm.publish("wind0/raw", doc.as<JsonObjectConst>());
}

static bool echo_isr(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata, void *arg)
{
    BaseType_t high_task_wakeup = pdFALSE;
    if (edata->cap_edge == MCPWM_POS_EDGE)
    {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample[cap_sig] = edata->cap_value;
        cap_val_end_of_sample[cap_sig] = cap_val_begin_of_sample[cap_sig];
    }
    else
    {
        cap_val_end_of_sample[cap_sig] = edata->cap_value;

        pulse p;
        p.pulse_count = cap_val_end_of_sample[cap_sig] - cap_val_begin_of_sample[cap_sig];
        p.cap_id = cap_sig;
        xQueueSendFromISR(measure_queue, (void *)&p, &high_task_wakeup);
        // Serial.println(high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}