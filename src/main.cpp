#include <Arduino.h>
#include <SailtrackModule.h>
#include <driver/mcpwm.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "esp_private/esp_clk.h"

#define ECHO1 GPIO_NUM_26
#define ECHO2 GPIO_NUM_33
#define ECHO3 GPIO_NUM_34

#define TRIGGER1 GPIO_NUM_18
#define TRIGGER2 GPIO_NUM_19
#define TRIGGER3 GPIO_NUM_23

#define LENGTH .475 //[m]
#define SPD(m1, m2) .5 * LENGTH *(1 / m1 - 1 / m2)

int cpu_freq = 0;

static bool echo_isr(mcpwm_unit_t mcpwm, mcpwm_capture_channel_id_t cap_sig, const cap_event_data_t *edata, void *arg);

static QueueHandle_t measure_queue;

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
    cpu_freq = ESP.getCpuFreqMHz();
    Serial.println(cpu_freq);

    // Setp PINs and interrupts
    pinMode(TRIGGER1, OUTPUT);
    pinMode(TRIGGER2, OUTPUT);
    pinMode(TRIGGER3, OUTPUT);
    // pinMode(ECHO1, INPUT_PULLDOWN);
    // pinMode(ECHO2, INPUT_PULLDOWN);
    // pinMode(ECHO3, INPUT_PULLDOWN);

    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, ECHO1));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, ECHO2));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, ECHO3));
    ESP_ERROR_CHECK(gpio_pulldown_en(ECHO1));
    ESP_ERROR_CHECK(gpio_pulldown_en(ECHO2));
    ESP_ERROR_CHECK(gpio_pulldown_en(ECHO3));

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
    if (measure_queue == NULL) {
        Serial.println("failed to alloc measure_queue");
        return;
    }
    Serial.println("Queue created...");
}

unsigned int echo[] = {ECHO1, ECHO2, ECHO3, ECHO2, ECHO1, ECHO3};
unsigned int trig1[] = {TRIGGER1, TRIGGER1, TRIGGER2, TRIGGER2, TRIGGER3, TRIGGER3};
unsigned int trig2[] = {TRIGGER2, TRIGGER2, TRIGGER3, TRIGGER3, TRIGGER1, TRIGGER1};
unsigned int idx = 0;

static uint32_t cap_val_begin_of_sample[] = {0, 0, 0};
static uint32_t cap_val_end_of_sample[] = {0, 0, 0};

void loop()
{
    uint32_t measure[6];
    Serial.println("In the loop!");

    for (size_t idx = 0; idx < 6; idx++)
    {
        pulse p;
        Serial.printf("Measure n-%d\n", idx);

        digitalWrite(trig1[idx], HIGH);
        digitalWrite(trig2[idx], HIGH);
        delayMicroseconds(10);
        digitalWrite(trig1[idx], LOW);
        digitalWrite(trig2[idx], LOW);

        // Serial.println("Sent pulse...");

        while (xQueueReceive(measure_queue, &p, 1000) == pdTRUE)
        {
            Serial.printf("%d: %.3f us\n",p.cap_id, p.pulse_count*(1000000.0 / esp_clk_apb_freq()));
        }

        delay(1000);
    }

    // StaticJsonDocument<500> payload;&high_task_wakeup
    // Serial.print(SPD(measure[0], measure[1]));
    // Serial.print(SPD(measure[2], measure[3]));
    // Serial.println(SPD(measure[5], measure[4]));
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