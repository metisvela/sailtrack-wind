#include <Arduino.h>

#include <ArduinoJson.h>

#include <freertos/queue.h>
#include <SailtrackModule.h>

#define ECHO1 35
#define ECHO2 33
#define ECHO3 34

#define TRIGGER1 18
#define TRIGGER2 19
#define TRIGGER3 23

#define DEBUG 15

#define LENGTH .475 //[m]

#define SENSOR_N 3
#define FILTER_N 100

#define SPD(m1, m2) .5 * LENGTH * 1e6 * (FILTER_N / m1 - FILTER_N / m2)
SailtrackModule STM;

void measureTask(void *parameter);
static void ICACHE_RAM_ATTR changeISR();

double m_carr1[FILTER_N] = {0};
double m_carr2[FILTER_N] = {0};
double m_carr3[FILTER_N] = {0};
double m_carr4[FILTER_N] = {0};
double m_carr5[FILTER_N] = {0};
double m_carr6[FILTER_N] = {0};


int i = 0;

int cpu_freq = 0;

// ISR variables
volatile unsigned long cpuTimeRising = 0;
volatile unsigned long elapsedCpuTime = 0;
volatile unsigned long cpuTimePlaceholder = 0;
volatile unsigned long ToF = 0;
volatile bool evalFlag = false;

QueueHandle_t raw_measure_queue, measure_queue;





void setup()
{
    STM.begin("wind", IPAddress(192, 168, 42, 104));

    // Get cpu frequency
    cpu_freq = ESP.getCpuFreqMHz();
    Serial.println(cpu_freq);

    // Setp PINs and interrupts
    pinMode(TRIGGER1, OUTPUT);
    pinMode(TRIGGER2, OUTPUT);
    pinMode(TRIGGER3, OUTPUT);
    pinMode(ECHO1, INPUT);
    pinMode(ECHO2, INPUT);
    pinMode(ECHO3, INPUT);

    pinMode(DEBUG, OUTPUT);

    // Create queues
    raw_measure_queue = xQueueCreate(1, sizeof(unsigned long));
    measure_queue = xQueueCreate(1, sizeof(double[3][3]));

    // Create tasks
    xTaskCreate(measureTask, "measureTask", TASK_BIG_STACK_SIZE, NULL, TASK_HIGH_PRIORITY, NULL);
}

void loop()
{
    double measure[SENSOR_N*2];
    DynamicJsonDocument payload(500);
    if (xQueueReceive(measure_queue, &measure, portMAX_DELAY) == pdPASS)
    {
        m_carr1[i] = measure[0];
        m_carr2[i] = measure[1];
        m_carr3[i] = measure[2];
        m_carr4[i] = measure[3];
        m_carr5[i] = measure[4];
        m_carr6[i] = measure[5];     

        double s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0;

        for (size_t j = 0; j < FILTER_N; j++)
        {
            s1 += m_carr1[j];
            s2 += m_carr2[j];
            s3 += m_carr3[j];
            s4 += m_carr4[j];
            s5 += m_carr5[j];
            s6 += m_carr6[j];
        }

        payload["A"] = SPD(s1, s2);
        payload["B"] = SPD(s3, s4);
        payload["C"] = SPD(s6, s5);
      
        STM.publish("sensor/wind0", payload);
        i = (i + 1) % FILTER_N;
    }
}

void measureTask(void *parameter)
{
    //fase di setup
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10 / portTICK_RATE_MS;

    xLastWakeTime = xTaskGetTickCount();

    unsigned long raw_elapsedCpuTime = 0;
    double measure[SENSOR_N*2];

    unsigned int echo[]  = {ECHO1, ECHO2, ECHO3, ECHO2, ECHO1, ECHO3};
    unsigned int trig1[] = {TRIGGER1, TRIGGER1, TRIGGER2, TRIGGER2, TRIGGER3, TRIGGER3};
    unsigned int trig2[] = {TRIGGER2, TRIGGER2, TRIGGER3, TRIGGER3, TRIGGER1, TRIGGER1};

    
    unsigned int idx = 0;
    //loop
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        attachInterrupt(digitalPinToInterrupt(echo[idx]), changeISR, CHANGE);

        digitalWrite(trig1[idx], HIGH);
        digitalWrite(trig2[idx], HIGH);
        delayMicroseconds(20);
        digitalWrite(trig1[idx], LOW);
        digitalWrite(trig2[idx], LOW);

        if (xQueueReceive(raw_measure_queue, &raw_elapsedCpuTime, portMAX_DELAY) == pdPASS)
        {
            cpuTimeRising = raw_elapsedCpuTime;
        }
        if (xQueueReceive(raw_measure_queue, &raw_elapsedCpuTime, portMAX_DELAY) == pdPASS)
        {
            elapsedCpuTime = (raw_elapsedCpuTime - cpuTimeRising);
            detachInterrupt(digitalPinToInterrupt(echo[idx]));
            measure[idx] = elapsedCpuTime / cpu_freq; // [us]
        }

        if (idx == SENSOR_N*2 - 1)
            xQueueSendToFront(measure_queue, (void *)&measure, 100 / portTICK_RATE_MS);

        idx = (idx + 1) % (SENSOR_N*2);
        taskYIELD();
    }
}

static void ICACHE_RAM_ATTR changeISR()
{
    //gets cpu timing when echo pin changes logic state
    cpuTimePlaceholder = ESP.getCycleCount(); //get cpu time before evaluating if statement
    // digitalWrite(DEBUG, !digitalRead(DEBUG));
    xQueueSendToBackFromISR(raw_measure_queue, (void *)&cpuTimePlaceholder, pdFALSE);
}