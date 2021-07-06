#include <Arduino.h>

#include <freertos/queue.h>

#define ECHO1 12
#define ECHO2 27
#define ECHO3 25

#define TRIGGER1 14
#define TRIGGER2 26
#define TRIGGER3 33

#define DEBUG 15

#define LENGTH .475 //[m]

#define SENSOR_N 2
#define FILTER_N 100

#define SPD(m1, m2) .5 * LENGTH * 1e6 * (FILTER_N / m1 - FILTER_N / m2)


void measureTask(void *parameter);
static void ICACHE_RAM_ATTR changeISR();

double m_carr1[FILTER_N] = {0};
double m_carr2[FILTER_N] = {0};

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
    // Enable serial communication
    Serial.begin(115200);

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
    xTaskCreate(measureTask, "measureTask", 10000, NULL, 1, NULL);
}

void loop()
{
    double measure[SENSOR_N][SENSOR_N];
    if (xQueueReceive(measure_queue, &measure, portMAX_DELAY) == pdPASS)
    {

        m_carr1[i] = measure[0][1];
        m_carr2[i] = measure[1][0];

        double s1 = 0, s2 = 0;

        for (size_t j = 0; j < FILTER_N; j++)
        {
            s1 += m_carr1[j];
            s2 += m_carr2[j];
        }
        Serial.printf("Measure : %e \n", .5 * LENGTH * 1e6 * (10. / s1 - 10. / s2));

        i = (i + 1) % FILTER_N;
    }
}

void measureTask(void *parameter)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 10 / portTICK_RATE_MS;

    xLastWakeTime = xTaskGetTickCount();

    unsigned long raw_elapsedCpuTime = 0;
    double measure[SENSOR_N][SENSOR_N];

    // unsigned int echo[]  = {ECHO1, ECHO2, ECHO3, ECHO2, ECHO1, ECHO3};
    // unsigned int trig1[] = {TRIGGER1, TRIGGER1, TRIGGER2, TRIGGER2, TRIGGER3, TRIGGER3};
    // unsigned int trig2[] = {TRIGGER2, TRIGGER2, TRIGGER3, TRIGGER3, TRIGGER1, TRIGGER1};

    unsigned int echo[] = {ECHO1, ECHO2};
    unsigned int trig1[] = {TRIGGER1, TRIGGER1};
    unsigned int trig2[] = {TRIGGER2, TRIGGER2};

    // unsigned int i[] = {0, 1, 1, 2, 2, 0};
    // unsigned int j[] = {1, 0, 2, 1, 0, 2};

    unsigned int i[] = {0, 1};
    unsigned int j[] = {1, 0};

    unsigned int idx = 0;

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        attachInterrupt(digitalPinToInterrupt(echo[idx]), changeISR, CHANGE);

        digitalWrite(trig1[idx], HIGH);
        digitalWrite(trig2[idx], HIGH);
        delayMicroseconds(20);
        digitalWrite(trig1[idx], LOW);
        digitalWrite(trig2[idx], LOW);
        // Serial.printf("Pulse %i \n", idx);

        if (xQueueReceive(raw_measure_queue, &raw_elapsedCpuTime, portMAX_DELAY) == pdPASS)
        {
            cpuTimeRising = raw_elapsedCpuTime;
        }
        if (xQueueReceive(raw_measure_queue, &raw_elapsedCpuTime, portMAX_DELAY) == pdPASS)
        {
            elapsedCpuTime = (raw_elapsedCpuTime - cpuTimeRising);
            detachInterrupt(digitalPinToInterrupt(echo[idx]));
            // Serial.printf("%d, %e \n", idx, (double) elapsedCpuTime/ (1000 * cpu_freq));
            measure[i[idx]][j[idx]] = elapsedCpuTime / cpu_freq; // [us]
        }

        if (idx == 1)
            xQueueSendToFront(measure_queue, (void *)&measure, 100 / portTICK_RATE_MS);

        idx = (idx + 1) % 2;
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