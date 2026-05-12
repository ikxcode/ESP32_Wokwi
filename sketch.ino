#include <Arduino.h>
#include <DHT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define DHT_PIN 4
#define DHT_TYPE DHT22

#define RELAY_PIN 18
#define BUZZER_PIN 19
#define STATUS_LED 2

#define EMERGENCY_PIN 27
#define OVERRIDE_PIN 26

#define ADC_PIN 34

#define TEMP_WARNING 40.0f
#define TEMP_CRITICAL 60.0f

#define FILTER_SIZE 10

DHT dht(DHT_PIN, DHT_TYPE);

SemaphoreHandle_t xSystemMutex;

volatile bool emergencyFlag = false;
volatile bool overrideFlag = false;

volatile uint32_t lastEmergencyInterrupt = 0;
volatile uint32_t lastOverrideInterrupt = 0;

typedef struct
{
    float dhtTemperature;
    float adcTemperature;
    float filteredTemperature;

    uint16_t adcRaw;

    bool emergencyStop;
    bool relayEnabled;
    bool sensorFault;
    bool overheat;
    bool warning;

} SystemData_t;

SystemData_t gSystemData;

void SensorTask(void *pvParameters);
void ControlTask(void *pvParameters);
void SafetyTask(void *pvParameters);
void LoggingTask(void *pvParameters);

void IRAM_ATTR emergencyISR(void);
void IRAM_ATTR overrideISR(void);

float movingAverage(float sample);
float readADCTemperature(void);

float movingAverage(float sample)
{
    static float buffer[FILTER_SIZE] = {0};
    static int index = 0;
    static float sum = 0;

    sum -= buffer[index];
    buffer[index] = sample;
    sum += sample;

    index = (index + 1) % FILTER_SIZE;

    return sum / FILTER_SIZE;
}

float readADCTemperature(void)
{
    uint16_t adcRaw = analogRead(ADC_PIN);

    float voltage = (adcRaw / 4095.0f) * 3.3f;

    float temperature = (voltage - 0.5f) * 100.0f;

    if (xSemaphoreTake(xSystemMutex, pdMS_TO_TICKS(50)))
    {
        gSystemData.adcRaw = adcRaw;
        xSemaphoreGive(xSystemMutex);
    }

    return temperature;
}

void IRAM_ATTR emergencyISR(void)
{
    uint32_t currentTime = millis();

    if ((currentTime - lastEmergencyInterrupt) > 200)
    {
        emergencyFlag = true;
        lastEmergencyInterrupt = currentTime;
    }
}

void IRAM_ATTR overrideISR(void)
{
    uint32_t currentTime = millis();

    if ((currentTime - lastOverrideInterrupt) > 200)
    {
        overrideFlag = true;
        lastOverrideInterrupt = currentTime;
    }
}

void SensorTask(void *pvParameters)
{
    dht.begin();

    while (1)
    {
        float dhtTemp = dht.readTemperature();
        float adcTemp = readADCTemperature();

        float filtered = movingAverage(adcTemp);

        bool fault = false;

        if (isnan(dhtTemp))
        {
            fault = true;
        }

        if (adcTemp < -40 || adcTemp > 125)
        {
            fault = true;
        }

        if (xSemaphoreTake(xSystemMutex, pdMS_TO_TICKS(50)))
        {
            gSystemData.dhtTemperature = dhtTemp;
            gSystemData.adcTemperature = adcTemp;
            gSystemData.filteredTemperature = filtered;
            gSystemData.sensorFault = fault;

            xSemaphoreGive(xSystemMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ControlTask(void *pvParameters)
{
    while (1)
    {
        float temp;
        bool emergency;
        bool fault;

        if (xSemaphoreTake(xSystemMutex, pdMS_TO_TICKS(50)))
        {
            temp = gSystemData.filteredTemperature;
            emergency = gSystemData.emergencyStop;
            fault = gSystemData.sensorFault;

            xSemaphoreGive(xSystemMutex);
        }

        if (emergency || fault)
        {
            digitalWrite(RELAY_PIN, LOW);
            digitalWrite(STATUS_LED, LOW);
        }
        else
        {
            if (temp > TEMP_WARNING)
            {
                digitalWrite(RELAY_PIN, HIGH);
                digitalWrite(STATUS_LED, HIGH);
            }
            else
            {
                digitalWrite(RELAY_PIN, LOW);
                digitalWrite(STATUS_LED, LOW);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void SafetyTask(void *pvParameters)
{
    while (1)
    {
        if (emergencyFlag)
        {
            if (xSemaphoreTake(xSystemMutex, pdMS_TO_TICKS(50)))
            {
                gSystemData.emergencyStop = true;
                xSemaphoreGive(xSystemMutex);
            }

            digitalWrite(RELAY_PIN, LOW);
            digitalWrite(BUZZER_PIN, HIGH);

            emergencyFlag = false;
        }

        if (overrideFlag)
        {
            if (xSemaphoreTake(xSystemMutex, pdMS_TO_TICKS(50)))
            {
                gSystemData.emergencyStop = false;
                gSystemData.sensorFault = false;
                gSystemData.overheat = false;

                xSemaphoreGive(xSystemMutex);
            }

            digitalWrite(BUZZER_PIN, LOW);

            overrideFlag = false;
        }

        if (xSemaphoreTake(xSystemMutex, pdMS_TO_TICKS(50)))
        {
            if (gSystemData.filteredTemperature > TEMP_CRITICAL)
            {
                gSystemData.overheat = true;
            }

            if (gSystemData.overheat)
            {
                digitalWrite(RELAY_PIN, LOW);
                digitalWrite(BUZZER_PIN, HIGH);
            }

            xSemaphoreGive(xSystemMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void LoggingTask(void *pvParameters)
{
    while (1)
    {
        if (xSemaphoreTake(xSystemMutex, pdMS_TO_TICKS(50)))
        {
            Serial.printf(
                "DHT: %.2f C | ADC: %.2f C | Filtered: %.2f C | Raw: %d | Emergency: %d | Fault: %d | Overheat: %d\n",
                gSystemData.dhtTemperature,
                gSystemData.adcTemperature,
                gSystemData.filteredTemperature,
                gSystemData.adcRaw,
                gSystemData.emergencyStop,
                gSystemData.sensorFault,
                gSystemData.overheat
            );

            xSemaphoreGive(xSystemMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void setup()
{
    Serial.begin(115200);

    pinMode(RELAY_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(STATUS_LED, OUTPUT);

    pinMode(EMERGENCY_PIN, INPUT_PULLUP);
    pinMode(OVERRIDE_PIN, INPUT_PULLUP);

    pinMode(ADC_PIN, INPUT);

    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    analogReadResolution(12);

    xSystemMutex = xSemaphoreCreateMutex();

    attachInterrupt(
        digitalPinToInterrupt(EMERGENCY_PIN),
        emergencyISR,
        FALLING
    );

    attachInterrupt(
        digitalPinToInterrupt(OVERRIDE_PIN),
        overrideISR,
        FALLING
    );

    xTaskCreatePinnedToCore(
        SensorTask,
        "SensorTask",
        4096,
        NULL,
        4,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        ControlTask,
        "ControlTask",
        4096,
        NULL,
        3,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        SafetyTask,
        "SafetyTask",
        4096,
        NULL,
        5,
        NULL,
        1
    );

    xTaskCreatePinnedToCore(
        LoggingTask,
        "LoggingTask",
        4096,
        NULL,
        2,
        NULL,
        1
    );
}

void loop()
{
}