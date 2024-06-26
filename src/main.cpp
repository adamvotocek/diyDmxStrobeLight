#include <Arduino.h>
#include <WrapperFreeRTOS.h>  // FreeRTOS C++ wrapper
#include <esp_dmx.h>
#include <esp_log.h>

#include "deviceConfig.hpp"
#include "dmxTask.hpp"
#include "logTask.hpp"

static const char *TAG = "main.cpp";

// Intervals for different effects
// channel 3: light duration
const uint8_t blackoutMin = 0;       // No light
const uint8_t blackoutMax = 4;       // No light
const uint8_t durationMin = 5;       // Shortest duration
const uint8_t durationMax = 250;     // Longest duration
const uint8_t lightFullOnMin = 251;  // Full on
const uint8_t lightFullOnMax = 255;  // Full on

struct strobeParameters {
    uint8_t dutyCycle;
    int lightOnTime;
    int lightOffTime;
    bool isLedOn;
    unsigned long lastSwitch;
};

DmxMode<3> effectParameters;
strobeParameters strobe;

// FreeRTOS stuff
// tasks
TaskHandle_t ledControlHandle = NULL;
TaskHandle_t dmxReceiveHandle = NULL;
// TaskHandle_t serialPrintTaskHandle = NULL;
// queues
// QueueHandle_t serialPrintQueue;
QueueHandle_t ledEffectParametersQueue;
// timers
TimerHandle_t ledOnTimer = NULL;
TimerHandle_t ledOffTimer = NULL;
TimerHandle_t oneShotSyncTimer = NULL;

// Function prototypes
void ledOnTimerCallback(TimerHandle_t xTimer);
void ledOffTimerCallback(TimerHandle_t xTimer);

void setLightPwm() {
    if (effectParameters.data[2] >= blackoutMin && effectParameters.data[2] <= blackoutMax) {
        ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
    } else {
        ledcWrite(DEVICECONF_PWM_CHANNEL, effectParameters.data[0]);
    }
}

void switchOnOff() {
    if (strobe.isLedOn) {
        ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
        strobe.isLedOn = false;
        strobe.lastSwitch = millis();
        logTask.queueLog(TAG, "LED OFF", LogLevel::VERBOSE);
    } else {
        setLightPwm();
        strobe.isLedOn = true;
        strobe.lastSwitch = millis();
        logTask.queueLog(TAG, "LED ON", LogLevel::VERBOSE);
    }
}

int getStrobePeriod() {
    return map(effectParameters.data[1], 0, 255, DEVICECONF_STROBE_PERIOD_MAX,
               DEVICECONF_STROBE_PERIOD_MIN);
}

void refreshLedOnTimer() {
    xTimerChangePeriod(ledOnTimer, pdMS_TO_TICKS(strobe.lightOffTime), portMAX_DELAY);
    if (ledOnTimer == NULL) {
        logTask.queueLog(TAG, "Error refreshing the led ON timer", LogLevel::ERROR);
    }
}

void refreshLedOffTimer() {
    xTimerChangePeriod(ledOffTimer, pdMS_TO_TICKS(strobe.lightOnTime), portMAX_DELAY);
    if (ledOffTimer == NULL) {
        logTask.queueLog(TAG, "Error refreshing the led OFF timer", LogLevel::ERROR);
    }
}

void ledOnTimerCallback(TimerHandle_t xTimer) {
    setLightPwm();
    strobe.isLedOn = true;
    strobe.lastSwitch = millis();
    refreshLedOffTimer();
}

void ledOffTimerCallback(TimerHandle_t xTimer) {
    ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
    strobe.isLedOn = false;
    strobe.lastSwitch = millis();
    refreshLedOnTimer();
}

void oneShotSyncTimerCallback(TimerHandle_t xTimer) {
    char message[100];
    logTask.queueLog(TAG, "SYNC timer expired", LogLevel::VERBOSE);
    switchOnOff();
    if (strobe.isLedOn) {
        if (xTimerChangePeriod(ledOffTimer, pdMS_TO_TICKS(strobe.lightOnTime), portMAX_DELAY) != pdPASS) {
            sniprintf(message, sizeof(message), "Error changing ledOFF timer period, tried %d",
                      strobe.lightOnTime);
            logTask.queueLog(TAG, message, LogLevel::ERROR);
        } else {
            snprintf(message, sizeof(message), "Changed ledOFF timer period to %d", strobe.lightOnTime);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    } else {
        if (xTimerChangePeriod(ledOnTimer, pdMS_TO_TICKS(strobe.lightOffTime), portMAX_DELAY) != pdPASS) {
            sniprintf(message, sizeof(message), "Error changing ledOFF timer period, tried %d",
                      strobe.lightOffTime);
            logTask.queueLog(TAG, message, LogLevel::ERROR);
        } else {
            snprintf(message, sizeof(message), "Changed the led ON timer period to %d", strobe.lightOffTime);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    }
}

void refreshSyncTimer() {
    // print the time from last switch
    char message[51];
    logTask.queueLog(TAG, "Refresh syncTimer called", LogLevel::VERBOSE);
    xTimerStop(ledOnTimer, portMAX_DELAY);
    xTimerStop(ledOffTimer, portMAX_DELAY);
    logTask.queueLog(TAG, "Stopped the led timers", LogLevel::VERBOSE);

    int timeFromLastSwitch = millis() - strobe.lastSwitch;

    if (strobe.isLedOn) {
        if ((strobe.lightOnTime - timeFromLastSwitch) < 1) {
            xTimerStop(oneShotSyncTimer, portMAX_DELAY);
            oneShotSyncTimerCallback(NULL);
            logTask.queueLog(TAG, "Timer called artificially to turn LED OFF", LogLevel::VERBOSE);
        } else {
            xTimerChangePeriod(oneShotSyncTimer, pdMS_TO_TICKS(strobe.lightOnTime - timeFromLastSwitch),
                               portMAX_DELAY);
            snprintf(message, sizeof(message), "Refreshing syncTimer to turn ledOFF exp-%d",
                     strobe.lightOnTime - timeFromLastSwitch);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    } else {
        if ((strobe.lightOnTime - timeFromLastSwitch) < 1) {
            xTimerStop(oneShotSyncTimer, portMAX_DELAY);
            oneShotSyncTimerCallback(NULL);
            logTask.queueLog(TAG, "Timer called artificially to turn LED ON", LogLevel::VERBOSE);
        } else {
            xTimerChangePeriod(oneShotSyncTimer, pdMS_TO_TICKS(strobe.lightOffTime - timeFromLastSwitch),
                               portMAX_DELAY);
            snprintf(message, sizeof(message), "Refreshing syncTimer to turn ledON exp-%d",
                     strobe.lightOffTime - timeFromLastSwitch);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    }
    if (oneShotSyncTimer == NULL) {
        logTask.queueLog(TAG, "Error refreshing the one shot sync timer", LogLevel::ERROR);
    }
}

void ledControlTask(void *pvParameters) {
    char message[70];
    DmxMode<3> lastEffectParameters;
    int strobePeriod;
    if (xQueueReceive(dmxTask.getQueueHandle(), &effectParameters, portMAX_DELAY) == pdTRUE) {
        strobePeriod = getStrobePeriod();
        strobe.dutyCycle = effectParameters.data[0];
        strobe.lightOnTime = map(effectParameters.data[2], durationMin, durationMax, 1, strobePeriod - 1);
        strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
        ledOnTimerCallback(NULL);
        lastEffectParameters = effectParameters;
    }
    while (true) {
        if (xQueueReceive(dmxTask.getQueueHandle(), &effectParameters, portMAX_DELAY) == pdTRUE) {
            strobePeriod = getStrobePeriod();
            strobe.dutyCycle = effectParameters.data[0];
            if (effectParameters.data[2] > blackoutMax && effectParameters.data[2] < lightFullOnMin) {
                strobe.lightOnTime =
                    map(effectParameters.data[2], durationMin, durationMax, 1, strobePeriod - 1);
                strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
            }
            if (effectParameters.data[1] != lastEffectParameters.data[1]) {
                refreshSyncTimer();
            }
            if (effectParameters.data[0] != lastEffectParameters.data[0] && strobe.isLedOn) {
                setLightPwm();
            }
            lastEffectParameters = effectParameters;
        } else {
            logTask.queueLog(TAG, "Error receiving ledEffectParameters from queue", LogLevel::ERROR);
        }
    }
}

void setup() {
    // LED PWM configuration
    ledcSetup(DEVICECONF_PWM_CHANNEL, DEVICECONF_PWM_FREQUENCY, 8);
    ledcAttachPin(DEVICECONF_STROBE_LED_PIN, DEVICECONF_PWM_CHANNEL);

    // Default state of the LED
    ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
    strobe.isLedOn = false;
    
    // Turn on the fan
    pinMode(DEVICECONF_FAN_PIN, OUTPUT);
    digitalWrite(DEVICECONF_FAN_PIN, HIGH);

    // Setup tasks
    logTask.setup();
    dmxTask.setup();

    // Start tasks
    logTask.start();
    dmxTask.start();

    // Create tasks
    xTaskCreatePinnedToCore(ledControlTask, "LEDControlTask", 2048, NULL, 1, &ledControlHandle, APP_CPU_NUM);

    // Create timers
    oneShotSyncTimer =
        xTimerCreate("OneShtSyncTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, oneShotSyncTimerCallback);
    ledOffTimer = xTimerCreate("LedOFFTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOffTimerCallback);
    ledOnTimer = xTimerCreate("LedONTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOnTimerCallback);
    if (oneShotSyncTimer == NULL || ledOffTimer == NULL || ledOnTimer == NULL) {
        logTask.queueLog(TAG, "Error creating the timers", LogLevel::ERROR);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP.restart();
    }


    logTask.queueLog(TAG, "Setup done", LogLevel::DEBUG);
    vTaskDelete(NULL); // Delete "setup and loop" task
}

void loop() {}  // this should never be executed