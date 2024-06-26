#include "ledControlTask.hpp"

#include <Arduino.h>
#include <esp_log.h>

#include "deviceConfig.hpp"
#include "dmxTask.hpp"
#include "logTask.hpp"

static const char *TAG = "LedConTask";

// Static variables
TimerHandle_t LedControlTask::m_ledOnTimerHandle = NULL;
TimerHandle_t LedControlTask::m_ledOffTimerHandle = NULL;
TimerHandle_t LedControlTask::m_syncTimerHandle = NULL;
DmxMode<3> LedControlTask::recievedDmx;
strobeParameters LedControlTask::strobe;

LedControlTask::LedControlTask(uint32_t stackSize, uint8_t priority, uint8_t coreId)
    : Task("LedControlTask", stackSize, priority) {
    setCore(coreId);
    m_ledOnTimerHandle = NULL;
    m_ledOffTimerHandle = NULL;
    m_syncTimerHandle = NULL;
}

void LedControlTask::createTimers() {
    // Create timers
    m_ledOnTimerHandle = xTimerCreate("LedONTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOnTimerCallback);
    m_ledOffTimerHandle =
        xTimerCreate("LedOFFTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOffTimerCallback);
    m_syncTimerHandle = xTimerCreate("SyncTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, syncTimerCallback);

    if (m_ledOnTimerHandle == NULL || m_ledOffTimerHandle == NULL || m_syncTimerHandle == NULL) {
        if (m_ledOnTimerHandle == NULL) {
            ESP_LOGE(TAG, "Error creating the led ON timer");
        }
        if (m_ledOffTimerHandle == NULL) {
            ESP_LOGE(TAG, "Error creating the led OFF timer");
        }
        if (m_syncTimerHandle == NULL) {
            ESP_LOGE(TAG, "Error creating the one shot sync timer");
        }
        restartCPU(TAG);
    }
}

void LedControlTask::setup()
{
    // LED PWM configuration
    ledcSetup(DEVICECONF_PWM_CHANNEL, DEVICECONF_PWM_FREQUENCY, 8);
    ledcAttachPin(DEVICECONF_STROBE_LED_PIN, DEVICECONF_PWM_CHANNEL);
    // Default state of the LED
    ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
    strobe.isLedOn = false;
    
    createTimers();
}

void LedControlTask::run(void *data)
{
    char message[70];
    DmxMode<3> lastEffectParameters;
    int strobePeriod;
    if (xQueueReceive(dmxTask.getQueueHandle(), &recievedDmx, portMAX_DELAY) == pdTRUE) {
        strobePeriod = getStrobePeriod();
        strobe.dutyCycle = recievedDmx.data[0];
        strobe.lightOnTime = map(recievedDmx.data[2], STROBE_DURATION_MIN, STROBE_DURATION_MAX, 1, strobePeriod - 1);
        strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
        ledOnTimerCallback(NULL);
        lastEffectParameters = recievedDmx;
    }
    while (true) {
        if (xQueueReceive(dmxTask.getQueueHandle(), &recievedDmx, portMAX_DELAY) == pdTRUE) {
            strobePeriod = getStrobePeriod();
            strobe.dutyCycle = recievedDmx.data[0];
            if (recievedDmx.data[2] > STROBE_BLACKOUT_MAX && recievedDmx.data[2] < STROBE_LIGHT_FULL_ON_MIN) {
                strobe.lightOnTime =
                    map(recievedDmx.data[2], STROBE_DURATION_MIN, STROBE_DURATION_MAX, 1, strobePeriod - 1);
                strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
            }
            if (recievedDmx.data[1] != lastEffectParameters.data[1]) {
                refreshSyncTimer();
            }
            if (recievedDmx.data[0] != lastEffectParameters.data[0] && strobe.isLedOn) {
                setLightPwm();
            }
            lastEffectParameters = recievedDmx;
        } else {
            logTask.queueLog(TAG, "Error receiving ledEffectParameters from queue", LogLevel::ERROR);
        }
    }
}

void LedControlTask::ledOnTimerCallback(TimerHandle_t xTimer)
{
    setLightPwm();
    strobe.isLedOn = true;
    strobe.lastSwitch = millis();
    refreshLedOffTimer();
}

void LedControlTask::ledOffTimerCallback(TimerHandle_t xTimer)
{
    ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
    strobe.isLedOn = false;
    strobe.lastSwitch = millis();
    refreshLedOnTimer();
}

void LedControlTask::syncTimerCallback(TimerHandle_t xTimer)
{
    char message[100];
    logTask.queueLog(TAG, "SYNC timer expired", LogLevel::VERBOSE);
    switchOnOff();
    if (strobe.isLedOn) {
        if (xTimerChangePeriod(m_ledOffTimerHandle, pdMS_TO_TICKS(strobe.lightOnTime), portMAX_DELAY) != pdPASS) {
            sniprintf(message, sizeof(message), "Error changing ledOFF timer period, tried %d",
                      strobe.lightOnTime);
            logTask.queueLog(TAG, message, LogLevel::ERROR);
        } else {
            snprintf(message, sizeof(message), "Changed ledOFF timer period to %d", strobe.lightOnTime);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    } else {
        if (xTimerChangePeriod(m_ledOnTimerHandle, pdMS_TO_TICKS(strobe.lightOffTime), portMAX_DELAY) != pdPASS) {
            sniprintf(message, sizeof(message), "Error changing ledOFF timer period, tried %d",
                      strobe.lightOffTime);
            logTask.queueLog(TAG, message, LogLevel::ERROR);
        } else {
            snprintf(message, sizeof(message), "Changed the led ON timer period to %d", strobe.lightOffTime);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    }
}

int LedControlTask::getStrobePeriod()
{
    return map(recievedDmx.data[1], 0, 255, DEVICECONF_STROBE_PERIOD_MAX, DEVICECONF_STROBE_PERIOD_MIN);
}

void LedControlTask::setLightPwm()
{
    if (recievedDmx.data[2] >= STROBE_BLACKOUT_MIN && recievedDmx.data[2] <= STROBE_BLACKOUT_MAX) {
        ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
    } else {
        ledcWrite(DEVICECONF_PWM_CHANNEL, recievedDmx.data[0]);
    }
}

void LedControlTask::switchOnOff()
{
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

void LedControlTask::refreshLedOnTimer()
{
    xTimerChangePeriod(m_ledOnTimerHandle, pdMS_TO_TICKS(strobe.lightOffTime), portMAX_DELAY);
    if (m_ledOnTimerHandle == NULL) {
        logTask.queueLog(TAG, "Error refreshing the led ON timer", LogLevel::ERROR);
    }
}

void LedControlTask::refreshLedOffTimer()
{
    xTimerChangePeriod(m_ledOffTimerHandle, pdMS_TO_TICKS(strobe.lightOnTime), portMAX_DELAY);
    if (m_ledOffTimerHandle == NULL) {
        logTask.queueLog(TAG, "Error refreshing the led OFF timer", LogLevel::ERROR);
    }
}

void LedControlTask::refreshSyncTimer()
{
    // print the time from last switch
    char message[51];
    logTask.queueLog(TAG, "Refresh syncTimer called", LogLevel::VERBOSE);
    xTimerStop(m_ledOnTimerHandle, portMAX_DELAY);
    xTimerStop(m_ledOffTimerHandle, portMAX_DELAY);
    logTask.queueLog(TAG, "Stopped the led timers", LogLevel::VERBOSE);

    int timeFromLastSwitch = millis() - strobe.lastSwitch;

    if (strobe.isLedOn) {
        if ((strobe.lightOnTime - timeFromLastSwitch) < 1) {
            xTimerStop(m_syncTimerHandle, portMAX_DELAY);
            syncTimerCallback(NULL);
            logTask.queueLog(TAG, "Timer called artificially to turn LED OFF", LogLevel::VERBOSE);
        } else {
            xTimerChangePeriod(m_syncTimerHandle, pdMS_TO_TICKS(strobe.lightOnTime - timeFromLastSwitch),
                               portMAX_DELAY);
            snprintf(message, sizeof(message), "Refreshing syncTimer to turn ledOFF exp-%d",
                     strobe.lightOnTime - timeFromLastSwitch);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    } else {
        if ((strobe.lightOnTime - timeFromLastSwitch) < 1) {
            xTimerStop(m_syncTimerHandle, portMAX_DELAY);
            syncTimerCallback(NULL);
            logTask.queueLog(TAG, "Timer called artificially to turn LED ON", LogLevel::VERBOSE);
        } else {
            xTimerChangePeriod(m_syncTimerHandle, pdMS_TO_TICKS(strobe.lightOffTime - timeFromLastSwitch),
                               portMAX_DELAY);
            snprintf(message, sizeof(message), "Refreshing syncTimer to turn ledON exp-%d",
                     strobe.lightOffTime - timeFromLastSwitch);
            logTask.queueLog(TAG, message, LogLevel::VERBOSE);
        }
    }
    if (m_syncTimerHandle == NULL) {
        logTask.queueLog(TAG, "Error refreshing the one shot sync timer", LogLevel::ERROR);
    }
}



LedControlTask ledControlTask(2048, 1, APP_CPU_NUM);
