#include <Arduino.h>
#include <WrapperFreeRTOS.h>  // FreeRTOS C++ wrapper
#include <esp_dmx.h>
#include <esp_log.h>

#include "deviceConfig.hpp"
#include "logTask.hpp"

static const char *TAG = "main.cpp";

// Intervals for different effects
// channel 1: intensity
const uint8_t intensityMin = 0;    // Lowest intensity
const uint8_t intensityMax = 255;  // Highest intensity
// channel 2: strobe speed
const uint8_t strobeSpeedMin = 0;    // Slowest speed
const uint8_t strobeSpeedMax = 255;  // Fastest speed
// channel 3: light duration
const uint8_t blackoutMin = 0;       // No light
const uint8_t blackoutMax = 4;       // No light
const uint8_t durationMin = 5;       // Shortest duration
const uint8_t durationMax = 250;     // Longest duration
const uint8_t lightFullOnMin = 251;  // Full on
const uint8_t lightFullOnMax = 255;  // Full on

LogTask logTask(2048, 1, PRO_CPU_NUM);

struct ledEffectParameters {
    uint8_t intensity;      // CH1
    uint8_t strobeSpeed;    // CH2
    uint8_t lightDuration;  // CH3

    // This is needed for the == operator to work
    bool operator==(const ledEffectParameters &other) const {
        return intensity == other.intensity && strobeSpeed == other.strobeSpeed &&
               (lightDuration == other.lightDuration ||
                (lightDuration >= blackoutMin && lightDuration <= blackoutMax &&
                 other.lightDuration >= blackoutMin && other.lightDuration <= blackoutMax) ||
                (lightDuration >= lightFullOnMin && lightDuration <= lightFullOnMax &&
                 other.lightDuration >= lightFullOnMin && other.lightDuration <= lightFullOnMax));
    }

    // This is needed for the != operator to work
    bool operator!=(const ledEffectParameters &other) const { return !(*this == other); }
};

struct strobeParameters {
    uint8_t dutyCycle;
    int lightOnTime;
    int lightOffTime;
    bool isLedOn;
    unsigned long lastSwitch;
};

ledEffectParameters effectParameters;
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
    if (effectParameters.lightDuration >= blackoutMin && effectParameters.lightDuration <= blackoutMax) {
        ledcWrite(DEVICECONF_PWM_CHANNEL, 0);
    } else {
        ledcWrite(DEVICECONF_PWM_CHANNEL, effectParameters.intensity);
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
    return map(effectParameters.strobeSpeed, strobeSpeedMin, strobeSpeedMax, DEVICECONF_STROBE_PERIOD_MAX,
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
    char message[70];
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
    ledEffectParameters lastEffectParameters;
    int strobePeriod;
    if (xQueueReceive(ledEffectParametersQueue, &effectParameters, portMAX_DELAY) == pdTRUE) {
        strobePeriod = getStrobePeriod();
        strobe.dutyCycle = effectParameters.intensity;
        strobe.lightOnTime =
            map(effectParameters.lightDuration, durationMin, durationMax, 1, strobePeriod - 1);
        strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
        ledOnTimerCallback(NULL);
        lastEffectParameters = effectParameters;
    }
    while (true) {
        if (xQueueReceive(ledEffectParametersQueue, &effectParameters, portMAX_DELAY) == pdTRUE) {
            strobePeriod = getStrobePeriod();
            strobe.dutyCycle = effectParameters.intensity;
            if (effectParameters.lightDuration > blackoutMax &&
                effectParameters.lightDuration < lightFullOnMin) {
                strobe.lightOnTime =
                    map(effectParameters.lightDuration, durationMin, durationMax, 1, strobePeriod - 1);
                strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
            }
            if (effectParameters.strobeSpeed != lastEffectParameters.strobeSpeed) {
                refreshSyncTimer();
            }
            if (effectParameters.intensity != lastEffectParameters.intensity && strobe.isLedOn) {
                setLightPwm();
            }
            lastEffectParameters = effectParameters;
        } else {
            logTask.queueLog(TAG, "Error receiving ledEffectParameters from queue", LogLevel::ERROR);
        }
    }
}

void dmxRecieveTask(void *pvParameters) {
    bool dmxIsConnected = false;
    static dmx_packet_t packet;     // the packet structure that will be filled with the data
                                    // from the DMX bus
    uint8_t data[DMX_PACKET_SIZE];  // the data read from the DMX bus
    char message[70];               // Buffer for formatting the message for the printing
    ledEffectParameters recievedParameters;
    ledEffectParameters lastSentParameters;

    while (true) {
        if (dmx_receive(DEVICECONF_DMX_PORT, &packet, DMX_TIMEOUT_TICK)) {
            // We should check to make sure that there weren't any DMX errors.
            if (!packet.err) {
                // If this is the first DMX data we've received, lets log it!
                if (!dmxIsConnected) {
                    logTask.queueLog(TAG, "DMX connected!", LogLevel::VERBOSE);
                    dmxIsConnected = true;
                }
                dmx_read(DEVICECONF_DMX_PORT, data, packet.size);
                recievedParameters.intensity = data[1];      // CH1
                recievedParameters.strobeSpeed = data[2];    // CH2
                recievedParameters.lightDuration = data[3];  // CH3

                if (recievedParameters != lastSentParameters) {
                    // Send the parameters to the led control task
                    if (xQueueSend(ledEffectParametersQueue, &recievedParameters, portMAX_DELAY) == pdPASS) {
                        lastSentParameters = recievedParameters;
                    } else {
                        logTask.queueLog(TAG, "Error sending ledEffectParameters to queue", LogLevel::ERROR);
                    }
                }
            } else {
                // A DMX error! This can happen when you connect or disconnect your DMX devices. If you are
                // consistently getting DMX errors, something may be wrong.
                logTask.queueLog(TAG, "DMX error occured!", LogLevel::ERROR);
            }
        } else  // if (dmxIsConnected)
        {
            // If DMX times out after having been connected, it likely means that the DMX
            // cable was unplugged.
            logTask.queueLog(TAG, "DMX disconnected!", LogLevel::INFO);
            dmxIsConnected = false;
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

    // First, use the default DMX configuration...
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    // ...install the DMX driver...
    dmx_driver_install(DEVICECONF_DMX_PORT, &config, DMX_INTR_FLAGS_DEFAULT);
    // ...and then set the communication pins
    dmx_set_pin(DEVICECONF_DMX_PORT, DEVICECONF_DMX_TX_PIN, DEVICECONF_DMX_RX_PIN, DEVICECONF_DMX_RTS_PIN);

    // Logging task
    logTask.setup();
    logTask.start();

    ledEffectParametersQueue = xQueueCreate(10, sizeof(ledEffectParameters));
    if (ledEffectParametersQueue == NULL) {
        ESP_LOGE(TAG, "Error creating the ledEffectParameters queue");
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // wait for 1 second
        ESP.restart();
    }

    // Create tasks
    xTaskCreatePinnedToCore(ledControlTask, "LEDControlTask", 2048, NULL, 1, &ledControlHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(dmxRecieveTask, "DMXRecieveTask", 4096, NULL, 2, &dmxReceiveHandle, PRO_CPU_NUM);

    // Create timers
    oneShotSyncTimer =
        xTimerCreate("OneShtSyncTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, oneShotSyncTimerCallback);
    ledOffTimer = xTimerCreate("LedOFFTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOffTimerCallback);
    ledOnTimer = xTimerCreate("LedONTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOnTimerCallback);
    if (oneShotSyncTimer == NULL || ledOffTimer == NULL || ledOnTimer == NULL) {
        logTask.queueLog(TAG, "Error creating the timers", LogLevel::VERBOSE);
        vTaskDelete(NULL);  // this should end the program since this is the only task running
    }
    //*/
    // fan
    pinMode(DEVICECONF_FAN_PIN, OUTPUT);
    digitalWrite(DEVICECONF_FAN_PIN, HIGH);

    logTask.queueLog(TAG, "Setup done", LogLevel::INFO);

    // Delete "setup and loop" task
    vTaskDelete(NULL);
}

void loop() {}  // this should never be executed