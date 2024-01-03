#include <Arduino.h>
#include <esp_dmx.h>

// led pwm configuration
const int pwmFrequency = 5000; // Hz
const uint8_t pwmChannel = 0;
const uint8_t ledPin = 13;    // the pin that the mosfet driver is connected to
const uint8_t resolution = 8; // max resolution of the DMX512 protocol values

// dmx configuration
const uint8_t tx_pin = 17;  // TXD_2 not used, i dont know if it is needed
const uint8_t rx_pin = 16;  // RXD_2
const uint8_t rts_pin = 21; // not used because the module does it automatically, and i dont know if it is needed
const dmx_port_t dmxPort = DMX_NUM_2;

// effect configuration
const int strobePeriodMin = 30;  // Fastest strobe
const int strobePeriodMax = 2000; // Slowest strobe

// Intervals for different effects
// channel 1: intensity
const uint8_t intensityMin = 0;   // Lowest intensity
const uint8_t intensityMax = 255; // Highest intensity
// channel 2: strobe speed
const uint8_t strobeSpeedMin = 0;   // Slowest speed
const uint8_t strobeSpeedMax = 255; // Fastest speed
// channel 3: light duration
const uint8_t blackoutMin = 0;      // No light
const uint8_t blackoutMax = 4;      // No light
const uint8_t durationMin = 5;      // Shortest duration
const uint8_t durationMax = 250;    // Longest duration
const uint8_t lightFullOnMin = 251; // Full on
const uint8_t lightFullOnMax = 255; // Full on

struct ledEffectParameters
{
    uint8_t intensity;     // CH1
    uint8_t strobeSpeed;   // CH2
    uint8_t lightDuration; // CH3

    // This is needed for the == operator to work
    bool operator==(const ledEffectParameters &other) const
    {
        return (intensity == other.intensity &&
                strobeSpeed == other.strobeSpeed &&

                ((lightDuration >= blackoutMin && lightDuration <= blackoutMax &&
                  other.lightDuration >= blackoutMin && other.lightDuration <= blackoutMax) ||
                 (lightDuration >= lightFullOnMin && lightDuration <= lightFullOnMax &&
                  other.lightDuration >= lightFullOnMin && other.lightDuration <= lightFullOnMax) ||
                 lightDuration == other.lightDuration));
    }

    // This is needed for the != operator to work
    bool operator!=(const ledEffectParameters &other) const
    {
        return !(*this == other);
    }
};

struct strobeParameters
{
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
TaskHandle_t serialPrintTaskHandle = NULL;
// queues
QueueHandle_t serialPrintQueue;
QueueHandle_t ledEffectParametersQueue;
// timers
TimerHandle_t ledOnTimer = NULL;
TimerHandle_t ledOffTimer = NULL;
TimerHandle_t oneShotSyncTimer = NULL;

// Function prototypes
void ledOnTimerCallback(TimerHandle_t xTimer);
void ledOffTimerCallback(TimerHandle_t xTimer);

void sendStringToPrintQueue(const char *string)
{
    char *message = (char *)malloc((strlen(string) + 1) * sizeof(char)); // Allocate memory for the string
    strcpy(message, string);                                             // Copy the string into the allocated memory

    if (xQueueSend(serialPrintQueue, &message, portMAX_DELAY) != pdPASS) // Send the pointer to the queue
    {
        // If xQueueSend failed, free the memory to prevent a leak
        free(message);
    }
}

void setLightPwm()
{
    if (effectParameters.lightDuration >= blackoutMin && effectParameters.lightDuration <= blackoutMax)
    {
        ledcWrite(pwmChannel, 0);
        // strobe.isLedOn = false;
    }
    else
    {
        ledcWrite(pwmChannel, effectParameters.intensity);
        // strobe.isLedOn = true;
    }
}

void switchOnOff()
{
    if (strobe.isLedOn)
    {
        ledcWrite(pwmChannel, 0);
        strobe.isLedOn = false;
        strobe.lastSwitch = millis();
        sendStringToPrintQueue("LED OFF");
    }
    else
    {
        setLightPwm();
        strobe.isLedOn = true;
        strobe.lastSwitch = millis();
        sendStringToPrintQueue("LED ON");
    }
}

int getStrobePeriod()
{
    return map(effectParameters.strobeSpeed, strobeSpeedMin, strobeSpeedMax, strobePeriodMax, strobePeriodMin);
}

void refreshLedOnTimer()
{
    xTimerChangePeriod(ledOnTimer, pdMS_TO_TICKS(strobe.lightOffTime), portMAX_DELAY);
    if (ledOnTimer == NULL)
    {
        sendStringToPrintQueue("Error refreshing the led ON timer\n");
    }
}

void refreshLedOffTimer()
{
    xTimerChangePeriod(ledOffTimer, pdMS_TO_TICKS(strobe.lightOnTime), portMAX_DELAY);
    if (ledOffTimer == NULL)
    {
        sendStringToPrintQueue("Error refreshing the led OFF timer\n");
    }
}

void ledOnTimerCallback(TimerHandle_t xTimer)
{
    setLightPwm();
    strobe.isLedOn = true;
    strobe.lastSwitch = millis();
    sendStringToPrintQueue("LED ON by timer");
    refreshLedOffTimer();
}

void ledOffTimerCallback(TimerHandle_t xTimer)
{
    ledcWrite(pwmChannel, 0);
    strobe.isLedOn = false;
    strobe.lastSwitch = millis();
    sendStringToPrintQueue("LED OFF by timer");
    refreshLedOnTimer();
}

void oneShotSyncTimerCallback(TimerHandle_t xTimer)
{
    char message[100];
    sendStringToPrintQueue("SYNC timer expired");
    switchOnOff();
    if (strobe.isLedOn)
    {
        if (xTimerChangePeriod(ledOffTimer, pdMS_TO_TICKS(strobe.lightOnTime), portMAX_DELAY) != pdPASS)
        {
            sniprintf(message, sizeof(message), "Error changing the led OFF timer period, tried %d", strobe.lightOnTime);
            sendStringToPrintQueue(message);
        }
        else
        {
            snprintf(message, sizeof(message), "Changed the led OFF timer period to %d", strobe.lightOnTime);
            sendStringToPrintQueue(message);
        }      
    }
    else
    {
        if (xTimerChangePeriod(ledOnTimer, pdMS_TO_TICKS(strobe.lightOffTime), portMAX_DELAY) != pdPASS)
        {
            sniprintf(message, sizeof(message), "Error changing the led ON timer period, tried %d", strobe.lightOffTime);
            sendStringToPrintQueue(message);
        }
        else
        {
            snprintf(message, sizeof(message), "Changed the led ON timer period to %d", strobe.lightOffTime);
            sendStringToPrintQueue(message);
        }
    }
}

void refreshSyncTimer()
{
    // print the time from last switch
    char message[70];
    sendStringToPrintQueue("Refresh syncTimer called");
    xTimerStop(ledOnTimer, portMAX_DELAY);
    xTimerStop(ledOffTimer, portMAX_DELAY);
    sendStringToPrintQueue("Stopped the led timers");

    int timeFromLastSwitch = millis() - strobe.lastSwitch;

    if (strobe.isLedOn)
    {
        // if ((strobe.lightOnTime - timeFromLastSwitch) < 1)
        // {
        //     oneShotSyncTimerCallback(NULL);
        //     sendStringToPrintQueue("Timer called artificially to turn LED OFF");
        // }
        // else
        // {
        //     xTimerChangePeriod(oneShotSyncTimer, pdMS_TO_TICKS(strobe.lightOnTime - timeFromLastSwitch), portMAX_DELAY);
        //     snprintf(message, sizeof(message), "Refreshing syncTimer to turn LED OFF, timerExpiring in %d", strobe.lightOnTime - timeFromLastSwitch);
        //     sendStringToPrintQueue(message);
        // }
        if ((strobe.lightOnTime - timeFromLastSwitch) < 1)
        {
            xTimerStop(oneShotSyncTimer, portMAX_DELAY);
            oneShotSyncTimerCallback(NULL);
            sendStringToPrintQueue("Timer called artificially to turn LED OFF");
            // snprintf(message, sizeof(message), "did nothing, timer length: %d", strobe.lightOnTime - timeFromLastSwitch);
            // sendStringToPrintQueue(message);
        }
        else
        {
            xTimerChangePeriod(oneShotSyncTimer, pdMS_TO_TICKS(strobe.lightOnTime - timeFromLastSwitch), portMAX_DELAY);
            snprintf(message, sizeof(message), "Refreshing syncTimer to turn LED OFF, timerExpiring in %d", strobe.lightOnTime - timeFromLastSwitch);
            sendStringToPrintQueue(message);
        }
    }
    else
    {
        if ((strobe.lightOnTime - timeFromLastSwitch) < 1)
        {
            xTimerStop(oneShotSyncTimer, portMAX_DELAY);
            oneShotSyncTimerCallback(NULL);
            sendStringToPrintQueue("Timer called artificially to turn LED ON");
            // snprintf(message, sizeof(message), "did nothing, timer length: %d", strobe.lightOffTime - timeFromLastSwitch);
            // sendStringToPrintQueue(message);
        }
        else
        {
            xTimerChangePeriod(oneShotSyncTimer, pdMS_TO_TICKS(strobe.lightOffTime - timeFromLastSwitch), portMAX_DELAY);
            snprintf(message, sizeof(message), "Refreshing syncTimer to turn LED ON. timerExpiring in %d", strobe.lightOffTime - timeFromLastSwitch);
            sendStringToPrintQueue(message);
        }
    }
    if (oneShotSyncTimer == NULL)
    {
        sendStringToPrintQueue("Error refreshing the one shot sync timer\n");
    }
}

void ledControlTask(void *pvParameters)
{
    char message[70];
    ledEffectParameters lastEffectParameters;
    int strobePeriod;
    if (xQueueReceive(ledEffectParametersQueue, &effectParameters, portMAX_DELAY) == pdTRUE)
    {
        strobePeriod = getStrobePeriod();
        strobe.dutyCycle = effectParameters.intensity;
        strobe.lightOnTime = map(effectParameters.lightDuration, durationMin, durationMax, 1, strobePeriod - 1);
        strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
        ledOnTimerCallback(NULL);
        lastEffectParameters = effectParameters;
    }
    while (true)
    {
        if (xQueueReceive(ledEffectParametersQueue, &effectParameters, portMAX_DELAY) == pdTRUE)
        {
            // Print the state of the led
            snprintf(message, sizeof(message), "LED state: %s", strobe.isLedOn ? "ON" : "OFF");
            sendStringToPrintQueue(message);

            strobePeriod = getStrobePeriod();
            strobe.dutyCycle = effectParameters.intensity;
            if (effectParameters.lightDuration > blackoutMax && effectParameters.lightDuration < lightFullOnMin)
            {
                strobe.lightOnTime = map(effectParameters.lightDuration, durationMin, durationMax, 1, strobePeriod - 1);
                strobe.lightOffTime = strobePeriod - strobe.lightOnTime;
            }
            if (effectParameters.strobeSpeed != lastEffectParameters.strobeSpeed)
            {
                refreshSyncTimer();
            }
            if (effectParameters.intensity != lastEffectParameters.intensity && strobe.isLedOn)
            {
                setLightPwm();
            }
            lastEffectParameters = effectParameters;
        }
        else
        {
            sendStringToPrintQueue("Error receiving ledEffectParameters from the queue\n");
        }
    }
}

void dmxRecieveTask(void *pvParameters)
{
    bool dmxIsConnected = false;
    static dmx_packet_t packet;    // the packet structure that will be filled with the data from the DMX bus
    uint8_t data[DMX_PACKET_SIZE]; // the data read from the DMX bus
    char message[70];              // Buffer for formatting the message for the printing
    ledEffectParameters recievedParameters;
    ledEffectParameters lastSentParameters;

    while (true)
    {
        if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK))
        {
            // We should check to make sure that there weren't any DMX errors.
            if (!packet.err)
            {
                // If this is the first DMX data we've received, lets log it!
                if (!dmxIsConnected)
                {
                    sendStringToPrintQueue("DMX connected!\n");
                    dmxIsConnected = true;
                }
                dmx_read(dmxPort, data, packet.size);
                recievedParameters.intensity = data[1];     // CH1
                recievedParameters.strobeSpeed = data[2];   // CH2
                recievedParameters.lightDuration = data[3]; // CH3

                if (recievedParameters != lastSentParameters)
                {
                    // Send the parameters to the led control task
                    if (xQueueSend(ledEffectParametersQueue, &recievedParameters, portMAX_DELAY) == pdPASS)
                    {
                        lastSentParameters = recievedParameters;
                        snprintf(message, sizeof(message), "SC: 0x%02X, CH1: %3d, CH2: %3d, CH3: %3d",
                                 data[0], recievedParameters.intensity,
                                 recievedParameters.strobeSpeed,
                                 recievedParameters.lightDuration);
                        sendStringToPrintQueue(message);
                    }
                    else
                    {
                        sendStringToPrintQueue("Error sending ledEffectParameters to the queue\n");
                    }
                }
            }
            else
            {
                // A DMX error occurred! This can happen when you connect or disconnect your DMX devices.
                // If you are consistently getting DMX errors, something may have gone wrong.
                sendStringToPrintQueue("DMX error occured!\n");
            }
        }
        else // if (dmxIsConnected)
        {
            // If DMX times out after having been connected, it likely means that the DMX cable was unplugged.
            sendStringToPrintQueue("DMX disconnected!\n");
            dmxIsConnected = false;
        }
    }
}

void serialPrintTask(void *parameter)
{
    char *message = NULL;
    while (true)
    {
        if (xQueueReceive(serialPrintQueue, &message, portMAX_DELAY))
        {
            Serial.println(message);
            free(message);  // Free the memory after printing the string
            message = NULL; // Reset the pointer after freeing the memory
        }
    }
}
/*void serialPrintTask(void *parameter)
{
    char *message;
    while (true)
    {
        if (xQueueReceive(serialPrintQueue, &message, portMAX_DELAY))
        {
            Serial.println(message);
            free(message); // Free the memory after printing the string
        }
    }
}*/

void setup()
{
    Serial.begin(115200);

    // LED PWM configuration
    ledcSetup(pwmChannel, pwmFrequency, resolution);
    ledcAttachPin(ledPin, pwmChannel);

    // Default state of the LED
    ledcWrite(pwmChannel, 0);
    strobe.isLedOn = false;

    // First, use the default DMX configuration...
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    // ...install the DMX driver...
    dmx_driver_install(dmxPort, &config, DMX_INTR_FLAGS_DEFAULT);
    // ...and then set the communication pins
    dmx_set_pin(dmxPort, tx_pin, rx_pin, rts_pin);

    // Create queue for printing
    serialPrintQueue = xQueueCreate(10, sizeof(char *));
    if (serialPrintQueue == NULL)
    {
        Serial.println("Error creating the serial print queue");
        vTaskDelete(NULL); // this should end the program since this is the only task running
    }
    ledEffectParametersQueue = xQueueCreate(10, sizeof(ledEffectParameters));
    if (ledEffectParametersQueue == NULL)
    {
        Serial.println("Error creating the ledEffectParameters queue");
        vTaskDelete(NULL); // this should end the program since this is the only task running
    }

    // Create tasks
    xTaskCreatePinnedToCore(serialPrintTask, "SerialPrintTask", 1024, NULL, 1, &serialPrintTaskHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ledControlTask, "LEDControlTask", 2048, NULL, 1, &ledControlHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(dmxRecieveTask, "DMXRecieveTask", 4096, NULL, 1, &dmxReceiveHandle, APP_CPU_NUM);

    // Create timers
    oneShotSyncTimer = xTimerCreate("OneShtSyncTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, oneShotSyncTimerCallback);
    ledOffTimer = xTimerCreate("LedOFFTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOffTimerCallback);
    ledOnTimer = xTimerCreate("LedONTimer", pdMS_TO_TICKS(9999), pdFALSE, NULL, ledOnTimerCallback);
    if (oneShotSyncTimer == NULL || ledOffTimer == NULL || ledOnTimer == NULL)
    {
        sendStringToPrintQueue("Error creating the timers\n");
        vTaskDelete(NULL); // this should end the program since this is the only task running
    }

    // Delete "setup and loop" task
    vTaskDelete(NULL);
}

void loop()
{
    // this should never be executed
}