#include <Arduino.h>
#include <esp_dmx.h>

// led pwm
const int pwmFrequency = 5000; // Hz
const int pwmChannel = 0;
const int ledPin = 13;
const int resolution = 8; // max resolution of the DMX512 protocol values
int dutyCycle = 255;

// dmx
const int tx_pin = 17;  // TXD_2 not used, i dont know if it is needed
const int rx_pin = 16;  // RXD_2
const int rts_pin = 21; // not used because the module does it automatically, and i dont know if it is needed

const dmx_port_t dmxPort = DMX_NUM_2;
bool dmxIsConnected = false;

uint8_t data[DMX_PACKET_SIZE]; // the data read from the DMX bus

unsigned long lastUpdate = millis();

// FreeRTOS stuff
// tasks
TaskHandle_t ledControlHandle = NULL;
TaskHandle_t dmxReceiveHandle = NULL;
TaskHandle_t serialPrintTaskHandle = NULL;
// queues
QueueHandle_t printQueue;
QueueHandle_t ledEffectParametersQueue;

// struct that will be sent to the led control task
struct ledEffectParameters
{
    uint8_t intensity;     // CH1
    uint8_t strobePeriod;  // CH2
    uint8_t lightDuration; // CH3
};

void printString(const char *string)
{
    char *message = (char *)malloc((strlen(string) + 1) * sizeof(char)); // Allocate memory for the string
    strcpy(message, string);                                             // Copy the string into the allocated memory

    if (xQueueSend(printQueue, &message, portMAX_DELAY) != pdPASS) // Send the pointer to the queue
    {
        // If xQueueSend failed, free the memory to prevent a leak
        free(message);
    }
}

void ledControlTask(void *pvParameters)
{
    ledEffectParameters effect;
    while (true)
    {
        ledcWrite(pwmChannel, dutyCycle);
    }
}

void dmxRecieveTask(void *pvParameters)
{
    static dmx_packet_t packet;
    char message[50]; // Buffer for formatting the message
    ledEffectParameters recievedParameters;

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
                    printString("DMX connected!\n");
                    dmxIsConnected = true;
                }

                dmx_read(dmxPort, data, packet.size);
                snprintf(message, sizeof(message), "Start code is 0x%02X and slot 1 is 0x%02X\n", data[0], data[1]);
                printString(message);
            }
            else
            {
                // A DMX error occurred! This can happen when you connect or disconnect your DMX devices.
                // If you are consistently getting DMX errors, something may have gone wrong.
                printString("DMX error occured!\n");
            }
        }
        /*else if (dmxIsConnected)
        {
            // If DMX times out after having been connected, it likely means that the
            // DMX cable was unplugged.
            printString("DMX disconnected!\n");
            dmx_driver_delete(dmxPort);

            // Stop the program.
            while (true)
                yield();
        }*/
    }
}

void serialPrintTask(void *parameter)
{
    char *message;
    while (true)
    {
        if (xQueueReceive(printQueue, &message, portMAX_DELAY))
        {
            Serial.println(message);
            free(message); // Free the memory after printing the string
        }
    }
}

void setup()
{
    Serial.begin(115200);

    // LED PWM configuration
    ledcSetup(pwmChannel, pwmFrequency, resolution);
    ledcAttachPin(ledPin, pwmChannel);

    // First, use the default DMX configuration...
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    // ...install the DMX driver...
    dmx_driver_install(dmxPort, &config, DMX_INTR_FLAGS_DEFAULT);
    // ...and then set the communication pins
    dmx_set_pin(dmxPort, tx_pin, rx_pin, rts_pin);

    // Create queue for printing
    printQueue = xQueueCreate(10, sizeof(char *));
    if (printQueue == NULL)
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
    xTaskCreatePinnedToCore(serialPrintTask, "Serial Print Task", 1024, NULL, 1, &serialPrintTaskHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(ledControlTask, "LED Control Task", 1024, NULL, 1, &ledControlHandle, APP_CPU_NUM);
    xTaskCreatePinnedToCore(dmxRecieveTask, "DMX Recieve Task", 2048, NULL, 1, &dmxReceiveHandle, APP_CPU_NUM);

    // Delete "setup and loop" task
    vTaskDelete(NULL);
}

void loop()
{
    // this should never be executed
}