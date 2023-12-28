#include <Arduino.h>
#include <esp_dmx.h>

// led pwm
const int pwmFrequency = 5000; // Hz
const int pwmChannel = 0;
const int ledPin = 13;
const int resolution = 8; // max resolution of the DMX512 protocol
int dutyCycle = 1;

// dmx
const int tx_pin = 17;  // TXD_2 not used, i dont know if it is needed
const int rx_pin = 16;  // RXD_2
const int rts_pin = 21; // not used because the module does it automatically, and i dont know if it is needed

const dmx_port_t dmxPort = DMX_NUM_2;
bool dmxIsConnected = false;

uint8_t data[DMX_PACKET_SIZE]; // the data read from the DMX bus

unsigned long lastUpdate = millis();
/*
//tasks
TaskHandle_t ledControl = NULL;
TaskHandle_t dmxReceive = NULL;

void ledControlTask(void *pvParameters)
{
    while (true)
    {
        ledcWrite(pwmChannel, dutyCycle);
        dutyCycle++;
        if (dutyCycle > 255)
        {
            dutyCycle = 0;
        }
        delay(5);
    }
}

void dmxRecieveTask(void *pvParameters)
{
    dmx_packet_t packet;

    while (true)
    {
        if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK))
        {
            //If this code gets called, it means we've received DMX data!

            unsigned long now = millis();

            //We should check to make sure that there weren't any DMX errors. 
            if (!packet.err)
            {
                // If this is the first DMX data we've received, lets log it! 
                if (!dmxIsConnected)
                {
                    Serial.println("DMX is connected!");
                    dmxIsConnected = true;
                }

                // Don't forget we need to actually read the DMX data into our buffer so
                // that we can print it out. 
                dmx_read(dmxPort, data, packet.size);

                if (now - lastUpdate > 10)
                {
                    // Print the received start code - it's usually 0.
                    Serial.printf("Start code is 0x%02X and slot 1 is 0x%02X\n", data[0], data[1]);
                    lastUpdate = now;
                }
            }
            else
            {
            //   A DMX error occurred! This can happen when you first
            //   connect or disconnect your DMX devices. If you are consistently getting
            //   DMX errors, then something may have gone wrong with your code or
            //   something is seriously wrong with your DMX transmitter. 
                Serial.println("A DMX error occurred.");
            }
        }
        else if (dmxIsConnected)
        {
            // If DMX times out after having been connected, it likely means that the
            // DMX cable was unplugged.
            Serial.println("DMX was disconnected.");
            dmx_driver_delete(dmxPort);

            // Stop the program.
            while (true)
                yield();
        }
    }
}*/

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
}

void loop()
{
    dmx_packet_t packet;

    if (dmx_receive(dmxPort, &packet, DMX_TIMEOUT_TICK)) 
    {
        /* If this code gets called, it means we've received DMX data! */

        unsigned long now = millis();

        /* We should check to make sure that there weren't any DMX errors. */
        if (!packet.err)
        {
            /* If this is the first DMX data we've received, lets log it! */
            if (!dmxIsConnected)
            {
                Serial.println("DMX is connected!");
                dmxIsConnected = true;
            }

            /* Don't forget we need to actually read the DMX data into our buffer so
              that we can print it out. */
            dmx_read(dmxPort, data, packet.size);

            if (now - lastUpdate > 10)
            {
                /* Print the received start code - it's usually 0. */
                Serial.printf("Start code is 0x%02X and slot 1 is 0x%02X\n", data[0], data[1]);
                lastUpdate = now;
            }
        }
        else
        {
            /* A DMX error occurred! This can happen when you first
              connect or disconnect your DMX devices. If you are consistently getting
              DMX errors, then something may have gone wrong with your code or
              something is seriously wrong with your DMX transmitter. */
            Serial.println("A DMX error occurred.");
        }
    }
    else if (dmxIsConnected)
    {
        /* If DMX times out after having been connected, it likely means that the
          DMX cable was unplugged.*/
        Serial.println("DMX was disconnected.");
        dmx_driver_delete(dmxPort);

        /* Stop the program. */
        while (true)
            yield();
    }

    ledcWrite(pwmChannel, dutyCycle);
    /*dutyCycle++;
    if (dutyCycle > 255)
    {
        dutyCycle = 0;
    }
    delay(5);*/
}