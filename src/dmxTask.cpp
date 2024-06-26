#include "dmxTask.hpp"

#include <Arduino.h>
#include <WrapperFreeRTOS.h>
#include <esp_dmx.h>

#include "deviceConfig.hpp"
#include "logTask.hpp"

static const char *TAG = "DmxTask";

DmxTask::DmxTask(uint32_t stackSize, uint8_t priority, uint8_t coreId)
    : Task("DmxTask", stackSize, priority) {
    setCore(coreId);
    m_queueHandle = NULL;
    m_dmxConnected = false;
}

void DmxTask::createQueue() {
    m_queueHandle = xQueueCreate(10, sizeof(DmxMode<3>));
    if (m_queueHandle == NULL) {
        ESP_LOGE(TAG, "Error creating the DmxMode queue");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP.restart();
    }
}

void DmxTask::setup() {
    // First, use the default DMX configuration...
    dmx_config_t config = DMX_CONFIG_DEFAULT;
    // ...install the DMX driver...
    dmx_driver_install(DEVICECONF_DMX_PORT, &config, DMX_INTR_FLAGS_DEFAULT);
    // ...and then set the communication pins
    dmx_set_pin(DEVICECONF_DMX_PORT, DEVICECONF_DMX_TX_PIN, DEVICECONF_DMX_RX_PIN, DEVICECONF_DMX_RTS_PIN);
    // Create the queue for outgoing DMX data
    createQueue();
}

void DmxTask::run(void *data) {
    static dmx_packet_t packet;  // the packet structure that will be filled with the data
                                 // from the DMX bus
    uint8_t lastData[3] = {0, 0, 0};

    while (true) {
        if (dmx_receive(DEVICECONF_DMX_PORT, &packet, DMX_TIMEOUT_TICK)) {
            // We should check to make sure that there weren't any DMX errors.
            if (!packet.err) {
                // If this is the first DMX data we've received, lets log it!
                if (!m_dmxConnected) {
                    logTask.queueLog(TAG, "DMX connected!", LogLevel::INFO);
                    m_dmxConnected = true;
                }
                dmx_read_offset(DEVICECONF_DMX_PORT, 1, m_3chMode.data, m_3chMode.m_nChannels);

                if (memcmp(m_3chMode.data, lastData, 3) != 0) {
                    // Send the parameters to the led control task
                    if (xQueueSend(m_queueHandle, &m_3chMode, portMAX_DELAY) == pdPASS) {
                        memcpy(lastData, m_3chMode.data, 3);
                    } else {
                        logTask.queueLog(TAG, "Error sending DMX data to the queue", LogLevel::ERROR);
                    }
                }
            } else {
                // A DMX error can happen when you connect/disconnect your DMX devices or if something goes
                // wrong.
                logTask.queueLog(TAG, "DMX error occured!", LogLevel::ERROR);
            }
        } else  // if (dmxIsConnected)
        {
            // DMX timeout after having been connected likely means that the DMX cable was unplugged.
            logTask.queueLog(TAG, "DMX disconnected!", LogLevel::INFO);
            m_dmxConnected = false;
        }
    }
}

// Global instance of the DmxTask
DmxTask dmxTask(4096, 2, PRO_CPU_NUM);
