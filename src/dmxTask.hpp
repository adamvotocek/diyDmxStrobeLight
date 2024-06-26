#pragma once

#include <Arduino.h>
#include <WrapperFreeRTOS.h>  // FreeRTOS C++ wrapper

template <size_t nChannels>
class DmxMode {
   public:
    static_assert(nChannels <= 512, "nChannels must be 512 or less");
    DmxMode() : m_nChannels(nChannels) {}

    uint8_t data[nChannels];  // Array size is determined at compile time
    uint16_t m_nChannels;
};

class DmxTask : public Task {
   public:
    DmxTask(uint32_t stackSize = 4096, uint8_t priority = 2, uint8_t coreId = PRO_CPU_NUM);

    void createQueue();
    void setup();
    void run(void *data);
    QueueHandle_t getQueueHandle(){ return m_queueHandle;};

   private:
    DmxMode<3> m_3chMode;
    QueueHandle_t m_queueHandle;
    bool m_dmxConnected;
};

extern DmxTask dmxTask;

