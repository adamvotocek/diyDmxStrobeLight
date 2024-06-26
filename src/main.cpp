#include <Arduino.h>
#include <esp_log.h>

// Configuration of things like pin numbers and other device specific settings
#include "deviceConfig.hpp"

// Import all the different tasks.
// Because of cross-dependencies, the implementations already construct a global instance of each task.
#include "dmxTask.hpp"
#include "ledControlTask.hpp"
#include "logTask.hpp"

// Tag for logging, this should be in every file that logs anything
static const char *TAG = "main.cpp";

void setup() {
    // Turn on the fan attached to the mosfet
    pinMode(DEVICECONF_FAN_PIN, OUTPUT);
    digitalWrite(DEVICECONF_FAN_PIN, HIGH);

    // Setup tasks
    logTask.setup();
    dmxTask.setup();
    ledControlTask.setup();

    // Start tasks
    logTask.start();
    dmxTask.start();
    ledControlTask.start();

    logTask.queueLog(TAG, "Setup complete", LogLevel::DEBUG);  // Log that the setup is complete
    vTaskDelete(NULL);                                         // Delete "setup and loop" task
}

void loop() {}  // This should never get executed as this task is deleted in setup