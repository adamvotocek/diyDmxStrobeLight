#pragma once

// PINS
//  Dmx module
// in my case I can only use TX2, because TX0 is connected to the esp-prog and used for serial communication
#define DEVICECONF_DMX_TX_PIN 17 // 1=TX0, 17=TX2
#define DEVICECONF_DMX_RX_PIN 16 // 3=RX0, 16=RX2
#define DEVICECONF_DMX_RTS_PIN 4
// LED mosfet driver
#define DEVICECONF_STROBE_LED_PIN 25
// Fan mosfet
#define DEVICECONF_FAN_PIN 26
// Display // TODO: Add display and menu control
#define DEVICECONF_DISPLAY_SDA_PIN 21
#define DEVICECONF_DISPLAY_SCL_PIN 22
// Buttons // TODO: Add buttons to control the device
#define DEVICECONF_MODE_BUTTON_PIN 34
#define DEVICECONF_PLUS_BUTTON_PIN 35
#define DEVICECONF_MINUS_BUTTON_PIN 32
#define DEVICECONF_ENTER_BUTTON_PIN 33

// PWM Led control
#define DEVICECONF_PWM_CHANNEL 0
#define DEVICECONF_PWM_FREQUENCY 5000 // Hz

// DMX 
#define DEVICECONF_DMX_PORT DMX_NUM_2

// Effects
// Strobe
#define DEVICECONF_STROBE_PERIOD_MIN 30 // ms
#define DEVICECONF_STROBE_PERIOD_MAX 999 // ms