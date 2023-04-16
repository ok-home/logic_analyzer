#pragma once

// definition test sample
// output pin ledc example
#define LEDC_OUTPUT_IO (18)
// output pin gpio blink example 
#define GPIO_BLINK (19)
// input wired example pins
#define IN_PORT_1 (22)
#define IN_PORT_2 (23)
// i2c sample example pins
#define I2C_PIN_SDA (26)
#define I2C_PIN_SCL (27)

// sigrok default sample rate
#define PULSEVIEW_MAX_SAMPLE_RATE 100000000
// logic analyzer max data buff size
#define MAX_CAPTURE_SIZE (32764 * 2)
// logic analyzer max count sample
#define MAX_SAMPLE_COUNT (MAX_CAPTURE_SIZE / 2)
// logic analyzer max sample clock HZ (sigrok not use this)
#define MAX_SAMPLE_RATE 20000000

// define logic analyzre channel pin
// -1 - pin disabled
#define LA_PIN_0 (IN_PORT_1)
#define LA_PIN_1 (LEDC_OUTPUT_IO)
#define LA_PIN_2 (IN_PORT_2)
#define LA_PIN_3 (GPIO_BLINK)
#define LA_PIN_4 (-1)
#define LA_PIN_5 (I2C_PIN_SDA)
#define LA_PIN_6 (I2C_PIN_SCL)
#define LA_PIN_7 (-1)
#define LA_PIN_8 (-1)
#define LA_PIN_9 (-1)
#define LA_PIN_10 (-1)
#define LA_PIN_11 (-1)
#define LA_PIN_12 (-1)
#define LA_PIN_13 (-1)
#define LA_PIN_14 (-1)
#define LA_PIN_15 (-1)

// define logic analyzrer trigger pin 
// -1 disable
// sigrok redefine this pin
#define LA_PIN_TRIGGER (-1)

// define logic analyzrer timeout freertos tick
// default 20 sek
#define LA__DEFAULT_TiMEOUT (2000)


