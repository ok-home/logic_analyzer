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

#define GPIO_IRQ_PIN_26 (26)
#define GPIO_IRQ_PIN_27 (27)



#define LA_CLK_SAMPLE_RATE 80000000
#define LA_MAX_SAMPLE_RATE 40000000
#define LA_MIN_SAMPLE_RATE 5000

#define LA_MAX_PIN 16
#define LA_MIN_GPIO -1
#define LA_MAX_GPIO 39
#define LA_MAX_SAMPLE_CNT 32764
#define LA_MIN_SAMPLE_CNT 100

// define logic analyzer channel pin
// -1 - pin disabled
#ifdef CONFIG_ANALYZER_USE_PORT_ASSIGN

#define LA_PIN_0 (CONFIG_ANALYZER_CHAN_0)
#define LA_PIN_1 (CONFIG_ANALYZER_CHAN_1)
#define LA_PIN_2 (CONFIG_ANALYZER_CHAN_2)
#define LA_PIN_3 (CONFIG_ANALYZER_CHAN_3)
#define LA_PIN_4 (CONFIG_ANALYZER_CHAN_4)
#define LA_PIN_5 (CONFIG_ANALYZER_CHAN_5)
#define LA_PIN_6 (CONFIG_ANALYZER_CHAN_6)
#define LA_PIN_7 (CONFIG_ANALYZER_CHAN_7)
#define LA_PIN_8 (CONFIG_ANALYZER_CHAN_8)
#define LA_PIN_9 (CONFIG_ANALYZER_CHAN_9)
#define LA_PIN_10 (CONFIG_ANALYZER_CHAN_10)
#define LA_PIN_11 (CONFIG_ANALYZER_CHAN_11)
#define LA_PIN_12 (CONFIG_ANALYZER_CHAN_12)
#define LA_PIN_13 (CONFIG_ANALYZER_CHAN_13)
#define LA_PIN_14 (CONFIG_ANALYZER_CHAN_14)
#define LA_PIN_15 (CONFIG_ANALYZER_CHAN_15)
#define LA_PIN_TRIGGER (CONFIG_ANALYZER_TRIG_PIN)
#define LA_PIN_EDGE (CONFIG_ANALYZER_TRIG_EDGE)
#define LA_SAMPLE_COUNT (CONFIG_ANALYZER_SAMPLES_COUNT)
#define LA_SAMPLE_RATE (CONFIG_ANALYZER_SAMPLE_RATE)
#define LA_DEFAULT_TiMEOUT (CONFIG_ANALYZER_TIMEOUT*100)

#else

#define LA_PIN_0 (IN_PORT_1)
#define LA_PIN_1 (LEDC_OUTPUT_IO)
#define LA_PIN_2 (IN_PORT_2)
#define LA_PIN_3 (GPIO_BLINK)
#define LA_PIN_4 (-1)
#define LA_PIN_5 (GPIO_IRQ_PIN_26)
#define LA_PIN_6 (GPIO_IRQ_PIN_27)
#define LA_PIN_7 (-1)
#define LA_PIN_8 (-1)
#define LA_PIN_9 (-1)
#define LA_PIN_10 (-1)
#define LA_PIN_11 (-1)
#define LA_PIN_12 (-1)
#define LA_PIN_13 (-1)
#define LA_PIN_14 (-1)
#define LA_PIN_15 (-1)
#define LA_PIN_TRIGGER (-1)
#define LA_PIN_EDGE (1)
#define LA_SAMPLE_COUNT (1000)
#define LA_SAMPLE_RATE (1000000)
#define LA_DEFAULT_TiMEOUT (2000)
#endif







