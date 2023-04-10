#pragma once

#include "logic_analizer.h"

#define LEDC_OUTPUT_IO (19)  
#define GPIO_BLINK (15)

void sump_task(void *arg);
void sump_la_cb(uint16_t *buf, int cnt, int clk);
