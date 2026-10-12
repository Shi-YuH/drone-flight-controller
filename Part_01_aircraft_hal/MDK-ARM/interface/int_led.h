#ifndef INT_LED_H
#define INT_LED_H

#include "main.h"

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} LED_STRUCT;

void int_led_on(LED_STRUCT *led);
void int_led_off(LED_STRUCT *led);
void int_led_toggle(LED_STRUCT *led);


#endif
