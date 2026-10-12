#include "int_led.h"

void int_led_on(LED_STRUCT *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}

void int_led_off(LED_STRUCT *led)
{
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}

void int_led_toggle(LED_STRUCT *led)
{
    HAL_GPIO_TogglePin(led->port, led->pin);
}

