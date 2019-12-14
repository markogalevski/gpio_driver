#ifndef _GPIO_H
#define GPIO_H

#include "stm32f411xe.h"
#include "gpio_stm32f4xx_config.h"
#include "assert.h"

void gpio_init(gpio_config_t * gpio_config);
uint32_t  gpio_pin_read(gpio_pin_t pin);
void gpio_pin_write(gpio_pin_t pin, gpio_pin_state_t value);
void gpio_pin_toggle(gpio_pin_t pin);
void gpio_register_write(uint32_t gpio_register, uint32_t value);
uint32_t gpio_register_read(uint32_t gpio_register);




#endif /* gpio_interface.h */
