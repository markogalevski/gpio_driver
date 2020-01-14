/*******************************************************************************
* Title                 :   General Purpose Input/Output (GPIO) Interface
* Filename              :   gpio_interface.h
* Author                :   Marko Galevski
* Origin Date           :   14/01/2020
* Version               :   1.0.0
* Compiler              :   None
* Target                :   None
* Notes                 :   None
*
*******************************************************************************/
/****************************************************************************
* Doxygen C Template
* Copyright (c) 2013 - Jacob Beningo - All Rights Reserved
*
* Feel free to use this Doxygen Code Template at your own risk for your own
* purposes.  The latest license and updates for this Doxygen C template can be
* found at www.beningo.com or by contacting Jacob at jacob@beningo.com.
*
* For updates, free software, training and to stay up to date on the latest
* embedded software techniques sign-up for Jacobs newsletter at
* http://www.beningo.com/814-2/
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Template.
*
*****************************************************************************/

/** @file gpio_interface.h
 *  @brief An interface which allows for a level of modularity when using the gpio
 *  between different architectures.
 *  Usage Notes: To port the driver, change the include config line to point instead
 *  to the new config.h file you want. The config file and the interface file together
 *  should provide all definitions to implement what you need in the machine specific c
 *  implementation file.
 */
#ifndef _GPIO_H
#define _GPIO_H


#include "gpio_stm32f411_config.h"
#include "assert.h"

void gpio_init(gpio_config_t *config_table);
gpio_pin_state_t gpio_pin_read(gpio_pin_t pin);
void gpio_pin_write(gpio_pin_t pin, gpio_pin_state_t value);
void gpio_pin_toggle(gpio_pin_t pin);

void gpio_register_write(uint32_t gpio_register, uint32_t value);
uint32_t gpio_register_read(uint32_t gpio_register);


#endif /* gpio_interface.h */
