
/*******************************************************************************
* Title                 :   General Purpose Input/Output (GPIO) STM32F411 Config
* Filename              :   gpio_stm32f411_config.h
* Author                :   Marko Galevski
* Origin Date           :   14/01/2020
* Version               :   1.0.0
* Compiler              :   GCC
* Target                :   STM32F411VE (ARM Cortex M4)
* Notes                 :   None
*
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

/** @file gpio_stm32f411_config.h
 *  @brief machine specific configuration enumerations and structures
 */
/******************************************************************************
* Includes
*******************************************************************************/
#ifndef _GPIO_STM32F4XX_CONFIG_H
#define _GPIO_STM32F4XX_CONFIG_H
#include <stdint.h>

/**
 * Contains all of the gpio pins on all of the ports. Intermediary calculations are used to separate port and pins.
 */
typedef enum
{
  GPIO_A_0, GPIO_A_1, GPIO_A_2, GPIO_A_3, GPIO_A_4, GPIO_A_5, GPIO_A_6, GPIO_A_7, GPIO_A_8, GPIO_A_9, GPIO_A_10,
  	  GPIO_A_11, GPIO_A_12, GPIO_A_13, GPIO_A_14, GPIO_A_15,

  GPIO_B_0, GPIO_B_1, GPIO_B_2, GPIO_B_3, GPIO_B_4, GPIO_B_5, GPIO_B_6, GPIO_B_7, GPIO_B_8, GPIO_B_9, GPIO_B_10,
	  GPIO_B_11, GPIO_B_12, GPIO_B_13, GPIO_B_14, GPIO_B_15,

  GPIO_C_0, GPIO_C_1, GPIO_C_2, GPIO_C_3, GPIO_C_4, GPIO_C_5, GPIO_C_6, GPIO_C_7, GPIO_C_8, GPIO_C_9, GPIO_C_10,
	  GPIO_C_11, GPIO_C_12, GPIO_C_13, GPIO_C_14, GPIO_C_15,

  GPIO_D_0, GPIO_D_1, GPIO_D_2, GPIO_D_3, GPIO_D_4, GPIO_D_5, GPIO_D_6, GPIO_D_7, GPIO_D_8, GPIO_D_9, GPIO_D_10,
  	  GPIO_D_11, GPIO_D_12, GPIO_D_13, GPIO_D_14, GPIO_D_15,

  GPIO_E_0, GPIO_E_1, GPIO_E_2, GPIO_E_3, GPIO_E_4, GPIO_E_5, GPIO_E_6, GPIO_E_7, GPIO_E_8, GPIO_E_9, GPIO_E_10,
 	  GPIO_E_11, GPIO_E_12, GPIO_E_13, GPIO_E_14, GPIO_E_15,
  NUM_GPIO_PINS

}gpio_pin_t;

/**
 * Contains both active states a pin can be in. Actual electrical behaviour depends on push-pull/open drain settings.
 */
typedef enum
{
	GPIO_PIN_LOW = 0UL,
	GPIO_PIN_HIGH = 1UL
}gpio_pin_state_t;

/**
 * Contains all the modes a specific pin can be in.
 */
typedef enum
{
  GPIO_INPUT, /**<The pin functions as a digital input */
  GPIO_OUTPUT, /**<The pin functions as a digital output */
  GPIO_ALTERNATE_FUNCTION, /**<The pin is multiplexed to allow another peripheral to control it @see gpio_mux_t*/
  GPIO_ANALOG,/**<The pin works as an analog, defined by the ADC peripheral */
  GPIO_MAX_MODE_OPTIONS /**< Redundant extra option. Can be used for assertions in super robust implementations
   	   	   	   	   	   	   	   where the strength of enums is in question. */
}gpio_mode_t;



/**
 * Contains the resistor options over a pin for both input and output modes.
 */
typedef enum
{
  GPIO_NO_RESISTOR, /**<No resistor. Pin is undefined unless driven actively */
  GPIO_PULL_UP, /**<Pull up resistor over pin. Will default to high unless driven*/
  GPIO_PULL_DOWN,/**<Pull down resistor over pin. Will default to low unless driven */
  GPIO_MAX_RESISTOR_OPTIONS /**< Redundant extra option. Can be used for assertions in super robust
   	   	   	   	   	   	   	   where the strength of enums is in question. */
}gpio_resistor_t;

/**
 * Defines the electrical behaviour of an output pin.
 */
typedef enum
{
  GPIO_PUSH_PULL,/**<The pin can drive to electrical defined 1 and 0 (Vdd and GND) */
  GPIO_OPEN_DRAIN,/**<The pin can only drive to GND. Output options are undefined and 0.*/
  GPIO_MAX_OUTPUT_OPTIONS/**< Redundant extra option. Can be used for assertions in super robust implementations
   	   	   	   	   	   	   	   where the strength of enums is in question. */
}gpio_output_type_t;

/**
 * Contains speed options for a pin's output. Actual speed is a factor Vdd and capacitor selection.
 * See pages 101-102 in the STM32F411xE datasheet for concrete numbers. All ranges given below are
 * implementation sensitive
 */
typedef enum
{
  GPIO_LOW_SPEED,/**<Output speed is between 2-8MHz */
  GPIO_MED_SPEED,/**<Output speed is between 12.5-50MHz */
  GPIO_FAST_SPEED,/**<Output speed is between 25-100MHz */
  GPIO_HIGH_SPEED,/**<Output speed is between 50-100MHz */
  GPIO_MAX_SPEED_OPTIONS/**< Redundant extra option. Can be used for assertions in super robust implementations
   	   	   	   	   	   	   	   where the strength of enums is in question. */
}gpio_output_speed_t;

/**
 * All alternate function values fed into the 4bit multiplexer. See Figure 17 in RM0383
 */
typedef enum
{
  GPIO_AF_0, GPIO_AF_1, GPIO_AF_2, GPIO_AF_3,
  GPIO_AF_4, GPIO_AF_5, GPIO_AF_6, GPIO_AF_7,
  GPIO_AF_8, GPIO_AF_9, GPIO_AF_10, GPIO_AF_11,
  GPIO_AF_12, GPIO_AF_13, GPIO_AF_14, GPIO_AF_15,
  GPIO_MAX_AF_OPTIONS
}gpio_mux_t;

/**
 * Configuration structure holding all values needed to configure a pin.
 */
typedef struct
{
  gpio_pin_t pin; /**<Which pin is being configured */
  gpio_mode_t mode; /**<Selected pin mode */
  gpio_resistor_t resistor; /**<Pull- up/down selection */
  gpio_output_type_t output_type; /**<Output type (only relevant in output mode)  */
  gpio_output_speed_t output_speed; /**<Output speed (only relevant in output mode) */
  gpio_mux_t mux; /**<Multiplexer signal used to select alternate function (only relevant in AF mode) */
}gpio_config_t;

const gpio_config_t *gpio_config_get(void);

#endif /* gpio_config for stm32f4xx series */
