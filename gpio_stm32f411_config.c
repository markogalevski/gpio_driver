/*******************************************************************************
* Title                 :   General Purpose Input/Output (GPIO) Config Table
* Filename              :   gpio_stm32f411_config.c
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

/** @file gpio_stm32f411_config.c
 *  @brief A file defining a config table which contains all information required by
 *  gpio_init to initialise the pins with the desired behaviour.
 */

#include "gpio_stm32f411_config.h"

/**
 * A table containing the settings required to eachieve the desired behaviours for each pin.
 * Irrelevant fields are ignored at a hardware level, so feel free to place zeros to save space.
 */
static const gpio_config_t gpio_config_table[] =
{
	//PIN		//MODE			//RESISTOR			//OUTPUT		//SPEED 		//MUX
	//e.g. for i2s
{	GPIO_A_15, 	GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR,  GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_6},
{	GPIO_B_3, GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR, GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_6},
{	GPIO_B_5, GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR, GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_6},
{ 	GPIO_B_10, GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR, GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_4},
{   GPIO_B_11, GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR, GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_4}

};


const uint32_t ACTIVE_GPIO_PINS = sizeof(gpio_config_table)/sizeof(gpio_config_t);
/**<Prevents iteration over 64 pins when only a few are used*/


/******************************************************************************
 * Function : gpio_config_get()
 *//**
 * \b Description:
 *	Retrieves the config table for the gpio peripheral, normally hidden statically within the
 *	config.c file.
 *
 * PRE-CONDITION: The config table has been populated/exists with a size greater than 0.
 *
 * POST-CONDITION: The returned value points to the base of the config table
 *
 * @return			const gpio_config_t *
 *
 * \b Example:
 *
 * @code
 * const gpio_config_t gpio_config_table = gpio_config_get(void);
 * gpio_init(gpio_config_table);
 * @endcode
 *
 * @see gpio_init
 *
 * <br><b> - CHANGE HISTORY - </b>
 *
 * <table align="left" style="width:800px">
 * <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
 * </table><br><br>
 * <hr>
 *
 */
const gpio_config_t *gpio_config_get(void)
{
	return (gpio_config_table);
}
