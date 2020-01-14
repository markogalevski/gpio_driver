/*******************************************************************************
* Title                 :   General Purpose Input/Output (GPIO) STM32F411 Implementation
* Filename              :   gpio_stm32f411.c
* Author                :   Marko Galevski
* Origin Date           :   14/01/2020
* Version               :   1.0.0
* Compiler              :   GCC
* Target                :   STM32F411VE (ARM Cortex M4)
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

/** @file gpio_stm32f411.c
 *  @brief machine specific implementation of gpio
 */
#include "gpio_interface.h"
#include "stm32f411xe.h"

#define PINS_PER_PORT 16 /**<The number of pins per letter named port */
/* covers A, B, C, D, E (as per 8.1 in STM32F411xC/E Ref. Manual, but no H */
#define NUM_GPIO_PORTS NUM_GPIO_PINS/PINS_PER_PORT /**<Number of letter named ports*/
#define MODERy_WIDTH 0x03UL /**<The width of the bitfield controlling a pin's mode*/
#define OTYPERy_WIDTH 0x01UL /**<The width of the bitfield controlling the output type of a pin */
#define OSPEEDRy_WIDTH 0x03UL /**<The width of the bitfield controlling the output speed of a pin */
#define PUPDRy_WIDTH 0x03UL /**<The width of the bitfield controlling pull up/pull down selection*/
#define AFLHRy_WIDTH 0x0FUL /**<The width of a the bitfield controlling the alternate function MUX */

extern const uint32_t ACTIVE_GPIO_PINS; /** Defined in the appropriate .c config file. Prevents iteration
 	 	 	 	 	 	 	 	 	 	 	 over 64 pins when only a few are used*/

/**
 * Array of pointers to the GPIO Pin Mode Registers
 */
static volatile uint32_t *const GPIO_MODER[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE, (uint32_t *)GPIOB_BASE, (uint32_t *)GPIOC_BASE,
	(uint32_t *)GPIOD_BASE, (uint32_t *)GPIOE_BASE
};

/**
 * Array of pointers to the Output Type Registers
 */
static volatile uint32_t *const GPIO_OTYPER[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x04UL, (uint32_t *)GPIOB_BASE + 0x04UL,
	(uint32_t *)GPIOC_BASE + 0x04UL, (uint32_t *)GPIOD_BASE + 0x04UL,
	(uint32_t *)GPIOE_BASE + 0x04UL
};

/**
 *  Array of pointers to the Output Speed Registers
 */
static volatile uint32_t *const GPIO_OSPEEDR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x08UL, (uint32_t *)GPIOB_BASE + 0x08UL,
	(uint32_t *)GPIOC_BASE + 0x08UL, (uint32_t *)GPIOD_BASE + 0x08UL,
	(uint32_t *)GPIOE_BASE + 0x08UL
};

/**
 * Array of pointers to the Pull Up/Pull Down Registers
 */
static volatile uint32_t *const GPIO_PUPDR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x0CUL, (uint32_t *)GPIOB_BASE + 0x0CUL,
	(uint32_t *)GPIOC_BASE + 0x0CUL, (uint32_t *)GPIOD_BASE + 0x0CUL,
	(uint32_t *)GPIOE_BASE + 0x0CUL
};

/**
 * Array of pointers to the Input Data Registers
 */
static volatile uint32_t *const GPIO_IDR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x10UL, (uint32_t *)GPIOB_BASE + 0x10UL,
	(uint32_t *)GPIOC_BASE + 0x10UL, (uint32_t *)GPIOD_BASE + 0x10UL,
	(uint32_t *)GPIOE_BASE + 0x10UL
};

/**
 * Array of pointers to the Output Data Registers
 */
static volatile uint32_t *const GPIO_ODR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x14UL, (uint32_t *)GPIOB_BASE + 0x14UL,
	(uint32_t *)GPIOC_BASE + 0x14UL, (uint32_t *)GPIOD_BASE + 0x14UL,
	(uint32_t *)GPIOE_BASE + 0x14UL
};

/**
 * Array of pointers to the Bit Set and Reset Registers
 */
static volatile uint32_t *const GPIO_BSRR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x18UL, (uint32_t *)GPIOB_BASE + 0x18UL,
	(uint32_t *)GPIOC_BASE + 0x18UL, (uint32_t *)GPIOD_BASE + 0x18UL,
	(uint32_t *)GPIOE_BASE + 0x18UL
};

/**
 * Array of pointers to the Alernate Function LOW Registers
 */
static volatile uint32_t *const GPIO_AFRL[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x20UL, (uint32_t *)GPIOB_BASE + 0x20UL,
	(uint32_t *)GPIOC_BASE + 0x20UL, (uint32_t *)GPIOD_BASE + 0x20UL,
	(uint32_t *)GPIOE_BASE + 0x20UL
};

/**
 * Array of pointers to the Alternate Function HIGH Registers
 */
static volatile uint32_t *const GPIO_AFRH[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x24UL, (uint32_t *)GPIOB_BASE + 0x24UL,
	(uint32_t *)GPIOC_BASE + 0x24UL, (uint32_t *)GPIOD_BASE + 0x24UL,
	(uint32_t *)GPIOE_BASE + 0x24UL

};

/******************************************************************************
* Function: gpio_init()
*//**
* \b Description:
*
* This function is used to initialise the gpio based on the configuration table
*  defined in the gpio_stm32f411_config.c
*
* PRE-CONDITION: Configuration table needs to populated (sizeof > 0) <br>
* PRE-CONDITION: PINS_PER_PORT > 0 <br>
* PRE-CONDITION: NUMBER_OF_PORTS > 0 <br>
* PRE-CONDITION: The RCC clocks for all planned ports must be configured and enabled.
*
* POST-CONDITION: The GPIO is ready for use with all active pins set up.
*
* @param  		config_table is a pointer to the configuration table that contains
*				the initialisation structures for each planned gpio pin.
*
* @return 		void
*
* \b Example:
* @code
* 	const gpio_config_t *gpio_config = gpio_config_get();
*		gpio_init(gpio_config);
* @endcode
*
* @see gpio_config_get
*
* <br><b> - CHANGE HISTORY - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* </table><br><br>
* <hr>
*******************************************************************************/
void gpio_init(gpio_config_t *config_table)
{
	assert(PINS_PER_PORT > 0);
	assert(NUM_GPIO_PORTS > 0);
	assert(sizeof(config_table) > 0);

	uint32_t pin_number = 0;
	uint32_t port_number = 0;

	for (int i = 0; i < ACTIVE_GPIO_PINS; i++)
	{
		assert(config_table[i].pin < NUM_GPIO_PINS);
		pin_number = config_table[i].pin % PINS_PER_PORT;
		port_number = config_table[i].pin / PINS_PER_PORT;


		*GPIO_MODER[port_number] &= ~(MODERy_WIDTH << (pin_number*2));
	    *GPIO_MODER[port_number] |= config_table[i].mode << (pin_number*2);

	    *GPIO_OTYPER[port_number] &= ~(OTYPERy_WIDTH << pin_number);
	    *GPIO_OTYPER[port_number] |= config_table[i].output_type << pin_number;

	    *GPIO_OSPEEDR[port_number] &= ~(OSPEEDRy_WIDTH << (pin_number*2));
	    *GPIO_OSPEEDR[port_number] |= config_table[i].output_speed << (pin_number*2);

	    *GPIO_PUPDR[port_number] &= ~(PUPDRy_WIDTH << (pin_number*2));
	    *GPIO_PUPDR[port_number] |= config_table[i].resistor << (pin_number*2);

	    if (pin_number < (PINS_PER_PORT / 2))
	    {
	    	*GPIO_AFRL[port_number] &= ~(AFLHRy_WIDTH << (pin_number*4));
	    	*GPIO_AFRL[port_number] |= config_table[i].mux << (pin_number*4);
	    }
	    else
	    {
	    	*GPIO_AFRH[port_number] &= ~(AFLHRy_WIDTH << ((pin_number - PINS_PER_PORT/2)*4));
	    	*GPIO_AFRH[port_number] |= config_table[i].mux << ((pin_number - PINS_PER_PORT/2)*4);
	    }
	}
}

/******************************************************************************
 * Function : gpio_pin_read()
 *//**
 * \b Description:
 *
 * This function reads the current state of the selected pin, regardless of
 *	whether it is in input or output mode.
 *
 * PRE-CONDITION: gpio_init() has run successfully with the selected pin configured
 * 	within the config table
 *
 * POST-CONDITION: The return value contains the requested pin state in 1/0 form.
 *
 * @param		pin is a member of the gpio_pin_t enumeration typedef
 *
 * @return		gpio_pin_state_t containing the pin's current state
 *
 *
 * \b Example:
 *
 * @code
 * gpio_pin_state_t current_state = gpio_pin_read(GPIO_E_4);
 * @endcode
 *
 * @see gpio_pin_write
 * @see gpio_pin_toggle
 *
 *
 * <br><b> - CHANGE HISTORY - </b>
 *
 * <table align="left" style="width:800px">
 * <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
 * </table><br><br>
 * <hr>
 */
gpio_pin_state_t gpio_pin_read(gpio_pin_t pin)
{
	uint32_t pin_number = pin % PINS_PER_PORT;
	uint32_t port_number = pin / PINS_PER_PORT;
	uint32_t register_value = (*GPIO_IDR[port_number] & (0x01 << pin_number));
	if (register_value == 0)
	{
		return (GPIO_PIN_LOW);
	}
	else
	{
		return (GPIO_PIN_HIGH);
	}
}

/******************************************************************************
 * Function : gpio_pin_write()
 *//**
 * \b Description:
 *	Writes the desired state to the pin operating in output mode.
 *
 * PRE-CONDITION: gpio_init has been carried out and configured the pin in output mode.
 *
 * POST-CONDITION: The pin takes on the desired state.
 *
 * @param			pin is the pin whose state we wish to change
 *
 * @param 			value is the state which we wish the pin to assume
 *
 *
 * @return			void
 *
 * \b Example:
 *
 * @code
 * gpio_pin_write(GPIO_C_0, GPIO_PIN_HIGH);
 * @endcode
 *
 * @see gpio_pin_read
 * @see gpio_pin_toggle
 *
 * <br><b> - CHANGE HISTORY - </b>
 *
 * <table align="left" style="width:800px">
 * <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
 * </table><br><br>
 * <hr>
 *
 */
void gpio_pin_write(gpio_pin_t pin, gpio_pin_state_t value)
{
	uint32_t pin_number = pin % PINS_PER_PORT;
	uint32_t port_number = pin / PINS_PER_PORT;
	uint32_t MODER_state = *GPIO_MODER[port_number];
	MODER_state = (MODER_state & (MODERy_WIDTH << (pin_number*2))) >> (pin_number*2);
	assert(MODER_state == GPIO_OUTPUT);

	if (value == GPIO_PIN_LOW)
	{
		*GPIO_BSRR[port_number] |= (0x01UL << (PINS_PER_PORT + pin_number));
	}
	else
	{
		*GPIO_BSRR[port_number] |= (0x01UL << pin_number);
	}
}
/******************************************************************************
 * Function : gpio_pin_toggle()
 *//**
 * \b Description:
 *	Toggles the state of the desired pin operating in output mode.
 *
 * PRE-CONDITION: gpio_init has been carried out and configured the pin in output mode.
 *
 * POST-CONDITION: The pin takes on the opposite state.
 *
 * @param			pin is the pin whose state we wish to change
 *
 *
 * @return			void
 *
 * \b Example:
 *
 * @code
 * gpio_pin_toggle(GPIO_D_15);
 * @endcode
 *
 * @see gpio_pin_read
 * @see gpio_pin_write
 *
 * <br><b> - CHANGE HISTORY - </b>
 *
 * <table align="left" style="width:800px">
 * <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
 * </table><br><br>
 * <hr>
 *
 */
void gpio_pin_toggle(gpio_pin_t pin)
{
	gpio_pin_state_t pin_state;
	pin_state = (gpio_pin_state_t) gpio_pin_read(pin);
	if (pin_state == GPIO_PIN_HIGH)
	{
		gpio_pin_write(pin, GPIO_PIN_LOW);
	}
	else
	{
		gpio_pin_write(pin, GPIO_PIN_HIGH);
	}
}


/******************************************************************************
 * Function : gpio_register_write()
 *//**
 * \b Description:
 *	Writes a desired value to the selected GPIO register in the GPIO memory space.
 *	This function can be used within a greater super function to access more advanced
 *	features of the GPIO, such as the LOCK.
 *
 * PRE-CONDITION: The value of GPIO_register lies within the memory map defined region
 * 					dedicated to GPIO
 *
 * POST-CONDITION: The register has been modified.
 *
 * @param			gpio_register is the register whose contents we wish to change
 *
 * @param 			value is the state which we wish the pin to assume
 *
 *
 * @return			void
 *
 * \b Example:
 *
 * @code
 * gpio_register_write(GPIOD_BASE + 0x1CUL, 0xDEADBEEF);
 * @endcode
 *
 * @see gpio_register_read
 *
 * <br><b> - CHANGE HISTORY - </b>
 *
 * <table align="left" style="width:800px">
 * <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
 * </table><br><br>
 * <hr>
 *
 */
void gpio_register_write(uint32_t gpio_register, uint32_t value)
{
	assert(gpio_register >= GPIOA_BASE && gpio_register < CRC_BASE);

	*((uint32_t *)gpio_register) = value;
}

/******************************************************************************
 * Function : gpio_register_read()
 *//**
 * \b Description:
 *	Reads a the value of the selected GPIO register in the GPIO memory space.
 *	This function can be used within a greater super function to access more advanced
 *	features of the GPIO, such as the LOCK.
 *
 * PRE-CONDITION: The value of GPIO_register lies within the memory map defined region
 * 					dedicated to GPIO
 *
 * POST-CONDITION: The registers contents are returned
 *
 * @param			gpio_register whose contents we wish to read
  *
 * @return			uint32_t the value within the register
 *
 * \b Example:
 *
 * @code
 * uint32_t contents = gpio_register_read(GPIOD_BASE + 0x1CUL);
 * @endcode
 *
 * @see gpio_register_write
 *
 * <br><b> - CHANGE HISTORY - </b>
 *
 * <table align="left" style="width:800px">
 * <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
 * </table><br><br>
 * <hr>
 *
 */
uint32_t gpio_register_read(uint32_t gpio_register)
{
	assert(gpio_register >= GPIOA_BASE && gpio_register < CRC_BASE);

	uint32_t value = *((uint32_t *)gpio_register);
	return (value);
}
