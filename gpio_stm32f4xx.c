#include "gpio_interface.h"
#include "stm32f411xe.h"

#define PINS_PER_PORT 16
/* covers A, B, C, D, E (as per 8.1 in STM32F411xC/E Ref. Manual, but no H */
#define NUM_GPIO_PORTS NUM_GPIO_PINS/PINS_PER_PORT

extern const uint32_t ACTIVE_GPIO_PINS;

static volatile uint32_t *const GPIO_MODER[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE, (uint32_t *)GPIOB_BASE, (uint32_t *)GPIOC_BASE,
	(uint32_t *)GPIOD_BASE, (uint32_t *)GPIOE_BASE
};

static volatile uint32_t *const GPIO_OTYPER[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x04UL, (uint32_t *)GPIOB_BASE + 0x04UL,
	(uint32_t *)GPIOC_BASE + 0x04UL, (uint32_t *)GPIOD_BASE + 0x04UL,
	(uint32_t *)GPIOE_BASE + 0x04UL
};

static volatile uint32_t *const GPIO_OSPEEDR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x08UL, (uint32_t *)GPIOB_BASE + 0x08UL,
	(uint32_t *)GPIOC_BASE + 0x08UL, (uint32_t *)GPIOD_BASE + 0x08UL,
	(uint32_t *)GPIOE_BASE + 0x08UL
};

static volatile uint32_t *const GPIO_PUPDR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x0CUL, (uint32_t *)GPIOB_BASE + 0x0CUL,
	(uint32_t *)GPIOC_BASE + 0x0CUL, (uint32_t *)GPIOD_BASE + 0x0CUL,
	(uint32_t *)GPIOE_BASE + 0x0CUL
};

static volatile uint32_t *const GPIO_IDR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x10UL, (uint32_t *)GPIOB_BASE + 0x10UL,
	(uint32_t *)GPIOC_BASE + 0x10UL, (uint32_t *)GPIOD_BASE + 0x10UL,
	(uint32_t *)GPIOE_BASE + 0x10UL
};

static volatile uint32_t *const GPIO_ODR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x14UL, (uint32_t *)GPIOB_BASE + 0x14UL,
	(uint32_t *)GPIOC_BASE + 0x14UL, (uint32_t *)GPIOD_BASE + 0x14UL,
	(uint32_t *)GPIOE_BASE + 0x14UL
};

static volatile uint32_t *const GPIO_BSRR[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x18UL, (uint32_t *)GPIOB_BASE + 0x18UL,
	(uint32_t *)GPIOC_BASE + 0x18UL, (uint32_t *)GPIOD_BASE + 0x18UL,
	(uint32_t *)GPIOE_BASE + 0x18UL
};

static volatile uint32_t *const GPIO_AFRL[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x20UL, (uint32_t *)GPIOB_BASE + 0x20UL,
	(uint32_t *)GPIOC_BASE + 0x20UL, (uint32_t *)GPIOD_BASE + 0x20UL,
	(uint32_t *)GPIOE_BASE + 0x20UL
};

static volatile uint32_t *const GPIO_AFRH[NUM_GPIO_PORTS] =
{
	(uint32_t *)GPIOA_BASE + 0x24UL, (uint32_t *)GPIOB_BASE + 0x24UL,
	(uint32_t *)GPIOC_BASE + 0x24UL, (uint32_t *)GPIOD_BASE + 0x24UL,
	(uint32_t *)GPIOE_BASE + 0x24UL

};

void gpio_init(gpio_config_t * config_table)
{
	uint32_t pin_number = 0;
	uint32_t port_number = 0;

	for (int i = 0; i < ACTIVE_GPIO_PINS; i++)
	{
		assert(config_table[i].pin < NUM_GPIO_PINS);
		pin_number = config_table[i].pin % PINS_PER_PORT;
		port_number = config_table[i].pin / PINS_PER_PORT;

		*GPIO_MODER[port_number] &= ~(0x03UL << (pin_number*2));
	    *GPIO_MODER[port_number] |= config_table[i].mode << (pin_number*2);

	    *GPIO_OTYPER[port_number] |= config_table[i].output << pin_number;

	    *GPIO_OSPEEDR[port_number] &= ~(0x03UL << (pin_number*2));
	    *GPIO_OSPEEDR[port_number] |= config_table[i].speed << (pin_number*2);

	    *GPIO_PUPDR[port_number] &= ~(0x03UL << (pin_number*2));
	    *GPIO_PUPDR[port_number] |= config_table[i].resistor << (pin_number*2);

	    if (pin_number < (PINS_PER_PORT / 2))
	    {
	    	*GPIO_AFRL[port_number] |= config_table[i].mux << (pin_number*4);
	    }
	    else
	    {
	    	*GPIO_AFRH[port_number] |= config_table[i].mux << ((pin_number - PINS_PER_PORT/2)*4);
	    }

	}
}

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

void gpio_pin_write(gpio_pin_t pin, gpio_pin_state_t value)
{
	uint32_t pin_number = pin % PINS_PER_PORT;
	uint32_t port_number = pin / PINS_PER_PORT;
	if (value == GPIO_PIN_LOW)
	{
		*GPIO_BSRR[port_number] |= (0x01UL << (PINS_PER_PORT + pin_number));
	}
	else
	{
		*GPIO_BSRR[port_number] |= (0x01UL << pin_number);
	}
}

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

void gpio_register_write(uint32_t gpio_register, uint32_t value)
{
	assert(gpio_register >= GPIOA_BASE && gpio_register < CRC_BASE);

	*((uint32_t *)gpio_register) = value;
}

uint32_t gpio_register_read(uint32_t gpio_register)
{
	assert(gpio_register >= GPIOA_BASE && gpio_register < CRC_BASE);

	uint32_t value = *((uint32_t *)gpio_register);
	return (value);
}
