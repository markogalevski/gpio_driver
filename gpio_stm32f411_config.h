#ifndef _GPIO_STM32F4XX_CONFIG_H
#define _GPIO_STM32F4XX_CONFIG_H
#include <stdint.h>
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

typedef enum
{
	GPIO_PIN_LOW = 0UL,
	GPIO_PIN_HIGH = 1UL
}gpio_pin_state_t;

typedef enum
{
  GPIO_INPUT,
  GPIO_OUTPUT,
  GPIO_ALTERNATE_FUNCTION,
  GPIO_ANALOG,
  GPIO_MAX_MODE_OPTIONS
}gpio_mode_t;

typedef enum
{
  GPIO_NO_RESISTOR,
  GPIO_PULL_UP,
  GPIO_PULL_DOWN,
  GPIO_MAX_RESISTOR_OPTIONS
}gpio_resistor_t;

typedef enum
{
  GPIO_PUSH_PULL,
  GPIO_OPEN_DRAIN,
  GPIO_MAX_OUTPUT_OPTIONS
}gpio_output_t;

typedef enum
{
  GPIO_LOW_SPEED,
  GPIO_MED_SPEED,
  GPIO_FAST_SPEED,
  GPIO_HIGH_SPEED,
  GPIO_MAX_SPEED_OPTIONS
}gpio_speed_t;

typedef enum
{
  GPIO_AF_0, GPIO_AF_1, GPIO_AF_2, GPIO_AF_3,
  GPIO_AF_4, GPIO_AF_5, GPIO_AF_6, GPIO_AF_7,
  GPIO_AF_8, GPIO_AF_9, GPIO_AF_10, GPIO_AF_11,
  GPIO_AF_12, GPIO_AF_13, GPIO_AF_14, GPIO_AF_15,
  GPIO_MAX_AF_OPTIONS
}gpio_mux_t;


typedef struct
{
  gpio_pin_t pin;
  gpio_mode_t mode;
  gpio_resistor_t resistor;
  gpio_output_t output;
  gpio_speed_t speed;
  gpio_mux_t mux;
}gpio_config_t;

const gpio_config_t * gpio_config_get(void);

#endif /* gpio_config for stm32f4xx series */
