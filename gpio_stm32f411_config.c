#include <gpio_stm32f411_config.h>

static const gpio_config_t gpio_config_table[] = {
	//PIN		//MODE			//RESISTOR			//OUTPUT		//SPEED 		//MUX
	//e.g. for i2s
{	GPIO_A_15, 	GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR,  GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_6},
{	GPIO_B_3, GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR, GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_6},
{	GPIO_B_5, GPIO_ALTERNATE_FUNCTION, GPIO_NO_RESISTOR, GPIO_PUSH_PULL, GPIO_HIGH_SPEED, GPIO_AF_6}
};
const uint32_t ACTIVE_GPIO_PINS = sizeof(gpio_config_table)/sizeof(gpio_config_t);

const gpio_config_t *gpio_config_get(void)
{
	return (gpio_config_table);
}
