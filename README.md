# GPIO_DRIVER 

After reading Jacob Beningo's book, Reusable Firmware Development, I've decided to begin the arduous process of building my own easily portable HAL to use for future projects. It feels as if all I've been developing for ages at this point is drivers.

The general principle is as follows:
* A general gpio_interface.h defines the api which will be exposed to applications. It will be this file which is included by the application. It is designed in a way to be 100% non-platform dependent. Changes required may be the modification of uint32_t types to uint16_t to match an architecture.

* The micrcontroller specific gpio_stm32f4xx.c file contains an MCU specific implementation of the peripheral. Accompanying it are a config .c/.h pair. These define a table of init structures for each instance of the peripheral, as well as all relevant typedefs. These files will need to be changed to port the driver. Time to port a gpio driver seems to be less than a day's worth of work.

* To port the driver: simply prepare the MCU specific c and config files and set gpio_interface.h to include the appropriate xxxxx_config.h file, and exchange the source files.


