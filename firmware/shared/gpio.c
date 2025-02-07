#include "gpio.h"

void gpio_configure(uint32_t pin, uint32_t mode, uint32_t otype, uint32_t pupd, uint32_t ospeed, uint32_t af) {
	gpio_set_mode(pin, mode);
	gpio_set_otype(pin, otype);
	gpio_set_pupd(pin, pupd);
	gpio_set_ospeed(pin, ospeed);
	gpio_set_af(pin, af);
}

void gpio_output(uint32_t pin) {
	gpio_configure(pin, GPIO_MODE_Output, GPIO_OTYPE_PushPull, GPIO_PUPD_None, GPIO_OSPEED_400kHz, GPIO_AFSEL_AF0);
}

void gpio_input(uint32_t pin, uint32_t pupd) {
	gpio_configure(pin, GPIO_MODE_Input, GPIO_OTYPE_OpenDrain, pupd, GPIO_OSPEED_400kHz, GPIO_AFSEL_AF0);
}

void gpio_analog(uint32_t pin) {
	gpio_configure(pin, GPIO_MODE_Analog, GPIO_OTYPE_OpenDrain, GPIO_PUPD_None, GPIO_OSPEED_2Mhz, GPIO_AFSEL_AF0);
}
