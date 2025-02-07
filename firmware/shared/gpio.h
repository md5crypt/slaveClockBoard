#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx.h"

#define COMP_CSR_COMP1INNSEL_VREFINT         (0<<COMP_CSR_COMP1INNSEL_Pos)
#define COMP_CSR_COMP1INNSEL_PA0             (1<<COMP_CSR_COMP1INNSEL_Pos)
#define COMP_CSR_COMP1INNSEL_PA4             (2<<COMP_CSR_COMP1INNSEL_Pos)
#define COMP_CSR_COMP1INNSEL_PA5             (3<<COMP_CSR_COMP1INNSEL_Pos)

#define COMP_CSR_COMP2INPSEL_PA3             (0<<COMP_CSR_COMP2INPSEL_Pos)
#define COMP_CSR_COMP2INPSEL_PB4             (1<<COMP_CSR_COMP2INPSEL_Pos)
#define COMP_CSR_COMP2INPSEL_PB5             (2<<COMP_CSR_COMP2INPSEL_Pos)
#define COMP_CSR_COMP2INPSEL_PB6             (3<<COMP_CSR_COMP2INPSEL_Pos)
#define COMP_CSR_COMP2INPSEL_PB7             (4<<COMP_CSR_COMP2INPSEL_Pos)
#define COMP_CSR_COMP2INPSEL_PA7             (5<<COMP_CSR_COMP2INPSEL_Pos)

#define COMP_CSR_COMP2INNSEL_VREFINT         (0<<COMP_CSR_COMP2INNSEL_Pos)
#define COMP_CSR_COMP2INNSEL_PA2             (1<<COMP_CSR_COMP2INNSEL_Pos)
#define COMP_CSR_COMP2INNSEL_PA4             (2<<COMP_CSR_COMP2INNSEL_Pos)
#define COMP_CSR_COMP2INNSEL_PA5             (3<<COMP_CSR_COMP2INNSEL_Pos)
#define COMP_CSR_COMP2INNSEL_VREF25          (4<<COMP_CSR_COMP2INNSEL_Pos)
#define COMP_CSR_COMP2INNSEL_VREF50          (5<<COMP_CSR_COMP2INNSEL_Pos)
#define COMP_CSR_COMP2INNSEL_VREF75          (6<<COMP_CSR_COMP2INNSEL_Pos)
#define COMP_CSR_COMP2INNSEL_PB3             (7<<COMP_CSR_COMP2INNSEL_Pos)

#define RCC_CCIPR_LPTIM1SEL_APB              (0<<RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_LSI              (1<<RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_HSI16            (2<<RCC_CCIPR_LPTIM1SEL_Pos)
#define RCC_CCIPR_LPTIM1SEL_LSE              (3<<RCC_CCIPR_LPTIM1SEL_Pos)

#define TIM2_OR_ETR_RMP_COMP1                (7<<TIM2_OR_ETR_RMP_Pos)
#define TIM2_OR_ETR_RMP_COMP2                (6<<TIM2_OR_ETR_RMP_Pos)
#define TIM2_OR_ETR_RMP_LSE                  (5<<TIM2_OR_ETR_RMP_Pos)
#define TIM2_OR_ETR_RMP_HSI16                (3<<TIM2_OR_ETR_RMP_Pos)
#define TIM2_OR_ETR_RMP_GPIO                 (0<<TIM2_OR_ETR_RMP_Pos)

#define TIM21_OR_ETR_RMP_GPIO                (0<<TIM21_OR_ETR_RMP_Pos)
#define TIM21_OR_ETR_RMP_COMP2_OUT           (1<<TIM21_OR_ETR_RMP_Pos)
#define TIM21_OR_ETR_RMP_COMP1_OUT           (2<<TIM21_OR_ETR_RMP_Pos)
#define TIM21_OR_ETR_RMP_LSE                 (3<<TIM21_OR_ETR_RMP_Pos)

#define RCC_ICSCR_MSIRANGE_65k536            (0<<RCC_ICSCR_MSIRANGE_Pos)
#define RCC_ICSCR_MSIRANGE_131k072           (1<<RCC_ICSCR_MSIRANGE_Pos)
#define RCC_ICSCR_MSIRANGE_262k144           (2<<RCC_ICSCR_MSIRANGE_Pos)
#define RCC_ICSCR_MSIRANGE_524k288           (3<<RCC_ICSCR_MSIRANGE_Pos)
#define RCC_ICSCR_MSIRANGE_1M048             (4<<RCC_ICSCR_MSIRANGE_Pos)
#define RCC_ICSCR_MSIRANGE_2M097             (5<<RCC_ICSCR_MSIRANGE_Pos)
#define RCC_ICSCR_MSIRANGE_4M194             (6<<RCC_ICSCR_MSIRANGE_Pos)

#define PWR_CR_VOS_1V8                       (1<<PWR_CR_VOS_Pos)
#define PWR_CR_VOS_1V5                       (2<<PWR_CR_VOS_Pos)
#define PWR_CR_VOS_1V2                       (3<<PWR_CR_VOS_Pos)

#define FLASH_PDKEYR_PDKEY1                  0x04152637
#define FLASH_PDKEYR_PDKEY2                  0xFAFBFCFD

#define SYSCFG_CFGR1_MEM_MODE_MainFlash      (0<<SYSCFG_CFGR1_MEM_MODE_Pos)
#define SYSCFG_CFGR1_MEM_MODE_SystemFlash    (1<<SYSCFG_CFGR1_MEM_MODE_Pos)
#define SYSCFG_CFGR1_MEM_MODE_SRAM           (3<<SYSCFG_CFGR1_MEM_MODE_Pos)

#define GPIO_MODE_Input                  0
#define GPIO_MODE_Output                 1
#define GPIO_MODE_Alternate              2
#define GPIO_MODE_Analog                 3

#define GPIO_OTYPE_PushPull              0
#define GPIO_OTYPE_OpenDrain             1

#define GPIO_OSPEED_400kHz               0
#define GPIO_OSPEED_2Mhz                 1
#define GPIO_OSPEED_10Mhz                2
#define GPIO_OSPEED_40Mhz                3

#define GPIO_PUPD_None                   0
#define GPIO_PUPD_PullUp                 1
#define GPIO_PUPD_PullDown               2

#define GPIO_AFSEL_AF0                   0
#define GPIO_AFSEL_AF1                   1
#define GPIO_AFSEL_AF2                   2
#define GPIO_AFSEL_AF3                   3
#define GPIO_AFSEL_AF4                   4
#define GPIO_AFSEL_AF5                   5
#define GPIO_AFSEL_AF6                   6
#define GPIO_AFSEL_AF7                   7

#define GPIO_PORT_NUM(x)                 (x >> 4)
#define GPIO_PIN_NUM(x)                  (x & 0x0F)

static inline GPIO_TypeDef* gpio_port(uint32_t pin) {
	return (GPIO_TypeDef*)(GPIOA_BASE + (GPIO_PORT_NUM(pin) * 0x400));
}

static inline void gpio_set_mode(uint32_t pin, uint32_t mode) {
	GPIO_TypeDef* port = gpio_port(pin);
	port->MODER = (port->MODER & ~(GPIO_MODER_MODE0_Msk << (2 * GPIO_PIN_NUM(pin)))) | (mode << (2 * GPIO_PIN_NUM(pin)));
}

static inline void gpio_set_otype(uint32_t pin, uint32_t otype) {
	GPIO_TypeDef* port = gpio_port(pin);
	port->OTYPER = (port->OTYPER & ~(1 << GPIO_PIN_NUM(pin))) | (otype << GPIO_PIN_NUM(pin));
}

static inline void gpio_set_pupd(uint32_t pin, uint32_t pupd) {
	GPIO_TypeDef* port = gpio_port(pin);
	port->PUPDR = (port->PUPDR & ~(GPIO_PUPDR_PUPD0_Msk << (2 * GPIO_PIN_NUM(pin)))) | (pupd << (2 * GPIO_PIN_NUM(pin)));
}

static inline void gpio_set_ospeed(uint32_t pin, uint32_t ospeed) {
	GPIO_TypeDef* port = gpio_port(pin);
	port->OSPEEDR = (port->OSPEEDR & ~(GPIO_OSPEEDER_OSPEED0_Msk << (2 * GPIO_PIN_NUM(pin)))) | (ospeed << (2 * GPIO_PIN_NUM(pin)));
}

static inline void gpio_set_af(uint32_t pin, uint32_t af) {
	GPIO_TypeDef* port = gpio_port(pin);
	if (GPIO_PIN_NUM(pin) > 7) {
		port->AFR[1] = (port->AFR[1] & ~(GPIO_AFRH_AFRH0_Msk << (4 * (GPIO_PIN_NUM(pin) - 8)))) | (af << (4 * (GPIO_PIN_NUM(pin) - 8)));
	} else {
		port->AFR[0] = (port->AFR[0] & ~(GPIO_AFRL_AFRL0_Msk << (4 * GPIO_PIN_NUM(pin)))) | (af << (4 * GPIO_PIN_NUM(pin)));
	}
}

void gpio_configure(uint32_t pin, uint32_t mode, uint32_t otype, uint32_t pupd, uint32_t ospeed, uint32_t af);

void gpio_output(uint32_t pin);

void gpio_input(uint32_t pin, uint32_t pupd);

void gpio_analog(uint32_t pin);

static inline void gpio_set(uint32_t pin) {
	gpio_port(pin)->BSRR = GPIO_BSRR_BS_0 << GPIO_PIN_NUM(pin);
}

static inline void gpio_clear(uint32_t pin) {
	gpio_port(pin)->BSRR = GPIO_BSRR_BR_0 << GPIO_PIN_NUM(pin);
}

static inline bool gpio_read(uint32_t pin) {
	return gpio_port(pin)->IDR & (GPIO_IDR_ID0 << GPIO_PIN_NUM(pin));
}

