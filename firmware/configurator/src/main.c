#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32l0xx.h"
#include "gpio.h"
#include "button.h"
#include "led.h"
#include "clock_config.h"

// #define DEBUG

#define BOOT_ADDR    0x08000800

#define PIN_BUTTON     0
#define PIN_LED        10

#define PIN_DEBUG      14

#define EEPROM_DATA  ((uint32_t*)0x08080000)
#define EEPROM_MAGIC 0xB16B00B5

#define INPUT_TIMEOUT 60000
#define BUTTON_COUNT  3

#define LED_VALUE_HIGH 100
#define LED_VALUE_LOW  10

enum {
	REG_PERIOD,          // 0000
	REG_PULSE,           // 0001
	REG_CHARGE,          // 0010
	REG_FIX,             // 0011
	REG_FIX_SIGN,        // 0100
	REG_OUTPUT_VOLTAGE,  // 0101
	REG_INPUT_VOLTAGE,   // 0110
	REG_POLARITY,        // 0111
	__REG_COUNT,

	CMD_RESET = 0b1101,
	CMD_DISCARD = 0b1110,
	CMD_SAVE = 0b1111
};

enum {
	BUTTON_0,
	BUTTON_1,
	BUTTON_M,
};

typedef struct {
	uint32_t magic;
	clock_config_t config;
	union {
		uint32_t raw[__REG_COUNT];
		clock_config_input_t data;
	} registers;
} eeprom_t;

static const led_command_t led_message_error[] = {
	LED_CMD(LOCK, 1),
	LED_CMD(WAIT, 500),
	LED_CMD(MOV, 4),
	LED_CMD(SET, LED_VALUE_HIGH),
	LED_CMD(WAIT, 200),
	LED_CMD(SET, 0),
	LED_CMD(WAIT, 200),
	LED_CMD(ADD, -1),
	LED_CMD(BRANCH, -5),
	LED_CMD(WAIT, 1000),
	LED_CMD(LOCK, 0),
	LED_CMD(WAIT, -1)
};

static const led_command_t led_message_ok[] = {
	LED_CMD(LOCK, 1),
	LED_CMD(WAIT, 500),
	LED_CMD(MOV, 2),
	LED_CMD(SET, LED_VALUE_HIGH),
	LED_CMD(WAIT, 200),
	LED_CMD(SET, 0),
	LED_CMD(WAIT, 200),
	LED_CMD(ADD, -1),
	LED_CMD(BRANCH, -5),
	LED_CMD(WAIT, 1000),
	LED_CMD(LOCK, 0),
	LED_CMD(WAIT, -1)
};

static const led_command_t led_message_idle[] = {
	LED_CMD(SET, LED_VALUE_LOW),
	LED_CMD(WAIT, 30),
	LED_CMD(SET, 0),
	LED_CMD(WAIT, 30),
	LED_CMD(JUMP, -4)
};

static const led_command_t led_message_cancel[] = {
	LED_CMD(SET, LED_VALUE_HIGH),
	LED_CMD(WAIT, 50),
	LED_CMD(SET, 0),
	LED_CMD(WAIT, 50),
	LED_CMD(JUMP, -4)
};

static const led_command_t led_message_button_active[] = {
	LED_CMD(SET, LED_VALUE_HIGH),
	LED_CMD(WAIT, -1)
};

static const led_command_t led_message_read[] = {
	LED_CMD(LOCK, 1),
	LED_CMD(WAIT, 500),
	LED_CMD(SET, LED_VALUE_HIGH),
	LED_CMD(WAIT, 200),
	LED_CMD(SET, 0),
	LED_CMD(WAIT, 200),
	LED_CMD(SET, LED_VALUE_HIGH),
	LED_CMD(WAIT, 200),
	LED_CMD(SET, 0),
	LED_CMD(WAIT, 2000),
	LED_CMD(ADD, -1),
	LED_CMD(SEQ, 0),
	LED_CMD(WAIT, 500),
	LED_CMD(SET, 0),
	LED_CMD(WAIT, 1000),
	LED_CMD(BRANCH, -5),
	LED_CMD(WAIT, 500),
	LED_CMD(LOCK, 0),
	LED_CMD(WAIT, -1)
};

static led_data_t led_data;
static eeprom_t eeprom_data;
static button_t button_data[BUTTON_COUNT];
static uint8_t led_sequence[32];
static bool scheduled_reset;

static struct {
	uint32_t timer;
	uint32_t bitmask;
	uint32_t size;
	bool active;
} input;

typedef struct {
	uint32_t value;
	uint8_t cmd;
	uint8_t has_value;
} user_input_t;

static void mcu_reset() {
	RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
	WWDG->CR = WWDG_CR_WDGA;
}

static void eeprom_read() {
	if (EEPROM_DATA[0] == EEPROM_MAGIC) {
		for (uint32_t i = 0; i < sizeof(eeprom_t) / 4; i += 1) {
			((uint32_t*)&eeprom_data)[i] = EEPROM_DATA[i];
		}
	} else {
		eeprom_data.registers.data = clock_config_default_input;
	}
}

static void eeprom_write() {
	FLASH->PEKEYR = 0x89ABCDEFU;
	FLASH->PEKEYR = 0x02030405U;
	for (uint32_t i = 0; i < sizeof(eeprom_t) / 4; i += 1) {
		while ((FLASH->SR & (FLASH_SR_READY | FLASH_SR_BSY)) != FLASH_SR_READY);
		*(EEPROM_DATA + i) = ((uint32_t*)&eeprom_data)[i];
	}
	while ((FLASH->SR & (FLASH_SR_READY | FLASH_SR_BSY)) != FLASH_SR_READY);
}

static uint32_t led_message_read_craete(uint8_t* data, uint32_t value) {
	uint32_t cnt = 0;
	do {
		uint32_t digit = value % 10;
		value /= 10;
		for (uint32_t i = 0; i < 4; i += 1) {
			data[cnt] = (digit & 1) ? LED_VALUE_HIGH : LED_VALUE_LOW;
			cnt += 1;
			digit >>= 1;
		}
	} while(value > 0);
	return cnt;
}

static bool input_process(user_input_t* result, uint32_t bitmask, uint32_t size) {
	if ((size & 3) != 0 || size == 0) {
		return false;
	}
	bitmask <<= 32 - size;
	result->cmd = bitmask >> 28;
	bitmask <<= 4;
	if (size == 4) {
		result->has_value = false;
		return true;
	}
	result->has_value = true;
	uint32_t value = 0;
	for (uint32_t i = 1; i < (size / 4); i += 1) {
		uint32_t digit = bitmask >> 28;
		if (digit > 9) {
			return false;
		}
		value = (value * 10) + digit;
		bitmask <<= 4;
	}
	result->value = value;
	return true;
}

static void input_reset() {
	input.timer = 0;
	input.bitmask = 0;
	input.size = 0;
	input.active = false;
}

static void input_handler() {
	user_input_t data;
	if (!input_process(&data, input.bitmask, input.size)) {
		led_set(&led_data, led_message_error);
		return;
	}
	if (data.cmd < __REG_COUNT) {
		if (!data.has_value) {
			led_set_squence(
				&led_data,
				led_message_read,
				led_sequence,
				led_message_read_craete(led_sequence, eeprom_data.registers.raw[data.cmd])
			);
		} else {
			uint32_t old_value = eeprom_data.registers.raw[data.cmd];
			eeprom_data.registers.raw[data.cmd] = data.value;
			if (clock_config_input_verify(&eeprom_data.registers.data)) {
				led_set(&led_data, led_message_ok);
			} else {
				eeprom_data.registers.raw[data.cmd] = old_value;
				led_set(&led_data, led_message_error);
			}
		}
	} else {
		switch (data.cmd) {
			case CMD_RESET:
				if (data.has_value) {
					led_set(&led_data, led_message_error);
				} else {
					eeprom_data.registers.data = clock_config_default_input;
					led_set(&led_data, led_message_ok);
				}
				break;
			case CMD_DISCARD:
				if (data.has_value) {
					led_set(&led_data, led_message_error);
				} else {
					scheduled_reset = true;
					led_set(&led_data, led_message_ok);
				}
				break;
			case CMD_SAVE:
				if (data.has_value) {
					led_set(&led_data, led_message_error);
				} else {
					eeprom_data.magic = EEPROM_MAGIC;
					clock_config_compute(&eeprom_data.registers.data, &eeprom_data.config);
					eeprom_write();
					scheduled_reset = true;
					led_set(&led_data, led_message_ok);
				}
				break;
			default:
				led_set(&led_data, led_message_error);
		}
	}
}

void SysTick_Handler() {
	if (led_update(&led_data)) {
		for (uint32_t i = 0; i < BUTTON_COUNT; i += 1) {
			button_reset(button_data + i);
		}
		return;
	}
	if (scheduled_reset) {
		mcu_reset();
	}
	bool pressed = false;
	for (uint32_t i = 0; i < BUTTON_COUNT; i += 1) {
		pressed |= button_update(button_data + i);
	}
	if (button_data[BUTTON_M].flags & BUTTON_FLAG_LONG_PRESS) {
		if (button_data[BUTTON_M].flags & BUTTON_FLAG_PRESSED) {
			button_data[BUTTON_M].flags &= ~(BUTTON_FLAG_PRESSED | BUTTON_FLAG_LONG_PRESS);
			led_set(&led_data, led_message_error);
			input_reset();
		} else {
			led_set(&led_data, led_message_cancel);
		}
		return;
	}
	if (input.active) {
		input.timer += 1;
		if (input.timer > INPUT_TIMEOUT) {
			input_reset();
			led_set(&led_data, led_message_error);
		}
		if (button_data[BUTTON_M].flags & BUTTON_FLAG_PRESSED) {
			input_handler();
			input_reset();
		} else if (button_data[BUTTON_0].flags & BUTTON_FLAG_PRESSED) {
			input.bitmask <<= 1;
			input.size += 1;
			led_set(&led_data, NULL);
		} else if (button_data[BUTTON_1].flags & BUTTON_FLAG_PRESSED) {
			input.bitmask <<= 1;
			input.bitmask |= 1;
			input.size += 1;
			led_set(&led_data, NULL);
		} else {
			if (pressed) {
				input.timer = 0;
			}
			led_set(&led_data, pressed ? led_message_button_active : NULL);
		}
	} else {
		if (button_data[BUTTON_M].flags & BUTTON_FLAG_PRESSED) {
			input.active = true;
			led_set(&led_data, NULL);
		} else {
			led_set(&led_data, pressed ? led_message_button_active : led_message_idle);
		}
	}
	button_data[BUTTON_0].flags &= ~(BUTTON_FLAG_PRESSED | BUTTON_FLAG_LONG_PRESS);
	button_data[BUTTON_1].flags &= ~(BUTTON_FLAG_PRESSED | BUTTON_FLAG_LONG_PRESS);
	button_data[BUTTON_M].flags &= ~(BUTTON_FLAG_PRESSED | BUTTON_FLAG_LONG_PRESS);
}

__attribute__((noreturn)) int main(){
#ifdef DEBUG
	SCB->VTOR = BOOT_ADDR;
#endif

	// enable peripherals clocks
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_WWDGEN | RCC_APB1ENR_LPTIM1EN;
	RCC->APB1SMENR |= RCC_APB1SMENR_LPTIM1SMEN;
	RCC->APB2ENR |= RCC_APB2ENR_TIM21EN | RCC_APB2ENR_ADC1EN;

	// set clock speed and core voltage
	RCC->ICSCR = RCC_ICSCR_MSIRANGE_4M194;
	PWR->CR = PWR_CR_VOS_1V5;
	while(PWR->CSR&PWR_CSR_VOSF);

	// start the LSE clock
	PWR->CR |= PWR_CR_DBP;
	RCC->CSR |= RCC_CSR_LSEON | RCC_CSR_LSEDRV_0;
	PWR->CR ^= PWR_CR_DBP;
	while(!(RCC->CSR&RCC_CSR_LSERDY));

	// set LSE as LPTIM1 clock
	RCC->CCIPR = RCC_CCIPR_LPTIM1SEL_LSE;

	// configure LPTIM1
	LPTIM1->CR = LPTIM_CR_ENABLE;
	LPTIM1->CMP = 32768 - 2;
	LPTIM1->ARR = 32768 - 1;

	// start timer
	while (!(LPTIM1->ISR & (LPTIM_ISR_ARROK | LPTIM_ISR_CMPOK)));
	LPTIM1->CR |= LPTIM_CR_CNTSTRT;

	// configure led pwm timer
	gpio_set_mode(PIN_LED, GPIO_MODE_Output);

	gpio_set_mode(PIN_LED, GPIO_MODE_Alternate);
	TIM21->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM21->CCER |= TIM_CCER_CC1E;
	TIM21->CR1 |= TIM_CR1_CEN;
	TIM21->ARR = 99;
	TIM21->CCR1 = 100;

	// configure buttons
	gpio_input(PIN_BUTTON, GPIO_PUPD_None);

	// wait for M button to become unpressed
	for (uint32_t i = 0; i < 250000; i += 1) {
		if (!gpio_read(PIN_BUTTON)) {
			i = 0;
		}
	}

	// switch over to analog
	gpio_analog(PIN_BUTTON);

#ifndef DEBUG
	// configure pps pwm output
	gpio_configure(PIN_DEBUG, GPIO_MODE_Alternate, GPIO_OTYPE_PushPull, GPIO_PUPD_None, GPIO_OSPEED_40Mhz, GPIO_AFSEL_AF1);
#endif

	button_data[BUTTON_0].value = 170;
	button_data[BUTTON_1].value = 128;
	button_data[BUTTON_M].value = 0;

	// read eeprom configuration
	eeprom_read();

	// configure ADC
	ADC1->CFGR1 |= ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD | ADC_CFGR1_RES_1;
	ADC1->CFGR2 |= (0b101 << ADC_CFGR2_OVSR_Pos) | (0b0110 << ADC_CFGR2_OVSS_Pos) | ADC_CFGR2_OVSE | (0b01 << ADC_CFGR2_CKMODE_Pos);
	ADC1->SMPR |= 0b010 << ADC_SMPR_SMP_Pos;
	ADC1->CHSELR |= ADC_CHSELR_CHSEL0;

	// calibrate ADC
	ADC1->CR |= ADC_CR_ADCAL;
	while (ADC1->CR & ADC_CR_ADCAL) {
		// wait for calibration to complete
	}

	// start ADC
	ADC1->CR |= ADC_CR_ADEN | ADC_CR_ADSTART;

	while (!(ADC1->ISR & ADC_ISR_EOC)) {
		// wait for first ADC result
	}

	NVIC_SetPriority(SysTick_IRQn, 255);
	SysTick->LOAD = 4194 - 1;
	SysTick->VAL = 4194 - 1;
	SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
	while(1) {
		__WFE();
	}
}
