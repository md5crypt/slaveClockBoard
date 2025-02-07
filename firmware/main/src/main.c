#include <stdint.h>
#include <stdbool.h>
#include "stm32l0xx.h"
#include "gpio.h"

#define PIN_BUTTON     0
#define PIN_LED        10
#define PIN_GATE_A     6
#define PIN_GATE_B     5

#define PIN_BOOST_LOW  7
#define PIN_BOOST_HIGH 1 + 0x10
#define PIN_CUTOFF     9
#define PIN_DEBUG      14

#define PIN_COMP_HIGH  1
#define PIN_COMP_LOW   3

#define BOOT_ADDR    0x08000800
#define EEPROM_DATA  ((uint32_t*)0x08080000)
#define EEPROM_MAGIC 0xB16B00B5

#define INPUT_VOLTAGE_1_5   0
#define INPUT_VOLTAGE_3_0   1

#define OUTPUT_VOLTAGE_LOW  0
#define OUTPUT_VOLTAGE_HIGH 1

#define POLARITY_SINGLE 0
#define POLARITY_DOUBLE 1

typedef enum {
	STATE_INITIAL,
	STATE_IDLE,
	STATE_CHARGE,
	STATE_PULSE
} state_t;

typedef struct {
	uint32_t reload;
	uint32_t pulse;
	uint32_t charge;
	uint32_t prescaler;
	uint32_t fix_period;
	int32_t fix_offset;
	int32_t fix_sign;
	uint32_t fix_precision;
	uint32_t output_voltage;
	uint32_t input_voltage;
	uint32_t polarity;
} clock_config_t;

static clock_config_t config;
static uint32_t polarity;
static uint32_t comp_enabled;
static uint32_t button_offset;
static uint32_t charge_time;

static inline void mcu_reset() {
	RCC->APB1ENR |= RCC_APB1ENR_WWDGEN;
	WWDG->CR = WWDG_CR_WDGA;
}

static inline void stop(bool ulp, bool fwu){
	PWR->CR =
		PWR_CR_DSEEKOFF |         /* do not enable flash after wake-up */
		(fwu ? PWR_CR_FWU : 0) |  /* fast wake-up */
		(ulp ? PWR_CR_ULP : 0) |  /* disable vref in sleep */
		PWR_CR_LPSDSR |           /* enable low-power sleep */
		PWR_CR_VOS_1V5 |
		PWR_CR_CWUF;              /* clear wake-up flag */
	__WFE();                      /* sleep */
}

static void comp1_start() {
	// enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// enable COMP1
	if (config.output_voltage == OUTPUT_VOLTAGE_HIGH) {
		COMP1->CSR = COMP_CSR_COMP1EN;
	} else {
		COMP1->CSR = COMP_CSR_COMP1EN | COMP_CSR_COMP1WM;
	}
	// unmask COMP1 event
	EXTI->EMR |= EXTI_EMR_EM21;
	while (!(SYSCFG->CFGR3 & SYSCFG_CFGR3_VREFINT_RDYF)) {
		// wait
	}
	// clear COMP1 pending event flag
	EXTI->PR = EXTI_PR_PIF21;
	if (COMP1->CSR & COMP_CSR_COMP1VALUE) {
		EXTI->SWIER = EXTI_SWIER_SWI21;
	}
	comp_enabled = true;
}

static void comp1_stop() {
	// disable COMP1
	COMP1->CSR = 0;
	// disable SYSCFG clock
	RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;
	// mask COMP1 event
	EXTI->EMR &= ~EXTI_EMR_EM21;
	comp_enabled = false;
}

static int32_t oscillator_calibration() {
	static uint32_t fix_cnt = 0;
	int32_t shift = config.fix_offset;
	if (config.fix_period > 0) {
		fix_cnt += config.fix_precision;
		if (fix_cnt >= config.fix_period) {
			fix_cnt -= config.fix_period;
			shift += config.fix_sign;
		}
	}
	return shift;
}

static void set_output() {
	if (config.polarity == POLARITY_SINGLE) {
		gpio_set(PIN_GATE_A);
	} else {
		if (polarity) {
			gpio_set(PIN_GATE_A);
		} else {
			gpio_set(PIN_GATE_B);
		}
		polarity ^= 1;
	}
}

static state_t next_state_single_cell(state_t state) {
	switch(state) {
		default:
		case STATE_INITIAL:
			LPTIM1->ARR = 1;
			return STATE_IDLE;
		case STATE_IDLE:
			gpio_clear(PIN_GATE_A);
			gpio_clear(PIN_GATE_B);
			gpio_set(PIN_BOOST_LOW);
			comp1_start();
			LPTIM1->ARR = config.reload - (config.pulse + config.charge + button_offset + 1) + oscillator_calibration();
			button_offset = 0;
			return STATE_CHARGE;
		case STATE_CHARGE:
			if (config.charge > 0) {
				gpio_set(PIN_BOOST_LOW);
				LPTIM1->ARR = config.charge - 1;
				return STATE_PULSE;
			}
			/* no break */
		case STATE_PULSE:
			gpio_set(PIN_CUTOFF);
			gpio_clear(PIN_BOOST_LOW);
			set_output();
			LPTIM1->ARR = config.pulse - 1;
			return STATE_IDLE;
	}
}

static state_t next_state_dual_cell(state_t state) {
	switch(state) {
		default:
		case STATE_INITIAL:
			LPTIM1->ARR = 1;
			return STATE_CHARGE;
		case STATE_IDLE:
			gpio_clear(PIN_GATE_A);
			gpio_clear(PIN_GATE_B);
			LPTIM1->ARR = config.reload - (config.pulse + config.charge + button_offset + charge_time + 1) + oscillator_calibration();
			button_offset = 0;
			return STATE_CHARGE;
		case STATE_CHARGE:
			comp1_start();
			gpio_set(PIN_CUTOFF);
			gpio_clear(PIN_BOOST_HIGH);
			LPTIM1->ARR = 0xFFFF;
			return STATE_PULSE;
		case STATE_PULSE:
			if (comp_enabled) {
				comp1_stop();
			}
			gpio_set(PIN_BOOST_HIGH);
			gpio_clear(PIN_CUTOFF);
			set_output();
			LPTIM1->ARR = config.pulse - 1;
			return STATE_IDLE;
	}
}

static state_t next_state(state_t state) {
	if (config.input_voltage == INPUT_VOLTAGE_1_5) {
		return next_state_single_cell(state);
	} else {
		return next_state_dual_cell(state);
	}
}

__attribute__((noreturn)) int main(){
	// enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	// mount SRAM to boot sector
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE_SRAM;
	// disable SYSCFG clock
	RCC->APB2ENR &= ~RCC_APB2ENR_SYSCFGEN;

	// enable GPIO clock
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN;

	// put all GPIO to analog mode
	GPIOA->MODER = 0xFFFFFFFF;

	// copy configuration from EEPROM
	if (EEPROM_DATA[0] == EEPROM_MAGIC) {
		for (uint32_t i = 0; i < sizeof(clock_config_t) / 4; i += 1) {
			((uint32_t*)&config)[i] = EEPROM_DATA[i + 1];
		}
	} else {
		// defaults
		config.reload = 30720;
		config.pulse = 50;
		config.prescaler = 6;
	}

	// set DC/DC converter enable
	if (config.output_voltage == OUTPUT_VOLTAGE_HIGH) {
		gpio_set(PIN_BOOST_HIGH);
		gpio_set_mode(PIN_BOOST_HIGH, GPIO_MODE_Output);
	} else {
		gpio_set(PIN_BOOST_LOW);
		gpio_set_mode(PIN_BOOST_LOW, GPIO_MODE_Output);
	}

	// set cuttoff
	gpio_set_mode(PIN_CUTOFF, GPIO_MODE_Output);

	// set gate output
	gpio_set_mode(PIN_GATE_A, GPIO_MODE_Output);
	if (config.polarity == POLARITY_DOUBLE) {
		gpio_set_mode(PIN_GATE_B, GPIO_MODE_Output);
	}

	// configure GPIO pin to check boot condition
	gpio_set_mode(PIN_BUTTON, GPIO_MODE_Input);

	// delay for a bit to wait for the pin to stabilize
	for (uint32_t i = 0; i < 50; i += 1) {
		__NOP();
	}

	// check pin status
	if (!gpio_read(PIN_BUTTON)) {
		// boot into configuration code
		SCB->VTOR = BOOT_ADDR;
		__set_MSP(((uint32_t*)BOOT_ADDR)[0]);
		((void (*)(void))((void*)((uint32_t*)BOOT_ADDR)[1]))();
	}

	// total flash power-down
	FLASH->PDKEYR = FLASH_PDKEYR_PDKEY1;
	FLASH->PDKEYR = FLASH_PDKEYR_PDKEY2;
	FLASH->ACR |= FLASH_ACR_RUN_PD;
	RCC->AHBENR &= ~RCC_AHBENR_MIFEN;

	// enable peripherals clocks
	RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_LPTIM1EN;
	RCC->APB1SMENR |= RCC_APB1SMENR_LPTIM1SMEN;

	// enable wake-on-event and deep sleep
	SCB->SCR |= SCB_SCR_SEVONPEND_Msk | SCB_SCR_SLEEPDEEP_Msk;

	// set clock speed and core voltage
	RCC->ICSCR = RCC_ICSCR_MSIRANGE_4M194;
	PWR->CR = PWR_CR_VOS_1V5;
	while(PWR->CSR&PWR_CSR_VOSF);

	// start the LSE clock
	PWR->CR |= PWR_CR_DBP;
	RCC->CSR |= RCC_CSR_LSEON | RCC_CSR_LSEDRV_0;
	PWR->CR ^= PWR_CR_DBP;
	while(!(RCC->CSR&RCC_CSR_LSERDY));

	// enable LPTIM1 event channel
	EXTI->EMR = (EXTI_EMR_EM0 << GPIO_PIN_NUM(PIN_BUTTON)) | EXTI_EMR_EM29;

	// falling edge sense on external int
	EXTI->FTSR = (EXTI_FTSR_FT0 << GPIO_PIN_NUM(PIN_BUTTON));

	// rising edge detection on COMP1
	EXTI->RTSR = EXTI_RTSR_RT21;

	// set LSE as LPTIM1 clock
	RCC->CCIPR = RCC_CCIPR_LPTIM1SEL_LSE;

	// configure LPTIM1
	LPTIM1->IER = LPTIM_IER_ARRMIE;
	LPTIM1->CFGR = (config.prescaler << LPTIM_CFGR_PRESC_Pos) | LPTIM_CFGR_TIMOUT; // reset timer on trigger
	LPTIM1->CR = LPTIM_CR_ENABLE;

	state_t state = next_state(STATE_INITIAL);

	// start timer
	while (!(LPTIM1->ISR&LPTIM_ISR_ARROK));
	LPTIM1->CR |= LPTIM_CR_CNTSTRT;

	// main loop
	while(1) {
		if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
			while (!(LPTIM1->ISR & LPTIM_ISR_ARROK));
			LPTIM1->ICR = LPTIM_ICR_ARRMCF | LPTIM_ICR_ARROKCF;
			NVIC_ClearPendingIRQ(LPTIM1_IRQn);
			state = next_state(state);
		} else if (comp_enabled && (COMP1->CSR & COMP_CSR_COMP1VALUE)) {
			if (config.input_voltage == INPUT_VOLTAGE_1_5) {
				comp1_stop();
				EXTI->EMR |= (EXTI_EMR_EM0 << PIN_BUTTON);
				gpio_clear(PIN_BOOST_LOW);
				gpio_clear(PIN_CUTOFF);
			} else {
				while (!(LPTIM1->ISR & LPTIM_ISR_ARROK));
				LPTIM1->ICR = LPTIM_ICR_ARROKCF;
				comp1_stop();
				charge_time = LPTIM1->CNT;
				LPTIM1->ARR = charge_time + (config.charge - 1);
			}
		} else if ((state == STATE_CHARGE) && (!gpio_read(PIN_BUTTON))) {
			while (!(LPTIM1->ISR & LPTIM_ISR_ARROK));
			LPTIM1->ICR = LPTIM_ICR_ARROKCF;
			uint32_t cnt = 1000;
			while (cnt > 0) {
				if (gpio_read(PIN_BUTTON)) {
					cnt -= 1;
				} else {
					cnt += 1;
				}
				if (cnt > 100000) {
					mcu_reset();
				}
			}
			button_offset = (8000) >> config.prescaler;
			LPTIM1->ARR = LPTIM1->CNT + button_offset;
		}
		stop(!comp_enabled, true);
	}
}
