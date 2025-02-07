#include "clock_config.h"
#include <stdlib.h>
#include <stddef.h>

const clock_config_input_t clock_config_default_input = {
	.period = 1000,
	.pulse = 100
};

static uint32_t uint32_log2(uint32_t a) {
	uint32_t result = 0;
	while (a > 0) {
		result += 1;
		a >>= 1;
	}
	return result;
}

uint32_t int32_abs(int32_t a) {
    return a > 0 ? a : -a;
}

uint64_t int64_abs(int64_t a) {
    return a > 0 ? a : -a;
}

int64_t int64_div_rounded(int64_t a, int64_t b) {
    if (b == 0) {
        return 0;
    }
    uint64_t result = (int64_abs(a) + (int64_abs(b) >> 1)) / int64_abs(b);
    return ((a > 0 && b > 0) || (a < 0 && b < 0)) ? result : -result;
}

static void calculate_calibration(div_t reload, uint32_t prescaler, int32_t fix, clock_config_t* output) {
	int64_t k = (86400000LL << prescaler);
	int64_t fix_scaled = (
		(((int64_t) fix * (int64_t) reload.quot) << prescaler) +
		int64_div_rounded((int64_t) fix *  (int64_t) reload.rem, 1000)
	);
	int64_t r_scaled = 86400LL * reload.rem;
	int64_t offset = int64_div_rounded(fix_scaled + r_scaled, k);
	uint32_t precision = 30 - uint32_log2(int32_abs(int64_div_rounded(k, (fix_scaled + r_scaled - k * offset))));
	int64_t value = int64_div_rounded(k << precision, fix_scaled + r_scaled - k * offset);
	output->fix_period = value > 0 ? value : -value;
	output->fix_sign = value > 0 ? 1 : -1;
	output->fix_offset = offset;
	output->fix_precision = 1 << precision;
}

static uint32_t calculate_prescaler(uint32_t period) {
	for (uint32_t i = 1; i < 7; i += 1) {
		if (period < (750U * (1U << i))) {
			return i - 1;
		}
	}
	return 7;
}

static div_t calculate_period(uint32_t value, uint32_t prescaler) {
	return div(value * 32768, 1000 << prescaler);
}

void clock_config_compute(clock_config_input_t* input, clock_config_t* output) {
	uint32_t prescaler = calculate_prescaler(input->period);
	div_t reload = calculate_period(input->period, prescaler);
	calculate_calibration(reload, prescaler, input->fix_sign ? -input->fix : input->fix, output);
	output->prescaler = prescaler;
	output->reload = reload.quot;
	output->pulse = calculate_period(input->pulse, prescaler).quot;
	output->charge = calculate_period(input->charge, prescaler).quot;
	output->input_voltage = input->input_voltage;
	output->output_voltage = input->output_voltage;
	output->polarity = input->polarity;
}

bool clock_config_input_verify(clock_config_input_t* input) {
	// check on / off values

	if (input->fix_sign > 1 || input->input_voltage > 1 || input->output_voltage > 1 || input->polarity > 1) {
		return false;
	}

	// can not be zero
	if (input->period == 0 || input->pulse == 0) {
		return false;
	}
	// max values
	if (input->period > 96000 || input->fix >= 65536) {
		return false;
	}
	// check if period > charge + pulse
	if (input->period <= (input->charge + input->pulse)) {
		return false;
	}
	return true;
}
