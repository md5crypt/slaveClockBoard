#include <stdint.h>
#include <stdbool.h>

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

typedef struct {
	uint32_t period;
	uint32_t pulse;
	uint32_t charge;
	uint32_t fix;
	uint32_t fix_sign;
	uint32_t output_voltage;
	uint32_t input_voltage;
	uint32_t polarity;
} clock_config_input_t;

extern const clock_config_input_t clock_config_default_input;

void clock_config_compute(clock_config_input_t* input, clock_config_t* output);
bool clock_config_input_verify(clock_config_input_t* input);
