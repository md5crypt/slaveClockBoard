#include <stddef.h>
#include "led.h"
#include "stm32l0xx.h"

void led_set(led_data_t* data, const led_command_t* message) {
	led_set_squence(data, message, NULL, 0);
}

void led_set_squence(led_data_t* data, const led_command_t* message, const uint8_t* sequance, uint32_t length) {
	if (data->ptr != message) {
		data->ptr = message;
		data->offset = 0;
		data->timer = 0;
		data->accumulator = length;
		data->sequance = sequance;
		data->lock = false;
		TIM21->CCR1 = 0;
	}
}

bool led_update(led_data_t* data) {
	if (data->timer > 0) {
		data->timer -= 1;
	} else if (data->ptr != NULL) {
		bool exit = false;
		while (!exit) {
			int32_t value = data->ptr[data->offset].value;
			switch (data->ptr[data->offset].cmd) {
				case LED_CMD_LOCK:
					data->lock = value;
					break;
				case LED_CMD_JUMP:
					data->offset += value - 1;
					break;
				case LED_CMD_SET:
					TIM21->CCR1 = value;
					break;
				case LED_CMD_SEQ:
					TIM21->CCR1 = data->sequance[data->accumulator];
					break;
				case LED_CMD_WAIT:
					exit = true;
					if (value < 0) {
						data->timer = 0;
						data->offset -= 1;
					} else {
						data->timer = value;
					}
					break;
				case LED_CMD_BRANCH:
					if (data->accumulator > 0) {
						data->offset += value - 1;
					}
					break;
				case LED_CMD_ADD:
					data->accumulator += value;
					break;
				case LED_CMD_MOV:
					data->accumulator = value;
					break;
			}
			data->offset += 1;
		}
	}
	return data->lock;
}
