#include "button.h"
#include "gpio.h"

void button_reset(button_t* button) {
	button->debounce_timer = 0;
	button->press_timer = 0;
	button->flags = 0;
}

bool button_update(button_t* button) {
	int32_t value = (int32_t)ADC1->DR - (int32_t)button->value;
	bool state = value < BUTTON_VALUE_JITTER && value > -BUTTON_VALUE_JITTER;
	if (button->press_timer) {
		if (state) {
			button->debounce_timer = 0;
			button->press_timer += 1;
			if (button->press_timer > BUTTON_LONG_PRESS_DELAY) {
				button->flags |= BUTTON_FLAG_LONG_PRESS;
			}
		} else {
			button->debounce_timer += 1;
			if (button->debounce_timer > BUTTON_DEBOUNCE_DELAY) {
				button->debounce_timer = 0;
				if (button->press_timer > BUTTON_IGNORE_DELAY) {
					button->flags |= BUTTON_FLAG_PRESSED;
				}
				button->press_timer = 0;
			}
		}
	} else if (state) {
		button->press_timer = 1;
	}
	return button->press_timer > BUTTON_IGNORE_DELAY;
}
