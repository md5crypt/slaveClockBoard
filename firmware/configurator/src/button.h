#include <stdint.h>
#include <stdbool.h>

#define BUTTON_IGNORE_DELAY      10
#define BUTTON_DEBOUNCE_DELAY    100
#define BUTTON_LONG_PRESS_DELAY  2000

#define BUTTON_VALUE_JITTER      16

enum {
	BUTTON_FLAG_PRESSED = 1,
	BUTTON_FLAG_LONG_PRESS = 2
};

typedef struct {
	uint16_t press_timer;
	uint16_t debounce_timer;
	uint8_t value;
	uint8_t flags;
} button_t;

void button_reset(button_t* button);
bool button_update(button_t* button);
