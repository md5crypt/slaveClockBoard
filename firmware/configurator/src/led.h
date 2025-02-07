#include <stdint.h>
#include <stdbool.h>

#define LED_CMD(x,y) (led_command_t){.cmd = LED_CMD_##x, .value = y}

enum {
	LED_CMD_LOCK,
	LED_CMD_JUMP,
	LED_CMD_SET,
	LED_CMD_SEQ,
	LED_CMD_WAIT,
	LED_CMD_BRANCH,
	LED_CMD_ADD,
	LED_CMD_MOV
};

typedef struct {
	uint16_t cmd;
	int16_t value;
} led_command_t;

typedef struct {
	const led_command_t* ptr;
	uint32_t offset;
	uint32_t timer;
	int32_t accumulator;
	const uint8_t* sequance;
	bool lock;
} led_data_t;

void led_set(led_data_t* data, const led_command_t* message);
void led_set_squence(led_data_t* data, const led_command_t* message, const uint8_t* sequance, uint32_t length);
bool led_update(led_data_t* data);
