/* Memories definition */
MEMORY {
	RAM (rwx)		: ORIGIN = 0x20000000, LENGTH = 2K
	FLASH (rx)		: ORIGIN = 0x08000800, LENGTH = 14K
}
