/* Memories definition */
MEMORY {
	BOOT (rwx)		: ORIGIN = 0x00000000, LENGTH = 2K
	SRAM (rwx)		: ORIGIN = 0x20000000, LENGTH = 2K
	FLASH (rx)		: ORIGIN = 0x08000000, LENGTH = 8K
}