/* Entry Point */
ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Sections */
SECTIONS {
	/* in-flash program code */
	.isr_vector : ALIGN(64) {
		KEEP(*(.isr_vector))
	} > FLASH

	/* program code */
	.text : ALIGN(4) {
		_stext = .;
		*(.text)
		*(.text*)
		. = ALIGN(4);
		_etext = .;
	} > FLASH

	/* read-only data */
	.rodata : ALIGN(4) {
		_srodata = .;
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
		_erodata = .;
	} > FLASH

	/* initialized global and static variables */
	.data : ALIGN(4) {
		_sdata = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM AT > FLASH
	_sidata = LOADADDR(.data);

	/* uninitialized global and static variables */
	.bss : ALIGN(4) {
		_sbss = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > RAM

	/DISCARD/ : {
		*(.init)
		*(.fini)
	}
}
