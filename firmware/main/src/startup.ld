/* Entry Point */
ENTRY(Reset_Handler)

_sram = ORIGIN(SRAM);  	          /* RAM (true) start address */
_estack = _sram + LENGTH(SRAM);   /* end of RAM */

/* Sections */
SECTIONS {
    /* Startup code */
	.isr_vector : AT (ADDR(.isr_vector) + ORIGIN(FLASH)) {
		KEEP(*(.isr_vector))
	} > BOOT
	. = ALIGN(4);
		
    /* program code */
    .text : AT (ADDR(.text) + ORIGIN(FLASH)) {
    	*(.text) 
    } > BOOT
    . = ALIGN(4);
    
    /* read-only data */
    .rodata : AT (ADDR(.rodata) + ORIGIN(FLASH)) {
    	*(.rodata) 
    } > BOOT
    . = ALIGN(4);
    
    /* initialized global and static variables */
    .data : AT (ADDR(.data) + ORIGIN(FLASH)) {
    	*(.data) 
    } > BOOT
   	. = ALIGN(4);
   	_edata = .;
   	
    /* uninitialized global and static variables */
    .bss : {
    	*(.bss*)
    } > BOOT
    . = ALIGN(4);
    _ebss = .;
}
