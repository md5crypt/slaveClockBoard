.syntax unified

.global Reset_Handler
.section .text.Reset_Handler
.type Reset_Handler, %function

Reset_Handler:
	/* Copy flash to SRAM */
	ldr r0, =_edata
	ldr r1, =_sdata
	subs r0, r1
	beq CopyFlashEnd
	ldr r2, =_sidata

CopyFlash:
	subs r0, #4
	ldr r3, [r2, r0]
	str r3, [r1, r0]
	bne CopyFlash

CopyFlashEnd:
	ldr r0, =_ebss
	ldr r1, =_sbss
	movs r3, #0
	subs r0, r1
	beq ZeroMemoryEnd

ZeroMemory:
	subs r0, #4
	str r3, [r1, r0]
	bne ZeroMemory

ZeroMemoryEnd:
	/* Call the application's entry point.*/
	bl main
