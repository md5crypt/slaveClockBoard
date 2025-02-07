.syntax unified
  .global Reset_Handler
  .section .text.Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
/* Copy flash to SRAM */
  movs r0, #0
  ldr r1, =_sram
  ldr r2, =_edata

CopyFlash:
  ldr r3, [r0]
  str r3, [r1, r0]
  adds r0, #4
  cmp r0, r2
  bcc CopyFlash

/* Zero fill the bss segment. */
  ldr r2, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r1, r0]
  adds r0, #4

LoopFillZerobss:
  cmp r0, r2
  bcc FillZerobss

/* Call the application's entry point.*/
  b main
