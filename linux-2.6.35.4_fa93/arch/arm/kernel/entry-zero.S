/*
 * By ccchang. This file is added for boot from address 0x0, 
 * it is due to the w99804 USB writer just could writes image to 0x0.
 */

	.section .zeroboot, "ax"
	mov pc, #0x8000		@ jump to 0x8000, the start address of the kernel
	.previous

