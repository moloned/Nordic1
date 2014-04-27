/* Memory configuration for nRF51822 */

ENTRY(Reset_Handler)

MEMORY
{
  	SRAM (rwx) 	: ORIGIN = 0x20000000, LENGTH = 16K
  	STACK (rwx)	: ORIGIN = 0x20003FFC, LENGTH = 0K		/* Top of the 16KB of SRAM minus 1 word */
  	FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 128K

}

SECTIONS {
	. = ORIGIN(FLASH);

	.vectors :
	{
		*(.isr_vector)			/* Vector table */
	} >FLASH

	.text :
	{
		*(.text)				/* Program code */
		*(.rodata)				/* Read only data */
		*(.rodata*)
	} >FLASH

	.data :
	{
		_start_data = .;
		*(.data)				/* Data memory */
		_end_data = .;
	} >SRAM AT >FLASH

    _start_data_flash = LOADADDR(.data);

	. = ALIGN(4);

	.bss :
	{
		_start_bss = .;
		*(.bss)				/* Zero-filled run time allocate data memory */
		_end_bss = .;
	} >SRAM

	. = ALIGN(4);

	_start_stack = .;
	_end_stack = ORIGIN(STACK);
}

_end = .;
PROVIDE(end = .);

