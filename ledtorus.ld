MEMORY
{
	/* The first 16 kByte flash are reserved for bootloader. */
	FLASH (rx) : ORIGIN = 0x8004000, LENGTH = 1008K
	RAM_MAIN (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
	RAM_CPU (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
}
_stack = 0x20020000;


SECTIONS
{
	.text :
	{
		KEEP(*(.vectors))
		*(.text*)
		*(.rodata*)
		. = ALIGN(4);
		_etext = .;
	} > FLASH

	/* Let's disable this stuff for now, just make it empty. */
	__preinit_array_start = .;
	__preinit_array_end = .;
	__init_array_start = .;
	__init_array_end = .;
	__fini_array_start = .;
	__fini_array_end = .;

	.data :
	{
		_data = .;
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM_MAIN AT > FLASH
	_data_loadaddr = LOADADDR(.data);

	.bss :
	{
		_sbss = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
	} > RAM_MAIN

	/* Check that there is room for stack. */
	._stack_free_space :
	{
		. = . + 4K;  /* Minimum 4 kByte stack free. */
	} > RAM_MAIN
}

ENTRY(reset_handler)
