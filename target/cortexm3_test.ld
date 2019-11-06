OUTPUT_ARCH(arm)

ENTRY(reset_handler)

MEMORY
{
    common (rx)  : ORIGIN = 0x00000000, LENGTH = 64K,
    sram   (rwx) : ORIGIN = 0x10000000, LENGTH = 8K
}

SECTIONS
{
    .text :
    {
	. = ALIGN(4);
        KEEP(*(.flash_isr_vector))
        *(.text*)
    } > common

    . = ALIGN(4);
    _etext = .;
    PROVIDE (__etext = .);

    .data :
    {
	. = ALIGN(4);
	PROVIDE (__data_start = .);
	KEEP(*(.sram_isr_vector))
        *(.rodata*)     /* Const data must be in SRAM for USB to see it */
	*(.data*)
	KEEP(*(.sram_function))
        _edata = .;
	PROVIDE(__data_end = .);
    } > common

    .bss :
    {
	. = ALIGN(4);
	 __bss_start = .;
	 *(.bss*)
	 *(COMMON)
	 __bss_end = .;
    } > common

    .stacks :
    {
	 . = ALIGN(8);
	 . += 4K;
	 __exception_stack = .;
	 . += 4K;
	 __main_stack = .;
	 . = ALIGN(8);
	 __heap_start = .;
    } > common

}
end = .;
