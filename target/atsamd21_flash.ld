OUTPUT_ARCH(arm)

ENTRY(reset_handler)

MEMORY
{
/*    /* 224k - 128b for imghdr */
    flash  (rx)  : ORIGIN = 0x00008080, LENGTH = 229248,
    sram   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K

    /* DEBUG - Standalone */
/*    flash  (rx)  : ORIGIN = 0x00000000, LENGTH = 256K,
    sram   (rwx) : ORIGIN = 0x20000000, LENGTH = 32K */
}

SECTIONS
{
    .text :
    {
	. = ALIGN(64);
        KEEP(*(.flash_isr_vector))
        *(.text*)
	. = ALIGN(4);
    } > flash

    _etext = .;
    PROVIDE (__etext = .);

    .data : AT (__etext)
    {
	. = ALIGN(4);
	PROVIDE (__data_start = .);
	KEEP(*(.sram_isr_vector))
        *(.rodata*)
	*(.data*)
	KEEP(*(.sram_function))
        _edata = .;
	PROVIDE(__data_end = .);
    } > sram

    .bss :
    {
	. = ALIGN(4);
	 __bss_start = .;
	 *(.bss*)
	 *(COMMON)
	 __bss_end = .;
    } > sram

    .stacks :
    {
	 . = ALIGN(8);
	 . += 4K;
	 __exception_stack = .;
	 . += 4K;
	 __main_stack = .;
	 . = ALIGN(8);
	 __heap_start = .;
    } > sram

}
end = .;
