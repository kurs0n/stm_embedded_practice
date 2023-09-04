ENTRY(Reset_handler)

MEMORY
{
    CCMRAM    (xrw)    : ORIGIN = 0x10000000,   LENGTH = 16K
    FLASH(rx):ORIGIN =0x08000000,LENGTH =512K
    SRAM(xrw):ORIGIN =0x20000000,LENGTH =64K 
}

SECTIONS 
{
    .text : 
    {
        *(.isr_vector)
        *(.text)
        *(.rodata)
        . = ALIGN(4);
        _etext = . ; 
    } >FLASH

    .data : /* location counter always trakcs vma of the section in which it is being used*/
    {
        _sdata = .;
        *(.data)
        . = ALIGN(4); 
        _edata = .;
    } >SRAM AT>FLASH 
    
    .bss : 
    {
        _sbss = .;
        *(.bss)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >SRAM
}