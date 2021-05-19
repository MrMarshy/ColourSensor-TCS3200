/*
	.text is for code & constants
	.bss is for uninitialized data
	.data is for initialized data
	.stack is for the stack
	
*/

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}

STACK_SIZE = 0x2000; /* 8Kb */

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector)) /* Keep at the very start of the .text section */
        *(.text*)
        *(.rodata*)
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx*)
        *(.gnu.linkonce.armexidx.*)
    } > FLASH

    _etext = .;

    .gnu_build_id : AT( _etext )
    {
        PROVIDE(g_note_build_id = .);
        *(.note.gnu.build-id)
    } > FLASH

    _begin_data = .;

    .data : AT( _begin_data )  /* .data section contains static variables which have initial value at boot */
    {
        _data = .;
        _ldata = LOADADDR (.data);
        *(vtable)
        *(.data*)
        _edata = .;
    } > SRAM

    .bss : /* .bss section contains static variables that must be zero'd at startup i.e. they are uninitialized in the code.*/
    {
        _bss = .;
        __bss_start__ = _bss;
        *(.bss*)
        *(COMMON) /* Special input section where the compiler puts global unitialized variables that go beyond the file scope */
        _ebss = .;
        __bss_end__ = _ebss;
            . = ALIGN(4);
        end = .;
        __end__ = .;
    } > SRAM

    .stack (NOLOAD):
    {
        . = ALIGN(8);
        . = . + STACK_SIZE;
        . = ALIGN(8);
        *(.stack*)
    } > SRAM

    __StackTop = ORIGIN(SRAM) + LENGTH(SRAM);
    __StackLimit = __StackTop - STACK_SIZE;
}

