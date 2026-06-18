/*
 * stm32h743vi_flash.ld
 * STM32H743VI 链接脚本 — Flash @ 0x08000000, RAM @ 0x20000000
 *
 * H743 内存映射:
 *   Flash:  2MB @ 0x08000000
 *   DTCM:   128KB @ 0x20000000
 *   SRAM1:  512KB @ 0x24000000 (AXI)
 *   SRAM2:  128KB @ 0x24080000
 *   ITCM:   64KB  @ 0x00000000
 *
 * 简化: 代码放 Flash, 数据放 DTCM + AXI SRAM
 */

ENTRY(Reset_Handler)

/* 栈顶 — DTCM 起点 + 128KB */
_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_Min_Heap_Size  = 0x200;   /* 512 bytes heap */
_Min_Stack_Size = 0x400;   /* 1KB min stack  */

MEMORY
{
    FLASH    (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
    DTCMRAM  (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
    RAM      (xrw) : ORIGIN = 0x24000000, LENGTH = 512K
    ITCMRAM  (xrw) : ORIGIN = 0x00000000, LENGTH = 64K
}

SECTIONS
{
    /* 中断向量表 */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    /* 代码 */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    /* 只读数据 */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    /* ARM 属性 */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    /* 启动代码用的 preinit/init/fini array */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } >FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } >FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } >FLASH

    /* _sidata = LOAD 地址, _sdata/_edata = RAM 地址 */
    _sidata = LOADADDR(.data);

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT> FLASH

    /* DTCM — FreeRTOS 任务栈 + 关键数据 */
    .dtcm :
    {
        . = ALIGN(4);
        *(.dtcm)
        *(.dtcm*)
        . = ALIGN(4);
    } >DTCMRAM

    /* BSS (未初始化变量) */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } >RAM

    /* FreeRTOS 堆 — 放 AXI SRAM.
     * _end 在此处定义, _sbrk 从 _end 向前分配 newlib heap.
     * _heap_limit 用 RAM region 末尾做边界 (不走 _estack, 它在 DTCM).
     */
    .heap :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        . = . + _Min_Heap_Size;
        . = ALIGN(8);
        _heap_limit = ORIGIN(RAM) + LENGTH(RAM);
    } >RAM

    /* 最小栈空间 — 放 DTCM (栈从 _estack 向下生长) */
    ._user_stack :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
    } >DTCMRAM

    /DISCARD/ :
    {
        libc.a (*)
        libm.a (*)
        libgcc.a (*)
    }

    .ARM.attributes 0 : { *(.ARM.attributes) }
}
