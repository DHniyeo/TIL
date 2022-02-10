
ENTRY(Reset_Handler)

_estack = 0x20020000; /* end of RAM (highest) 128*1024를 계산하면 딱 Hex값 20000 */

MEMORY
{
	RAM (xrw)	: ORIGIN = 0x20000000,	LENGTH = 128k
	ROM (rx)	: ORIGIN = 0x08000000,	LENGTH = 512k
}

SECTIONS
{
	.isr_vector :
	{
		. = ALIGN(4);
		KEEP(*(.isr_vector))
		. = ALIGN(4);
	} >ROM

	.text :
	{
		. = ALIGN(4);
		*(.text)
		. = ALIGN(4);
	} >ROM

}
