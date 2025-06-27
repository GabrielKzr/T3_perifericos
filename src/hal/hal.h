#include <stddef.h>

extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */

#define ID_UNIQUE_ADDRESS		0x1FFF7A10
#define ID_UNIQUE_8(x)			((x >= 0 && x < 12) ? (*(uint8_t *) (ID_UNIQUE_ADDRESS + (x))) : 0)

typedef uint32_t jmp_buf[20];

extern uint32_t _stack_start;		/* Start of the STACK memory. */
extern uint32_t _stack_end;		/* End of the STACK memory. */
extern uint32_t _heap_start;		/* Start of the HEAP memory. */
extern uint32_t _heap_end;		/* End of the HEAP memory (one byte past the last byte of this memory). */
extern uint32_t _heap_size;		/* Size of HEAP memory. */
extern uint32_t _sidata;		/* Start address for the contents initialization of the .data
						section. defined in linker script. */
extern uint32_t _sdata;			/* Start address for the .data section, defined in linker script */
extern uint32_t _edata;			/* End address for the .data section, defined in linker script. */
extern uint32_t _sbss;			/* Start address for the .bss section, defined in linker script. */
extern uint32_t _ebss;			/* End address for the .bss section, defined in linker script. */
extern uint32_t _end;			/* Start address of the heap memory, defined in linker script. */

int32_t setjmp(jmp_buf env);
void longjmp(jmp_buf env, int32_t val);
// void delay_us(const uint32_t us);
void delay_ms(const uint32_t ticks_to_wait);
uint64_t ticks_ms(void);
void _ei(void);
void _di(void);

