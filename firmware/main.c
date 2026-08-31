#define GPIO_OUT        (*(volatile unsigned int *)0x10000000)
#define SYS_STATUS      (*(volatile unsigned int *)0x10000004)
#define CYCLE_COUNT_LO  (*(volatile unsigned int *)0x10000008)
#define CYCLE_COUNT_HI  (*(volatile unsigned int *)0x1000000C)
#define DEBUG_CTRL      (*(volatile unsigned int *)0x10000010)
#define INSTR_COUNT     (*(volatile unsigned int *)0x10000014)
#define MEM_COUNT       (*(volatile unsigned int *)0x10000018)
#define MMIO_COUNT      (*(volatile unsigned int *)0x1000001C)

#define POST_RUNNING    0x01
#define POST_GPIO_OK    0x02
#define POST_STATUS_OK  0x04
#define POST_TIMER_OK   0x08
#define POST_DEBUG_OK   0x10
#define POST_PASS       0x3F

// fault codes set bit 6 (0x40) plus a test-specific bit, so a failure
// is visually distinct on the LEDs from any POST_* progress code above
#define ERR_STATUS      0x41
#define ERR_TIMER       0x42
#define ERR_DEBUG       0x44

static void fault(unsigned int code)
{
    GPIO_OUT=code;

    while(1)
    {
    }
}

static void delay_cycles(unsigned int cycles)
{
    unsigned int start=CYCLE_COUNT_LO;

    while((CYCLE_COUNT_LO-start)<cycles)
    {
    }
}

int main(void)
{
    unsigned int status;
    unsigned int start;
    unsigned int end;
    unsigned int debug_value;

    unsigned int instructions;
    unsigned int memory_accesses;
    unsigned int mmio_accesses;

    GPIO_OUT=POST_RUNNING;

    //GPIO write path is working if execution reaches here
    GPIO_OUT=POST_GPIO_OK;

    //System status test
    status=SYS_STATUS;

    if((status&0x01)==0)
        fault(ERR_STATUS);

    GPIO_OUT=POST_STATUS_OK;

    //Hardware cycle counter test
    start=CYCLE_COUNT_LO;

    delay_cycles(1000);

    end=CYCLE_COUNT_LO;

    if(end<=start)
        fault(ERR_TIMER);

    GPIO_OUT=POST_TIMER_OK;

    //Debug register read/write test
    DEBUG_CTRL=0x12345678;
    debug_value=DEBUG_CTRL;

    if(debug_value!=0x12345678)
        fault(ERR_DEBUG);

    GPIO_OUT=POST_DEBUG_OK;

    //Execution monitor test
    instructions=INSTR_COUNT;
    memory_accesses=MEM_COUNT;
    mmio_accesses=MMIO_COUNT;

    if(instructions==0)
        fault(0x48);

    if(memory_accesses==0)
        fault(0x49);

    if(mmio_accesses==0)
        fault(0x4A);

    //Short visible delay before final PASS state
    delay_cycles(5000000);

    GPIO_OUT=POST_PASS;

    while(1)
    {
    }

    return 0;
}