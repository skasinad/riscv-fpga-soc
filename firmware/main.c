#define GPIO_OUT        (*(volatile unsigned int *)0x10000000)
#define SYS_STATUS      (*(volatile unsigned int *)0x10000004)
#define CYCLE_COUNT_LO  (*(volatile unsigned int *)0x10000008)
#define CYCLE_COUNT_HI  (*(volatile unsigned int *)0x1000000C)
#define DEBUG_CTRL      (*(volatile unsigned int *)0x10000010)

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

    GPIO_OUT=0x01;

    status=SYS_STATUS;

    if((status&0x01)==0)
    {
        GPIO_OUT=0x80;

        while(1)
        {
        }
    }

    start=CYCLE_COUNT_LO;

    delay_cycles(5000000);

    end=CYCLE_COUNT_LO;

    if(end>start)
        GPIO_OUT=0x03;
    else
        GPIO_OUT=0x40;

    DEBUG_CTRL=0x12345678;

    while(1)
    {
    }

    return 0;
}