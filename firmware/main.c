#define GPIO_OUT (*(volatile unsigned int *)0x10000000)
#define SYS_STATUS (*(volatile unsigned int *)0x10000004)
#define CYCLE_COUNT_LO (*(volatile unsigned int *)0x10000008)
#define CYCLE_COUNT_HI (*(volatile unsigned int *)0x1000000C)
#define DEBUG_CTRL (*(volatile unsigned int *)0x10000010)
#define INSTR_COUNT (*(volatile unsigned int *)0x10000014)
#define MEM_COUNT (*(volatile unsigned int *)0x10000018)
#define MMIO_COUNT (*(volatile unsigned int *)0x1000001C)

#define POST_RUNNING 0x01
#define POST_GPIO_OK 0x02
#define POST_STATUS_OK 0x04
#define POST_TIMER_OK 0x08
#define POST_DEBUG_OK 0x10
#define POST_PASS 0x3F

//fault codes have bit 6 set plus a specific bit, so they look different
//on the leds from any of the POST_* codes above
#define ERR_STATUS 0x41
#define ERR_TIMER 0x42
#define ERR_DEBUG 0x44

#define SNAP_CTRL (*(volatile unsigned int *)0x10000020)
#define SNAP_CYCLE_LO (*(volatile unsigned int *)0x10000024)
#define SNAP_CYCLE_HI (*(volatile unsigned int *)0x10000028)
#define SNAP_INSTR (*(volatile unsigned int *)0x1000002C)
#define SNAP_MEM (*(volatile unsigned int *)0x10000030)
#define SNAP_MMIO (*(volatile unsigned int *)0x10000034)

#define WORKLOAD_SELECT (*(volatile unsigned int *)0x10000038)
#define DATA_RAM_COUNT (*(volatile unsigned int *)0x1000003C)
#define WORKLOAD_SCRATCH (*(volatile unsigned int *)0x10000044)

#define WL_IDLE 0
#define WL_ALU 1
#define WL_MEMORY 2
#define WL_BRANCH 3
#define WL_MMIO 4
#define WL_MIXED 5

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

//store sink for the register-only workloads (idle/alu/branch) so -Os can't
//prove the loop does nothing and optimize it away
static volatile unsigned int workload_sink;

//64 words is plenty to show real ram traffic, way under the 4kb budget
#define WORKLOAD_BUF_WORDS 64
static volatile unsigned int workload_buf[WORKLOAD_BUF_WORDS];

static void run_idle(void)
{
    unsigned int i,x=0;

    //a plain empty counting loop gets collapsed by -Os straight into
    //workload_sink=20000, saw this in the disassembly. xoring i into x
    //has no closed form gcc can figure out so the loop actually survives
    for(i=0;i<20000;i++)
        x=x^i;

    workload_sink=x;
}

static void run_alu(void)
{
    unsigned int a=1,b=2,c=3,i;

    for(i=0;i<2000;i++)
    {
        a=a+b;
        b=b^c;
        c=c-a;
        a=a<<1;
        b=b>>1;
        c=(c==a);
    }

    //one store after the loop, not per iteration, keeps this register only
    //so DATA_RAM_COUNT stays near zero
    workload_sink=a+b+c;
}

static void run_memory(void)
{
    unsigned int i,iter;

    for(iter=0;iter<200;iter++)
    {
        for(i=0;i<WORKLOAD_BUF_WORDS;i++)
            workload_buf[i]=workload_buf[i]+1;
    }
}

static void run_branch(void)
{
    unsigned int i,x=0,state=0;

    for(i=0;i<4000;i++)
    {
        if(state==0)
            state=(i&1) ? 2 : 1;
        else if(state==1)
        {
            state=(x<10) ? 2 : 0;
            x++;
        }
        else
        {
            state=0;
            x=(x>0) ? x-1 : 0;
        }
    }

    workload_sink=x+state;
}

static void run_mmio(void)
{
    unsigned int i;

    for(i=0;i<2000;i++)
        WORKLOAD_SCRATCH=i;
}

static void run_mixed(void)
{
    unsigned int i,a=1,b=2;

    for(i=0;i<500;i++)
    {
        a=a+b;
        b=a^i;

        workload_buf[i&(WORKLOAD_BUF_WORDS-1)]=a;

        if((i&7)==0)
            WORKLOAD_SCRATCH=b;
    }

    workload_sink=a+b;
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
    unsigned int snap_cycle_lo;
    unsigned int snap_instr;
    unsigned int snap_mem;
    unsigned int snap_mmio;

    GPIO_OUT=POST_RUNNING;

    //gpio write path works if we made it here
    GPIO_OUT=POST_GPIO_OK;

    //system status test
    status=SYS_STATUS;

    if((status&0x01)==0)
        fault(ERR_STATUS);

    GPIO_OUT=POST_STATUS_OK;

    //hardware cycle counter test
    start=CYCLE_COUNT_LO;

    delay_cycles(1000);

    end=CYCLE_COUNT_LO;

    if(end<=start)
        fault(ERR_TIMER);

    GPIO_OUT=POST_TIMER_OK;

    //debug register read/write test
    DEBUG_CTRL=0x12345678;
    debug_value=DEBUG_CTRL;

    if(debug_value!=0x12345678)
        fault(ERR_DEBUG);

    GPIO_OUT=POST_DEBUG_OK;

    //execution monitor test
    instructions=INSTR_COUNT;
    memory_accesses=MEM_COUNT;
    mmio_accesses=MMIO_COUNT;

    if(instructions==0)
        fault(0x48);

    if(memory_accesses==0)
        fault(0x49);

    if(mmio_accesses==0)
        fault(0x4A);
    SNAP_CTRL=1;

    snap_cycle_lo=SNAP_CYCLE_LO;
    snap_instr=SNAP_INSTR;
    snap_mem=SNAP_MEM;
    snap_mmio=SNAP_MMIO;

    if(snap_cycle_lo==0)
        fault(0x4B);

    if(snap_instr==0)
        fault(0x4C);

    if(snap_mem==0)
        fault(0x4D);

    if(snap_mmio==0)
        fault(0x4E);
        //short visible delay before final pass state
        delay_cycles(5000000);

    GPIO_OUT=POST_PASS;

    //polling WORKLOAD_SELECT once per run_*() call instead of inside each
    //loop, each call is only a few thousand instructions so a workload
    //change still shows up in a couple ms without reading mmio every iter
    for(;;)
    {
        switch(WORKLOAD_SELECT)
        {
            case WL_ALU: run_alu(); break;
            case WL_MEMORY: run_memory(); break;
            case WL_BRANCH: run_branch(); break;
            case WL_MMIO: run_mmio(); break;
            case WL_MIXED: run_mixed(); break;
            default: run_idle(); break;
        }
    }

    return 0;
}