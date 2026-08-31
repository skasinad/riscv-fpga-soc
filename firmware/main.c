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

#define SNAP_CTRL       (*(volatile unsigned int *)0x10000020)
#define SNAP_CYCLE_LO   (*(volatile unsigned int *)0x10000024)
#define SNAP_CYCLE_HI   (*(volatile unsigned int *)0x10000028)
#define SNAP_INSTR      (*(volatile unsigned int *)0x1000002C)
#define SNAP_MEM        (*(volatile unsigned int *)0x10000030)
#define SNAP_MMIO       (*(volatile unsigned int *)0x10000034)

#define WORKLOAD_SELECT  (*(volatile unsigned int *)0x10000038)
#define DATA_RAM_COUNT   (*(volatile unsigned int *)0x1000003C)
#define WORKLOAD_SCRATCH (*(volatile unsigned int *)0x10000044)

#define WL_IDLE   0
#define WL_ALU    1
#define WL_MEMORY 2
#define WL_BRANCH 3
#define WL_MMIO   4
#define WL_MIXED  5

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

// Anti-optimization anchor for workloads whose loops are otherwise pure
// register traffic (IDLE, ALU, BRANCH) -- one store per call, never read
// back, just enough to stop -Os from proving the loop has no effect.
static volatile unsigned int workload_sink;

// 64 words is plenty to show real RAM traffic and is nowhere near the 4KB
// budget (firmware is well under 1KB total, stack starts at 0x1000 and
// only goes a few dozen bytes deep with no recursion anywhere in this file).
#define WORKLOAD_BUF_WORDS 64
static volatile unsigned int workload_buf[WORKLOAD_BUF_WORDS];

static void run_idle(void)
{
    unsigned int i,x=0;

    // a plain empty-bodied counting loop has a statically-known trip
    // count and no other effect, so -Os collapses it straight into
    // "workload_sink=20000" and skips the loop entirely -- confirmed by
    // disassembly. XORing the counter into x has no closed form GCC
    // recognizes, so the loop survives while staying pure register
    // traffic (no RAM/MMIO access anywhere in the body)
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

    // one store after the loop, not per iteration -- keeps the actual
    // computation register-only so DATA_RAM_COUNT stays near zero
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
        //Short visible delay before final PASS state
        delay_cycles(5000000);

    GPIO_OUT=POST_PASS;

    // WORKLOAD_SELECT is polled once per run_*() call rather than inside
    // each loop -- each call is a few thousand instructions at most, so a
    // workload change is visible within a few milliseconds without paying
    // MMIO-read overhead on every iteration
    for(;;)
    {
        switch(WORKLOAD_SELECT)
        {
            case WL_ALU:    run_alu();    break;
            case WL_MEMORY: run_memory(); break;
            case WL_BRANCH: run_branch(); break;
            case WL_MMIO:   run_mmio();   break;
            case WL_MIXED:  run_mixed();  break;
            default:        run_idle();   break;
        }
    }

    return 0;
}