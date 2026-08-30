#define LED_REG (*(volatile unsigned int *)0x10000000)

int main(void)
{
    LED_REG=1;

    while(1)
    {
    }

    return 0;
}