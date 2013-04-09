#include <xc.h>
#include "UART.h"

void UartConfig()
{
    U6MODEbits.ON=1;
    U6STAbits.UTXEN=1;
    U6STAbits.URXEN=1;
    U6STAbits.ADM_EN=1;
    U6MODEbits.PDSEL=0x00;
    U6MODEbits.STSEL=0;
    U6MODEbits.BRGH=1;
    U6BRG=42;
}
void UARTrepeat()
{
    send(get());
}
char get(void)
{
    if(U6STAbits.URXDA)
    {
        return(U6RXREG);
    }
}
void _mon_putc(char a)
{
    send(a);
}
void send(char a)
{
    while(U6STAbits.UTXBF){}
    if(!U6STAbits.UTXBF)
    {
        U6TXREG = a;
    }
}