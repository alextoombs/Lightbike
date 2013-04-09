/* 
 * File:   UART.h
 * Author: mmellitt
 *
 * Created on February 11, 2013, 7:10 PM
 */

#ifndef UART_H
#define	UART_H

#ifdef	__cplusplus
extern "C" {
#endif

    void UartConfig();
    void UARTrepeat();
    char get();
    void _mon_putc(char a);
    void send(char a);
    


#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

