#include <stdint.h>
#include "stm32f4xx.h"
#include "stm32f4xx_uart.h"
#include "delay.h"

int main()
{
    uart_handle_t uart2 = {
        .uart = USART2,
        .conf.mode = MODE_TX,
        .conf.parity = EVEN_PARITY,
        .conf.stop_bits = STOP_BIT_1,
        .conf.over_sample = OVER16,
        .conf.m_bits = MBIT_8,
        .conf.baudrate = 9600
    };

    uart_init(uart2);
    init_systick(); 
    while(1)
    {
        //char hell[] = "hello hi \r\n";
        //uart_write(uart2, hell, sizeof(hell));
       // delay(2);
    }
    return 0;
}
