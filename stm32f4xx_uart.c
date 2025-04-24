#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "stm32f4xx_uart.h"

USART_Reg_Def_t *UART;
#define PERI_CLK 16000000
uint8_t tx_buf[512];
uint8_t rx_buf[512];
int tx_head = 0;
int tx_tail = 0;
int rx_head = 0;
int rx_tail = 0;
bool tx_busy = false;
void uart_irq_handler()
{
    if (UART->USART_SR & ( 1<< 6)) {
        if (tx_head == tx_tail) {
            UART->USART_SR &= ~( 1 << 6);
            tx_busy = false;
        } else {
        UART->USART_DR = tx_buf[tx_tail++];
        if (tx_tail >= sizeof(tx_buf)) 
            tx_tail = 0;
        }
    }
}

static uint32_t calculate_baudrate(uint32_t peri_clk, uint32_t baudrate) 
{
    return ((peri_clk + (baudrate / 2U))/baudrate);
}

static void set_baudrate(uart_handle_t uart, uint32_t peri_clk, uint32_t baudrate)
{
    uart.uart->USART_BRR = calculate_baudrate(peri_clk, baudrate);
}

void uart_init(uart_handle_t uart)
{
    if (uart.uart == USART1) {
        UART = USART1;
        USART1_PCLK_EN();
        GPIOA_PCLK_EN();
        GPIOA->MODER &= ~(0xf << ( 2* 9)); 
        GPIOA->MODER |= (0xa << (2 * 9));
        GPIOA->AFR[1] &= ~(0xff << (4* 1));
        GPIOA->AFR[1] |= (0x77 << (4* 1));
        uint8_t reg_ind = 37 / 32;
        uint8_t isr_num = 37 % 32;
        NVIC->NVIC_ISER[reg_ind] |= (1 << isr_num);
    } else if (uart.uart == USART2) {
        UART = USART2;
        USART2_PCLK_EN();
        GPIOA_PCLK_EN();
        GPIOA->MODER &= ~(0xf << ( 2* 2)); 
        GPIOA->MODER |= (0xa << (2 * 2));
        GPIOA->AFR[0] &= ~(0xff << (4* 2));
        GPIOA->AFR[0] |= (0x77 << (4* 2));
        uint8_t reg_ind = 38 / 32;
        uint8_t isr_num = 38 % 32;
        NVIC->NVIC_ISER[reg_ind] |= (1 << isr_num);
    } else if (uart.uart == USART3) {
        UART = USART3;
        USART3_PCLK_EN();
        GPIOB_PCLK_EN();
        GPIOB->MODER &= ~(0xf << ( 2* 10)); 
        GPIOB->MODER |= (0xa << (2 * 10));
        GPIOB->AFR[1] &= ~(0xff << (4* 2));
        GPIOB->AFR[1] |= (0x77 << (4* 2));
        uint8_t reg_ind = 39 / 32;
        uint8_t isr_num = 39 % 32;
        NVIC->NVIC_ISER[reg_ind] |= (1 << isr_num);
    } else if (uart.uart == USART4) {
        UART = USART4;
        USART4_PCLK_EN();
        GPIOA_PCLK_EN();
        GPIOA->MODER &= ~(0xf << ( 2* 0)); 
        GPIOA->MODER |= (0xa << (2 * 0));
        GPIOA->AFR[0] &= ~(0xff << (4* 0));
        GPIOA->AFR[0] |= (0x88 << (4* 0));
        uint8_t reg_ind = 52 / 32;
        uint8_t isr_num = 52 % 32;
        NVIC->NVIC_ISER[reg_ind] |= (1 << isr_num);
    } else if (uart.uart == USART5) {
        UART = USART5;
        USART5_PCLK_EN();
        GPIOC_PCLK_EN();
        GPIOD_PCLK_EN();
        GPIOC->MODER &= ~(0x3 << ( 2* 12)); 
        GPIOC->MODER |= (0x2 << (2 * 12));
        GPIOD->MODER &= ~(0x3 << ( 2* 2)); 
        GPIOD->MODER |= (0x2 << (2 * 2));
        GPIOC->AFR[1] &= ~(0xf << (4* 4));
        GPIOC->AFR[1] |= (0x8 << (4* 4));
        GPIOC->AFR[0] &= ~(0xf << (4* 2));
        GPIOC->AFR[0] |= (0x8 << (4* 2));
        uint8_t reg_ind = 53 / 32;
        uint8_t isr_num = 53  % 32;
        NVIC->NVIC_ISER[reg_ind] |= (1 << isr_num);
    } else if (uart.uart == USART6) {
        UART = USART6;
        USART6_PCLK_EN();
        GPIOC_PCLK_EN();
        GPIOC->MODER &= ~(0xf << ( 2* 6)); 
        GPIOC->MODER |= (0xa << (2 * 6));
        GPIOC->AFR[0] &= ~(0xff << (4* 6));
        GPIOC->AFR[0] |= (0x88 << (4* 6));
        uint8_t reg_ind = 71 / 32;
        uint8_t isr_num = 71 % 32;
        NVIC->NVIC_ISER[reg_ind] |= (1 << isr_num);
    }
    if (uart.conf.parity == EVEN_PARITY) {
        uart.uart->USART_CR1 &= ~(1 << 9);
    } else if (uart.conf.parity == ODD_PARITY) {
        uart.uart->USART_CR1 |= (1 << 9);
    }
    if (uart.conf.m_bits == MBIT_8) {
        uart.uart->USART_CR1 &= ~(1 << 12);
    } else if (uart.conf.m_bits == MBIT_9) {
        uart.uart->USART_CR1 |= (1 << 12);
    }
    if (uart.conf.stop_bits == STOP_BIT_1) {
        uart.uart->USART_CR2 &= ~(3 << 12);
    } else if (uart.conf.stop_bits == STOP_BIT_0_5) {
        uart.uart->USART_CR2 &= ~(3 << 12);
        uart.uart->USART_CR2 |= (1 << 12);
    } else if (uart.conf.stop_bits == STOP_BIT_2) {
        uart.uart->USART_CR2 &= ~(3 << 12);
        uart.uart->USART_CR2 |= (1 << 13);
    } else if (uart.conf.stop_bits == STOP_BIT_1_5) {
        uart.uart->USART_CR2 |= (3 << 12);
    }
    if (uart.conf.over_sample == OVER16 ) {
        uart.uart->USART_CR1 &= ~(1 << 15);
    } else if (uart.conf.over_sample == OVER8) {
        uart.uart->USART_CR1 |= (1 << 15);
    }
    //uart.uart->USART_BRR  = uart.conf.baudrate;
    //uart.uart->USART_BRR  = 0x683;
    set_baudrate(uart, PERI_CLK, uart.conf.baudrate);
    if (uart.conf.mode == MODE_RX) {
         uart.uart->USART_CR1 |= ( 1<< 2);
         uart.uart->USART_CR1 |= ( 1<< 5);
    } else if (uart.conf.mode == MODE_TX) {
         uart.uart->USART_CR1 |= ( 1<< 3);
         uart.uart->USART_CR1 |= ( 1<< 6);
    } else if (uart.conf.mode == MODE_RX_TX) {
         uart.uart->USART_CR1 |= ( 1<< 2);
         //uart.uart->USART_CR1 |= ( 1<< 5);
         uart.uart->USART_CR1 |= ( 1<< 3);
         //uart.uart->USART_CR1 |= ( 1<< 6);
    }
    uart.uart->USART_CR1 |= ( 1<< 13);
    
}

void uart_tx(uart_handle_t uart, uint8_t data) {
   while(!(uart.uart->USART_SR & ( 1 << 6)));
   uart.uart->USART_DR = data;
}


uint8_t uart_rx(uart_handle_t uart) {
   while(!(uart.uart->USART_SR & ( 1 << 5)));
    return uart.uart->USART_DR;
}

//void uart_write(uart_handle_t uart, void *data, int len) 
//{
//    if (data == NULL) {
//        return;
//    }
//    uint8_t *buf = (uint8_t *) data;
//    for (int i = 0;i<len;i++) {
//        uart_tx(uart,buf[i]); 
//        //uart_tx(uart, *(((uint8_t *)data) + i);
//        //uart_tx(uart, ((uint8_t *)data)[i]);
//    }
//}
//
//int  uart_read(uart_handle_t uart, void *data, int len) 
//{
//    if (data == NULL) {
//        return 0;
//    }
//    int i = 0;
//    while ( i < len) {
//        ((uint8_t *)data)[i++] = uart_rx(uart);
//    }
//    return i;
//}

void uart_write(uart_handle_t uart, void *data, int len) 
{
    if (data == NULL) {
        return;
    }
    uint8_t *buf = (uint8_t *) data;
    for (int i = 0;i<len;i++) {
        tx_buf[tx_head++] = buf[i];
        if (tx_head >= sizeof(tx_buf))
            tx_head = 0;
    }
    if(!tx_busy) {
       uart.uart->USART_SR |= ( 1 << 6);
       tx_busy = true;
    }

}

int  uart_read(uart_handle_t uart, void *data, int len) 
{
    if (data == NULL) {
        return 0;
    }
    int i = 0;
    while ( i < len) {
        if (rx_tail == rx_head) {
            return i;
        }
        ((uint8_t *)data)[i++] = rx_buf[rx_tail++];
        if (rx_tail >= sizeof(rx_buf)) 
            rx_tail = 0;
    }
    return i;
}
