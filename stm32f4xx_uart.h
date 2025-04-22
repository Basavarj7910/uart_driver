#ifndef __USART__
#define __USART__
#include <stdint.h>
#include "stm32f4xx.h"
typedef struct {
    uint8_t mode;
    uint8_t parity;
    uint8_t stop_bits;
    uint8_t over_sample;
    uint8_t m_bits;
    uint32_t baudrate;
}uart_conf_t;

typedef struct {
    uart_conf_t conf;
    USART_Reg_Def_t *uart;
}uart_handle_t;


typedef enum {
    MODE_RX = 0,
    MODE_TX = 1,
    MODE_RX_TX = 2
}usart_mode_t;

typedef enum {
   MBIT_8 = 0,
   MBIT_9 = 1
}usart_mbit_t;

typedef enum {
    OVER16 = 0,
    OVER8 = 1,
}over_t;

typedef enum {
    EVEN_PARITY = 0,
    ODD_PARITY = 1,
}usart_parity_t;

typedef enum {
    STOP_BIT_1 = 0,
    STOP_BIT_0_5 = 1,
    STOP_BIT_2 = 2,
    STOP_BIT_1_5 = 3,
}usart_stop_bits_t;


/* *************************************************************
 * @fn            - uart_init
 *
 * @brief         - Function to initialize the usart
 * 
 * @param[in]     - handle of the usart
 * 
 * @return        - null
 * note           - null
 */

void uart_init(uart_handle_t uart);

/* *************************************************************
 * @fn            - uart_tx
 *
 * @brief         - Function to send the data
 * 
 * @param[in]     - handle of the usart
 * @param[in]     - data going to trasmit
 * 
 * @return        - null
 * note           - null
 */

void uart_tx(uart_handle_t uart, uint8_t data);

/* *************************************************************
 * @fn            - uart_rx
 *
 * @brief         - Function to send the data
 * 
 * @param[in]     - handle of the usart
 * 
 * @return        - received data
 * note           - null
 */

uint8_t uart_rx(uart_handle_t uart);
/* *************************************************************
 * @fn            - uart_write
 *
 * @brief         - Function to send array of data 
 * 
 * @param[in]     - handle of the usart
 * @param[in]     - pointer to buffer
 * @param[in]     - length of data
 * 
 * @return        - void
 * note           - null
 */

void uart_write(uart_handle_t uart, void *data, int len);
/* *************************************************************
 * @fn            - uart_read
 *
 * @brief         - Function to read array of data
 * 
 * @param[in]     - handle of the usart
 * @param[in]     - pointer to buffer
 * @param[in]     - length of data
 * 
 * @return        - length of data recieved
 * note           - null
 */

int  uart_read(uart_handle_t uart, void *data, int len);
#endif
