#ifndef __DEBUG_USART_HAL_H__
#define __DEBUG_USART_HAL_H__


#include "usart.h"
#include "main.h"


#define  data_len  100

typedef struct
{
    uint8_t  RX_flag;        //IDLE receive flag
    uint16_t RX_Size;          //receive length
    uint8_t  RX_pData[data_len];
}ETH_TYPE;



#endif








