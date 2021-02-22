/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "platform_config.h"

extern uint8_t TxBuffer1[]; 
extern uint8_t TxBuffer2[]; 
extern uint8_t RxBuffer1[];
extern uint8_t RxBuffer2[];
extern __IO uint8_t TxCounter1;
extern __IO uint8_t TxCounter2;
extern __IO uint8_t RxCounter1; 
extern __IO uint8_t RxCounter2;
extern uint8_t NbrOfDataToTransfer1;
extern uint8_t NbrOfDataToTransfer2;
extern uint8_t NbrOfDataToRead1;
extern uint8_t NbrOfDataToRead2;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
int i = 0;

void USARTy_IRQHandler(void)
{
  if(USART_GetITStatus(USARTy, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USARTy, TxBuffer1[TxCounter1++]);

    if(TxCounter1 == NbrOfDataToTransfer1)
    {
      /* Disable the USARTy Transmit interrupt */
      USART_ITConfig(USARTy, USART_IT_TXE, DISABLE);
    }    
  }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
