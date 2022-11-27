 /**
   ****************************(C) COPYRIGHT 2020 NCIST****************************
   * @file       rc.c
   * @brief      			
   *             		遥控及接收机使用USART2
   *             		PA3：RX
   *             
   * @note       
   * @history
   *  Version    Date            Author          Modification
   *  V1.0.0     2021-07-17     	   RM              1. 完成
   *
   @verbatim
   ==============================================================================

   ==============================================================================
   @endverbatim
   ****************************(C) COPYRIGHT 2020 NCIST****************************
   */
//#include "main.h"
#include "rc.h"
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)//数组0 1   缓存长度
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);// | RCC_AHB1Periph_DMA1
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);//RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART2, DISABLE);
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, rc_GPIO_AF_USARTx); //PB11  USART2 rx
    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO_Speed_100MHz
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        USART_DeInit(rc_USARTx);

        USART_InitStructure.USART_BaudRate = 100000;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(rc_USARTx, &USART_InitStructure);

        USART_DMACmd(rc_USARTx, USART_DMAReq_Rx, ENABLE);

        USART_ClearFlag(rc_USARTx, USART_FLAG_IDLE);
        USART_ITConfig(rc_USARTx, USART_IT_IDLE, ENABLE);

        USART_Cmd(rc_USARTx, ENABLE);
    }

    /* -------------- Configure NVIC ---------------------------------------*/
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = rc_USARTx_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }


    /* -------------- Configure DMA -----------------------------------------*/
    {
        DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(rc_DMAx_Streamx);

        DMA_InitStructure.DMA_Channel = rc_DMA_Channel_x;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (rc_USARTx->DR);
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
        DMA_InitStructure.DMA_BufferSize = dma_buf_num;//
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(rc_DMAx_Streamx, &DMA_InitStructure);
        DMA_DoubleBufferModeConfig(rc_DMAx_Streamx, (uint32_t)rx2_buf, DMA_Memory_0);
        DMA_DoubleBufferModeCmd(rc_DMAx_Streamx, ENABLE);
        DMA_Cmd(rc_DMAx_Streamx, DISABLE); //Add a disable  DMA_Cmd(rc_DMAx_Streamx, DISABLE)
        DMA_Cmd(rc_DMAx_Streamx, ENABLE);//   DMA_Cmd(DMA  1_Stream5, ENABLE)
    }
}
void RC_unable(void)
{
    USART_Cmd(rc_USARTx, DISABLE);
}
void RC_restart(uint16_t dma_buf_num)
{
    USART_Cmd(rc_USARTx, DISABLE);
    DMA_Cmd(rc_DMAx_Streamx, DISABLE);
    DMA_SetCurrDataCounter(rc_DMAx_Streamx, dma_buf_num);

    USART_ClearFlag(rc_USARTx, USART_FLAG_IDLE);

    DMA_ClearFlag(rc_DMAx_Streamx, DMA_FLAG_TCIF4);
    DMA_ClearITPendingBit(rc_DMAx_Streamx, DMA_IT_TCIF4);
    DMA_Cmd(rc_DMAx_Streamx, ENABLE);
    USART_Cmd(rc_USARTx, ENABLE);
}




