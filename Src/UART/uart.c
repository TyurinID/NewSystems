#include	"uart.h"

extern uint32_t	GTimerDebug;

UART_HandleTypeDef hUart3;
uint32_t GTahoPeriod;
uint32_t GOvalPeriod;

static uint8_t	uartTxReady;
static uint8_t	buffer_Tx[TXBUFFERSIZE] = {'A', 'N', 'S', 0x00, 0x00};
static uint8_t	buffer_Rx[RXBUFFERSIZE];
static uint8_t	answer_done = 1;


static void HandleAnswer(void);
static uint8_t	GetMessage(uint8_t *message);
static void HandleReceive(uint8_t *message);


void USART3_Init(void)						// Configure the UART peripheral.
{
	
	// Configure protocol parameters: 
	hUart3.Instance 					= USART3;
  hUart3.Init.BaudRate 			= 115200;
  hUart3.Init.WordLength 		= UART_WORDLENGTH_8B;
  hUart3.Init.StopBits 			= UART_STOPBITS_1;
  hUart3.Init.Parity 				= UART_PARITY_NONE;
  hUart3.Init.Mode 					= UART_MODE_TX_RX;
  hUart3.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;
  hUart3.Init.OverSampling 	= UART_OVERSAMPLING_16;
	
  if (HAL_UART_Init(&hUart3) != HAL_OK)
  {
//    Error_Handler();
  }
	
	HAL_UART_Receive_DMA(&hUart3, (uint8_t *)buffer_Rx, RXBUFFERSIZE);			// Start recieving.
	uartTxReady = 1;
	
	GTahoPeriod = 300;
	GOvalPeriod = 150;
}

void	USART3_Process(void)
{
	if(answer_done == 0)
	{
		if(uartTxReady == 1)
		{
			uartTxReady = 0;
			HandleAnswer();
			answer_done = 1;
		}
	}
	
	if( __HAL_UART_GET_FLAG(&hUart3, UART_FLAG_IDLE) == SET)   // check idle state
	{
		__HAL_UART_CLEAR_IDLEFLAG(&hUart3);
		for(uint8_t i = 0; i < 10; i++)
			__NOP();
		
		uint8_t message[MAX_MESSAGE_LENGTH];
		
		if(GetMessage(message) == 1)
		{	
			HandleReceive(message);
			answer_done = 0;
		}
	}	
}

void HandleAnswer(void)
{
	buffer_Tx[4] = (uint8_t)(GTahoPeriod >> 24);
	buffer_Tx[5] = (uint8_t)(GTahoPeriod >> 16);
	buffer_Tx[6] = (uint8_t)(GTahoPeriod >> 8);
	buffer_Tx[7] = (uint8_t)(GTahoPeriod);
	
	buffer_Tx[8] = (uint8_t)(GOvalPeriod >> 24);
	buffer_Tx[9] = (uint8_t)(GOvalPeriod >> 16);
	buffer_Tx[10] = (uint8_t)(GOvalPeriod >> 8);
	buffer_Tx[11] = (uint8_t)(GOvalPeriod);
	
	HAL_UART_Transmit_DMA(&hUart3, (uint8_t*)buffer_Tx, 12); 
}

uint8_t	GetMessage(uint8_t *message)
{
	uint8_t	begin_ptr = 0xff;
		
	for(uint8_t i = 0; i < RXBUFFERSIZE; i++)
	{
		if('E' == buffer_Rx[i])
		{
			begin_ptr = i;
			break;
		}
	}
	if(begin_ptr != 0xff)
	{		
		for(uint8_t i=0; i<MAX_MESSAGE_LENGTH; i++)
		{																				
			uint8_t temp_ptr = i+begin_ptr;				
			if(temp_ptr >= RXBUFFERSIZE)	
				temp_ptr -= RXBUFFERSIZE;
			message[i] = buffer_Rx[temp_ptr];	
			
			buffer_Rx[temp_ptr] = 0xff;
		}
		return 1;
	}
	else
		return 0;
}

void HandleReceive(uint8_t *message)
{
//	GUART_Error = UART_OK;
	
	if( 'E' == message[0] && 'M' == message[1] && 'U' == message[2] && 0x00 == message[3] )  //  check start frame delimeter
	{	
		uint32_t temp1 = ((((uint32_t)message[4])<<24UL) | (((uint32_t)message[5])<<16UL) | (((uint32_t)message[6])<<8UL) | ((uint32_t)message[7]));
		if(temp1 < 20 || temp1 > 600)
		{
//			GUART_Error = VERIFY_ERR;
		}
		else
				GTahoPeriod = temp1;
		
		temp1 = ((((uint32_t)message[8])<<24UL) | (((uint32_t)message[9])<<16UL) | (((uint32_t)message[10])<<8UL) | ((uint32_t)message[11]));
		if(temp1 < 150 || temp1 > 150000)
		{
//			GUART_Error = VERIFY_ERR;
		}
		else
				GOvalPeriod = temp1;
	}
//	else
//	GUART_Error = VERIFY_ERR;
}




/******* ======= INTERRUPTS ====== *********/
/*******************************************/

/**
  * @brief  This function handles DMA RX interrupt request.  
  * @param  None
  * @retval None   
  */
void USARTx_DMA_RX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hUart3.hdmarx);
}

/**
  * @brief  This function handles DMA TX interrupt request.
  * @param  None
  * @retval None   
  */
void USARTx_DMA_TX_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hUart3.hdmatx);
	  uartTxReady = 1; 
}

/**
  * @brief  This function handles USARTx interrupt request.
  * @param  None
  * @retval None
  */
void USARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(&hUart3);
	
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle. 
  * @note   This example shows a simple way to report end of DMA Tx transfer, and 
  *         you can add your own implementation. 
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  uartTxReady = 1; 
}
