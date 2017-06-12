/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

#include "nRF24.h";
#include "main.h";

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

volatile int timer_ms;

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


void     HAL_SYSTICK_Callback(void)
{
	timer_ms++;
}




#include <stdarg.h>

void Printf(const char *fmt, ...)
{
	static char buf[256];
	char *p;
	va_list lst;

	va_start(lst, fmt);
	vsprintf(buf, fmt, lst);
	va_end(lst);

	p = buf;
	while(*p) {
		HAL_UART_Transmit(&huart1, (unsigned char *)p, 1, 500);
		p++;
	}
}


#define	RX_DR			0x40
#define	TX_DS			0x20
#define	MAX_RT			0x10

#define TX_PLOAD_WIDTH 32

unsigned char TX_ADDRESS[TX_ADR_WIDTH] = {0xb2,0xb2,0xb3,0xb4,0x01};
unsigned char RX_BUF[TX_PLOAD_WIDTH];
unsigned char TX_BUF[TX_PLOAD_WIDTH];


#define CE(x) HAL_GPIO_WritePin(GPIOB, CE_Pin, x)
#define CSN(x) HAL_GPIO_WritePin(GPIOB, CSN_Pin, x)
#define IRQ HAL_GPIO_ReadPin(GPIOB, IRQ_Pin)


unsigned char SPI_Receive_byte(unsigned char reg)
{
	HAL_SPI_TransmitReceive(&hspi1, &reg, &reg, 1, 100);
   return reg;
}

unsigned char SPI_Send_byte(unsigned char reg)
{
	HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
   return reg;
}

unsigned char  SPI_Write_Buf(unsigned char  reg, unsigned char  *pBuf, unsigned char  bytes)
{
	unsigned char  status,byte_ctr;
	CSN(0);
	status= SPI_Receive_byte(reg);
	HAL_Delay(10); //delayMicroseconds(10);//delay1us(1);
	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
		SPI_Send_byte(*pBuf++);
	CSN(1);
	return(status);
}

unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char bytes)
{
	unsigned char status;//,byte_ctr;
	CSN(0);
	status=SPI_Receive_byte(reg);
	unsigned char * bufer[32];
	HAL_SPI_TransmitReceive(&hspi1, bufer, pBuf, bytes, 100);
	CSN(1);
	return(status);
}

unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;
	CSN(0);
	status=SPI_Receive_byte(reg);   //select register  and write value to it
	SPI_Send_byte(value);
	CSN(1);
	return(status);
}

unsigned char SPI_Read_Reg(unsigned char reg)
{
	unsigned char status;
	CSN(0);
	SPI_Send_byte(reg);
	status=SPI_Receive_byte(0);   //select register  and write value to it
	CSN(1);
	return(status);
}


void TX_Mode(unsigned char * tx_buf)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // {0xb2,0xb2,0xb3,0xb4,0x01}
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH); 			 // wr payload
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);       // Enable AA to 1-5 pipe
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);   // Enable rx data to 1-5 pipe
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + SETUP_RETR, 0x0a);  // number retr to error
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);         // work channel
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);    // no carrier;  no encoder; 1Mbps; 0dbm
	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // num width payload
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0e);      // Enable CRC; 2byte CRC; Power UP; PTX;
	CE(1);
	//delay(1);//delay1us(10);
}

void RX_Mode(void)
{
	CE(0);
  	SPI_Write_Buf(WRITE_REG_NRF24L01 + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // address 5 byte is {0xb2,0xb2,0xb3,0xb4,0x01} to pipe 0
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RX_PW_P0, TX_PLOAD_WIDTH);  // 

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_AA, 0x00);//0x00);//0x3f);               // enable ask
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + EN_RXADDR, 0x3f);           // set address rx
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_CH, 125);                 // channel

  	SPI_RW_Reg(WRITE_REG_NRF24L01 + RF_SETUP, 0x07);            // 0dbm 1MBps
  	SPI_RW_Reg(WRITE_REG_NRF24L01 + CONFIG, 0x0f);               // Enable CRC; 2byte CRC; Power UP; PRX;
  	CE(1);
}


void NRF24L01_Receive(void)
{
    unsigned char status=0x01;

	CE(0);
	HAL_Delay(2);//delayMicroseconds(10);//delay1us(10);

	status=SPI_Read_Reg(STATUS);

	if(status & 0x40)
	{
		SPI_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer

		Printf("RX --");
		Printf("data zerro is: 0x%x\r\n",RX_BUF[0]);
		//int t=0; for (t=0; t<32; t++)printf("0x%x,",RX_BUF[t]); printf("\r\n");

		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x40);
	}
	else if(status&TX_DS)
	{
		Printf("data sended INRX\r\n");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);
	}
	else if(status&MAX_RT)
	{
	Printf("NO SEND..INRX\n\r");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);
	}

	CE(1);

}

void NRF24L01_Send(void)
{
    unsigned char status=0x00;

	unsigned char bbuuff[32] = {0x33,0x12,0x31,0x54,0x25,0x16,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

	TX_Mode(bbuuff);

	while(IRQ == GPIO_PIN_SET);

	CE(0);
	HAL_Delay(2);//delayMicroseconds(10);//delay1us(10);

	status=SPI_Read_Reg(STATUS);	 
	if(status&TX_DS)	/*tx_ds == 0x20*/
	{
		Printf("data sended f\r\n");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x20);     

	}
	if(status&MAX_RT)
	{
		Printf("NO SEND.. f\n\r");
		SPI_RW_Reg(WRITE_REG_NRF24L01 + STATUS, 0x10);  
	}

	CE(1);

}






/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  Printf("Started..\n\r");



  CSN(GPIO_PIN_RESET);
  CE(GPIO_PIN_SET);



  RX_Mode();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if (IRQ == GPIO_PIN_RESET)
		  {
		  	  NRF24L01_Receive();
		  	  RX_Mode(); // Switch to RX mode
		  }

	  if (timer_ms>1000){
		  if (SPI_Read_Reg(CD) != 0) {continue;}	// If RF line is busy - no send
		  NRF24L01_Send();
		  RX_Mode();		// Switch to RX mode
		  timer_ms=0;
	  }





  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
