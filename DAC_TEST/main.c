#include "stm32f4xx.h"
#include "system_timetick.h"
#include "math.h"
#include<string.h>
#include <stdlib.h>
#include <stdbool.h>

#define		BUFF_SIZE_RX		8
uint8_t data_Rx[BUFF_SIZE_RX];

#define 	PI 3.1415926

void TIM2_Config(uint16_t Frequency, uint16_t Ns);
void TIM7_Config(uint16_t Frequency, uint16_t Ns);

void DAC_Ch1_WaveConfig(uint16_t* value, uint16_t BUFF);
void DAC_Ch2_WaveConfig(uint16_t* value, uint16_t BUFF);
void DAC_Ch1_TriangleConfig(void);	// chuong trinh tao xung tam giac CHANNEL 1
void DAC_Ch2_TriangleConfig(void); // chuong trinh tao xung tam giac CHANNEL 2

#define		BUFF_SIZE_DAC_SIN			100		// SO LAN LAU MAU SONG SIN
void get_sinvalue_Ch1 ( float voltage);
uint16_t sinvalue_Ch1[BUFF_SIZE_DAC_SIN];
void get_sinvalue_Ch2 ( float voltage);
uint16_t sinvalue_Ch2[BUFF_SIZE_DAC_SIN];

#define		BUFF_SIZE_DAC_SRC			100	// SO LAN LAU MAU SONG RANG CUA
void get_srcvalue_Ch1( float voltage);
uint16_t srcvalue_Ch1[BUFF_SIZE_DAC_SRC];
void get_srcvalue_Ch2( float voltage);
uint16_t srcvalue_Ch2[BUFF_SIZE_DAC_SRC];

#define		BUFF_SIZE_DAC_SV			100		// SO LAN LAU MAU SONG VUONG
void get_svvalue_Ch1( float voltage);
uint16_t svvalue_Ch1[BUFF_SIZE_DAC_SV]; 	
void get_svvalue_Ch2( float voltage);
uint16_t svvalue_Ch2[BUFF_SIZE_DAC_SV]; 	

uint16_t wavevalue_Ch2[2];
void get_wavevalue_Ch2( float voltage);

void UART_Common_Config(void);	// cau hinh chung cho MODE UART_TX
void Display1( uint8_t* Addr_txbuff, uint16_t BUFF   );
void UART_Rx_Config(void);			//Cau hinh cho MODE UART_RX
void TIM2_IRQHandler(void);
void	IntToStr4 (uint16_t u, uint8_t *y);

DMA_InitTypeDef  	DMA_InitStructure;

void Test_led(void);	//Cai hinh led hoat dong

int ATD(uint8_t A[], uint8_t i); //Chuyen mang ASCII sang so

uint8_t Rx_Hz[4]="0000";
uint8_t Rx_V[2]="00";
uint16_t Hz1=0;	//tan so kenh 1
float V1=0;		// bien do kenh 1
uint16_t Hz2=0;
float V2=0;

int main(void)
{
		// Cau hinh chan
	GPIO_InitTypeDef GPIO_InitStructure; 	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE); 
	// 2 kenh DAC PA4,PA5
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN; //Chon mode analog
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	Test_led();
	UART_Common_Config();
	UART_Rx_Config();
	while(1){

					}
}

void UART_Rx_Config(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;  
	DMA_InitTypeDef   DMA_InitStructure_Rx;
  NVIC_InitTypeDef  NVIC_InitStructure;	
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
  /* Enable USART */
  USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
	/* DMA1 Stream2 Channel4 for USART4 Rx configuration */			
  DMA_InitStructure_Rx.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure_Rx.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure_Rx.DMA_Memory0BaseAddr = (uint32_t)&data_Rx;
  DMA_InitStructure_Rx.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure_Rx.DMA_BufferSize = BUFF_SIZE_RX; //
  DMA_InitStructure_Rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure_Rx.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure_Rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure_Rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure_Rx.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Circular;
  DMA_InitStructure_Rx.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure_Rx.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure_Rx.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure_Rx.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure_Rx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &DMA_InitStructure_Rx);
  DMA_Cmd(DMA1_Stream2, ENABLE);
	/* Enable DMA Interrupt to the highest priority */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Transfer complete interrupt mask */
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
}
void DMA1_Stream2_IRQHandler(void)	//NHAN du 8 BYTE se chay CT NGAT, 8 BYTE do luu trong MANG data_Rx
{
  /* Clear the DMA1_Stream2 TCIF2 pending bit */
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
	DMA_Cmd(DMA1_Stream2, ENABLE);
	/* Kiem tra khi nao xay ra ngat */
	GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
		if(!(data_Rx[2] - 0x31))
		{	// TAN SO
			for(int i =0; i<4 ;i++)
			{
			Rx_Hz[i]=data_Rx[i+3];
			}
			Hz1=ATD(Rx_Hz,4);
			//BIEN DO
			for(int i=0;i<2;i++)
			{
			Rx_V[i]=data_Rx[i];
			}
			V1=ATD(Rx_V,2);
			// CAU HINH XUNG
			switch(data_Rx[7]-0x30)			//Chon kenh cau hinh
			{
				case 0:					//SONG SIN
				TIM2_Config(Hz1, BUFF_SIZE_DAC_SIN);
				get_sinvalue_Ch1(V1/10);
				DAC_Ch1_WaveConfig(sinvalue_Ch1, BUFF_SIZE_DAC_SIN);
				get_wavevalue_Ch2(V1/10);
				TIM7_Config(Hz1, BUFF_SIZE_DAC_SIN);
				DAC_Ch2_WaveConfig(wavevalue_Ch2,2);
				break;
				
				case 1:				//SONG VUONG
				get_svvalue_Ch1(V1/10);
				DAC_Ch1_WaveConfig(svvalue_Ch1, BUFF_SIZE_DAC_SV);				
				TIM2_Config(Hz1, BUFF_SIZE_DAC_SV);
				get_wavevalue_Ch2(V1/10);
				TIM7_Config(Hz1, BUFF_SIZE_DAC_SV);
				DAC_Ch2_WaveConfig(wavevalue_Ch2,2);
				break;
		
				case 2:		//	SONG RANG CUA
				TIM2_Config(Hz1, BUFF_SIZE_DAC_SRC);
				get_srcvalue_Ch1(V1/10);
				DAC_Ch1_WaveConfig(srcvalue_Ch1, BUFF_SIZE_DAC_SRC);
				get_wavevalue_Ch2(V1/10);
				TIM7_Config(Hz1, BUFF_SIZE_DAC_SRC);				
				DAC_Ch2_WaveConfig(wavevalue_Ch2,2);
				break;
				
				case 3:		//SONG TAM GIAC
				TIM2_Config(Hz1,2000);	
				DAC_Ch1_TriangleConfig();
				get_wavevalue_Ch2(V1/10);
				TIM7_Config(Hz1, 2046);
				DAC_Ch2_WaveConfig(wavevalue_Ch2,2);
				break;
				default:	// KHONG XUAT XUNG
				
				break;
			}
			// CT CHUNG	
			DAC_DMACmd(DAC_Channel_1, ENABLE);
			DMA_Cmd(DMA1_Stream5, ENABLE);		
			DAC_DMACmd(DAC_Channel_2, ENABLE);
			DMA_Cmd(DMA1_Stream6, ENABLE);
			TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	// DE BAT XUAT DAC LÊN UART
		}
		else
		{
		DAC_DeInit();
		TIM_DeInit(TIM2);
		TIM_DeInit(TIM7);
		}
}
void Display1( uint8_t* Addr_txbuff, uint16_t BUFF )
{
	  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)Addr_txbuff;
		DMA_InitStructure.DMA_BufferSize = BUFF;//BUFF_SIZE_UART;
	  DMA_Init(DMA1_Stream4, &DMA_InitStructure);
		DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
	  DMA_Cmd(DMA1_Stream4, ENABLE);		// phai cho phep truyen lai
		while(DMA_GetFlagStatus(DMA1_Stream4, DMA_FLAG_TCIF4)  == RESET );	
}

void UART_Common_Config(void)
{
  GPIO_InitTypeDef 	GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;   
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* Enable UART clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* Enable DMA1 clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  /* Connect UART4 pins to AF2 */  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_UART4); 
  /* GPIO Configuration for UART4 Tx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* GPIO Configuration for USART Rx */
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
       
  /* USARTx configured as follow:
		- BaudRate = 115200 baud  
    - Word Length = 8 Bits
    - One Stop Bit
    - No parity
    - Hardware flow control disabled (RTS and CTS signals)
    - Receive and transmit enabled
  */
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(UART4, &USART_InitStructure);
	/* Enable USART */
  USART_Cmd(UART4, ENABLE);
	/* Enable UART4 DMA */
  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE); 
		/* DMA1 Stream4 Channel4 for UART4 Tx configuration */			
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UART4->DR;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
}
void TIM2_Config(uint16_t Frequency, uint16_t Ns)
{
// Cau hình TIMER 2
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Period = 1000000/(Frequency*Ns)-1;        //10000 la tan so sau khi chia Prsacler  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // THEM TU EXAMPLE thay cho 3  dong duoi
	NVIC_InitTypeDef NVIC_InitStructure; 
	TIM_Cmd(TIM2, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&NVIC_InitStructure);
}
void TIM2_IRQHandler(void) // GUI MA ASCII DAC LÊN UART_TX
{	
	uint16_t DAC_value;
	uint8_t String_DAC_value[]="";
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) 
	{
	DAC_value = DAC->DOR1;
	IntToStr4(DAC_value,String_DAC_value);
	Display1(String_DAC_value, 4);
	}
TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
}
void	IntToStr4 (uint16_t u, uint8_t *y)
{
	uint16_t a;
	a = u;
	y[3] = a % 10 + 0x30;
	a = a/10;
	y[2] = a % 10 + 0x30;
	a = a/10;
	y[1] = a % 10 + 0x30;
	a = a/10;
	y[0] = a + 0x30;
}

void TIM7_Config(uint16_t Frequency, uint16_t Ns)
{
// Cau hình TIMER 2
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); 
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;  
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_Period = 1000000/(Frequency*Ns)-1;     
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_SelectOutputTrigger(TIM7, TIM_TRGOSource_Update);
	TIM_Cmd(TIM7, ENABLE);
}

void DAC_Ch1_WaveConfig(uint16_t* value, uint16_t BUFF)
{
	// Cau hinh DAC
	DAC->CR |= (6<<0)|(1<<5);		// Mode kich hoat ngoai TIM2; co bo dem
//Enable DMA1 clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);// cho phep DMA1-CA 2 KENH DAC
// cau hình DMA1 cho DAC1 channel 7 stream 5
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)value;//(uint32_t)sinvalue;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->DHR12R1);//(uint32_t)0x40007408;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = BUFF;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//do phan giai DAC 12bit nen DMA chon HALFWORD
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream5, &DMA_InitStructure);
	DAC_Cmd (DAC_Channel_1, ENABLE);
}
void DAC_Ch2_WaveConfig(uint16_t* value, uint16_t BUFF)
{
	DAC->CR |= (6<<16)|(1<<20);		// Mode kich hoat ngoai TIM7; co bo dem
	//Enable DMA1 clock
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);// cho phep DMA1-CA 2 KENH DAC
	// cau hình DMA1 cho DAC1 channel 7 stream 6
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Stream6);
	DMA_InitStructure.DMA_Channel = DMA_Channel_7;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)value;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(DAC->DHR12R2);	//khong phan biet DAC1 và DAC2
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = BUFF;//BUFF_SIZE_DAC_SIN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//do phan giai DAC 12bit nen DMA chon HALFWORD
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	//DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStructure);
	DAC_Cmd (DAC_Channel_2, ENABLE);
}
void DAC_Ch1_TriangleConfig(void)
{
	DAC_InitTypeDef DAC_InitStructure;
 /* DAC channel2 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T2_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
  /* Set DAC channel2 DHR12RD register */
  DAC_SetChannel2Data(DAC_Align_12b_R, 1023);
}
void DAC_Ch2_TriangleConfig(void)
{
	DAC_InitTypeDef DAC_InitStructure;
 /* DAC channel2 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T7_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);
  /* Enable DAC Channel2 */
  DAC_Cmd(DAC_Channel_2, ENABLE);
  /* Set DAC channel2 DHR12RD register */
  DAC_SetChannel2Data(DAC_Align_12b_R, 0);
}
void get_sinvalue_Ch1 ( float voltage)	// TAO MANG SONG SIN CHANNEL 1
{
	for(int i =0;i<BUFF_SIZE_DAC_SIN;i++)
	{
		sinvalue_Ch1[i] = (sin(i*2*PI/BUFF_SIZE_DAC_SIN)+1)*(((0xFFF+1)*voltage)/(2*4*1.05));
	}
}
void get_srcvalue_Ch1( float voltage) // TAO MANG SONG RANG CUA
{
	for(int i =0;i<BUFF_SIZE_DAC_SRC;i++)
	{
		srcvalue_Ch1[i] = (i)*(((0xFFF)*voltage)/(4*1.01*(BUFF_SIZE_DAC_SRC-1)));
	}
}
void get_svvalue_Ch1( float voltage) // TAO MANG SONG VUONG
{
	for(int i =0;i<BUFF_SIZE_DAC_SV/2;i++)
	{
		svvalue_Ch1[i] = 0;
	}
	
	for(int i =BUFF_SIZE_DAC_SV/2;i<BUFF_SIZE_DAC_SV;i++)
	{
		svvalue_Ch1[i] = (((0xFFF)*voltage)/(4*1.08));
	}
}
	void get_sinvalue_Ch2 ( float voltage)	// TAO MANG SONG SIN CHANNEL2
{
	for(int i =0;i<BUFF_SIZE_DAC_SIN;i++)
	{
		sinvalue_Ch2[i] = (sin(i*2*PI/BUFF_SIZE_DAC_SIN)+1)*(((0xFFF+1)*voltage)/(2*3.3));
	}
}
void get_srcvalue_Ch2( float voltage) // TAO MANG SONG RANG CUA
{
	for(int i =0;i<BUFF_SIZE_DAC_SRC;i++)
	{
		srcvalue_Ch2[i] = (i)*((0xFFF*voltage)/3.3*(BUFF_SIZE_DAC_SRC-1));
	}
}
void get_svvalue_Ch2( float voltage) // TAO MANG SONG VUONG
{
	for(int i =0;i<BUFF_SIZE_DAC_SV;i++)
	{
		svvalue_Ch2[i] = i*((0xFFF*voltage)/3.3);
	}
}
void get_wavevalue_Ch2( float voltage) // TAO MANG SONG VUONG
{
	for(int i =0;i<2;i++)
	{
		wavevalue_Ch2[i] = ((0xFFF*voltage)/(8));
	}
}
void Test_led(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14|GPIO_Pin_15; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //Chon mode analog
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

int ATD(uint8_t A[], uint8_t i)//Chuyen chuoi sang ma ASCII
{
	uint16_t D=0;
	for(int j=0;j<i;j++)
	{
		D=D+(A[j]-0x30)*pow(10,i-1-j);
	}
	return D;
}
