//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

//Function prototypes
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void ADC_Init(void);
void DAC_Init(void);
void SPI_Init(void);
void LCD_Init(void);
void LCD_SendBits(uint8_t);
void LCD_Send_Message(uint16_t);
void LCD_Write(uint16_t, uint16_t);

//Global variables
int timerTriggered = 0;
float fs = 48000000;
float freq = 0;
SPI_HandleTypeDef SPI_Handle;

void SystemClock48MHz( void )
{
//
// Disable the PLL
//
    RCC->CR &= ~(RCC_CR_PLLON);
//
// Wait for the PLL to unlock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != 0 );
//
// Configure the PLL for a 48MHz system clock
//
    RCC->CFGR = 0x00280000;

//
// Enable the PLL
//
    RCC->CR |= RCC_CR_PLLON;

//
// Wait for the PLL to lock
//
    while (( RCC->CR & RCC_CR_PLLRDY ) != RCC_CR_PLLRDY );

//
// Switch the processor to the PLL clock source
//
    RCC->CFGR = ( RCC->CFGR & (~RCC_CFGR_SW_Msk)) | RCC_CFGR_SW_PLL;

//
// Update the system with the new clock frequency
//
    SystemCoreClockUpdate();

}

int main(int argc, char* argv[])
{

	SystemClock48MHz();
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	ADC_Init();			/*Initialize ADC*/
	DAC_Init();			/*Initialize DAC*/
	SPI_Init();			/*Initialize SPI*/
	LCD_Init();			/*Initialize LCD*/

	//Variables for resistance and frequency for LCD screen
	uint16_t resistance = 0;
	float frequency = 0;

	while (1)
	{
		//Start conversions
		ADC1->CR |= ADC_CR_ADSTART;
		//Wait for EOC flag
		while(!(ADC1->ISR & ADC_ISR_EOC));

		//Calculate resistance from ADC value
		resistance = (ADC1->DR*5000)/4095;
		//Frequency placeholder for LCD
		frequency = freq;

		//Load DAC buffer with ADC data
		DAC->DHR12R1 = ADC1->DR;
		//Trigger software trigger to load DAC
		DAC->SWTRIGR = DAC_SWTRIGR_SWTRIG1;

		//Wait for DAC conversion
		while(DAC->SWTRIGR);
		//Write resistance and frequency to LCD
		LCD_Write(resistance, (uint16_t)frequency);
	}

	return 0;
}

void myGPIOA_Init(){
	//Enable GPIOA
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//Enable ADC1 clock
	RCC->APB2ENR |= (1<<9);

	//PA0 as input for pot
	GPIOA->MODER &= ~(GPIO_MODER_MODER0);

	//PA1 as input for 555 timer
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);

	//PA4 as general-purpose output mode for DAC output
	GPIOA->MODER  |= GPIO_MODER_MODER4;

	//Ensure no pull-up/pull-down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
}

void myGPIOB_Init(){
	//Enable GPIOB
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	//Set PB3 to alternate function mode
	//PB3 as SPI clock
	GPIOB->MODER |= GPIO_MODER_MODER3_1;

	//PB4 as general-purpose output mode
	//PB4 as SPI latch
	GPIOB->MODER |= GPIO_MODER_MODER4_0;

	//Set PB5 to alternate function mode
	//PB5 as MOSI
	GPIOB->MODER |= GPIO_MODER_MODER5_1;

	//Ensure no pull-up/pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR5);
}

void myTIM2_Init(){
	//Enable clock for TIM2 peripheral
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	//Enable timer interrupts
	TIM2->CR1 = ((uint16_t)0x008C);

	//Set clock prescaler value
	TIM2->PSC = myTIM2_PRESCALER;

	//Set auto-reloaded delay
	TIM2->ARR = myTIM2_PERIOD;

	//Update timer registers
	TIM2->EGR = ((uint16_t)0x0001);

	//Assign TIM2 interrupt priority = 0 in NVIC
	NVIC_SetPriority(TIM2_IRQn, 0);

	//Enable TIM2 interrupts in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);

	//Enable update interrupt generation
	TIM2->DIER |= 0x1;
}

void TIM2_IRQHandler(){
	//Check if update interrupt flag is set
	if ((TIM2->SR & TIM_SR_UIF) != 0){
		trace_printf("\n*** Overflow! ***\n");

		//Clear update interrupt flag
		TIM2->SR &= ~(TIM_SR_UIF);

		//Restart stopped timer
		TIM2->CR1 |= (1<<0);
	}
}

void myEXTI_Init(){
	//Map EXTI1 line to PA1
	SYSCFG->EXTICR[0] |= 0x80;

	//EXTI1 line interrupts: set rising-edge trigger
	EXTI->RTSR |= (1<<1) ;

	//Unmask interrupts from EXTI1 line
	EXTI->IMR |= (1<<1);

	//Assign EXTI1 interrupt priority = 0 in NVIC
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	//Enable EXTI1 interrupts in NVIC
	NVIC_EnableIRQ(EXTI0_1_IRQn);
}



void EXTI0_1_IRQHandler(){
	//Check if EXTI1 interrupt pending flag is set
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		//First timer rising edge; start count
		if(timerTriggered == 0){
			//Mask interrupt
			EXTI->IMR &= ~(1<<1);

			//Restart timer
			TIM2->CR1 |= (1<<0);
			//Reset count
			TIM2->CNT = 0;

			//Unmask interrupt
			EXTI->IMR |= (1<<1);
			//Count rising edge count
			timerTriggered = 1;
		}
		//Second timer rising edge; stop count
		else {
			//Mask interrupt
			EXTI->IMR &= ~(1<<1);

			//Stop timer
			TIM2->CR1 &= ~(1<<0);

			//Read count; calculate frequency
			unsigned long int count = TIM2->CNT;
			freq = fs/count;

			//Un-mask interrupt
			EXTI->IMR |= (1<<1);
			//Reset rising edge count
			timerTriggered = 0;
		}

		//Reset interrupt flag
		EXTI -> PR |= (1<<1);
	}
}

void ADC_Init(){
	ADC1->CR |= ADC_CR_ADEN; //Enable ADC
	ADC1->CR &= ~ADC_CR_ADSTART; //Disable start conversions
	ADC1->CFGR1 |= ADC_CFGR1_CONT; //Continuous conversion mode
	ADC1->CHSELR = ADC_CHSELR_CHSEL0; //Select channel 0 (disable others)
	ADC1->CFGR2 = ADC_CFGR2_CKMODE; //ADC Asynchronous clock mode
	ADC1->SMPR = ADC_SMPR_SMP_0; //Sampling time selection
	ADC1->CFGR1 &= ~ADC_CFGR1_AWDEN; //Disable watchdog
	ADC1->CFGR1 &= ~ADC_CFGR1_SCANDIR; //Scan sequence direction (upward scan)
	ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN; //Data alignment (right)
	ADC1->CFGR1 &= ~ADC_CFGR1_EXTEN; //Disable external trigger conversion mode
	ADC1->CFGR1 &= ~ADC_CFGR1_OVRMOD; //Overrun management mode
	ADC1->CFGR1 &= ~ADC_CFGR1_WAIT; //Disable wait conversion mode
	ADC1->CFGR1 &= ~ADC_CFGR1_DISCEN; //Disable discontinuous mode
	ADC1->IER |= ADC_IER_ADRDYIE; //ADC Ready interrupt (enable)
	ADC1->IER |= ADC_IER_EOCIE; //End of conversion interrupt (enable)
	ADC1->IER |= ADC_IER_EOSMPIE; //End of sampling interrupt (enable)
	ADC->CCR |=  ADC_CCR_VREFEN; //Vref internal (enable)
	ADC1->CFGR1 &= ~ADC_CFGR1_RES; //Set resolution (12)
}

void DAC_Init()
{
	//Enable DAC clock
	RCC->APB1ENR |= 0x20000000;
	//Enable the DAC Channel 1
	DAC->CR |= 0x1;
	//Set the trigger to software
	DAC->CR |= DAC_CR_TSEL1;
}

void SPI_Init(){
	//Enable clock for SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	//Configure SPI1
	SPI_Handle.Instance = SPI1;
	SPI_Handle.Init.Direction = SPI_DIRECTION_1LINE;
	SPI_Handle.Init.Mode = SPI_MODE_MASTER;
	SPI_Handle.Init.DataSize = SPI_DATASIZE_8BIT;
	SPI_Handle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SPI_Handle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SPI_Handle.Init.NSS = SPI_NSS_SOFT;
	SPI_Handle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	SPI_Handle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SPI_Handle.Init.CRCPolynomial = 7;

	//Initialize SPI instance with desired configuration
	HAL_SPI_Init(&SPI_Handle);
	//Enable SPI
	__HAL_SPI_ENABLE(&SPI_Handle);
}

void LCD_SendBits(uint8_t Bits_8){
	//Set latch bit for PB4 low
	GPIOB->BSRR |= (uint32_t)0x100000;

	//Wait for SPI buffer to be empty
	while((SPI1->SR & (uint16_t)0x2) == 0 ){};

	//Transmit 8 bits of data
	HAL_SPI_Transmit(&SPI_Handle, (uint8_t*)&Bits_8, sizeof(Bits_8), HAL_MAX_DELAY);

	//Wait for SPI buffer to be empty
	while((SPI1->SR & (uint16_t)0x2) == 0 ){};

	//Set latch bit for PB4 high
	GPIOB->BSRR |= (uint32_t)0x10;
}

void LCD_Send_Message(uint16_t message){
	//Isolate RS bit
	uint8_t RS = (message & 0x0200)>>9;
	//Isolate High bits [7:4]
	uint8_t High = (message & 0x00F0)>> 4;
	//Isolate Low bits [3:0]
	uint8_t Low = (message & 0x000F);

	//Create first high message (EN = 0)
	uint8_t High0 = ((RS << 6 ) | High);
	//Create second high message (EN = 1)
	uint8_t High1 = ((1 << 7) | (RS << 6) | High);

	//Create first low message (EN = 0)
	uint8_t Low0 = ((RS << 6 ) | Low);
	//Create second low message (EN = 1)
	uint8_t Low1  = ((1 << 7) | (RS << 6) | Low);

	//Send 3 high messages
	LCD_SendBits(High0);
	LCD_SendBits(High1);
	LCD_SendBits(High0);

	//Send 3 low messages
	LCD_SendBits(Low0);
	LCD_SendBits(Low1);
	LCD_SendBits(Low0);
}

void LCD_Init(){
	// Set to 4 bit mode; No low sent
	LCD_SendBits(0x02);
    LCD_SendBits(0x82);
	LCD_SendBits(0x02);

	//Set LCD to use 2 lines of characters
	LCD_Send_Message(0x0028);

	//Turn of cursor; turn on LCD
	LCD_Send_Message(0x000C);
	//Display shift off; auto-increment on
	LCD_Send_Message(0x0006);
	//Clear display
	LCD_Send_Message(0x0001);
}

void LCD_Write(uint16_t resistance, uint16_t frequency){
	// delay to allow for smooth update
	for(int i = 0; i < 500000; i++){}
	//Limit data to 4 digits for LCD space provided
	resistance = resistance % 10000;
	frequency = frequency % 10000;

	//Break data down into individual digits
	uint16_t res4 = resistance % 10;
	uint16_t res3 = ((resistance - res4) % 100)/10;
	uint16_t res2 = ((resistance - res3 - res4) % 1000)/100;
	uint16_t res1 = resistance/1000;
	uint16_t freq4 = frequency % 10;
	uint16_t freq3 = ((frequency - freq4) % 100)/10;
	uint16_t freq2 = ((frequency - freq3 - freq4) % 1000)/100;
	uint16_t freq1 = frequency/1000;

	// delay to allow for smooth update
	for(int i = 0; i < 500000; i++){}

	//Select first line of LCD
	LCD_Send_Message(0x0080);
	//Write 'F:' to LCD
	LCD_Send_Message(0x0200 | 0x0046);
	LCD_Send_Message(0x0200 | 0x003A);
	//Write frequency digits to LCD
	LCD_Send_Message(0x0200 | 0x0030 | freq1);
	LCD_Send_Message(0x0200 | 0x0030 | freq2);
	LCD_Send_Message(0x0200 | 0x0030 | freq3);
	LCD_Send_Message(0x0200 | 0x0030 | freq4);
	//Write 'Hz' to LCD
	LCD_Send_Message(0x0200 | 0x0048);
	LCD_Send_Message(0x0200 | 0x007A);

	//Select second line of LCD
	LCD_Send_Message(0x00C0);
	//Write 'R:' to LCD
	LCD_Send_Message(0x0200 | 0x0052);
	LCD_Send_Message(0x0200 | 0x003A);
	//Write resistance digits to LCD
	LCD_Send_Message(0x0200 | 0x0030 | res1);
	LCD_Send_Message(0x0200 | 0x0030 | res2);
	LCD_Send_Message(0x0200 | 0x0030 | res3);
	LCD_Send_Message(0x0200 | 0x0030 | res4);
	//Write ohm symbol to LCD
	LCD_Send_Message(0x0200 | 0x00F4);
}




#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
