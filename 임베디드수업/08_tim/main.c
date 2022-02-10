#include "STM32FDiscovery.h"
#include <stdio.h>

unsigned char rec, adc_val;
unsigned int count = 0;
char buf[5];
int len;
void sendStr(char buf[], int max);
unsigned int tim2tick;

unsigned int uart_data[423]= {  // each is ascii code
32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 
32, 95, 95, 95, 95, 95, 32, 32, 32, 32, 
32, 32, 95, 95, 95, 95, 95, 32, 95, 95, 
95, 32, 32, 32, 32, 32, 32, 32, 32, 95, 
95, 95, 95, 95, 32, 32, 32, 95, 32, 32, 
32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 
32, 32, 32, 95, 95, 95, 95, 32, 32, 10, 
32, 32, 32, 32, 32, 47, 92, 32, 32, 32, 
124, 95, 32, 32, 32, 95, 124, 32, 32, 32, 
32, 47, 32, 95, 95, 95, 95, 124, 95, 95, 
32, 92, 32, 32, 32, 32, 32, 32, 47, 32,
95, 95, 95, 95, 124, 32, 124, 32, 124, 32, 
32, 32, 32, 32, 32, 32, 32, 47, 92, 32,
32, 32, 124, 32, 32, 95, 32, 92, 32, 10, 
32, 32, 32, 32, 47, 32, 32, 92, 32, 32,
32, 32, 124, 32, 124, 95, 95, 95, 95, 95,
124, 32, 40, 95, 95, 95, 32, 32, 32, 32,
41, 32, 124, 95, 95, 95, 32, 124, 32, 124, 
32, 32, 32, 32, 32, 32, 124, 32, 124, 32, 
32, 32, 32, 32, 32, 32, 47, 32, 32, 92, 
32, 32, 124, 32, 124, 95, 41, 32, 124, 10, 
32, 32, 32, 47, 32, 47, 92, 32, 92, 32, 
32, 32, 124, 32, 124, 95, 95, 95, 95, 95,
95, 92, 95, 95, 95, 32, 92, 32, 32, 47, 
32, 47, 47, 32, 95, 32, 92, 124, 32, 124, 
32, 32, 32, 32, 32, 32, 124, 32, 124, 32, 
32, 32, 32, 32, 32, 47, 32, 47, 92, 32, 
92, 32, 124, 32, 32, 95, 32, 60, 32, 10, 
32, 32, 47, 32, 95, 95, 95, 95, 32, 92, 
32, 95, 124, 32, 124, 95, 32, 32, 32, 32, 
32, 95, 95, 95, 95, 41, 32, 124, 47, 32, 
47, 124, 32, 40, 95, 41, 32, 124, 32, 124, 
95, 95, 95, 95, 32, 32, 124, 32, 124, 95, 
95, 95, 95, 32, 47, 32, 95, 95, 95, 95, 
32, 92, 124, 32, 124, 95, 41, 32, 124, 10, 
32, 47, 95, 47, 32, 32, 32, 32, 92, 95, 
92, 95, 95, 95, 95, 95, 124, 32, 32, 32, 
124, 95, 95, 95, 95, 95, 47, 124, 95, 95, 
95, 95, 92, 95, 95, 95, 47, 32, 92, 95, 
95, 95, 95, 95, 124, 32, 124, 95, 95, 95, 
95, 95, 95, 47, 95, 47, 32, 32, 32, 32, 
92, 95, 92, 95, 95, 95, 95, 47, 32, 10,
10, 10, 10};

void clk(void)
{
	RCC_CR = 0;
	RCC_PLLCFGR = 0;
	RCC_CFGR = 0;
		
	RCC_CR |= (1<<16); // HSE set
	while( (RCC_CR & (1<<17) ) == 0 ); // wait until HSE ready
	
	RCC_PLLCFGR |= 8;//0x00000008; // set PLLM
	RCC_PLLCFGR |= (336<<6);//|= (336<<6); // 		set PLLN
	RCC_PLLCFGR |= (0<<16); // set PLLP
	RCC_PLLCFGR |= (7<<24);//0x07000000; // set PLLQ

	RCC_PLLCFGR |= (1<<22); // set PLL src HSE
	

	RCC_CR |= (1<<24); // PLL ON
	while( (RCC_CR & (1<<25)) == 0); // wait until PLL ready
	
	FLASH_ACR |= 5;
	RCC_CFGR |= 2; // set PLL to system clock
	
		
	while( (RCC_CFGR & (12) ) != 8); // wait until PLL ready
	
	RCC_CFGR |= (1<<12) | (1<<10); // set APB1 div 4
	RCC_CFGR |= (1<<15); // set APB2 div2	
}


// ADC1, channel 1, PA1
void set_adc1(){
	RCC_AHB1ENR	|= 0x00000001; // RCC clock enable
	GPIOA_MODER	|= 3<<2;			// PA1 analog mode
	RCC_APB2ENR	|= 1<<8;			// ADC1 clock enable
	RCC_CFGR		|= 1<<15 | 1<<13; // set APB2 div4 = 42 MHZ
	
	ADC1_CR2		|= 1<<0;			// ADC1 enable

	ADC1_SMPR2	|= 3<<0;			// channel 1 sampling cycle 56 cyccle
	ADC1_CR1		|= 2<<24 | 1<<5;	// 8bit resolution
											// end-ofconversion interrupt enable
	ADC1_CCR		|= 1<<16;		// PCLK2 div 4
	ADC1_SQR1	|= 0<<20;		// channel 1 : 1 conversion
	ADC1_SQR3	|= 1<<0;			// 1st conversion : channel 1

	NVIC_ISER0	|= 1<<18;		// enable interrupt
}

void set_usart2(){ // second module
	//USART PA2, PA3
	RCC_AHB1ENR	|= 1<<0;
	GPIOA_MODER	|= (1<<5) | (1<<7); // Alternate Function Mode
	GPIOA_AFRL	|= (7<<8) | (7<<12);

	//set USART2
	RCC_APB1ENR	|= (1<<17); //usart2 clk enable
	USART2_CR1	|= (0<<12);
	USART2_CR2	|= (0<<13) | (0<<12);

	USART2_BRR	= (unsigned int)(42000000/115200);
	
	USART2_CR1 |= (1<<3) | (1<<2);
	USART2_CR1 |= 1<<5; // usart interrupt
	USART2_CR1 |= 1<<13; // usart enable

	// USART interrupt enable
	NVIC_ISER1 |= 1<<6; // address + 6	
}

void USART2_IRQHandler(){
	if( USART2_SR & (1<<5)){ // send complete
		rec = USART2_DR; // read DR 8bit


		USART2_DR =rec;  // resend what I entered
		while(!(USART2_SR & (1<<7))); 
		while(!(USART2_SR & (1<<6)));
		//GPIOD_ODR ^= 1<<12;

		USART2_CR1 |= (1<<5); // set again
	}
}

void EXTI0_IRQHandler() {
	GPIOD_ODR ^= 1 << 13;
	GPIOD_ODR ^= 1 << 14;
	GPIOD_ODR ^= 1 << 15;

	EXTI_PR |= 1<<0;	// clear pending bit for EXTI0 , intr bit clear. 12.3.6
}

void ADC1_IRQHandler(){
	if( ADC1_SR & 1<<1 ){
		adc_val = ADC1_DR & 0xFF;
		
		len = sprintf(buf, "%3d\n", adc_val);
		sendStr(buf, len);
	}
	//	ADC1_CR2		|= 1<<30; // interrupt
}

void set_timer2(){
	RCC_APB1ENR	|= 1<<0; 
	TIM2_CR1		 = 0;				// initialize
	
	TIM2_PSC		 = 84-1;	
	TIM2_ARR		 = 1000-1;
	TIM2_DIER	|= 1<<0;
	TIM2_EGR		|= 1<<0;			// 1000 >> 0 >> 1000
	TIM2_CR1		|= 1<<0;

	NVIC_ISER0	|= 1<<28;
}

void TIM2_IRQHandler(){
	TIM2_SR &= 0x00000000;
	tim2tick++;
	if( tim2tick == 100 ){ // 1 ms * 1000 = 1 second
		GPIOD_ODR ^= 1<<12; // green led
		ADC1_CR2		|= 1<<30;
		tim2tick = 0;

	}
}

int main (void)
{
	
	clk();
	
	RCC_CFGR |= 0x04600000;

    /* PORT A */
	RCC_AHB1ENR  |= 1<<0; //RCC clock enable register	
    GPIOA_MODER  |= 0<<0; // input mode
    GPIOA_OTYPER |= 0<<0; // output push-pull
    GPIOA_PUPDR  |= 0<<0; // no pull-up, pull-down
	
	/* button intr set */ 
	SYSCFG_EXTICR1	|= 0<<0; // EXTI0 connect to PA0
	EXTI_IMR			|= 1<<0; // MASK EXTI0 
	EXTI_RTSR		|= 1<<0;	// risingg edge trigger enable
	EXTI_FTSR		|= 0<<0;	// falling edge trigger disable
	NVIC_ISER0		|= 1<<6; // enable EXTI0 interrupt
	
	/* PORT D */
	RCC_AHB1ENR  |= 1<<3;		// PORTD enable
	GPIOD_MODER  |= 1<<24;		// PORTD 12 general output mode
	GPIOD_MODER  |= 1<<26;		// PORTD 13 general output mode
	GPIOD_MODER  |= 1<<28;		// PORTD 14 general output mode
	GPIOD_MODER  |= 1<<30;		// PORTD 15 general output mode
	GPIOD_OTYPER |= 0x00000000;
	GPIOD_PUPDR	 |= 0x00000000;
	

	GPIOD_ODR |= 1 << 12;

	set_timer2();
	set_adc1();
	set_usart2();

	while(count <423){
		USART2_DR = uart_data[count++]; // DR >> Data send
		while(!(USART2_SR & (1<<7)));
		while(!(USART2_SR & (1<<6)));
	}

	ADC1_CR2		|= 1<<30;

	while(1) {
	}
}
///////////////////////////////////////////////////////////

void sendStr(char buf[], int max){
	int cnt = 0;
	while( cnt < max ){
		USART2_DR = buf[cnt++]; 
		while(!(USART2_SR & (1<<7)));
		while(!(USART2_SR & (1<<6)));
	}
}
