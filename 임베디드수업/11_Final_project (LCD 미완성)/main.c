#include "STM32FDiscovery.h"
#include <stdio.h>

#define LCD_ADDR 0x27
#define PIN_RS 	(1<<0)
#define PIN_EN 	(1<<2)
#define BACKLIGHT 	(1<<3)


unsigned char rec, adc_val;
unsigned int count = 0;
char buf[7];
int len;
unsigned int tim2tick;

void sendStr(char buf[], int max);

float Filtered_val;
float Distance;
float V;
float sensitivity = 0.1;

// data to Distance parameter
const float A = 0.008271;
const float B = 939.6;
const float C = -3.398;
const float D = 17.339;

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

	SCB_CPACR |= (3<<20) | (3<<22);
}

/* Button */
void EXTI0_IRQHandler() {
	GPIOD_ODR ^= 1 << 13;
	GPIOD_ODR ^= 1 << 14;
	GPIOD_ODR ^= 1 << 15;

	EXTI_PR |= 1<<0;	// clear pending bit for EXTI0 , intr bit clear. 12.3.6
}
///////////////

/* UART, PA2, PA3 */
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
///////////////

/* I2C + LCD */
void i2c_start(){
	I2C1_CR1 |= 1<<8;
	while(!(I2C1_SR1 & (1<<0)));
}
void i2c_stop(){
	I2C1_CR1 |= 1<<9;
	while(!(I2C1_SR2 & (1<<1)));
}
void i2c_write(uint8_t regaddr, uint8_t data){
	// send start condition
	i2c_start();
	I2C1_DR = LCD_ADDR;
	
	while(!(I2C1_SR1 & (1<<1)));
	(void)I2C1_SR2;
	
	I2C1_DR = regaddr;
	while(!(I2C1_SR1 & (1<<2)));
	
	//send data
	I2C1_DR = data;
	while (!(I2C1_SR1 & (1<<2)));
	
	i2c_stop();
}
uint8_t i2c_read(uint8_t regaddr){
	uint8_t reg;
	
	i2c_start();
	I2C1_DR = LCD_ADDR;
	
	while(!(I2C1_SR1 & (1<<1)));
	(void)I2C1_SR2;
	
	I2C1_DR = regaddr;
	while(!(I2C1_SR1 & (1<<2)));
	
	i2c_stop();
	i2c_start();
	
	I2C1_DR = LCD_ADDR | 0x01; // read
	
	while(!(I2C1_SR1 & (1<<1)));
	(void)I2C1_SR2;
	
	while(!(I2C1_SR1 & (1<<6)));
	reg = (uint8_t)I2C1_DR;
	
	i2c_stop();
	
	return reg;
}
void I2C1_ER_IRQHandler(){
	GPIOD_ODR |= (1<<14); // red LED On
}
	
void set_i2c1(){
	RCC_AHB1ENR	|= 1<<1; // GPIO B enable
	
	RCC_AHB1ENR	|= 1<<21;
	GPIOB_MODER	|= (3<<12) | (3<<18); // pin 6,9
	GPIOB_MODER	|= (2<<12) | (2<<18);
	GPIOB_OTYPER	|= (1<<6) | (1<<9);
	
	GPIOB_AFRL	|= (4<<6*4);
	GPIOB_AFRH	|= (4<<(9-8)*4);

	I2C1_CR1 = (1<<15);
	I2C1_CR1 = 0;
	
	I2C1_CR2 |= (1<<8); // enable error interrupt
	
	I2C1_CR2 |= (10<<0); // 10Mhz
	I2C1_CCR |= (50<<0);
	
	I2C1_TRISE |= (11<<0);
	
	I2C1_OAR1 |= (0X00<<1);
	I2C1_OAR1 |= (1<<14);
	
	NVIC_ISER1 |= 1<<6;
	
	I2C1_CR1 |= (1<<0); // enable i2c
}


/* ADC1, channel 1, PA1 */
void set_adc1(){
	RCC_AHB1ENR	|= 0x00000001;		// RCC clock enable
	GPIOA_MODER	|= 3<<2;		// PA1 analog mode
	RCC_APB2ENR	|= 1<<8;		// ADC1 clock enable
	RCC_CFGR	|= 1<<15 | 1<<13;	// set APB2 div4 = 42 MHZ
	
	ADC1_CR2	|= 1<<0;		// ADC1 enable

	ADC1_SMPR2	|= 3<<0;		// channel 1 sampling cycle 56 cyccle
	ADC1_CR1	|= 2<<24 | 1<<5;	// 8bit resolution
							// end-of conversion interrupt enable
	ADC1_CCR	|= 1<<16;		// PCLK2 div 4
	ADC1_SQR1	|= 0<<20;		// channel 1 : 1 conversion
	ADC1_SQR3	|= 1<<0;		// 1st conversion : channel 1

	NVIC_ISER0	|= 1<<18;		// enable interrupt
}
void ADC1_IRQHandler(){
	if( ADC1_SR & 1<<1 ){
		adc_val = ADC1_DR & 0xFF;
		Filtered_val = Filtered_val * (1-sensitivity) + adc_val * sensitivity;
		V = Filtered_val*5/256;
		
		Distance = (A+B*V)/(1+C*V+D*V*V);
		
		len = sprintf(buf, "%.1lf\n", Distance);
		sendStr(buf, len);
	}
	//ADC1_CR2	|= 1<<30; // interrupt
}
///////////////


/* Timer */
void set_timer2(){
	RCC_APB1ENR	|= 1<<0; 
	TIM2_CR1	= 0;		// initialize
	
	TIM2_PSC	= 84-1;	
	TIM2_ARR	= 1000-1;
	TIM2_DIER	|= 1<<0;
	TIM2_EGR	|= 1<<0;	// 1000 >> 0 >> 1000
	TIM2_CR1	|= 1<<0;

	NVIC_ISER0	|= 1<<28;
}

void TIM2_IRQHandler(){
	TIM2_SR &= 0x00000000;
	tim2tick++;
	if( tim2tick == 10 ){ // 1 ms * tim2tick
		ADC1_CR2		|= 1<<30;
		tim2tick = 0;
	}
}
///////////////

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
	GPIOD_PUPDR  |= 0x00000000;
	

	GPIOD_ODR |= 1 << 12;

	set_timer2();
	set_adc1();
	set_usart2();

	ADC1_CR2	|= 1<<30;
	adc_val = ADC1_DR & 0xFF;
	Filtered_val = adc_val;
	
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
