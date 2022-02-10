#include "STM32FDiscovery.h"

unsigned char rec[20];
unsigned int count = 0;
unsigned int i=0;

unsigned int uart_data[108]= {  // each is ascii code
47, 42, 42, 42, 42, 42, 42, 42, 42, 42,
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 47, 10, 47, 42, 32, 76, 
69, 68, 32, 99, 111, 110, 116, 114, 111, 108, 
32, 83, 104, 101, 108, 108, 32, 102, 111, 114, 
32, 83, 84, 77, 51, 50, 70, 52, 32, 42, 
47, 10, 47, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 42, 42, 42, 42, 42, 42, 
42, 42, 42, 42, 42, 42, 47, 10 };

unsigned char help_guide[]= "* Command List \n - LEDON\n \t + argument [1,2,3,4]\n - LEDOFF\n \t + argument [1,2,3,4]\n etc...\n";
unsigned char wrong[]="worng command!!\n";
void clk(void)
{
	RCC_CR = 0;
	RCC_PLLCFGR = 0;
	RCC_CFGR = 0;
		
	RCC_CR |= (1<<16); // HSE set
	while( (RCC_CR & ( 1<<17) ) == 0 ); // wait until HSE ready
	
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
void help_exe(){
unsigned int cnt=0;
	while(help_guide[cnt]!='\0'){
		USART2_DR = help_guide[cnt++]; // DR >> Data send
		while(!(USART2_SR & (1<<7)));
		while(!(USART2_SR & (1<<6)));
	}
}

void LEDON_exe(unsigned char a){
	unsigned int k=a-'0';
	GPIOD_ODR |= 1 << (11+k);
}

void LEDOFF_exe(unsigned char a){
	unsigned int k=a-'0';
	GPIOD_ODR &= ~(1 << (11+k));
}
void Wrong_exe(){
unsigned int cnt=0;
	while(wrong[cnt]!='\0'){
		USART2_DR = wrong[cnt++]; // DR >> Data send
		while(!(USART2_SR & (1<<7)));
		while(!(USART2_SR & (1<<6)));
	}
}
void USART2_IRQHandler(){
	if( USART2_SR & (1<<5)){ // send complete
		rec[i] = USART2_DR; // read DR 8bit
		USART2_DR=rec[i];  // resend what I entered
		while(!(USART2_SR & (1<<7))); 
		while(!(USART2_SR & (1<<6)));
		GPIOD_ODR ^= 1<<12;	
		if(rec[i]!='\r'){
			i++;
		}
		else {
			if(rec[0]=='h'&&rec[1]=='e'&&rec[2]=='l'&&rec[3]=='p'&&rec[4]=='\r')
				help_exe(); //help exe
			else if(rec[0]=='L'&&rec[1]=='E'&&rec[2]=='D'&&rec[3]=='O'&&rec[4]=='N'
				&&rec[5]==' '&&rec[7]=='\r')
				LEDON_exe(rec[6]); //LEDON exe
			else if(rec[0]=='L'&&rec[1]=='E'&&rec[2]=='D'&&rec[3]=='O'&&rec[4]=='F'
				&&rec[5]=='F'&&rec[6]==' '&&rec[8]=='\r')
				LEDOFF_exe(rec[7]); //LEDOFF exe
			else
				Wrong_exe(); // Wrong exe

			USART2_DR = '>';
			while(!(USART2_SR & (1<<7)));
			while(!(USART2_SR & (1<<6)));
			i=0;
		}
		USART2_CR1 |= (1<<5); // set again
	}

}

void EXTI0_IRQHandler() {
	GPIOD_ODR ^= 1 << 13;
	GPIOD_ODR ^= 1 << 14;
	GPIOD_ODR ^= 1 << 15;

	EXTI_PR |= 1<<0;	// clear pending bit for EXTI0 , intr bit clear. 12.3.6
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
	set_usart2();

	while(count <108){
		USART2_DR = uart_data[count++]; // DR >> Data send
		while(!(USART2_SR & (1<<7)));
		while(!(USART2_SR & (1<<6)));
	}
	USART2_DR = '>';
	while(!(USART2_SR & (1<<7)));
	while(!(USART2_SR & (1<<6)));
	
	while(1) {
	
	}
}
