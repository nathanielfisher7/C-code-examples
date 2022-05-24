// LAB4:  Interrupts
// Author: Nathaniel Fisher
// Date: 2/15/2021
///////////////////////////////////////////////////////////////////
#include "stm32f411xe.h"


/////////////////////////////////////
/*      Functions Declaration      */
/////////////////////////////////////
void delay_ms(uint32_t val);
void delay_us(uint32_t val);
void freq_gen(uint16_t val);
void edge_counter();
void compare_timer(uint32_t val);



// Helping Functions
void USART2_init (void);
void LED_init();
void LED_toggle();


//////////////////////////////////
/*         Main Function        */
//////////////////////////////////
int main(void) {
    LED_init();    // Initialize LED at PA5 as an output
    USART2_init();



    // TEST CASE 4:
     edge_counter();


    while (1) {}
}


//////////////////////////////////
/*   User Defined Function      */
//////////////////////////////////
void delay_ms(uint32_t val){
    // Using SysTick Timer:
    //        A delay function that can stall CPU 1msec to 100 sec, depending on val.
    //
    //
    // The concept here is to make a delay block using SysTick timer for a delay of 1 msec.
    // The 1 msec delay will be inside a for loop that will loop for val times which will
    // result in a delay of as short as 1msec (for val=1) and as long as 1msec*0xffff_ffff (4,294,967.295 sec)

    // Here are the steps to set the SysTick to delay 1 msec
    //   1- Set the load register to achieve 1msec. Note that you have two options to source your
    //      timer clock. One is to use the HSI clock of 16MHz while the other to use 16MHz/8.
    //   2- Set the counter current value to 0 so that the counter start
    //   3- Enable the counter and the bit to select which clock you want to use
    // Now the counter is counting down once it reaches 0, a flag in the control register
    // will be set -- use that flag.

	// If having the external clock set, 16MHz/8, one timer count is 1/2MHz = 5.0E^-7 seconds.
	// to get this value to 1msec .001/ 5.0E^07 = 2000
	// n-1 = 2000-1 = 1999 = Load value for Reload Value Register
    SysTick->LOAD = 2000-1;      /* reload with number of clocks per millisecond (use N-1)*/
    SysTick->VAL = 0;          /* clear current value register */
    SysTick->CTRL = 1;        /* Enable the timer */

    for (uint32_t i=0; i<val; i++){
            while((SysTick->CTRL & 0x10000) == 0); /* wait until the COUNTFLAG is set */
        }

    SysTick->CTRL &= 0;       /* Stop the timer (Enable = 0) */
}

void SysTick_init(void){
	 SysTick->LOAD = 16000-1;      /* reload with number of clocks per millisecond (use N-1)*/
	 SysTick->VAL = 0;          /* clear current value register */
	 SysTick->CTRL = 1;        /* Enable the timer */

}





/////////////////////
// Helping Functions
void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= (1<<17);    /* Enable USART2 clock */

    /* Configure PA3 for USART2 RX */
    GPIOA->AFR[0] &= ~0xF000;
    GPIOA->AFR[0] |=  0x7000;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00C0;
    GPIOA->MODER  |=  0x0080;   /* enable alternate function for PA3 */

    USART2->BRR = 0x008B;       /* 9600 baud @ 16 MHz */
    USART2->CR1 = 0x0004;       /* enable Rx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}


void LED_init(){
    // configure PB0 through PB10 as output to drive the 10-segment bargraph LED
    RCC->AHB1ENR |=  2;             /* enable GPIOB clock */
    GPIOB->MODER &= ~0x003FFFFF;    /* clear pin mode for pins 1-10 */
    GPIOB->MODER |=  0x00155555;    /* set pin 1-10 to output mode  */
    GPIOB->BSRR   = (0x3FF<<16);    /* Turn each pin off            */
}


void LED_toggle(char num){

    GPIOA->ODR ^=0x1<<num;              /* Toggle LED            */
}
