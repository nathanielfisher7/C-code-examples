/*
 * Author: Nathaniel Fisher
 * Date: 2/9/2021
 * Description: 	This code sets up and provides information on the various processes required to use USART
 * 			   	on the STM32F411RE device. Specifically two functions were created, one to initialize the UART2
 * 			   	on the device and one to initialize the GPIO pins to be used. Sepcifically, the main goal was to
 * 			   	use and show the operation of the boards UART connection to its programing computer. The board
 * 			   	is meant to send letters stored in a string array to the computer at the push of a button connected
 * 			   	to pin C13. The board is then also meant to recieve, translate, and use information sent from the
 * 			   	programming computer. This data is then used to turn on an RGB led to control its color. If the
 * 			   	character 'r' is sent, the LED turns red, if the character 'g' is sent the LED turns green, if the
 * 			   	character 'g' is sent, the LED turns blue. If any other character is sent, the LED turns off.
 */

#include "stm32f446xx.h"

// Initialize functions to be used.
void UART2_init();
void GPIOC_init();
void GPIOA_init();
void UART2_tx();
void UART_Error();
uint32_t UART2_rx();

int main(void) {
	char txBuf[2] = "Nathaniel Fisher"; // Character array to be sent to the host computer.
	int txCntr = 0; 		// Counter used to iterate through the above array.
	char characterReceived; // Holder for any value recieved from the host computer.
	int button = 1; 		// Counter to monitor the button connected to C13.
	int prevButton = 1; 	// Monitor used to observe the previous VS new value of the button connected
							// to C13.
	UART2_init(); 			// Initialize the STM32F411RE's UART 2 for use between it and its host computer.
	GPIOC_init(); 			// Initialize pins C8, C6, and C5 as output pins to control an RGB LED.
							// Initialize C13 as.
				  	  	  	// an input pin to monitor the button it's connected to.
	GPIOA_init();

	// Infinite loop
	while (1) {
		UART_Error(); // Check for Errors

		// Check if ready to send data.
		// if TXE bit in Status Register is 0 then Transmit.
		//		Data register is not empty (we need to stall
		//		the CPU).
		// if TXE bit is 1, then Transmit Data register is empty.
		//		(we can write and send the next packet)
		while (!(USART2->SR & 0x0080)); 	// Wait until Tx buffer is empty.
		// We escaped the loop, the Transmit data register is empty.

		button = GPIOC->IDR >> 13; // Assign the value of the button connected to C13 to it value place holder.

		// If statements to monitor if the button has been pressed and then released. Looking for the occurrence
		// of a rising edge. If the button value is 1, the button has either not been pressed, or has just been
		// released.
		if (button == 1) {
			// If the button has been pressed.
			if (prevButton == 0) {
				// If the data being sent is at the end of the string. Restart and go to next line.
				// Else, send next character data.
				if (txBuf[txCntr % 100] == '\0') {
					txCntr = 0; 			// Restart string iterator.
					UART2_tx(' '); 			// Write the Packet with a space to separate the end of the string from the start.
				} else {
					UART2_tx(txBuf[(txCntr++) % 100]); // Write the Packet to be sent, one letter at a time.
				}
				prevButton = 1; 			// Save the value of the buttons current state.
			}

		} else if (button == 0) {
			prevButton = 0; 				// Save the value of the buttons current state.
		}

		// Add some delay.
		for (int i = 0; i < 100000; i++) {
			// Check if there is data in the Receive Data register.
			if ((USART2->SR) & (1 << 5)) {
				characterReceived = UART2_rx(); // Properly read packet.
			}
		}
		// Check what letter has been sent by the host computer. If r, g, or b characters are sent, turn on the
		// red, green, or blue characters respectively.
		if (characterReceived == 'g') {
			GPIOC->ODR &= !(1011 << 5); 	// Clear the output bits.
			GPIOC->ODR |= (1 << 6); 		// Turn on C6 which turns on the green LED.
		} else if (characterReceived == 'b') {
			GPIOC->ODR &= !(1011 << 5); 	// Clear the output bits.
			GPIOC->ODR |= (1 << 5); 		// Turn on C5 which turns on the blue LED.
		} else if (characterReceived == 'r') {
			GPIOC->ODR &= !(1011 << 5); 	// Clear the output bits.
			GPIOC->ODR |= (1 << 8); 		// Turn on C8 which turns on the red LED.
		} else {
			GPIOC->ODR &= !(1011 << 5); 	// Clear all bits if anything other than r, g, or b is sent
											// from the host.
		}
	}
}

/*
 * Configure and initialize UART2 with 115,200 bps baud rate.
 */
void UART2_init() {
	//Enable the clock for UART2.
	// UART2 is connected to the APB1 (Advanced Peripheral 1) bus.
	RCC->APB1ENR |= (1 << 17);
	RCC->AHB1ENR |= 1;			// Enable the clock of GPIOA.

	// Connect MCU UART2 TX to pin PA2.
	// Set PA2 to use alternate function.
	// Must set PA2's AF to 7 to connect it to UART2's TX.
	GPIOA->MODER &= ~(0x3 << (2 * 2)); 	// Clear the two bits for PA2.
	GPIOA->MODER |= (0x2 << (2 * 2)); 	// Set the mode to AF (10--> 0x2).
	// Set AF to 0x7 to connect the UART2 TX to the GPIOA.2
	GPIOA->AFR[0] &= ~(0xF << (4 * 2)); // Clear PA2's 4 AF bits.
	GPIOA->AFR[0] |= (0x7 << (4 * 2)); 	// Set PA2's AF 4 bits to 0x7.

	// Connect MCU UART2 RX to MCU pin PA3.
	// Set PA3 Pin mode to AF.
	// Must set PA3's AF to 7 to connect it to UART2's RX.
	GPIOA->MODER &= ~(0x3 << (2 * 3)); 	// Clear PA3's two mode bits.
	GPIOA->MODER |= (0x2 << (2 * 3)); 	// Set PA3's mode bits to AF (10).
	// Set PA3's AF to 7.
	GPIOA->AFR[0] &= ~(0xF << (4 * 3)); // Clear PA3's AF 4 bits.
	GPIOA->AFR[0] |= (0x7 << (4 * 3)); 	// Set PA3's AF 4 bits to 0x7.

	// USART2 Configuration.
	//		Will be using default 16 bit oversampling.
	//		UART2 input clock is the HSI(high speed internal clock) which is 16MHz.
	// Desired baud rate = 115,200bps.
	// 8bit data send and receive.
	// 1 stop bit.
	// no parity.
	// no hw flow control.
	// BaudRate calc = 16,000,000/(16*115200) = 8.621.
	// 		8 for mantissa.
	//		round (0.681*16)=11 for the fraction.
	//		BaudRate[12:4] == mantissa.
	//		BaudRate[3:0] == fraction.
	//		BR register set to 0x8B.
	USART2->BRR = 0x8B; 			// Set BRR to 0x8B.
	USART2->CR1 |= (1 << 3); 		// Enable Transmitter.
	USART2->CR1 |= (1 << 2); 		// Enable Receiver.
	USART2->CR2 = 0x0000; 			// 1 stop bit and all other features left to default (0).
	USART2->CR3 = 0x0000; 			// no flow control and all other features left to default (0).

	// After setting our settings enable the USART2 module.
	USART2->CR1 |= (1 << 13); 		// Enable USART2.
}

/*
 * Configure GPIOC pins.
 * C8 turns RGB LED.
 */
void GPIOC_init() {
	RCC->AHB1ENR |= 1 << 2;			// Enable the clock of GPIOC.

	// Enable Pins C8, C6, and C5 for RGB output control.
	// Configure pins C8,6,5.
	GPIOC->MODER &= ~(0x6F << 10);	// Clear MODER bits for pins C8,6,5 (11001111 << 10, or 0x3F << 10).
	GPIOC->MODER |= 0x45 << 10;		// Set MODER bits for pins C8,6,5. Set as output pins.
									// (01000101 << 10, or  0x45<<10).

	// Configure C13 bit for input
	GPIOC->MODER &= ~(0x3 << 26); 	// Clear MODER bits for C13.

	GPIOC->MODER |= (0x0 << 26); 	// Set MODER bits for C13. Set as input pin.
}

/*
 * Initializes GPIOA to allow the use of PA5.
 */
void GPIOA_init() {
	RCC->AHB1ENR |= 1;				// Enable the clock of GPIOA.

	// Enable Pin A5 to output.
	GPIOA->MODER &= ~(0x3 << 10); 	// Clear A5's MODER bits.
	GPIOA->MODER |= 0x1 << 10; 		// Set A5's MODER bits to output mode.
}

/*
 * Reads the data sent from the host computer on the rx line.
 */
uint32_t UART2_rx() {
	uint32_t n = 0; // Initialize return value.
	n = (USART2->DR) & 0xFF; 		//Reading the packet automatically clears the RXNE.
	return n; // Return the value.
}

/*
 * Writes the data specified by the user. Sending it to the host computer.
 */
void UART2_tx(uint32_t n) {
	USART2->DR = n; 				// Write the packet to be sent with the value specified.
}

/*
 * If an error occurs when using the STM32's UART2, turn on A5.
 */
void UART_Error() {
	// If an error occurs turn on PA5 which turns on LD2.
	if ( USART2->SR >> 3 == 1 || USART2->SR >> 2 == 1 || USART2->SR >> 1 == 1 || USART2->SR == 1) {
		GPIOA->ODR |= (1 << 5); 	// Turn on PA5.
	}
}
