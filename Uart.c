#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "Uart.h"

/**********************************************************************************************/
//                Lista de funciones:
/**********************************************************************************************/
void ConfigUART0( void );
uint8_t HayCarRx( void ); 
void TxCar( uint8_t Car );
uint8_t RxCar( void );
void TxCadena( uint8_t Cadena[] );

/**********************************************************************************************/
//                ConfigUART0()
/**********************************************************************************************/
void ConfigUART0( void ){	  	
	uint32_t temmp;

  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0;                   // Se activa el reloj del UART
  temmp = SYSCTL_RCGC2_R;                                 // Espera de unos ciclos de reloj 
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA;                   // Se activa el reloj del puerto A 
  // PA0 (U0Rx) PA1( U0Tx)  	
  temmp = SYSCTL_RCGC2_R;                                 // Espera de unos ciclos de reloj
  UART0_CTL_R &= ~ UART_CTL_UARTEN;                       // Se desactiva el UART 
  UART0_IBRD_R = (UART0_IBRD_R & ~UART_IBRD_DIVINT_M)|104;// Se configura DIVINT 
  // 16MHz/(16*9600) 
  // Parte entera 
  UART0_FBRD_R = (UART0_FBRD_R & ~UART_FBRD_DIVFRAC_M)|11;//Se configura DIVFRAC
  // Parte fraccionaria*64 
  UART0_LCRH_R = ((UART0_LCRH_R & ~0x000000FF)|(UART_LCRH_WLEN_8)|(UART_LCRH_FEN)); 
  // Se configuran los bits de datos, 1 bit de parada, sin paridad y habilita el FIFO
  UART0_CTL_R |= UART_CTL_UARTEN;                  // Se habilita el UART 
  GPIO_PORTA_AMSEL_R &= ~(0x03);                   // Desactivamos modo analógico en PA0 y PA1 
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)|0x00000011;// Conectamos UART0 a PA0 y PA1 

  GPIO_PORTA_AFSEL_R |= 0x03;                      // Activamos funciones alternas en PA0 y PA1 

  GPIO_PORTA_DEN_R |= 0x03;                        // Activamos funciones digitales en PA0 y PA1 

}

/**********************************************************************************************/
//                HayCarRx()
/**********************************************************************************************/
uint8_t HayCarRx( void ){
 if ((UART0_FR_R & UART_FR_RXFE) != 0 )
 return 0; //pila vacía
 else
 return 1; //Pila no vacía ->hay carácter recibido
}

/**********************************************************************************************/
//                RxCar()
/**********************************************************************************************/
uint8_t RxCar( void ){ 
  uint8_t camp; 
  camp= UART0_DR_R&0xFF;                  // Se toman solo 8 bits
  return camp; 
}

/**********************************************************************************************/
//                TxCar()
/**********************************************************************************************/
void TxCar( uint8_t Car ){
  while((UART0_FR_R & UART_FR_TXFF) != 0 ) ; //Espera mientras pila llena
  UART0_DR_R = Car; 
}

/**********************************************************************************************/
//                TxCadena()
/**********************************************************************************************/
void TxCadena( uint8_t Cadena[] ){
  uint8_t i;

  for( i = 0; Cadena[i] != '\0'; i++ )
    TxCar( Cadena[i] );
}




