#ifndef _UART_H_
#define _UART_H_
#include <stdint.h>

/**********************************************************************************************/
//                ConfigUART0()
/**********************************************************************************************/
// Descripci�n: Funci�n que configura el UART0 para que se pueda recibir y transmitir 
//              car�cteres o mensajes a trav�s del teclado y ventana serial UART#1.
//              Para ello se configuran lo pines PA0 y PA1 del microncontrolador.
//              Adem�s, para este caso los par�metros de configuraci�n serial son: 
//              9600 bps, 8 bits de datos, sin paridad, 1 bit de parada y sin control de flujo. 
//
// Entradas:    Ninguna
// Salidas:     Ninguna
/**********************************************************************************************/
void ConfigUART0( void );    

/**********************************************************************************************/
//                HayCarRx()
/**********************************************************************************************/
// Descripci�n: Funci�n que permite saber si se ha ingresado alg�n car�cter. 
//
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo uint8_t: 
//              0. No se ha ingresado car�cter (pila vac�a)
//              1. Se ha ingresado car�cter (pila no vac�a)
/**********************************************************************************************/
uint8_t HayCarRx( void );

/**********************************************************************************************/
//                RxCar()
/**********************************************************************************************/
// Descripci�n: Funci�n que recibe un car�cter 
//
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo uint8_t: Car�cter ingresado por el teclado.
/**********************************************************************************************/
uint8_t RxCar( void );

/**********************************************************************************************/
//                TxCar()
/**********************************************************************************************/
// Descripci�n: Funci�n que transmite un car�cter
//
// Entradas:    
//              Variable de tipo uint8_t: Car�cter que se desea transmitir
// Salidas:     Ninguna
/**********************************************************************************************/
void TxCar( uint8_t Car );

/**********************************************************************************************/
//                TxCadena()
/**********************************************************************************************/
// Descripci�n: Funcion que transmite una cadena ASCII-Z
//
// Entradas:    
//              Arreglo de tipo uint8_t: Cadena de caracteres("") que se desea transmitir
// Salidas:     Ninguna
/**********************************************************************************************/
void TxCadena( uint8_t Cadena[] );

#endif

