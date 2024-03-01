#ifndef _UART_H_
#define _UART_H_
#include <stdint.h>

/**********************************************************************************************/
//                ConfigUART0()
/**********************************************************************************************/
// Descripción: Función que configura el UART0 para que se pueda recibir y transmitir 
//              carácteres o mensajes a través del teclado y ventana serial UART#1.
//              Para ello se configuran lo pines PA0 y PA1 del microncontrolador.
//              Además, para este caso los parámetros de configuración serial son: 
//              9600 bps, 8 bits de datos, sin paridad, 1 bit de parada y sin control de flujo. 
//
// Entradas:    Ninguna
// Salidas:     Ninguna
/**********************************************************************************************/
void ConfigUART0( void );    

/**********************************************************************************************/
//                HayCarRx()
/**********************************************************************************************/
// Descripción: Función que permite saber si se ha ingresado algún carácter. 
//
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo uint8_t: 
//              0. No se ha ingresado carácter (pila vacía)
//              1. Se ha ingresado carácter (pila no vacía)
/**********************************************************************************************/
uint8_t HayCarRx( void );

/**********************************************************************************************/
//                RxCar()
/**********************************************************************************************/
// Descripción: Función que recibe un carácter 
//
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo uint8_t: Carácter ingresado por el teclado.
/**********************************************************************************************/
uint8_t RxCar( void );

/**********************************************************************************************/
//                TxCar()
/**********************************************************************************************/
// Descripción: Función que transmite un carácter
//
// Entradas:    
//              Variable de tipo uint8_t: Carácter que se desea transmitir
// Salidas:     Ninguna
/**********************************************************************************************/
void TxCar( uint8_t Car );

/**********************************************************************************************/
//                TxCadena()
/**********************************************************************************************/
// Descripción: Funcion que transmite una cadena ASCII-Z
//
// Entradas:    
//              Arreglo de tipo uint8_t: Cadena de caracteres("") que se desea transmitir
// Salidas:     Ninguna
/**********************************************************************************************/
void TxCadena( uint8_t Cadena[] );

#endif

