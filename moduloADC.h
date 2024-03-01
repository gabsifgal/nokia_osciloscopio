#ifndef _MODULOADC_H_
#define _MODULOADC_H_
#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************************/
//                 configADC()
/**********************************************************************************************/
// Descripci�n: Funci�n que configura el m�dulo ADC para que reciba datos del potenci�metro.
//              La entrada del m�dulo ADC es el pin B5, canal 11. Es por ello que
//              se configura el uso del canal 11 y el pin B5 a modo anal�gico y
//              se habilita la generaci�n de peticiones en el secuenciador 3.
//              
// Entradas:    Ninguna
// Salidas:     Ninguna
/**********************************************************************************************/
void configADC(void);			

/**********************************************************************************************/
//                 finalizada_conversion()
/**********************************************************************************************/
// Descripci�n: Funci�n que indica si los datos anal�gicos han sido convertidos a c�digos.
//              CODIGO= 4096 * DATO / Vref(3.3 V)
//              
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo bool:   
//              0. La conversi�n no ha finalizado
//              1. La conversi�n ha finalizado
/**********************************************************************************************/
bool finalizada_conversion(void);		

/**********************************************************************************************/
//                 adquirir_muestra()
/**********************************************************************************************/
// Descripci�n: Funci�n que env�a el c�digo del dato anal�gico le�do una vez que se haya 
//              concluido su conversi�n.
//              
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo uint16: C�digo del dato recibido
/**********************************************************************************************/
uint16_t adquirir_muestra(void);

#endif



