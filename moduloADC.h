#ifndef _MODULOADC_H_
#define _MODULOADC_H_
#include <stdint.h>
#include <stdbool.h>

/**********************************************************************************************/
//                 configADC()
/**********************************************************************************************/
// Descripción: Función que configura el módulo ADC para que reciba datos del potenciómetro.
//              La entrada del módulo ADC es el pin B5, canal 11. Es por ello que
//              se configura el uso del canal 11 y el pin B5 a modo analógico y
//              se habilita la generación de peticiones en el secuenciador 3.
//              
// Entradas:    Ninguna
// Salidas:     Ninguna
/**********************************************************************************************/
void configADC(void);			

/**********************************************************************************************/
//                 finalizada_conversion()
/**********************************************************************************************/
// Descripción: Función que indica si los datos analógicos han sido convertidos a códigos.
//              CODIGO= 4096 * DATO / Vref(3.3 V)
//              
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo bool:   
//              0. La conversión no ha finalizado
//              1. La conversión ha finalizado
/**********************************************************************************************/
bool finalizada_conversion(void);		

/**********************************************************************************************/
//                 adquirir_muestra()
/**********************************************************************************************/
// Descripción: Función que envía el código del dato analógico leído una vez que se haya 
//              concluido su conversión.
//              
// Entradas:    Ninguna
// Salidas:     
//              Dato de tipo uint16: Código del dato recibido
/**********************************************************************************************/
uint16_t adquirir_muestra(void);

#endif



