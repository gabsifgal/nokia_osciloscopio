#ifndef _PUERTOSDFYTIMER_H_
#define _PUERTOSDFYTIMER_H_

/**********************************************************************************************/
//                config_portF ()
/**********************************************************************************************/
// Descripción: Función que configura el Puerto F para el uso de dos pulsadores y el led RGB.
//              Se configuran los pines PF4 y PF0 (Pulsadores SW1 y SW2) como entradas y los 
//              pines PF1, PF2, PF3 (Led RBG) como salidas. Todos estos pines se habilitan como
//              señales digitales y se configuran como GPIO.
//
// Entradas:    Ninguna
// Salidas:     Ninguna
/**********************************************************************************************/
void config_portF (void);

/**********************************************************************************************/
//                configPulsadores_D()
/**********************************************************************************************/
// Descripción: Función que configura el Puerto D para el uso de pulsadores/llaves.
//              Se configuran los pines PD0, PD1, PD2 y PD3 (llaves Key0, key1, key2 y key3) 
//              como entradas, se habilitan como señales digitales y se configuran como GPIO.
//
// Entradas:    Ninguna
// Salidas:     Ninguna
/**********************************************************************************************/
void configPulsadores_D(void);

/**********************************************************************************************/
//                ConfiguraTimer_1ms()
/**********************************************************************************************/
// Descripción: Función para configurar el timer a 1ms y permite interrupciones.
//              Establecemos un tiempo base de 1ms para sondear el timer. Esto resulta 
//              conveniente cuando queremos sondear varias tareas que se ejecutan 
//              a tiempos distintos, por lo que no tendremos que modificar el timer
//
// Entradas:    Ninguna
// Salidas:     Ninguna
/**********************************************************************************************/
void ConfiguraTimer_1ms(void);

#endif


