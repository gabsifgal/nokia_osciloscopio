/****************************************************************************************/
/****************************************************************************************/
/***** PONTIFICIA UNIVERSIDAD CATÓLICA DEL PERÚ 																		*****/
/***** FACULTAD DE CIENCIAS E INGENIERÍA 																						*****/
/***** SISTEMAS DIGITALES 																													*****/
/****************************************************************************************/
/***** Proyecto: Osciloscopio																												*****/
/****************************************************************************************/
/***** Microcontrolador: TM4C123GH6PM 																							*****/
/***** EvalBoard: Tiva C Series TM4C123G LaunchPad 																	*****/
/***** Autores: Gabriel Sifuentes, Talita Ricaldi															  		*****/
/***** Fecha: 1 de julio de 2022																										*****/
/****************************************************************************************/
/****************************************************************************************/


/**********************************************************************************************/
//                      Librerías del proyecto 
/**********************************************************************************************/
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "ElementosGraficos.h"
#include "Uart.h"
#include "moduloADC.h"
#include "PuertosDFyTimer.h"
#include "../bitmap.h"


/**********************************************************************************************/
//                Defininimos variables del proyecto 
/**********************************************************************************************/
uint16_t cursor_on=0;                  // Para mostrar u ocultar los cursores en el Nokia
uint16_t x_cursor_1=20, x_cursor_2=40; // Para determinar las posiciones de los cursores 1 y 2 
uint16_t dez=0, amp=0;                 // Generan desplazamiento horizontal(dez) y vertical(amp)
uint16_t escala_1=1, escala=1;         // Para aumentar y disminuir la escala de tiempo
uint16_t view=1;                       // Para mejorar la nitidez de la gráfica voltaje-tiempo
uint16_t esp=83;                       // Para mostrar solo los datos pertinentes en el Nokia 
uint16_t horizontal=3;							   // Definir 5 escalas de tiempo 
uint8_t ActualizarPantalla;            // Para actualizar los datos del Voltaje en el nokia
uint16_t contActualizarPantalla=0;      // Contador de tiempo para la actualización de pantalla 
uint8_t pausa=0; 										   // Para pausar o reanudar la lectura de voltaje
uint16_t TiempoDeLectura=30;           // Tiempo empleado para leer un dato del módulo ADC
uint16_t contTiempoDeLectura;          // Contador para la lectura de datos
uint16_t x_pos=0;                      // Posición "x" de la lectura voltaje en la pantalla

/**********************************************************************************************/
//                Declaramos arreglos del proyecto 
/**********************************************************************************************/
uint16_t muestras_y_pos[84];    // Almacena las posiciones "y" de las mediciones de voltaje 
uint16_t muestras[84];          // Almacena los códigos de las mediciones de voltaje
uint32_t framecnt = 0;

uint8_t handle_pixel(uint8_t color){
	return ((color/16)>framecnt);
}

uint8_t tempo[504];

// Funcion para controlar la lectura de voltaje y la actualizacion de pantalla LCD
void SysTick_Handler(void){
	// Se espera a que el contador sea igual al Tiempo de espera para la lectura de un nuevo valor 
	/*
	if(++contTiempoDeLectura >= TiempoDeLectura) { 
		contTiempoDeLectura=0; // Reinicia el contador de lectura de voltaje 
		if(pausa==0){ //Si el sistema está pausado no se procede a realizar una nueva medición
			if(x_pos<84){ // Mientras no desaparezca del cuadro
				x_pos= x_pos+1;// Se mueve un pixel a la derecha
			} else{ // En el momento en que se ubica al limite de la pantalla nokia
				x_pos=0; // Se reinicia a cero
			}
		}
	}
	*/
		
	// Actualizar la pantalla cada 15ms
	if(++contActualizarPantalla>500){
		contActualizarPantalla=0; // Reinicia el contador de tiempo para la actualización  
		ActualizarPantalla=1; // Se autoriza la actualización de pantalla
	}
	/*
	for (int i = 0; i < 504; i++){
			tempo[i] = handle_pixel(bitmap0[i]);
	}
	framecnt++;
	if (framecnt==15) framecnt = 0;
	*/
}



/**********************************************************************************************/
//                Funciones principal
/**********************************************************************************************/
int main(void){

	// Inicializacion
	float V1, V2;
	float amplitud=1;
	uint16_t settings;
	Nokia5110_Init();
	Nokia5110_ClearBuffer();	
  configADC();	
	configPulsadores_D();																				
	ConfiguraTimer_1ms();
  ConfigUART0();
  config_portF();
	uint8_t Screen[504];
	
	while(1){
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap0);
		}
		/*
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap1);
		}
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap2);
		}
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap3);
		}
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap4);
		}
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap3);
		}
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap2);
		}
		if(ActualizarPantalla == 1){GPIO_PORTF_DATA_R^=0x02; ActualizarPantalla=0;
			Nokia5110_Clear();
			Nokia5110_DrawFullImage((unsigned char*)bitmap1);
		}
		*/
	}
}



