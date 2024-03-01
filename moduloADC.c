#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "moduloADC.h"

/**********************************************************************************************/
//                Lista de funciones:
/**********************************************************************************************/
void configADC(void);																						
bool finalizada_conversion(void);												
uint16_t adquirir_muestra(void);


/**********************************************************************************************/
//                 configADC()
/**********************************************************************************************/
void configADC(void){
 	SYSCTL_RCGCADC_R |= 1; // Reloj ADC0
	
	SYSCTL_RCGCGPIO_R |= 1<<1;     // Activamos reloj del puerto B
	while ((SYSCTL_PRGPIO_R & (1<<1)) == 0);
	
	// Canal 11 por pin PB5, así que lo configuramos a modo analógico
	GPIO_PORTB_DEN_R &= ~(1<<5);
	GPIO_PORTB_AFSEL_R |= 1<<5;
	GPIO_PORTB_AMSEL_R |= 1<<5;
	
	// antes de configurar hay que desactivar el secuenciador
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;
	ADC0_EMUX_R = (ADC0_EMUX_R &~ADC_EMUX_EM3_M) | ADC_EMUX_EM3_PROCESSOR;     // Disparo por SW
	ADC0_SSMUX3_R = (ADC0_SSMUX3_R & ~0xF) | 11;            // Secuenciador 3, Elegimos canal 11
	
	// habilitamos generación peticiones en secuenciador 3, pero no las elevamos al controlador
	// Primera y última muestra
	ADC0_SSCTL3_R = (ADC0_SSCTL3_R & ~0xF) |ADC_SSCTL3_IE0 | ADC_SSCTL3_END0;  
	// Activamos secuenciador 3	
  ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                                           
}

/**********************************************************************************************/
//                 finalizada_conversion()
/**********************************************************************************************/
bool finalizada_conversion(void){
	return (ADC0_RIS_R & ADC_RIS_INR3)!= 0; 
}
 
/**********************************************************************************************/
//                 adquirir_muestra()
/**********************************************************************************************/
uint16_t adquirir_muestra(void){	
	ADC0_PSSI_R = ADC_PSSI_SS3; // se inicia captura en secuenciador 3. NOTAR QUE NO SE ENMASCARA
					 while (!finalizada_conversion()); 
	         // hay que borrar bit escribiendo 1 en bit IN3, directo, sin enmascarar
					 ADC0_ISC_R = ADC_ISC_IN3;   
return ADC0_SSFIFO3_R;
}





