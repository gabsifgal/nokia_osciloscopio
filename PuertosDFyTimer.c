#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "PuertosDFyTimer.h"

/**********************************************************************************************/
//                Lista de funciones:
/**********************************************************************************************/
void config_portF (void);
void configPulsadores_D(void);
void ConfiguraTimer_1ms(void);

/**********************************************************************************************/
//                config_portF ()
/**********************************************************************************************/
void config_portF (void) {
	
 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;             // Activamos la señal de reloj del puerto F
 while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)==0){}     // Espera a que el puerto F esté operativo

 // Configuraciones generales
 GPIO_PORTF_LOCK_R = 0x4C4F434B;                      // Desbloquear puerto F
 GPIO_PORTF_AMSEL_R &= ~0X1F;                         // Se deshabilitan las funciones analógicas de los pines 	
 GPIO_PORTF_AFSEL_R &= ~(0x11);                       // No utilizar funciones alternas

 // Para SW1 y SW2 (PF4 PF0)
 GPIO_PORTF_DIR_R &= ~(0x11);                         // Configura PF0 y PF4 como entradas
 GPIO_PORTF_PUR_R |= 0x11;                            // Activa resistencias de pull-up de PF0 y PF4
 GPIO_PORTF_DEN_R |= 0x11;                            // Habilita señales digitales en PF0 y PF4

 // Para Led D1 (PF3 PF2 PF1)
 GPIO_PORTF_DIR_R |= 0x0E;                            // Configura PF1, PF2 y PF3 como salidas
 GPIO_PORTF_DR8R_R |= 0x0E;                           // Activa el driver de 8 mA en PF1, PF2 y PF3
 GPIO_PORTF_DEN_R |= 0x0E;                            // Habilita señales digitales en PF1, PF2 y PF3
 
 // Apaga los LEDs conectados a PF1, PF2 y PF3
 GPIO_PORTF_DATA_R &=~(0x0E); 
}

/**********************************************************************************************/
//                configPulsadores_D()
/**********************************************************************************************/
void configPulsadores_D(void) {
 
	 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3; // Se habilita la señal de reloj del Puerto D
	 while( (SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3)==0) {} // Espera a que se active
		 
	 GPIO_PORTD_AMSEL_R &= ~0x0F; // Se deshabilitan las funciones analógicas de los pines 	
	 GPIO_PORTD_DIR_R &= ~0x0F; // PD0. PD1 y PD2 Y PD3 como entradas
	 GPIO_PORTD_AFSEL_R &= ~0x0F; // Se desactivan las funciones alternas
	 GPIO_PORTD_DEN_R |= 0x0F;  //Se activan las funciones digitales
	 GPIO_PORTD_PUR_R &= ~0x0F; //Se desactivan las resistencias pull-up de PD0, PD1, PD2 y PD3
	 GPIO_PORTD_PDR_R &= ~0x0F; //Se desactivan las resistencias pull-down de PD0, PD1, PD2 y PD3
}

/**********************************************************************************************/
//                ConfiguraTimer_1ms()
/**********************************************************************************************/
void ConfiguraTimer_1ms(void){        // Temporiza 1mseg, Fclksys=16MHz
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
	
	//16000-1 = 15999, en hexadecimal es 00003E7F
	NVIC_ST_RELOAD_R = (NVIC_ST_RELOAD_R&0xFF000000)|0x00003E7F;
	
	// Iniciamos el contador con cero (escribiendo cualquier valor)
	NVIC_ST_CURRENT_R &= ~(0x00FFFFFF);
	
	// Habilitamos el módulo SysTick
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;
}
