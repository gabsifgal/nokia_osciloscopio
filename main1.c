/****************************************************************************************/
/****************************************************************************************/
/***** PONTIFICIA UNIVERSIDAD CATÓLICA DEL PERÚ 																		*****/
/***** FACULTAD DE CIENCIAS E INGENIERÍA 																						*****/
/***** SISTEMAS DIGITALES 																													*****/
/****************************************************************************************/
/***** Tema: LAB 9 Avance 2 del proyecto final																			*****/
/***** Proyecto: Osciloscopio																												*****/
/****************************************************************************************/
/***** Microcontrolador: TM4C123GH6PM 																							*****/
/***** EvalBoard: Tiva C Series TM4C123G LaunchPad 																	*****/
/***** Autores: Talita Ricaldi, Gabriel Sifuentes																		*****/
/***** Fecha: 25 de junio de 2022																										*****/
/****************************************************************************************/
/****************************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "osciloscopio.h"

/*****************************************************************************/
//                Defininimos variables del proyecto 
/****************************************************************************/
uint16_t cursor_on=0; //------------------------------------------------------Variables para el uso de los cursores
uint16_t x_cursor_1=20,x_cursor_2=40;
uint16_t dez=0, amp=0, ant_amp=0;//------------------------------------------------------Variables para el desplazamiento horizontal y vertical
uint16_t escala=1,escala_1=1;
uint16_t esp=83, view=1;//----------------------------------------------------Variables para el cambio de escala
uint16_t horizontal=3;
uint8_t ActualizarPantalla; //------------------------------------------------Variables para actualizar los datos del Voltaje en el nokia
uint8_t contActualizarPantalla=0;
uint8_t dir=1;
uint16_t velocidaMillis=30;
uint16_t contadorVelocidad; 
uint16_t xEnemigo=0;  // xEnemigo es la posicion de la última lectura del voltaje




/*****************************************************************************/
//               Declaramos arreglos del proyecto 
/****************************************************************************/
uint8_t buffer[2];
uint16_t muestras[84]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
												 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
												 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
												 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint16_t muestras_ypos[84]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
																0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
																0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
																0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	
/*****************************************************************************/
//           Definimos elementos graficos empleados en mapas de bits
/****************************************************************************/
const unsigned char obs [] = {0x03, 	0x03,	0x03,	0x03,};
const unsigned char linea_cur [] = {0x0F,0x0F,0x0F,0x0F,0x0F,0x0F,};

	
/*****************************************************************************/
//          Declaramos las funciones empleadas
/****************************************************************************/	
void ConfigUART0( void );//-------------------------------------------------- Configuracion de puertos F, D y módulos ADC y UART0
void configADC(void);																							
void config_portF(void);
void configPulsadores_D(void);																				
void ConfiguraSysTick(void); //---------------------------------------------Interrupciones y timer		
void SysTick_Handler(void);																						
uint16_t analizar_codigo(uint16_t y);//-------------------------------------Calcular Voltaje
void iniciar_conversion(void);
bool finalizada_conversion(void);												
uint16_t adquirir_muestra(void);	
//-------------------------------------------------Para la pantalla LCD - nokia 5110																		
void mostrar_voltaje(void);
uint8_t HayCarRx( void ); //------------------------------------------------Para recibir y transmitir datos por el teclado
void TxCar( uint8_t Car );
uint8_t RxCar( void );
void TxCadena( uint8_t Cadena[] );
void leer_cadena(uint32_t long_buffer, uint8_t buffer[]);
void cursores(void); //---------------------------------------------------------Para mover los cursores
void calcular_escala(void); //----------------------------------------------Para cambiar escalas
void pause_play(void);

/*****************************************************************************/
//          Funciones empleadas
/****************************************************************************/
// Funcion para configurar el UART
void ConfigUART0( void ){	  	
	uint32_t temmp;

  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART0; // Se activa el reloj del UART
  temmp = SYSCTL_RCGC2_R;                // Espera de unos ciclos de reloj 
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // Se activa el reloj del puerto A 
  // PA0 (U0Rx) PA1( U0Tx)  	
  temmp = SYSCTL_RCGC2_R;                // Espera de unos ciclos de reloj
  UART0_CTL_R &= ~ UART_CTL_UARTEN;     // Se desactiva el UART 
  UART0_IBRD_R = (UART0_IBRD_R & ~UART_IBRD_DIVINT_M)|104; // Se configura DIVINT 
  // 16MHz/(16*9600) 
  // Parte entera 
  UART0_FBRD_R = (UART0_FBRD_R & ~UART_FBRD_DIVFRAC_M)|11; //Se configura DIVFRAC
  // Parte fraccionaria*64 
  UART0_LCRH_R = ((UART0_LCRH_R & ~0x000000FF)|(UART_LCRH_WLEN_8)|(UART_LCRH_FEN)); 
  // Se configuran los bits de datos, 1 bit de parada, sin paridad y habilita el
  //FIFO 
  UART0_CTL_R |= UART_CTL_UARTEN;       // Se habilita el UART 
  // Desactivamos modo analógico en PA0 y PA1 
  GPIO_PORTA_AMSEL_R &= ~(0x03); 
  // Conectamos UART0 a PA0 y PA1 
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFFFFFF00)|0x00000011; 
  // Activamos funciones alternas en PA0 y PA1 
  GPIO_PORTA_AFSEL_R |= 0x03; 
  // Activamos funciones digitales en PA0 y PA1 
  GPIO_PORTA_DEN_R |= 0x03;
}

// Funcion para configurar el Puerto F
void config_portF (void) {
	
 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;             // Activamos la señal de reloj del puerto F
 while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)==0){}     // Espera a que el puerto F esté operativo

 // Configuraciones generales
 GPIO_PORTF_LOCK_R = 0x4C4F434B;                      // Desbloquear puerto F
 GPIO_PORTF_AMSEL_R &= ~0X1F;                         // Se deshabilitan las funciones analógicas de los pines 	
 GPIO_PORTF_AFSEL_R &= ~(0x11);                       // No utilizar funciones alternas

 // Para SW1 y SW2 (PF4 PF0)
 GPIO_PORTF_DIR_R &= ~(0x11); // Configura PF0 y PF4 como entradas
 GPIO_PORTF_PUR_R |= 0x11; // Activa resistencias de pull-up de PF0 y PF4
 GPIO_PORTF_DEN_R |= 0x11; // Habilita señales digitales en PF0 y PF4

 // Para Led D1 (PF3 PF2 PF1)
 GPIO_PORTF_DIR_R |= 0x0E; // Configura PF1, PF2 y PF3 como salidas
 GPIO_PORTF_DR8R_R |= 0x0E; // Activa el driver de 8 mA en PF1, PF2 y PF3
 GPIO_PORTF_DEN_R |= 0x0E; // Habilita señales digitales en PF1, PF2 y PF3
 
 // Apaga los LEDs conectados a PF1, PF2 y PF3
 GPIO_PORTF_DATA_R &=~(0x0E); 
}
// Funcion para configurar el ADC
void configADC(void){
 	SYSCTL_RCGCADC_R |= 1; // reloj ADC0
	//	o también SYSCTL_RCGC0_R |= SYSCTL_RCGC0_ADC0;
	//	while ((SYSCTL_PRADC_R & 1) == 0);
	
	SYSCTL_RCGCGPIO_R |= 1<<1;     // activamos reloj del puerto B
	while ((SYSCTL_PRGPIO_R & (1<<1)) == 0);
	
	// Canal 11 por pin PB5, así que lo configuramos a modo analógico
	GPIO_PORTB_DEN_R &= ~(1<<5);
	GPIO_PORTB_AFSEL_R |= 1<<5;
	GPIO_PORTB_AMSEL_R |= 1<<5;
	
	// antes de configurar hay que desactivar el secuenciador
	ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;
	           // = (ADC0_EMUX_R &~0xF000);
	ADC0_EMUX_R = (ADC0_EMUX_R &~ADC_EMUX_EM3_M) | ADC_EMUX_EM3_PROCESSOR; // disparo por SW
	ADC0_SSMUX3_R = (ADC0_SSMUX3_R & ~0xF) | 11;   // secuenciador 3, elegimos canal 11
	
	// habilitamos generación peticiones en secuenciador 3, pero no las elevamos al controlador
	ADC0_SSCTL3_R = (ADC0_SSCTL3_R & ~0xF) |ADC_SSCTL3_IE0 | ADC_SSCTL3_END0;  // primera y última muestra

  ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;    // Activamos secuenciador 3	
}

// Funcion para configurar el Puerto D
void configPulsadores_D(void) {
 
	 SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;             // Se habilita la señal de reloj del Puerto D
	 while( (SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3)==0) {}   // Espera a que se active
		 
	 GPIO_PORTD_AMSEL_R &= ~0x0F;             // Se deshabilitan las funciones analógicas de los pines 	
	 GPIO_PORTD_DIR_R &= ~0x0F;              	// PD0. PD1 y PD2 Y PD3 como entradas
	 GPIO_PORTD_AFSEL_R &= ~0x0F;           	// Se desactivan las funciones alternas
	 GPIO_PORTD_DEN_R |= 0x0F;                // Se activan las funciones digitales
	 GPIO_PORTD_PUR_R &= ~0x0F;             	// Se desactivan las resistencias pull-up de PD0, PD1 Y PD2 Y PD3
	 GPIO_PORTD_PDR_R &= ~0x0F; 		          // Se desactivan las resistencias pull-down de PD0, PD1 Y PD2 Y PD3
}

// Funcion que configura el timer 1ms y permite interrupciones
void ConfiguraSysTick(void){ // Temporiza 1mseg, Fclksys=16MHz
	NVIC_ST_CTRL_R &= ~NVIC_ST_CTRL_ENABLE;
	NVIC_ST_RELOAD_R = (NVIC_ST_RELOAD_R&0xFF000000)|0x00003E7F;//16000-1
	NVIC_ST_CURRENT_R &= ~(0x00FFFFFF);
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;
}

// Funcion para controlar las interrupciones y la actualizacion de pantalla LCD
void SysTick_Handler(void){
				if(++contadorVelocidad >= velocidaMillis) { // Cada cuantos ms se mueve el enemigo
					contadorVelocidad=0; // Reinicia el contador
					if(dir==1){ //Movimiento hacia la derecha del enemigo
						if(xEnemigo<84){ // Mientras no desaparezca del cuadro
							xEnemigo= xEnemigo+1;
						} // Se mueve un pixel a la derecha
						else{ // En el momento en que se ubica al limite de la pantalla nokia
							xEnemigo=0; // Cambia de direccion de movimiento
						}
					}
				}
	
	
// Actualizar la pantalla cada 16ms
	if(++contActualizarPantalla>15){
		contActualizarPantalla=0;
		ActualizarPantalla=1;
	}
}

// Funcion para relacionar el voltaje obtenido con su posicion en el Nokia
uint16_t analizar_codigo(uint16_t y){
	return 39-(y/103)+8;
}

// Funciones para controlar la conversion de los datos obtenidos del ADC
void iniciar_conversion(void){	
	ADC0_PSSI_R = ADC_PSSI_SS3; // se inicia captura en secuenciador 3. NOTAR QUE NO SE ENMASCARA
}
bool finalizada_conversion(void){
	return (ADC0_RIS_R & ADC_RIS_INR3)!= 0; 
}
 
// Funciones para encontrar el Voltaje
uint16_t adquirir_muestra(void){	
	ADC0_PSSI_R = ADC_PSSI_SS3; // se inicia captura en secuenciador 3. NOTAR QUE NO SE ENMASCARA
					 while (!finalizada_conversion()); 
					 ADC0_ISC_R = ADC_ISC_IN3;   // hay que borrar bit escribiendo 1 en bit IN3, directo, sin enmascarar
return ADC0_SSFIFO3_R;
}

//Funcion para mostrar la lectura del voltaje
void mostrar_voltaje(void){
	// ESCALA 1
	// Subida 1: 21 pixeles
	drawBitmap((0/escala)*escala_1, muestras_ypos[(((0+ dez)>83)*(0+dez-84)+((0+ dez)<=83)*(0+dez))]+amp, obs, 1*view, view*((xEnemigo+1>=1)&(esp+1>=1))); 
  //drawLine(0, 39, 0, muestras_ypos[(((0+ dez)>83)*(0+dez-84)+((0+ dez)<=83)*(0+dez))]+amp);	

	for (int i = 1; i<84 ; i++){
		if(((muestras_ypos[(((i+ dez)>83)*(i+dez-84)+((i+ dez)<=83)*(i+dez))]+amp)<=47)){

			drawBitmap((i/escala)*escala_1, muestras_ypos[(((i+ dez)>83)*(i+dez-84)+((i+ dez)<=83)*(i+dez))]+amp, obs, 1*view, view*((xEnemigo>=i)&(esp>=i)));
			
		// drawLine((((i-1)/escala)*escala_1)*(xEnemigo>=i), (muestras_ypos[(((i-1+ dez)>83)*(i-1+dez-84)+((i-1+ dez)<=83)*(i-1+dez))]+amp)*(xEnemigo>=i), ((i/escala)*escala_1)*(xEnemigo>=i), (muestras_ypos[(((i+ dez)>83)*(i+dez-84)+((i+ dez)<=83)*(i+dez))]+amp)*(xEnemigo>=i));	

		} 
	} 
	
		if(cursor_on==1){
			drawBitmap(x_cursor_1, 0, linea_cur, 1, 6); 
			drawBitmap(x_cursor_2, 0, linea_cur, 1, 6);	
		}
}


// Funcion para verificar si se escribió en el UART
uint8_t HayCarRx( void ){
 if ((UART0_FR_R & UART_FR_RXFE) != 0 )
 return 0; //pila vacía
 else
 return 1; //hay caracter recibido
}

// Función que recibe un caracter
uint8_t RxCar( void ){ 
  uint8_t camp; 
  //while ((UART0_FR_R & UART_FR_RXFE)!=0); // Se espera que llegue un dato
  camp= UART0_DR_R&0xFF;                  // Se toman solo 8 bits
  return camp; 
}

// Funcion que transmite un caracter
void TxCar( uint8_t Car ){
  while((UART0_FR_R & UART_FR_TXFF) != 0 ) ; //Espera mientras pila llena
  UART0_DR_R = Car; 
}
// Funcion que transmite una cadena ASCII-Z
void TxCadena( uint8_t Cadena[] ){
  uint8_t i;

  for( i = 0; Cadena[i] != '\0'; i++ )
    TxCar( Cadena[i] );
}

// Funcion que recide una cadena ASCII-Z
void leer_cadena(uint32_t long_buffer, uint8_t buffer[]){
	uint8_t c;
	int a=0;
	while((c=RxCar())!= 0x0D && a < long_buffer-1){
		buffer[a]=c;
		TxCar(c);
		a=a+1;
	}
}


void calcular_escala(void){
	// Escala 1:4
					if(horizontal==1){
						escala=1;
						escala_1=4;
						view=5;
						esp=20;
	// Escala 1:2
					}else if(horizontal==2){
						escala = 1;
						escala_1=2;
						view=3;
						esp=40;
	// Escala 1:1
					} else if(horizontal==3){
						escala=1;
						escala_1=1;
						view=1;
						esp=83;
	// Escala 2:1
					}else if(horizontal==4){
						escala=2;
						escala_1=1;
						view=1;
						esp=83;
	// Escala 4:1
					}else if(horizontal==5){
						escala = 4;
						escala_1=1;
						view=1;
						esp=83;
					}
}


int main(void){

	// Inicializacion
	float V1, V2;
	float amplitud=1;
	uint16_t settings;
	Nokia5110_Init();
	Nokia5110_ClearBuffer();	
  configADC();	
	configPulsadores_D();																				
	ConfiguraSysTick();
  ConfigUART0();
  config_portF();
	Nokia5110_DrawFullImage((char*)init);	
	
	while(1){

		// Realiza el refresco de la pantalla Nokia
		if(ActualizarPantalla == 1){
			GPIO_PORTF_DATA_R^=0x02; // Invierte el valor del LED rojo
			Nokia5110_ClearBuffer();
			ActualizarPantalla=0;
			// Mostrar voltaje
			muestras_ypos[xEnemigo]=analizar_codigo(adquirir_muestra());
			muestras[xEnemigo]=adquirir_muestra();	
			mostrar_voltaje();
			


			// Cálculo de los voltajes según posición X de los cursores y de la frecuencia, luego son convertidos a arreglos de caracteres para
			// mostrarse en la pantalla 
			V1 = (float)((amplitud*muestras[x_cursor_1])/4095);
			char voltchar1[4];
			snprintf(voltchar1, sizeof voltchar1, "%f", V1);
			
			V2 = (float)((amplitud*muestras[x_cursor_2])/4095);
			char voltchar2[4];
			snprintf(voltchar2, sizeof voltchar2, "%f", V2);
			
			float frec = 1000/(float)velocidaMillis;
			char frecchar[6];
			snprintf(frecchar, sizeof frecchar, "%f", frec);
			
			char escalachar1[1] = {escala + '0'};
			char escalachar2[1] = {escala_1 + '0'};
			
			// Se muestra el texto en la pantalla
			text(0,0,frecchar,1);
			text(30,0," Hz",1);
			text(50,0,escalachar1,1);
			text(54,0,":",1);
			text(58,0,escalachar2,1);
			text(2,41,voltchar1,1);
			text(30,41,voltchar2,1);			
			Nokia5110_DisplayBuffer();

		}
		copyToScreen((unsigned char*)init);
		
		//Verificar si los cursores se mueven o si se pausa la lectura de voltaje

		if((GPIO_PORTF_DATA_R & 0x11) == 0x01){
				 dir=0;
			while((GPIO_PORTF_DATA_R & 0x11) == 0x01){}

		// Si se presiona el SW1 disminuye la velocidad en 100ms
		}else if((GPIO_PORTF_DATA_R & 0x11) == 0x10){
			

			while((GPIO_PORTF_DATA_R & 0x11) == 0x10){}
			 dir=1;
				
		}
		if(cursor_on==1){
			configPulsadores_D();	
			// KEY0 Cursor 1 izquierda
			if((GPIO_PORTD_DATA_R & 0x0F) == 0x01){
				while((GPIO_PORTD_DATA_R & 0x0F) == 0x01){}
				if(x_cursor_1>2){
				  x_cursor_1 = x_cursor_1-2;
					V1 = (float)muestras[x_cursor_1]/4095;
					char voltchar1[4];
					snprintf(voltchar1, sizeof voltchar1, "%f", V1);
					text(2,41,voltchar1,1);
				}else{
					x_cursor_1=0;
				}	
			// KEY1 Cursor 1 derecha
			} else if((GPIO_PORTD_DATA_R & 0x0F) == 0x02){
				while((GPIO_PORTD_DATA_R & 0x0F) == 0x02){}
				if(x_cursor_1<82){
				  x_cursor_1 = x_cursor_1+2;
					V1 = (float)muestras[x_cursor_1]/4095;
					char voltchar1[4];
					snprintf(voltchar1, sizeof voltchar1, "%f", V1);
					text(2,41,voltchar1,1);
				}else{
					x_cursor_1=83;
				}
			// KEY2 Cursor 2 izquierda
			} else if((GPIO_PORTD_DATA_R & 0x0F) == 0x04){
				while((GPIO_PORTD_DATA_R & 0x0F) == 0x04){}
				if(x_cursor_2>2){
				  x_cursor_2 = x_cursor_2-2;
					V2 = (float)muestras[x_cursor_2]/4095;
					char voltchar2[4];
					snprintf(voltchar2, sizeof voltchar2, "%f", V2);
					text(30,41,voltchar2,1);	
				}else{
					x_cursor_2=0;
				}		
			// KEY3 Cursor 2 derecha
			} else if((GPIO_PORTD_DATA_R & 0x0F) == 0x08){
				while((GPIO_PORTD_DATA_R & 0x0F) == 0x08){}
				if(x_cursor_2<82){
				  x_cursor_2 = x_cursor_2+2;
					V2 = (float)muestras[x_cursor_2]/4095;
					char voltchar2[4];
					snprintf(voltchar2, sizeof voltchar2, "%f", V2);
					text(30,41,voltchar2,1);	
				}else{
					x_cursor_2=83;
				}
			}
		}
		//Verificar si se ordena cambio de escala, desplazamiento o implementacion de cursores
		if(HayCarRx()){
		settings = RxCar();
		TxCadena( "\r\n" );
		
			switch(settings){
				case 'a': // Desplazar a la izquierda
					if(dez==0){
						dez=dez+11;
					}else if(dez>0 && dez<72){
						dez=dez+12;
					}else if(dez>72){
						dez=83;
					}
					mostrar_voltaje();	
					break;
				case 'd': // Desplazar a la derecha
					if(dez==11){
						dez=dez-11;
					}else if(dez>11 && dez<84){
						dez=dez-12;
					}else if(dez<11){
						dez=0;
					}
					mostrar_voltaje();	
					break;
				case 'w': // Desplazar arriba
					if(amp>0){
						amp=amp-1;	
					}else{
						amp=0;
					}
					mostrar_voltaje();
					break;
				case 's': // Desplazar abajo
					if(amp<=15){
						amp=amp+1;
					}
				  mostrar_voltaje();
					break;
				case 'j': // Disminuir escala
					if(horizontal<=5){
						horizontal=horizontal+1;
					} else{
						horizontal=5;
					}
					calcular_escala();
					mostrar_voltaje();
					break;
				case 'l': // Aumentar escala
					if(horizontal>0){
						horizontal=horizontal-1;
					} else{
						horizontal=1;
					}
					calcular_escala();
					mostrar_voltaje();
					break;
				case 'n': // Mostrar cursores
					cursor_on=1;
					mostrar_voltaje();
					break;
				case 'f': // Ocultar cursores
					cursor_on=0;
					mostrar_voltaje();
					break;
				case '1': // Aumentar frecuencia
					if(velocidaMillis>10){
						velocidaMillis-=5;
					}else{
						velocidaMillis=velocidaMillis;
					}
					break;
				case '2': // Disminuir frecuencia
					if(velocidaMillis<=65530){
						velocidaMillis+=5;
					}else{
						velocidaMillis=velocidaMillis;					
					}
					break;
				default:
					break;
				}

		}

	}
}



