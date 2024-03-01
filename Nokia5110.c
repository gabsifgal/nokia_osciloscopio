// Nokia5110.c (File Modified by Valentin Sarmiento)
// Runs on LM4F120/TM4C123
// Use SSI0 to send an 8-bit code to the Nokia5110 48x84
// pixel LCD to display text, images, or other information.
// Daniel Valvano
// September 16, 2013

// Font table, initialization, and other functions based
// off of Nokia_5110_Example from Spark Fun:
// 7-17-2011
// Spark Fun Electronics 2011
// Nathan Seidle
// http://dlnmh9ip6v2uc.cloudfront.net/datasheets/LCD/Monochrome/Nokia_5110_Example.pde

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// Blue Nokia 5110
// ---------------
// Signal        (Nokia 5110) LaunchPad pin
// Reset         (RST, pin 1) connected to PA7
// SSI0Fss       (CE,  pin 2) connected to PA3
// Data/Command  (DC,  pin 3) connected to PA6
// SSI0Tx        (Din, pin 4) connected to PA5
// SSI0Clk       (Clk, pin 5) connected to PA2
// 3.3V          (Vcc, pin 6) power
// back light    (BL,  pin 7) not connected, consists of 4 white LEDs which draw ~80mA total
// Ground        (Gnd, pin 8) ground

// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// SSI0Fss       (SCE, pin 3) connected to PA3
// Reset         (RST, pin 4) connected to PA7
// Data/Command  (D/C, pin 5) connected to PA6
// SSI0Tx        (DN,  pin 6) connected to PA5
// SSI0Clk       (SCLK, pin 7) connected to PA2
// back light    (LED, pin 8) not connected, consists of 4 white LEDs which draw ~80mA total

#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "Nokia5110.h"
#include "TivaES.h"
#include "Terminal3.h"
#include "Terminal6.h"
#include "Terminal12.h"


#define DC                      (*((volatile unsigned long *)0x40004100))
#define DC_COMMAND              0
#define DC_DATA                 0x40
#define RESET                   (*((volatile unsigned long *)0x40004200))
#define RESET_LOW               0
#define RESET_HIGH              0x80
//#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
//#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
//#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
//#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
//#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
//#define SSI0_CR0_R              (*((volatile unsigned long *)0x40008000))
//#define SSI0_CR1_R              (*((volatile unsigned long *)0x40008004))
//#define SSI0_DR_R               (*((volatile unsigned long *)0x40008008))
//#define SSI0_SR_R               (*((volatile unsigned long *)0x4000800C))
//#define SSI0_CPSR_R             (*((volatile unsigned long *)0x40008010))
//#define SSI0_CC_R               (*((volatile unsigned long *)0x40008FC8))
#define SSI_CR0_SCR_M           0x0000FF00  // SSI Serial Clock Rate
#define SSI_CR0_SPH             0x00000080  // SSI Serial Clock Phase
#define SSI_CR0_SPO             0x00000040  // SSI Serial Clock Polarity
#define SSI_CR0_FRF_M           0x00000030  // SSI Frame Format Select
#define SSI_CR0_FRF_MOTO        0x00000000  // Freescale SPI Frame Format
#define SSI_CR0_DSS_M           0x0000000F  // SSI Data Size Select
#define SSI_CR0_DSS_8           0x00000007  // 8-bit data
#define SSI_CR1_MS              0x00000004  // SSI Master/Slave Select
#define SSI_CR1_SSE             0x00000002  // SSI Synchronous Serial Port
                                            // Enable
#define SSI_SR_BSY              0x00000010  // SSI Busy Bit
#define SSI_SR_TNF              0x00000002  // SSI Transmit FIFO Not Full
#define SSI_CPSR_CPSDVSR_M      0x000000FF  // SSI Clock Prescale Divisor
#define SSI_CC_CS_M             0x0000000F  // SSI Baud Clock Source
#define SSI_CC_CS_SYSPLL        0x00000000  // Either the system clock (if the
                                            // PLL bypass is in effect) or the
                                            // PLL output (default)
//#define SYSCTL_RCGC1_R          (*((volatile unsigned long *)0x400FE104))
//#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC1_SSI0       0x00000010  // SSI0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control

char Screen[SCREENW*SCREENH/8]; // buffer stores the next image to be printed on the screen
char null[SCREENW*SCREENH/8] = {0};

enum typeOfWrite{
  COMMAND,                              // the transmission is an LCD command
  DATA                                  // the transmission is data
};
// The Data/Command pin must be valid when the eighth bit is
// sent.  The SSI module has hardware input and output FIFOs
// that are 8 locations deep.  Based on the observation that
// the LCD interface tends to send a few commands and then a
// lot of data, the FIFOs are not used when writing
// commands, and they are used when writing data.  This
// ensures that the Data/Command pin status matches the byte
// that is actually being transmitted.
// The write command operation waits until all data has been
// sent, configures the Data/Command pin for commands, sends
// the command, and then waits for the transmission to
// finish.
// The write data operation waits until there is room in the
// transmit FIFO, configures the Data/Command pin for data,
// and then adds the data to the transmit FIFO.

// This is a helper function that sends an 8-bit message to the LCD.
// inputs: type     COMMAND or DATA
//         message  8-bit code to transmit
// outputs: none
// assumes: SSI0 and port A have already been initialized and enabled
void static lcdwrite(enum typeOfWrite type, char message){
  if(type == COMMAND){
                                        // wait until SSI0 not busy/transmit FIFO empty
    while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
    DC = DC_COMMAND;
    SSI0_DR_R = message;                // command out
                                        // wait until SSI0 not busy/transmit FIFO empty
    while((SSI0_SR_R&SSI_SR_BSY)==SSI_SR_BSY){};
  } else{
    while((SSI0_SR_R&SSI_SR_TNF)==0){}; // wait until transmit FIFO not full
    DC = DC_DATA;
    SSI0_DR_R = message;                // data out
  }
}

//********Nokia5110_Init*****************
// Initialize Nokia 5110 48x84 LCD by sending the proper
// commands to the PCD8544 driver.  One new feature of the
// LM4F120 is that its SSIs can get their baud clock from
// either the system clock or from the 16 MHz precision
// internal oscillator.
// inputs: none
// outputs: none
// assumes: system clock rate of 80 MHz
void Nokia5110_Init(void){
  volatile unsigned long delay;
  SYSCTL_RCGC1_R |= SYSCTL_RCGC1_SSI0;  // activate SSI0
  SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA; // activate port A
  delay = SYSCTL_RCGC2_R;               // allow time to finish activating
  GPIO_PORTA_DIR_R |= 0xC0;             // make PA6,7 out
  GPIO_PORTA_AFSEL_R |= 0x2C;           // enable alt funct on PA2,3,5
  GPIO_PORTA_AFSEL_R &= ~0xC0;          // disable alt funct on PA6,7
  GPIO_PORTA_DEN_R |= 0xEC;             // enable digital I/O on PA2,3,5,6,7
                                        // configure PA2,3,5 as SSI
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0xFF0F00FF)+0x00202200;
                                        // configure PA6,7 as GPIO
  GPIO_PORTA_PCTL_R = (GPIO_PORTA_PCTL_R&0x00FFFFFF)+0x00000000;
  GPIO_PORTA_AMSEL_R &= ~0xEC;          // disable analog functionality on PA2,3,5,6,7
  SSI0_CR1_R &= ~SSI_CR1_SSE;           // disable SSI
  SSI0_CR1_R &= ~SSI_CR1_MS;            // master mode
                                        // configure for system clock/PLL baud clock source
  SSI0_CC_R = (SSI0_CC_R&~SSI_CC_CS_M)+SSI_CC_CS_SYSPLL;
                                        // clock divider for 3.33 MHz SSIClk (80 MHz PLL/24)
                                        // SysClk/(CPSDVSR*(1+SCR))
                                        // 80/(24*(1+0)) = 3.33 MHz (slower than 4 MHz)
  SSI0_CPSR_R = (SSI0_CPSR_R&~SSI_CPSR_CPSDVSR_M)+24; // must be even number
  SSI0_CR0_R &= ~(SSI_CR0_SCR_M |       // SCR = 0 (3.33 Mbps data rate)
                  SSI_CR0_SPH |         // SPH = 0
                  SSI_CR0_SPO);         // SPO = 0
                                        // FRF = Freescale format
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_FRF_M)+SSI_CR0_FRF_MOTO;
                                        // DSS = 8-bit data
  SSI0_CR0_R = (SSI0_CR0_R&~SSI_CR0_DSS_M)+SSI_CR0_DSS_8;
  SSI0_CR1_R |= SSI_CR1_SSE;            // enable SSI

  RESET = RESET_LOW;                    // reset the LCD to a known state
  for(delay=0; delay<10; delay=delay+1);// delay minimum 100 ns
  RESET = RESET_HIGH;                   // negative logic

  lcdwrite(COMMAND, 0x21);              // chip active; horizontal addressing mode (V = 0); use extended instruction set (H = 1)
                                        // set LCD Vop (contrast), which may require some tweaking:
  lcdwrite(COMMAND, CONTRAST);          // try 0xB1 (for 3.3V red SparkFun), 0xB8 (for 3.3V blue SparkFun), 0xBF if your display is too dark, or 0x80 to 0xFF if experimenting
  lcdwrite(COMMAND, 0x04);              // set temp coefficient
  lcdwrite(COMMAND, 0x14);              // LCD bias mode 1:48: try 0x13 or 0x14

  lcdwrite(COMMAND, 0x20);              // we must send 0x20 before modifying the display control mode
  lcdwrite(COMMAND, 0x0C);              // set display control to normal mode: 0x0D for inverse
}

void setContrast(unsigned char value) {
    if (value > 0x7f) value = 0x7f;

    lcdwrite(COMMAND, 0x21);
    lcdwrite(COMMAND, 0x80 + value);
    lcdwrite(COMMAND, 0x20);
}

//********Nokia5110_OutChar*****************
// Print a character to the Nokia 5110 48x84 LCD.  The
// character will be printed at the current cursor position,
// the cursor will automatically be updated, and it will
// wrap to the next row or back to the top if necessary.
// One blank column of pixels will be printed on either side
// of the character for readability.  Since characters are 8
// pixels tall and 5 pixels wide, 12 characters fit per row,
// and there are six rows.
// inputs: data  character to print
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutChar(unsigned char data){
  int i;
    
  lcdwrite(DATA, 0x00);                 // blank vertical line padding
  for(i=0; i<5; i=i+1){
    lcdwrite(DATA, ASCII[data - 0x20][i]);
  }
     //lcdwrite(DATA, 0x00);                 // blank vertical line padding
    lcdwrite(DATA, 0x00);                 // blank vertical line padding
}


void Nokia5110_OutChar2(unsigned char data){
  int i;
  lcdwrite(DATA, 0x00);                 // blank vertical line padding
  for(i=0; i<5; i=i+1){
    lcdwrite(DATA, ASCII[data - 0x20][i]);
  }
     //lcdwrite(DATA, 0x00);                 // blank vertical line padding
    lcdwrite(DATA, 0x00);                 // blank vertical line padding
    for(i=1; i<4; i=i+1){
    lcdwrite(DATA, ASCII['|' - 0x20][i]);
  }
}

//********Nokia5110_OutString*****************
// Print a string of characters to the Nokia 5110 48x84 LCD.
// The string will automatically wrap, so padding spaces may
// be needed to make the output look optimal.
// inputs: ptr  pointer to NULL-terminated ASCII string
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutString(char *ptr){
  while(*ptr){
    Nokia5110_OutChar(*ptr);
    ptr = ptr + 1;
  }
}

//********Nokia5110_OutUDec*****************
// Output a 16-bit number in unsigned decimal format with a
// fixed size of five right-justified digits of output.
// Inputs: n  16-bit unsigned number
// Outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_OutUDec(unsigned short n){
  if(n < 10){
    Nokia5110_OutString("    ");
    Nokia5110_OutChar(n+'0'); /* n is between 0 and 9 */
  } else if(n<100){
    Nokia5110_OutString("   ");
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  } else if(n<1000){
    Nokia5110_OutString("  ");
    Nokia5110_OutChar(n/100+'0'); /* hundreds digit */
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  }
  else if(n<10000){
    Nokia5110_OutChar(' ');
    Nokia5110_OutChar(n/1000+'0'); /* thousands digit */
    n = n%1000;
    Nokia5110_OutChar(n/100+'0'); /* hundreds digit */
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  }
  else {
    Nokia5110_OutChar(n/10000+'0'); /* ten-thousands digit */
    n = n%10000;
    Nokia5110_OutChar(n/1000+'0'); /* thousands digit */
    n = n%1000;
    Nokia5110_OutChar(n/100+'0'); /* hundreds digit */
    n = n%100;
    Nokia5110_OutChar(n/10+'0'); /* tens digit */
    Nokia5110_OutChar(n%10+'0'); /* ones digit */
  }
}

//********Nokia5110_SetCursor*****************
// Move the cursor to the desired X- and Y-position.  The
// next character will be printed here.  X=0 is the leftmost
// column.  Y=0 is the top row.
// inputs: newX  new X-position of the cursor (0<=newX<=11)
//         newY  new Y-position of the cursor (0<=newY<=5)
// outputs: none
void Nokia5110_SetCursor(unsigned char newX, unsigned char newY){
  if((newX > 11) || (newY > 5)){        // bad input
    return;                             // do nothing
  }
  // multiply newX by 7 because each character is 7 columns wide
  lcdwrite(COMMAND, 0x80|(newX*7));     // setting bit 7 updates X-position
  lcdwrite(COMMAND, 0x40|newY);         // setting bit 6 updates Y-position
}
void Nokia5110_SetCursorChar(unsigned char newX, unsigned char newY,unsigned char data){
    
  if((newX > 6) || (newY > 5)){        // bad input
    return;                             // do nothing
  }
    if(newX==6)
    {
    lcdwrite(COMMAND, 0x80|(newX*13));     // setting bit 7 updates X-position
  lcdwrite(COMMAND, 0x40|newY);         // setting bit 6 updates Y-position
    Nokia5110_OutChar(data);
    }
    else
    {
  // multiply newX by 13 because each character is 13 columns wide
  lcdwrite(COMMAND, 0x80|(newX*13));     // setting bit 7 updates X-position
  lcdwrite(COMMAND, 0x40|newY);         // setting bit 6 updates Y-position
    Nokia5110_OutChar2(data);
    
    }
}

//********Nokia5110_Clear*****************
// Clear the LCD by writing zeros to the entire screen and
// reset the cursor to (0,0) (top left corner of screen).
// inputs: none
// outputs: none
void Nokia5110_Clear(void){
  int i;
  for(i=0; i<(MAX_X*MAX_Y/8); i=i+1){
    lcdwrite(DATA, 0x00);
  }
  Nokia5110_SetCursor(0, 0);
}

//********Nokia5110_DrawFullImage*****************
// Fill the whole screen by drawing a 48x84 bitmap image.
// inputs: ptr  pointer to 504 byte bitmap
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_DrawFullImage(const char *ptr){
  int i;
  Nokia5110_SetCursor(0, 0);
  for(i=0; i<(MAX_X*MAX_Y/8); i=i+1){
    lcdwrite(DATA, ptr[i]);
  }
}

//********Nokia5110_PrintBMP*****************
// Bitmaps defined above were created for the LM3S1968 or
// LM3S8962's 4-bit grayscale OLED display.  They also
// still contain their header data and may contain padding
// to preserve 4-byte alignment.  This function takes a
// bitmap in the previously described format and puts its
// image data in the proper location in the buffer so the
// image will appear on the screen after the next call to
//   Nokia5110_DisplayBuffer();
// The interface and operation of this process is modeled
// after RIT128x96x4_BMP(x, y, image);
// inputs: xpos      horizontal position of bottom left corner of image, columns from the left edge
//                     must be less than 84
//                     0 is on the left; 82 is near the right
//         ypos      vertical position of bottom left corner of image, rows from the top edge
//                     must be less than 48
//                     2 is near the top; 47 is at the bottom
//         ptr       pointer to a 16 color BMP image
//         threshold grayscale colors above this number make corresponding pixel 'on'
//                     0 to 14
//                     0 is fine for ships, explosions, projectiles, and bunkers
// outputs: none
void Nokia5110_PrintBMP(unsigned char xpos, unsigned char ypos, const unsigned char *ptr, unsigned char threshold){
  long width = ptr[18], height = ptr[22], i, j;
  unsigned short screenx, screeny;
  unsigned char mask;
  // check for clipping
  if((height <= 0) ||              // bitmap is unexpectedly encoded in top-to-bottom pixel order
     ((width%2) != 0) ||           // must be even number of columns
     ((xpos + width) > SCREENW) || // right side cut off
     (ypos < (height - 1)) ||      // top cut off
     (ypos > SCREENH))           { // bottom cut off
    return;
  }
  if(threshold > 14){
    threshold = 14;             // only full 'on' turns pixel on
  }
  // bitmaps are encoded backwards, so start at the bottom left corner of the image
  screeny = ypos/8;
  screenx = xpos + SCREENW*screeny;
  mask = ypos%8;                // row 0 to 7
  mask = 0x01<<mask;            // now stores a mask 0x01 to 0x80
  j = ptr[10];                  // byte 10 contains the offset where image data can be found
  for(i=1; i<=(width*height/2); i=i+1){
    // the left pixel is in the upper 4 bits
    if(((ptr[j]>>4)&0xF) > threshold){
      Screen[screenx] |= mask;
    } else{
      Screen[screenx] &= ~mask;
    }
    screenx = screenx + 1;
    // the right pixel is in the lower 4 bits
    if((ptr[j]&0xF) > threshold){
      Screen[screenx] |= mask;
    } else{
      Screen[screenx] &= ~mask;
    }
    screenx = screenx + 1;
    j = j + 1;
    if((i%(width/2)) == 0){     // at the end of a row
      if(mask > 0x01){
        mask = mask>>1;
      } else{
        mask = 0x80;
        screeny = screeny - 1;
      }
      screenx = xpos + SCREENW*screeny;
      // bitmaps are 32-bit word aligned
      switch((width/2)%4){      // skip any padding
        case 0: j = j + 0; break;
        case 1: j = j + 3; break;
        case 2: j = j + 2; break;
        case 3: j = j + 1; break;
      }
    }
  }
}
// There is a buffer in RAM that holds one screen
// This routine clears this buffer
void Nokia5110_ClearBuffer(void){int i;
  for(i=0; i<SCREENW*SCREENH/8; i=i+1){
    Screen[i] = 0;              // clear buffer
  }
}

//********Nokia5110_DisplayBuffer*****************
// Fill the whole screen by drawing a 48x84 screen image.
// inputs: none
// outputs: none
// assumes: LCD is in default horizontal addressing mode (V = 0)
void Nokia5110_DisplayBuffer(void){
  Nokia5110_DrawFullImage(Screen);
}




/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
/******************************************************************************************************************/
void clearBuffer() {
    for (int i=0; i<(SCREENW*SCREENH/8); i++){
        Screen[i] = 0;
    }
}

void fillBuffer(unsigned char* ptr) {
    for (int i=0; i<(SCREENW*SCREENH/8); i++){
        Screen[i] = ptr[i];
    }
}
/*
void copyToScreen(unsigned char* ptr) {
    int i;
    for (i=0; i<(SCREENW*SCREENH/8); i++){
        lcdwrite(DATA, Screen[i]|ptr[i]);
    }
}
*/

void copyToScreen(void) {
    int i;
    for (i=0; i<(SCREENW*SCREENH/8); i++){
        lcdwrite(DATA, Screen[i]);
    }
}

void setPixel(int x, int y){
    int by, bi;

    if ((x>=0) && (x<SCREENW) && (y>=0) && (y<SCREENH)){
        by = ((y/8)*SCREENW) + x;
        bi = y % 8;

        Screen[by] = Screen[by] | (1<<bi);
    }
}

void clearPixel(int x, int y){
    int by, bi;

    if ((x>=0) && (x<SCREENW) && (y>=0) && (y<SCREENH)){
        by = ((y/8) * SCREENW) + x;
        bi = y % 8;

        Screen[by] = Screen[by] & ~(1<<bi);
    }
}

int readPixel(int x, int y){
    int by, bi, tssb = 0;
  unsigned char tss = 0;

    if ((x>=0) && (x<84) && (y>=0) && (y<48)){
        by = ((y/8)*84) + x;
        bi = y % 8;

        tss = Screen[by] & (1<<bi);
    tssb = tss>>bi;
    }
    return tssb;
}

void invertPixel(int x, int y){
    int by, bi;

    if ((x>=0) && (x<84) && (y>=0) && (y<48)){
        by = ((y/8)*84) + x;
        bi = y % 8;

        if ((Screen[by] & (1<<bi))==0)
            Screen[by] = Screen[by] | (1<<bi);
        else
            Screen[by] = Screen[by] & ~(1<<bi);
    }
}

void drawHLine(int x, int y, int l){
    int by, bi, cx;

    if ((x>=0) && (x<84) && (y>=0) && (y<48)){
        for (cx=0; cx<l; cx++){
            by = ((y/8)*84) + x;
            bi = y % 8;

            Screen[by+cx] |= (1<<bi);
        }
    }
}

void clearHLine(int x, int y, int l){
    int by, bi, cx;

    if ((x>=0) && (x<84) && (y>=0) && (y<48)){
        for (cx=0; cx<l; cx++){
            by = ((y/8)*84) + x;
            bi = y % 8;

            Screen[by+cx] &= ~(1<<bi);
        }
    }
}

void drawVLine(int x, int y, int l){
    int cy;

    if ((x>=0) && (x<84) && (y>=0) && (y<48)){
        for (cy=0; cy<l; cy++){
            setPixel(x, y+cy);
        }
    }
}

void clearVLine(int x, int y, int l){
    int cy;

    if ((x>=0) && (x<84) && (y>=0) && (y<48)){
        for (cy=0; cy<l; cy++){
            clearPixel(x, y+cy);
        }
    }
}

void drawLine(int x1, int y1, int x2, int y2){
    int tmp, i;
    double delta, tx, ty;
    //double m, b, dx, dy;
    
    if (((x2-x1)<0)){
        tmp=x1;
        x1=x2;
        x2=tmp;
        tmp=y1;
        y1=y2;
        y2=tmp;
    }

  if (((y2-y1)<0)){
        tmp=x1;
        x1=x2;
        x2=tmp;
        tmp=y1;
        y1=y2;
        y2=tmp;
    }

    if (y1==y2) {
        if (x1>x2){
            tmp=x1;
            x1=x2;
            x2=tmp;
        }
        drawHLine(x1, y1, x2-x1);
    }
    else if (x1==x2){
        if (y1>y2){
            tmp=y1;
            y1=y2;
            y2=tmp;
        }
        drawVLine(x1, y1, y2-y1);
    }
    else if (abs(x2-x1)>abs(y2-y1)){
        delta = ((double)(y2-y1)/(double)(x2-x1));
        ty = (double)(y1);
        if (x1>x2){
            for (i=x1; i>=x2; i--){
                setPixel(i, (int)(ty+0.5));
        ty = ty - delta;
            }
        }
        else{
            for (i=x1; i<=x2; i++){
                setPixel(i, (int)(ty+0.5));
        ty = ty + delta;
            }
        }
    }
    else{
        delta = ((float)(x2-x1)/(float)(y2-y1));
        tx = (float)(x1);
    if (y1>y2){
            for (i=y2+1; i>y1; i--){
                setPixel((int)(tx+0.5), i);
        tx = tx + delta;
            }
    }
    else{
            for (int i=y1; i<y2+1; i++){
                setPixel((int)(tx+0.5), i);
        tx = tx + delta;
            }
    }
    }
}

void clearLine(int x1, int y1, int x2, int y2){
    int tmp, i;
    double delta, tx, ty;
    //double m, b, dx, dy;
    
    if (((x2-x1)<0)){
        tmp=x1;
        x1=x2;
        x2=tmp;
        tmp=y1;
        y1=y2;
        y2=tmp;
    }
  
    if (((y2-y1)<0)){
        tmp=x1;
        x1=x2;
        x2=tmp;
        tmp=y1;
        y1=y2;
        y2=tmp;
    }

    if (y1==y2){
        if (x1>x2){
            tmp=x1;
            x1=x2;
            x2=tmp;
        }
        clearHLine(x1, y1, x2-x1);
    }
    else if (x1==x2){
        if (y1>y2){
            tmp=y1;
            y1=y2;
            y2=tmp;
        }
        clearVLine(x1, y1, y2-y1);
    }
    else if (abs(x2-x1)>abs(y2-y1)){
        delta=((double)(y2-y1)/(double)(x2-x1));
        ty = (double)(y1);
        if (x1>x2){
            for (i=x1; i>=x2; i--){
                clearPixel(i, (int)(ty+0.5));
        ty = ty - delta;
            }
        }
        else{
            for (i=x1; i<=x2; i++){
                clearPixel(i, (int)(ty+0.5));
        ty = ty + delta;
            }
        }
    }
    else{
        delta=((float)(x2-x1)/(float)(y2-y1));
        tx = (float)(x1);
    if (y1>y2){
            for (i=y2+1; i>y1; i--){
                clearPixel((int)(tx+0.5), i);
        tx=tx+delta;
            }
    }
    else{
            for (i=y1; i<y2+1; i++){
                clearPixel((int)(tx+0.5), i);
        tx = tx + delta;
            }
    }
    }
}

void drawRectangle(int x1, int y1, int x2, int y2){
    int tmp;

    if (x1>x2){
        tmp=x1;
        x1=x2;
        x2=tmp;
    }
    if (y1>y2){
        tmp=y1;
        y1=y2;
        y2=tmp;
    }

    drawHLine(x1, y1, x2-x1);
    drawHLine(x1, y2, x2-x1);
    drawVLine(x1, y1, y2-y1);
    drawVLine(x2, y1, y2-y1+1);
}

void clearRectangle(int x1, int y1, int x2, int y2){
    int tmp;

    if (x1>x2){
        tmp=x1;
        x1=x2;
        x2=tmp;
    }
    if (y1>y2){
        tmp=y1;
        y1=y2;
        y2=tmp;
    }

    clearHLine(x1, y1, x2-x1);
    clearHLine(x1, y2, x2-x1);
    clearVLine(x1, y1, y2-y1);
    clearVLine(x2, y1, y2-y1+1);
}

void drawFilledRectangle(int x1, int y1, int x2, int y2) {
// Fill in all pixels in the rectangular area from x1,y1 to x2,y2
    int i;
    int sx=x2-x1;
    int tmp;
   
    if (x1>x2){
      tmp=x1;
      x1=x2;
      x2=tmp;
    }
    if (y1>y2){
      tmp=y1;
      y1=y2;
      y2=tmp;
    }
    
    for (i=y1; i<y2; i++) {
      drawHLine(x1, i, sx);
    }    
}

void clearFilledRectangle(int x1, int y1, int x2, int y2) {
    int i;
    int sx=x2-x1;
    int tmp;
   
    if (x1>x2){
      tmp=x1;
      x1=x2;
      x2=tmp;
    }
    if (y1>y2){
      tmp=y1;
      y1=y2;
      y2=tmp;
    }
    
    for (i=y1; i<y2; i++) {
      clearHLine(x1, i, sx);
    }    
}

void drawRoundRectangle(int x1, int y1, int x2, int y2){
    int tmp;

    if (x1>x2){
        tmp=x1;
        x1=x2;
        x2=tmp;
    }
    if (y1>y2){
        tmp=y1;
        y1=y2;
        y2=tmp;
    }
    if ((x2-x1)>4 && (y2-y1)>4){
        setPixel(x1+1,y1+1);
        setPixel(x2-1,y1+1);
        setPixel(x1+1,y2-1);
        setPixel(x2-1,y2-1);
        drawHLine(x1+2, y1, x2-x1-3);
        drawHLine(x1+2, y2, x2-x1-3);
        drawVLine(x1, y1+2, y2-y1-3);
        drawVLine(x2, y1+2, y2-y1-3);
    }
}

void clearRoundRectangle(int x1, int y1, int x2, int y2){
    int tmp;

    if (x1>x2){
        tmp=x1;
        x1=x2;
        x2=tmp;
    }
    if (y1>y2){
        tmp=y1;
        y1=y2;
        y2=tmp;
    }
    if ((x2-x1)>4 && (y2-y1)>4){
        clearPixel(x1+1,y1+1);
        clearPixel(x2-1,y1+1);
        clearPixel(x1+1,y2-1);
        clearPixel(x2-1,y2-1);
        clearHLine(x1+2, y1, x2-x1-3);
        clearHLine(x1+2, y2, x2-x1-3);
        clearVLine(x1, y1+2, y2-y1-3);
        clearVLine(x2, y1+2, y2-y1-3);
    }
}

void drawFilledRoundRectangle(int x1, int y1, int x2, int y2){
    int tmp;
    int i;

    if (x1>x2){
        tmp=x1;
        x1=x2;
        x2=tmp;
    }
    if (y1>y2){
        tmp=y1;
        y1=y2;
        y2=tmp;
    }
    if ((x2-x1)>4 && (y2-y1)>4){
        drawHLine(x1+1,y1+1,x2-1-x1-1);
        drawHLine(x1+1,y2-1,x2-1-x1-1);
        drawHLine(x1+2, y1, x2-x1-3);
        drawHLine(x1+2, y2, x2-x1-3);
        for(i=x1; i<x2; i++){
            drawVLine(i, y1+2, y2-y1-3);
        }
    }
}

void clearFilledRoundRectangle(int x1, int y1, int x2, int y2){
    int tmp;
  int i;

    if (x1>x2){
        tmp=x1;
        x1=x2;
        x2=tmp;
    }
    if (y1>y2){
        tmp=y1;
        y1=y2;
        y2=tmp;
    }
    if ((x2-x1)>4 && (y2-y1)>4){
        clearHLine(x1+1,y1+1,x2-1-x1-1);
        clearHLine(x1+1,y2-1,x2-1-x1-1);
        clearHLine(x1+2, y1, x2-x1-3);
        clearHLine(x1+2, y2, x2-x1-3);
        for(i=x1; i<x2; i++){
            clearVLine(i, y1+2, y2-y1-3);
        }
    }
}

void ellipsePlotPoints (int xCenter, int yCenter, int x, int y){
// ***RBO***
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
    setPixel (xCenter + x, yCenter + y);
    setPixel (xCenter - x, yCenter + y);
    setPixel (xCenter + x, yCenter - y);
    setPixel (xCenter - x, yCenter - y);
}

void drawEllipse(int xCenter, int yCenter, int Rx, int Ry){
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
  int Rx2 = Rx*Rx;
  int Ry2 = Ry*Ry;
  int twoRx2 = 2*Rx2;
  int twoRy2 = 2*Ry2;
  int p;
  int x = 0;
  int y = Ry;
  int px = 0;
  int py = twoRx2*y;

  /* Plot the first set of points */
  ellipsePlotPoints(xCenter, yCenter, x, y);
  p = (Ry2 - (Rx2*Ry) + (0.25*Rx2))+0.5;

  /* Region 1 */
  while (px < py) {
    x++;
    px += twoRy2;
    if (p < 0){
      p += Ry2 + px;
    }
    else 
    {
      y--;
      py -= twoRx2;
      p += Ry2 + px - py;
    }
    ellipsePlotPoints(xCenter, yCenter, x, y);
   }


  /* Region 2 */
  p = (Ry2*(x+0.5)*(x+0.5) + Rx2*(y-1)*(y-1) - Rx2*Ry2)+0.5;
  while (y > 0) {
    y--;
    py -= twoRx2;
    if (p > 0){
      p += Rx2 - py;
    }
    else {
      x++;
      px += twoRy2;
      p += Rx2 - py + px;
    }
    ellipsePlotPoints(xCenter, yCenter, x, y);
  }
}

void drawRealCircle(int xCenter, int yCenter, int radius){   
    drawEllipse(xCenter, yCenter, radius, radius*0.9); 
}

void drawCirclePoints(int cx, int cy, int x, int y){
// ***RBO***
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
        if (x == 0) {
                setPixel(cx, cy + y);
                setPixel(cx, cy - y);
                setPixel(cx + y, cy);
                setPixel(cx - y, cy);
        } else 
        if (x == y) {
                setPixel(cx + x, cy + y);
                setPixel(cx - x, cy + y);
                setPixel(cx + x, cy - y);
                setPixel(cx - x, cy - y);
        } else 
        if (x < y) {
                setPixel(cx + x, cy + y);
                setPixel(cx - x, cy + y);
                setPixel(cx + x, cy - y);
                setPixel(cx - x, cy - y);
                setPixel(cx + y, cy + x);
                setPixel(cx - y, cy + x);
                setPixel(cx + y, cy - x);
                setPixel(cx - y, cy - x);
        }
}

void drawCircle(int xCenter, int yCenter, int radius){
// ***RBO***
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
  int x = 0;
    int y = radius;
    int p = (5 - radius*4)/4;

    drawCirclePoints(xCenter, yCenter, x, y);
    while (x < y) {
            x++;
            if (p < 0) {
                    p += 2*x+1;
            } else {
                    y--;
                    p += 2*(x-y)+1;
            }
            drawCirclePoints(xCenter, yCenter, x, y);
    }
}

void drawCircleLines(int cx, int cy, int x, int y){
// ***RBO***
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
        if (x == 0) {
                drawLine(cx, cy + y, cx, cy - y);
                // drawLine(cx + y, cy, cx - y, cy);
        } else 
        if (x == y) {
                drawLine(cx + x, cy + y, cx + x, cy - y);
                drawLine(cx - x, cy + y, cx - x, cy - y);
        } else 
        if (x < y) {
                drawLine(cx + x, cy + y, cx + x, cy - y);
                drawLine(cx - x, cy + y, cx - x, cy - y);
                drawLine(cx + y, cy + x, cx + y, cy - x);
                drawLine(cx - y, cy + x, cx - y, cy - x);
        }
}

void ellipsePlotLines (int xCenter, int yCenter, int x, int y){
    drawLine(xCenter + x, yCenter + y, xCenter + x, yCenter - y);
    drawLine(xCenter - x, yCenter + y, xCenter - x, yCenter - y);
}

void drawFilledEllipse(int xCenter, int yCenter, int Rx, int Ry){
// ***RBO***
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
  int Rx2 = Rx*Rx;
  int Ry2 = Ry*Ry;
  int twoRx2 = 2*Rx2;
  int twoRy2 = 2*Ry2;
  int p;
  int x = 0;
  int y = Ry;
  int px = 0;
  int py = twoRx2*y;

  /* Plot the first set of points */
  ellipsePlotLines(xCenter, yCenter, x, y);
  p = (Ry2 - (Rx2*Ry) + (0.25*Rx2))+0.5;

  /* Region 1 */
  while (px < py) {
    x++;
    px += twoRy2;
    if (p < 0){
      p += Ry2 + px;
    }
    else 
    {
      y--;
      py -= twoRx2;
      p += Ry2 + px - py;
    }
    ellipsePlotLines(xCenter, yCenter, x, y);
   }


  /* Region 2 */
  p = (Ry2*(x+0.5)*(x+0.5) + Rx2*(y-1)*(y-1) - Rx2*Ry2)+0.5;
  while (y > 0) {
    y--;
    py -= twoRx2;
    if (p > 0){
      p += Rx2 - py;
    }
    else {
      x++;
      px += twoRy2;
      p += Rx2 - py + px;
    }
    ellipsePlotLines(xCenter, yCenter, x, y);
  }
}

void drawRealFilledCircle(int xCenter, int yCenter, int radius){   
     drawFilledEllipse(xCenter, yCenter, radius, radius*0.9); 
}

void drawFilledCircle(int xCenter, int yCenter, int radius){
        int x = 0;
        int y = radius;
        int p = (5 - radius*4)/4;

        drawCircleLines(xCenter, yCenter, x, y);
        while (x < y) {
                x++;
                if (p < 0) {
                        p += 2*x+1;
                } else {
                        y--;
                        p += 2*(x-y)+1;
                }
                drawCircleLines(xCenter, yCenter, x, y);
        }
}
    
void ellipseClearLines (int xCenter, int yCenter, int x, int y){
    clearLine(xCenter + x, yCenter + y, xCenter + x, yCenter - y);
    clearLine(xCenter - x, yCenter + y, xCenter - x, yCenter - y);
}

void clearFilledEllipse(int xCenter, int yCenter, int Rx, int Ry){
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
  int Rx2 = Rx*Rx;
  int Ry2 = Ry*Ry;
  int twoRx2 = 2*Rx2;
  int twoRy2 = 2*Ry2;
  int p;
  int x = 0;
  int y = Ry;
  int px = 0;
  int py = twoRx2*y;

   /* Plot the first set of points */
  ellipseClearLines(xCenter, yCenter, x, y);
  p = (Ry2 - (Rx2*Ry) + (0.25*Rx2))+0.5;

  /* Region 1 */
  while (px < py) {
    x++;
    px += twoRy2;
    if (p < 0){
      p += Ry2 + px;
    }
    else 
    {
      y--;
      py -= twoRx2;
      p += Ry2 + px - py;
    }
    ellipseClearLines(xCenter, yCenter, x, y);
   }


  /* Region 2 */
  p = (Ry2*(x+0.5)*(x+0.5) + Rx2*(y-1)*(y-1) - Rx2*Ry2)+0.5;
  while (y > 0) {
    y--;
    py -= twoRx2;
    if (p > 0){
      p += Rx2 - py;
    }
    else {
      x++;
      px += twoRy2;
      p += Rx2 - py + px;
    }
    ellipseClearLines(xCenter, yCenter, x, y);
  }
}

void clearCircleLines(int cx, int cy, int x, int y){
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
        if (x == 0) {
                clearLine(cx, cy + y, cx, cy - y);
                // drawLine(cx + y, cy, cx - y, cy);
        } else 
        if (x == y) {
                clearLine(cx + x, cy + y, cx + x, cy - y);
                clearLine(cx - x, cy + y, cx - x, cy - y);
        } else 
        if (x < y) {
                clearLine(cx + x, cy + y, cx + x, cy - y);
                clearLine(cx - x, cy + y, cx - x, cy - y);
                clearLine(cx + y, cy + x, cx + y, cy - x);
                clearLine(cx - y, cy + x, cx - y, cy - x);
        }
}

void clearRealFilledCircle(int xCenter, int yCenter, int radius){   
    clearFilledEllipse(xCenter, yCenter, radius, radius*0.9); 
} 

void clearFilledCircle(int xCenter, int yCenter, int radius){
        int x = 0;
        int y = radius;
        int p = (5 - radius*4)/4;

        clearCircleLines(xCenter, yCenter, x, y);
        while (x < y) {
                x++;
                if (p < 0) {
                        p += 2*x+1;
                } else {
                        y--;
                        p += 2*(x-y)+1;
                }
                clearCircleLines(xCenter, yCenter, x, y);
        }
}

void clearCirclePoints(int cx, int cy, int x, int y){
        if (x == 0) {
                clearPixel(cx, cy + y);
                clearPixel(cx, cy - y);
                clearPixel(cx + y, cy);
                clearPixel(cx - y, cy);
        } else 
        if (x == y) {
                clearPixel(cx + x, cy + y);
                clearPixel(cx - x, cy + y);
                clearPixel(cx + x, cy - y);
                clearPixel(cx - x, cy - y);
        } else 
        if (x < y) {
                clearPixel(cx + x, cy + y);
                clearPixel(cx - x, cy + y);
                clearPixel(cx + x, cy - y);
                clearPixel(cx - x, cy - y);
                clearPixel(cx + y, cy + x);
                clearPixel(cx - y, cy + x);
                clearPixel(cx + y, cy - x);
                clearPixel(cx - y, cy - x);
        }
}

void ellipseClearPoints (int xCenter, int yCenter, int x, int y){
// from Computer Graphics by Hearn & Baker
// https://docs.google.com/file/d/0B_YZ665nBRhlYmNiOTU5ZDItMmU2OC00YTVmLThiNmMtMjg3Y2E3ZTgwZDYw/edit?hl=en_US&pref=2&pli=1
    clearPixel (xCenter + x, yCenter + y);
    clearPixel (xCenter - x, yCenter + y);
    clearPixel (xCenter + x, yCenter - y);
    clearPixel (xCenter - x, yCenter - y);
}

void clearEllipse(int xCenter, int yCenter, int Rx, int Ry){
  int Rx2 = Rx*Rx;
  int Ry2 = Ry*Ry;
  int twoRx2 = 2*Rx2;
  int twoRy2 = 2*Ry2;
  int p;
  int x = 0;
  int y = Ry;
  int px = 0;
  int py = twoRx2*y;

   /* Plot the first set of points */
  ellipseClearPoints(xCenter, yCenter, x, y);
  p = (Ry2 - (Rx2*Ry) + (0.25*Rx2))+0.5;

  /* Region 1 */
  while (px < py) {
    x++;
    px += twoRy2;
    if (p < 0){
      p += Ry2 + px;
    }
    else 
    {
      y--;
      py -= twoRx2;
      p += Ry2 + px - py;
    }
    ellipseClearPoints(xCenter, yCenter, x, y);
   }


  /* Region 2 */
  p = (Ry2*(x+0.5)*(x+0.5) + Rx2*(y-1)*(y-1) - Rx2*Ry2)+0.5;
  while (y > 0) {
    y--;
    py -= twoRx2;
    if (p > 0){
      p += Rx2 - py;
    }
    else {
      x++;
      px += twoRy2;
      p += Rx2 - py + px;
    }
    ellipseClearPoints(xCenter, yCenter, x, y);
  }
}

void clearRealCircle(int xCenter, int yCenter, int radius){   
     clearEllipse(xCenter, yCenter, radius, radius*0.9); 
}    

void clearCircle(int xCenter, int yCenter, int radius){
        int x = 0;
        int y = radius;
        int p = (5 - radius*4)/4;

        clearCirclePoints(xCenter, yCenter, x, y);
        while (x < y) {
                x++;
                if (p < 0) {
                        p += 2*x+1;
                } else {
                        y--;
                        p += 2*(x-y)+1;
                }
                clearCirclePoints(xCenter, yCenter, x, y);
        }
}

void drawByte(unsigned char b, int x, int y) {
// ***RBO*** 
// Print byte at x, y (Graphics co-ordinates)
// x>=0 x<84
// y>=0 y<48  k=y/8  m=y%8 
    int k, m, n;
    unsigned char nb1, nb2;

    k = y/8;
    m = y%8;
    if( m==0 ){        
        n = k*SCREENW + x;
        Screen[n]= b | Screen[n];
    }
    else{        
        n = k*SCREENW + x;
        nb1 = b<<m % 0xFF; 
        Screen[n] = nb1 | Screen[n];
        k = k+1;
        n = k*SCREENW + x;
        nb2 = b>>(8-m);
        Screen[n] = nb2 | Screen[n];       
    }
}

void clearBitmap(int x, int y, const unsigned char arr[], int w, int length) {
// Clear only those pixels set in bitmap  
    int i, j, h=8*length/w, x1=x, x2=x+w, y1=y, y2=y+h;
    int by1, by2, bi, bt_x=0, bt_y=0, bt_my, bt_ky; //v, 
    int bt_ind, bt_b, bt_r;
         
    for (i=y1; i<y2; i++) {
        bt_y = i - y1;  // y-coord inside bitmap
        bt_my = bt_y/8;
        bt_ky = bt_y%8;     
        bi = i % 8;
        by1 = ((i/8)*84);
        for (j=x1; j<x2; j++){
            bt_x = j - x1; // x-coord inside bitmap
            bt_ind = bt_my*w + bt_x;
            bt_b = arr[bt_ind]>>bt_ky;
            bt_r = bt_b & 1;  // value of pixel at bt_x, bt_y inside bitmap
            by2 = by1+j;      
            //v = Screen[by2] & (1<<bi);
            if(bt_r>0){
                 Screen[by2] = Screen[by2] & ~(1<<bi);
             }
        }       
    }    
}

void drawBitmap(int x, int y, const unsigned char arr[], int w, int length) {
    // w is the width of the bitmap 
    // length = total number of bytes in bitmap definition
    int i, xp, yp=y;
    unsigned char b;

    for (i=0; i<length; i++) {
        b = arr[i];
        xp = x + i%w;
        drawByte(b,xp,yp);
        if( (i+1)%w == 0 ){
            yp = yp + 8; 
        }    
    }   
}

void text(int x, int y, const unsigned char s[], unsigned int font) {
// Print text at x, y (Graphics co-ordinates)
// x>=0 x<84
// y>=0 y<48  k=y/8  m=y%8
// i.e. co-ordinates ar the same as for every other graphic.

// (Original used co-ordinates 0<=x<=14 and 0<=y<=5
// for printing text only)
    int i, j, k, m, n, maxi;
    int len = strlen((char *)s);
    unsigned char b, b1, b2;

    if (font==0) {
        k = y/8;
        m = y%8;
        if( m==0 ){
            // setXY(x, k);
            for (j=0; j<len; j++) {
                n = k*SCREENW + j*4+x;
                for (i=0; i<3; i++){ 
                    maxi = x + j*4 + i;
                    b = (Terminal3x5[s[j]-' '][i]);
                    if(maxi<=83){
                        Screen[n+i]= b | Screen[n+i];
                    }
                }
                if(maxi+1 <= (SCREENW-1) ){
                    Screen[n+3] = 0x00 | Screen[n+3];
                }
            }
        }
        else{
            // setXY(x, k);
            for (j=0; j<len; j++) {
            n = k*SCREENW + j*4 + x;
            for (i=0; i<3; i++){
                maxi = x + j*4 + i;
                b = (Terminal3x5[s[j]-' '][i]) << m%0xFF;
                if( maxi <= (SCREENW-1) ){
                    Screen[n+i] = b | Screen[n+i];
                }
            }
            if( maxi+1 <= (SCREENW-1) ){
                Screen[n+3] = 0x00 | Screen[n+3];
            }
            }
            // setXY(x, k+1);
            k = k+1;
            for (j=0; j<len; j++) {
                n = k*SCREENW + j*4 + x;
                for (i=0; i<3; i++){ 
                    maxi = x + j*4 + i;
                    b = (Terminal3x5[s[j]-' '][i])>>(8-m);
                    if( maxi <= (SCREENW-1) ){
                        Screen[n+i] = b | Screen[n+i]; 
                    }
                }
                if( maxi+1 <= (SCREENW-1) ){
                    Screen[n+3] = 0x00 | Screen[n+3];
                }
            }
        }
    }

    else if (font==1) {
        k = y/8;
        m = y%8;
        if(m==0){
            // setXY(x, k);
            for (j=0; j<len; j++) {
                n = k*SCREENW + j*(SCREENH/8) + x;
                for (i=0; i<5; i++){ 
                    maxi = x + j*6 + i;
                    b = (Terminal6x8[s[j]-' '][i]);
                    if( maxi<=83 ){
                        Screen[n+i] = b | Screen[n+i];
                    }
                }
                if( maxi+1 <= 83 ){
                    Screen[n+5] = 0x00 | Screen[n+5];
                }
            }
        }
        else{
            // setXY(x, k);
             for (j=0; j<len; j++) {
                 n = k*84 + j*6 + x;
                    for (i=0; i<5; i++){
                        maxi = x + j*6 + i;
                        b = (Terminal6x8[s[j]-' '][i])<<m %0xFF;
                        if(maxi<=83){
                            Screen[n+i] = b | Screen[n+i];
                        }
                    }
                    if(maxi+1<=83){
                        Screen[n+5] = 0x00 | Screen[n+5];
                    }
             }
            // setXY(x, k+1);
             k=k+1;
             for (j=0; j<len; j++) {
                    n = k*84 + j*6 + x;
                    for (i=0; i<5; i++){ 
                        maxi = x + j*6 + i;
                        b = (Terminal6x8[s[j]-' '][i])>>(8-m);
                        if(maxi<=83){
                            Screen[n+i] = b | Screen[n+i]; 
                        }
                    }
                    if(maxi+1<=83){
                        Screen[n+5] = 0x00 | Screen[n+5];
                    }
             }
            
        }
    }
    else if (font==2) {
        k = y/8;
        m = y%8;
        if(m==0){
                for (j=0; j<len; j++) {
                     n = k*84 + j*12 + x;
                     for (i=0; i<11; i++) {
                         maxi = x + j*12 + i;
                         b = Terminal11x16[s[j]-' '][2*i];  
                         if(maxi<=83){              
                             Screen[n+i] = b | Screen[n+i];
                         }
                     }
                     if(maxi+1<=83){
                         Screen[n+11] = 0x00 | Screen[n+11];
                     }
                }

             // setXY(6*x, y+1);
             k=k+1;
             for (j=0; j<len; j++) {
                     int n = k*84+j*12+x;
                     for (i=0; i<11; i++) {
                            maxi=x+j*12+i;
                            b = Terminal11x16[s[j]-' '][2*i+1];
                            if(maxi<=83){               
                             Screen[n+i] = b | Screen[n+i];
                            }
                     }
                     if(maxi+1<=83){
                         Screen[n+11] = 0x00 | Screen[n+11];
                     }
             } 
        }
        else{
            for (j=0; j<len; j++) {
                 n = k*84 + j*12 + x;
                 for (i=0; i<11; i++) {
                     maxi = x + j*12 + i;
                     b= (Terminal11x16[s[j]-' '][2*i])<<m %0xFF;
                     if(maxi<=83){                
                         Screen[n+i] = b | Screen[n+i];
                     }
                 }
                 if(maxi+1<=83){
                     Screen[n+11] = 0x00 | Screen[n+11];
                 }
            }

            k=k+1;
            for (j=0; j<len; j++) {
                 n = k*84 + j*12 + x;
                 for (i=0; i<11; i++) {
                     maxi = x + j*12 + i;
                     b1=(Terminal11x16[s[j]-' '][2*i])>>(8-m);
                     b2=(Terminal11x16[s[j]-' '][2*i+1])<<m %0xFF; 
                     b = b1 | b2;
                     if(maxi<=83){               
                         Screen[n+i] = b | Screen[n+i];
                     }
                 }
                 if(maxi+1<=83){
                     Screen[n+11] = 0x00 | Screen[n+11];
                 }
            }

            k=k+1;
            for (j=0; j<len; j++) {
                 int n=k*84+j*12+x;
                 for (i=0; i<11; i++) {
                     maxi=x+j*12+i;
                     b = (Terminal11x16[s[j]-' '][2*i+1])>>(8-m);
                     if(maxi<=83){                
                         Screen[n+i] = b | Screen[n+i];
                     }
                 }
                 if(maxi+1<=83){
                     Screen[n+11] = 0x00 | Screen[n+11];
                 }
            }
        }
    }
}
















//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
//****************************************************************************************************************************
void Nokia5110_TestFunctions(const unsigned char *ptr){
    clearBuffer();
    drawByte(0x0F, 1, 1);
    setPixel(2, 2);
    clearPixel(1, 2);
    drawBitmap(50,4,ptr,30,150);
    text(4, 9,  (unsigned char *)"Hello", 0);  
    text(4, 19, (unsigned char *)"Hello", 1); 
    text(4, 30, (unsigned char *)"Hello", 2);
    drawRealCircle(50, 9, 9);   
    //copyToScreen(nullptr);
}

uint32_t readADC(){
    uint32_t result; 
    ADC0_PSSI_R |= 0x0008;            // start a conversion sequence 3 - Start conversion by SS3
    while((ADC0_RIS_R & 0x08) == 0);  // Wait for conversion complete
    result = ADC0_SSFIFO3_R & 0xFFF;  // Read conversion results
    ADC0_ISC_R = 0x0008;              // clear completion flag
    return result;
}

void Nokia5110_Runner(unsigned char *frames[], int Wframe, int Hframe, int Xpos, int Ypos){
    int i = 0, len = Wframe*Hframe/8;
    unsigned char *bck = frames[9];
    unsigned char status;
    uint32_t previous = 0;
    uint32_t j, N = 1500;
    int n = 0xFFF/(84-Wframe);
    int x = 0; 
    char str[32];
    sprintf(str, "X:%0.2d\r\n", x); // convert result to string
    
    while(1){
        for(j=0;j<=N;j++){   // Lazo de control FOR para generar un retardo. 
            status = TivaES_LeePulsador(SW2);
            if( status == NO_PRESIONADO ){
                return;
            }

            x = readADC() / n;
            if( x != previous ){ // move if x position is different
                previous = x;
                sprintf(str, "X:%0.2d\r\n", x); // convert result to string
            }
        }

        //fillBuffer(bck);  //drawBitmap(0, 0, bck, SCREENW, SCREENW*SCREENH/8);
        drawBitmap(x, Ypos, frames[i], Wframe, len);
        text(69, 43,  (unsigned char *)str, 0);
        copyToScreen();
        
        // i++;
        if( ++i == 9 ) i = 0;
    }
}

