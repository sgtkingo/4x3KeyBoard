/*
 * File:   Core.c
 * Author: kater
 *
 * Created on 15. b?ezna 2018, 10:30
 */

// PIC18F46K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (Power up timer enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)
// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic18lf46k22.h>

#define _XTAL_FREQ 1000000
#define CLEAR PORTD=0x00;

#define FAIL 99

#define FALSE 0
#define TRUE 1

#define OUT 0
#define IN 1

#define TEST_ZEROandERROR 0
#define TEST_R1 1
#define TEST_R2 2
#define TEST_R3 3
#define TEST_R4 4

#define S1 PORTBbits.RB2
#define S2 PORTBbits.RB0
#define S3 PORTBbits.RB4

#define R1 PORTBbits.RB1
#define R2 PORTBbits.RB6
#define R3 PORTBbits.RB5
#define R4 PORTBbits.RB3

#define LED PORTBbits.RB7

#define R1IO     TRISBbits.RB1
#define R2IO     TRISBbits.RB6
#define R3IO     TRISBbits.RB5
#define R4IO     TRISBbits.RB3

void InitDevice();

unsigned int KeyValue=0,Pre_KeyValue=0, TempState;
int Counter=TEST_R4;
bit EndOfDetection=FALSE;
bit RotateDirection;

void main(void) {
    InitDevice();
    CLEAR;
    //PORTD=0b10000000;
    while (1)
    {
    asm ("NOP");
    //PORTD!=FALSE ? LED=TRUE : LED=~LED;
    KeyValue==FAIL ? LED=~LED : (PORTD==FALSE ? LED=FALSE : LED=TRUE);
    /*PORTD==0x01 ? RotateDirection=FALSE : RotateDirection=TRUE;
    RotateDirection ? PORTD=PORTD>>1 : PORTD=PORTD<<1 ;*/
    __delay_ms(250);
    }
}

void InitDevice()
{
    TRISD=0; //All Port D pins are out
    ANSELD=0; //All Port D pins are dig
    
    TRISBbits.RB0=1; //Set port B as input
    TRISBbits.RB1=1; //Set port B as input
    TRISBbits.RB2=1; //Set port B as input
    TRISBbits.RB3=1; //Set port B as input
    TRISBbits.RB4=1; //Set port B as input
    TRISBbits.RB5=1; //Set port B as input
    TRISBbits.RB6=1; //Set port B as input
    TRISBbits.RB7=0; //Set port B as output
    ANSELB=0; //Set port B as digital
    
    
    INTCON2bits.nRBPU=0; //WPU settings
    //WPUB=1;
    WPUBbits.WPUB4=1;
    WPUBbits.WPUB2=1;
    WPUBbits.WPUB0=1;
    
    OSCCONbits.OSTS=1; //Set default oscilator to internal
    OSCCONbits.SCS=0b11; //System clock select bit    
    OSCCONbits.IRCF=0b011; //f=1 MHz
    
    INTCON=0b11100000; //Timer 0 Interupt
    T0CON=0b11010010; //Timer 0 setting
}

void interrupt ISR(void)
{
    if(INTCONbits.T0IF){
        TMR0IE=0;
        TempState=PORTB;
        if (Counter<0) 
            Counter=TEST_R4;

        if (KeyValue==FAIL||KeyValue==FALSE) KeyValue=Pre_KeyValue;

        if (KeyValue!=Pre_KeyValue)  
        {
        KeyValue=Pre_KeyValue=FAIL;
        Counter=FAIL;    
        EndOfDetection=TRUE;
        }  
        if (EndOfDetection==FALSE)
        {
        switch (Counter)
        {
            case TEST_ZEROandERROR: 
            {
                R1IO=R2IO=R3IO=R4IO=OUT;
                R1=R2=R3=R4=0;
                asm("NOP");
                if (S1&&S2&&S3)
                    KeyValue=Pre_KeyValue=FALSE;
                if ((!S1&&!S2)||(!S1&&!S3)||(!S2&&!S3))
                    KeyValue=Pre_KeyValue=FAIL;
                R1IO=R2IO=R3IO=R4IO=IN;

                EndOfDetection=TRUE;
                Counter=FAIL;
                break;
            }
            case TEST_R1:
            {
                R1IO=OUT;
                asm("NOP");
                R1=0;
                asm("NOP");
                if (!S1)Pre_KeyValue=1;
                if (!S2)Pre_KeyValue=2;
                if (!S3)Pre_KeyValue=3;
                R1IO=IN;
                break;
            }
            case TEST_R2:
            {
                R2IO=OUT;
                asm("NOP");
                R2=0;
                asm("NOP");
                if (!S1)Pre_KeyValue=4;
                if (!S2)Pre_KeyValue=5;
                if (!S3)Pre_KeyValue=6;
                R2IO=IN;
                break;
            }
            case TEST_R3:
            {
                R3IO=OUT;
                asm("NOP");
                R3=0;
                asm("NOP");
                if (!S1)Pre_KeyValue=7;
                if (!S2)Pre_KeyValue=8;
                if (!S3)Pre_KeyValue=9;
                R3IO=IN;
                break;
            }
            case TEST_R4:
            {
                R4IO=OUT; //R4=0;R3=R2=R1=1;
                asm("NOP");
                R4=0;
                asm("NOP");
                if (!S1/*&&S2&&S3*/)Pre_KeyValue=10; //*
                if (!S2)Pre_KeyValue=11; //0
                if (!S3)Pre_KeyValue=12; //#
                R4IO=IN;
                break;
            }
            default: 
            {
                Counter=TEST_R4;
                break;
            }
        }
        }
        if (EndOfDetection)
        {
            switch (KeyValue)
        {
            case 0: {CLEAR;break;}
            case 1: {PORTD=0x31;break;}
            case 2: {PORTD=0x32;break;}
            case 3: {PORTD=0x33;break;}
            case 4: {PORTD=0x34;break;}
            case 5: {PORTD=0x35;break;}
            case 6: {PORTD=0x36;break;}
            case 7: {PORTD=0x37;break;}
            case 8: {PORTD=0x38;break;}
            case 9: {PORTD=0x39;break;}
            case 10: {PORTD=0x2A;break;}
            case 11: {PORTD=0x30;break;}
            case 12: {PORTD=0x23;break;}
            default: {CLEAR;KeyValue=FAIL;break;}
        }
        KeyValue!=FAIL ? KeyValue=Pre_KeyValue=FALSE : KeyValue;
        EndOfDetection=FALSE;
        }
        Counter--;
        TMR0IF=0;
        TMR0IE=1; 
    }
}