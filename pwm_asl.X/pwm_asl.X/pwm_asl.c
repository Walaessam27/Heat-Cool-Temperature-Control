#define _XTAL_FREQ   4000000UL     // needed for the delays, set to 4 MH= your crystal frequency
// CONFIG1H
#pragma config OSC = XT         // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
#include <xc.h>
#include <stdio.h>
#include <pic18f4620.h>
#include "my_ser.h"
#include "my_adc.h"
#include "my_pwm.h"
#include "lcd_x8.h"
//function prototypes
#define STARTVALUE  3036

unsigned int RPS_count = 0;
unsigned char mode = 0; // Mode: 0=OFF, 1=Cool, 2=Heat, 3=Auto Cool
unsigned char hysteresis = 0;
int H = 0 ;
int C = 0 ;

void setupPorts(void) {
    ADCON0 = 0;
    ADCON1 = 0b00001100; //3 analog channels, change this according to your application

    TRISB = 0xFF; // all pushbuttons are inputs
    TRISC = 0x80; // RX input , others output
    TRISA = 0xFF; // All inputs
    TRISD = 0x00; // All outputs
    TRISE = 0x00; // All outputs
}

void PWM_Disable(void) {
    CCP1CON = 0x00;        // Disable PWM
    T2CONbits.TMR2ON = 0;  // Turn off Timer2
    LATCbits.LATC2 = 0;    // Set RC2 pin to low to turn off the fan
}

// This function is needed for measuring speed
void initTimers01(void) {
    T3CON = 0x29; // Timer3 ON, prescaler = 4
    PIE2bits.TMR3IE = 1; // Enable Timer3 interrupt
    
    TMR3H = 0;
    TMR3L = 0;
    
    T2CON=0x07;
    TMR2 = 0;

    CCP2CON=9;
    CCP1CON=0x0c;
    INTCON=INTCON3=INTCON2=0;
    INTCONbits.PEIE=1;
    INTCONbits.GIE = 1; //enable only timer 0 interrupt
    INTCONbits.INT0IE=1;
    
    INTCON3bits.INT2IE=1;
    PIE1bits.TMR2IE=0;
    PIE2bits.TMR3IE=1;
    PIE2bits.CCP2IE=1;
    IPR1=IPR2=PIR1=PIR2=0;
    
}
void set_compare_value(unsigned int compare_value) {
    CCPR2L = compare_value >> 8;
    CCPR2H = compare_value & 0xFF;
}
// used also for measuring speed
//void interrupt high_priority highIsr(void)//old syntax
void __interrupt(high_priority) highIsr(void) {
    if (INTCONbits.INT0IF) {
        __delay_ms(500);
        mode = (mode + 1) % 4; // Cycle through modes
        INTCONbits.INT0IF = 0; // Clear INT0 interrupt flag
    }
    if (INTCON3bits.INT2IF) {
     //   INTCON3bits.INT2IE=0;
        __delay_ms(500);
       // INTCON3bits.INT2IE=1;
        hysteresis = (hysteresis + 1) % 5; // Cycle through hysteresis values
        INTCON3bits.INT2IF = 0; // Clear INT2 interrupt flag
    }
    if (PIR2bits.TMR3IF) {
        PIR2bits.TMR3IF = 0; // Clear Timer3 interrupt flag
        PORTCbits.RC5=1;
    }
    if(PIR2bits.CCP2IF){
        PIR2bits.CCP2IF=0;
        PORTCbits.RC5=0; 
    }
    
}

void main(void) {
    //ADCON1 = 0b00001100; //3 analog channels, change this according to your application
    char Buffer[32]; // for sprintf
    float AN[3];     // To store the voltages of AN0, AN1, AN2
    int raw_val;
    unsigned char channel;
    float voltage;
    setupPorts();
    setupSerial();
    lcd_init();
    init_adc_no_lib();
    init_pwm1();
    lcd_putc('\f'); //clears the display
    int RPS;
     initTimers01();   // These will be used to measure the speed
    while (1) {
        CLRWDT(); // no need for this inside the delay below
        delay_ms(200); //read ADC AN0,AN1, AN2 every 2 seconds
        for (channel = 0; channel < 3; channel++) {
            // read the adc voltage
            voltage = read_adc_voltage((unsigned char) channel);
            AN[channel] = voltage; // store in array AN0--AN2
        } 
        //set_pwm1_raw(raw_val);  // set the Pwm to that value 0--1023
        
        float RT = (float)AN[2] * 100.0;
        lcd_gotoxy(1, 1);
        sprintf(Buffer, "RT:%5.1fC    H C\n", RT);
        lcd_puts(Buffer);
        
        
        float sp = AN[0] * 20 ;
        //sp=read_adc_voltage(2);
        lcd_gotoxy(1, 2);
        sprintf(Buffer, "SP:%5.1f \n", sp);
        lcd_puts(Buffer);
        
       
        float HC = (float)AN[1] *100 / 5.0;
        lcd_gotoxy(1, 3);
        sprintf(Buffer, "HS: %d HC:%5.1f %% \n", hysteresis , HC);
        lcd_puts(Buffer);
        
        if(PORTBbits.RB3==0){
            mode=0;
        }
    raw_val = read_adc_raw_no_lib(1);
    lcd_gotoxy(1, 4);
    switch (mode) {
            case 0: // OFF mode
                PWM_Disable();
                set_pwm1_raw(0);
                PIE1bits.CCP1IE = 0; // Disable CCP1 interrupt
                PIE2bits.CCP2IE = 0;
                sprintf(Buffer, "MD: OFF MODE \n");
                H=C=0;
                break;
            case 1: // Cool mode
                CCP1CON = 0x0c;        // Disable PWM
                
                
                T2CONbits.TMR2ON=1;
                T3CONbits.TMR3ON=0;
                PORTCbits.RC5=0;
                
                PIE1bits.CCP1IE = 1;
                //int R = (unsigned int)AN[1];
                set_pwm1_raw(raw_val);
                sprintf(Buffer, "MD: COOL     \n");
                C=1;
                H=0;
                break;
            case 2: // Heat mode
                PWM_Disable();
                T2CONbits.TMR2ON=0;
                T3CONbits.TMR3ON=1;
                
                PIE2bits.CCP2IE = 1;
                set_pwm1_raw(0);
                set_compare_value(raw_val * 64);
                C=0;
                H=1;
                sprintf(Buffer, "MD: HEAT     \n");
                break;
            case 3: // Auto Cool mode
                H=C=0;
                if (RT > sp) {
                    CCP1CON = 0x0c;        // Disable PWM
                    T2CONbits.TMR2ON=1;
                    T3CONbits.TMR3ON=0;
                    PORTCbits.RC5=0;
                    
                    unsigned int cool_error = RT - sp;
                    unsigned int pwm_value = (cool_error * 100) / 10;
                    if (pwm_value < 25) pwm_value = 25;
                    set_pwm1_percent(pwm_value);
                } else if (RT < (sp - hysteresis)) {
                    T2CONbits.TMR2ON=0;
                    T3CONbits.TMR3ON=1;
                    PIE2bits.CCP2IE = 1;
                    PWM_Disable();
                    set_pwm1_raw(0);
                    set_compare_value(raw_val* 64); // Force heater to 50%
                }
                sprintf(Buffer, "MD: AUTO COOL \n");
                break;
        }
    
    lcd_puts(Buffer);
    lcd_gotoxy(14, 2);
    if(H==0)
        lcd_puts("N");
    else
        lcd_puts("Y");
    lcd_gotoxy(16, 2);
    if(C==0)
        lcd_puts("N");
    else
        lcd_puts("Y");
    
    }
}