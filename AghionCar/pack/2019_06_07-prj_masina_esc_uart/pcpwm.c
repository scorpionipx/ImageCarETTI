#include <xc.h>
// CONFIG1H
//#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (clock frequency = 4 x FOSC1))
//#pragma config FCMEN = OFF       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
//#pragma config IESO = OFF        // Internal External Oscillator Switchover bit (Internal External Switchover mode enabled)
//
//// CONFIG2L
//#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
//#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
//// BORV = No Setting
//
//// CONFIG2H
//#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
//#pragma config WDPS = 32768     // Watchdog Timer Postscale Select bits (1:32768)
//#pragma config WINEN = OFF      // Watchdog Timer Window Enable bit (WDT window disabled)
//
//// CONFIG3L
//#pragma config PWMPIN = OFF     // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
//#pragma config LPOL = HIGH      // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
//#pragma config HPOL = HIGH      // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
//#pragma config T1OSCMX = ON     // Timer1 Oscillator MUX (Low-power Timer1 operation when microcontroller is in Sleep mode)
//
//// CONFIG3H
//#pragma config FLTAMX = RD4     // FLTA MUX bit (FLTA input is multiplexed with RD4)
//#pragma config SSPMX = RD1      // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RD3 and RD2, respectively. SDO output is multiplexed with RD1.)
//#pragma config PWM4MX = RB5     // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
//#pragma config EXCLKMX = RD0    // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RD0)
//#pragma config MCLRE = OFF      // MCLR Pin Enable bit (Disabled)
//
//// CONFIG4L
//#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
//#pragma config LVP = ON         // Low-Voltage ICSP Enable bit (Low-voltage ICSP enabled)
//
//// CONFIG5L
//#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
//#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFF) not code-protected)
//#pragma config CP2 = OFF        // Code Protection bit (Block 2 (002000-002FFFh) not code-protected)
//#pragma config CP3 = OFF        // Code Protection bit (Block 3 (003000-003FFFh) not code-protected)
//
//// CONFIG5H
//#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
//#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)
//
//// CONFIG6L
//#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
//#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFF) not write-protected)
//#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (002000-002FFFh) not write-protected)
//#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (003000-003FFFh) not write-protected)
//
//// CONFIG6H
//#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
//#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
//#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
//
//// CONFIG7L
//#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
//#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)
//#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (002000-002FFFh) not protected from table reads executed in other blocks)
//#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (003000-003FFFh) not protected from table reads executed in other blocks)
//
//// CONFIG7H
//#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (clock frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal External Oscillator Switchover bit (Internal External Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDPS = 32768     // Watchdog Timer Postscale Select bits (1:32768)
#pragma config WINEN = OFF      // Watchdog Timer Window Enable bit (WDT window disabled)

// CONFIG3L
#pragma config PWMPIN = OFF     // PWM output pins Reset state control (PWM outputs disabled upon Reset (default))
#pragma config LPOL = HIGH      // Low-Side Transistors Polarity (PWM0, 2, 4 and 6 are active-high)
#pragma config HPOL = HIGH      // High-Side Transistors Polarity (PWM1, 3, 5 and 7 are active-high)
#pragma config T1OSCMX = OFF    // Timer1 Oscillator MUX (Standard (legacy) Timer1 oscillator operation)

// CONFIG3H
#pragma config FLTAMX = RC1     // FLTA MUX bit (FLTA input is multiplexed with RC1)
#pragma config SSPMX = RD1      // SSP I/O MUX bit (SCK/SCL clocks and SDA/SDI data are multiplexed with RD3 and RD2, respectively. SDO output is multiplexed with RD1.)
#pragma config PWM4MX = RB5     // PWM4 MUX bit (PWM4 output is multiplexed with RB5)
#pragma config EXCLKMX = RC3    // TMR0/T5CKI External clock MUX bit (TMR0/T5CKI external clock input is multiplexed with RC3)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (Disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Low-Voltage ICSP Enable bit (Low-voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000200-000FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (001000-001FFF) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (002000-002FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (003000-003FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFF) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (002000-002FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (003000-003FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFF) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (002000-002FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (003000-003FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)

void PWMInit()
{
    PTCON0 = 0b00000000;
    PTCON1 = 0b10000000;
    PWMCON0 = 0b01110000;
    PWMCON1 = 0b00000001;
    //OVDCOND = 0B00000000;
    OVDCONS = 0b00000000;
    PTPERL = 0xFF;
    PTPERH = 0x01;
    OVDCONDbits.POVD7 = 1;
}

void DC0pcpwm(unsigned int dutycycle)
{
	PWMCON1bits.UDIS = 1;
	PDC0L = dutycycle;
	PDC0H = (dutycycle >>8) & 0x3f;
	PWMCON1bits.UDIS = 0;
}

void DC1pcpwm(unsigned int dutycycle)
{
	PWMCON1bits.UDIS = 1;
	PDC1L = dutycycle;
	PDC1H = (dutycycle >>8) & 0x3f;
	PWMCON1bits.UDIS = 0;
}
void DC2pcpwm(unsigned int dutycycle)
{
	PWMCON1bits.UDIS = 1;
	PDC2L = dutycycle;
	PDC2H = (dutycycle >>8) & 0x3f;
	PWMCON1bits.UDIS = 0;
}

void DC3pcpwm(unsigned int dutycycle)
{
	PWMCON1bits.UDIS = 1;
	PDC3L = dutycycle;
	PDC3H = (dutycycle >>8) & 0x3f;
	PWMCON1bits.UDIS = 0;
}

