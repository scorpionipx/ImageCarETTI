#include <xc.h>

#define Idle 0
#define Manual 1

#define Start 1
#define EmergencyStop 2

unsigned char posMS = 1, speed1 = 200, receive, direction = 0, mode = Idle, status = EmergencyStop;
unsigned int pwm, j = 500, power=350, bin;
unsigned int count_step, speed_desire, k=0, speed, spi_delay;
unsigned char rec_uart[5], spi_cnt, rec_uart1, delete, adc_var=0, read_adc_com;

void Timer0Init();
void PortInit();
void InterruptInit();
//void interrupt isr();
//void init_SPI(void);
void init_UART(void);
void USARTWriteByte(char ch);
void USARTWriteString(const char *str);
void PWMInit();
void DC0pcpwm(unsigned int dutycycle);
void DC1pcpwm(unsigned int dutycycle);
void DC2pcpwm(unsigned int dutycycle);
void DC3pcpwm(unsigned int dutycycle);
void init_SPI(void);
void ADCInit();
unsigned int ADCRead1(); 


//void  __interrupt(low_priority) myLowIsr(void)
//void  __interrupt(high_priority) myHighIsr(void)
//void __interrupt () my_isr_routine (void)
//void interrupt isr()
void  __interrupt(low_priority) myLowIsr(void)
{
//    spi_delay++;
    if (PIR1bits.RCIF == 1) // enable USART rx interrupt)        // 
    {   
        if(spi_delay > 100) {spi_cnt=0;  rec_uart[0] = 0; rec_uart[1] = 0;}// each 10ms
        if(spi_cnt==0) {rec_uart[0] = RCREG; spi_delay=0;}
        if(spi_cnt==1) {rec_uart[1] = RCREG; spi_delay=0;}
        spi_cnt++;
        if(rec_uart[0] == 0x5A) {read_adc_com = 1;}        
        if(rec_uart[0] == 0x39) {speed_desire = rec_uart[1]; direction = 1; }
        if(rec_uart[0] == 0x35) {speed_desire = rec_uart[1]; direction = 2; PORTCbits.RC1 = ~PORTCbits.RC1;}  
        if(rec_uart[0] == 0x20) {    power = rec_uart[1];
                                     power <<= 2;      
                                }
        
        delete = RCREG;
        rec_uart1=1;
        PIR1bits.RCIF = 0;      
//        PORTCbits.RC1 = ~PORTCbits.RC1;
    }
}

void  __interrupt(high_priority) myHighIsr(void)
{
    spi_delay++;
    if (TMR0IF == 1)        // 100us
    {
//        PORTCbits.RC0 = ~PORTCbits.RC0;
        PORTCbits.RC0 = 1;

        count_step++;
        if(count_step>100)   // each ms
        {
            count_step=0;
            if(speed > speed_desire)
                speed--;
            else speed++;
        }

       k++;
        if(k >= speed)
        {
            switch(posMS)
            {
                case 1: PORTB = 0x00; OVDCOND = 0b10001000; PORTB = 0b00000001;
                if(direction == 1)
                    posMS = 2;
                else if(direction == 2)
                    posMS = 6;
                break;

                case 2: PORTB = 0x00; OVDCOND = 0b10001000; PORTB = 0b00100000;
                if(direction == 1)
                    posMS = 3;
                else if(direction == 2)
                    posMS = 1;
                break;

                case 3: PORTB = 0x00; OVDCOND = 0b10000010; PORTB = 0b00100000;
                if(direction == 1)
                    posMS = 4;
                else if(direction == 2)
                    posMS = 2;
                break;

                case 4: PORTB = 0x00; OVDCOND = 0b10000010; PORTB = 0b00000100;
                if(direction == 1)    
                    posMS = 5;
                else if(direction == 2)
                    posMS = 3;
                break;

                case 5: PORTB = 0x00; OVDCOND = 0b10100000; PORTB = 0b00000100;
                if(direction == 1)
                    posMS = 6;
                else if(direction == 2)
                    posMS = 4;
                break;

                case 6: PORTB = 0x00; OVDCOND = 0b10100000; PORTB = 0b00000001;
                if(direction == 1)
                    posMS = 1;
                else if(direction == 2)
                    posMS = 5;
                break;
            }
            k=0;        
        }

        TMR0IF = 0;
        TMR0IE = 1;
        PORTCbits.RC0 = 0;
    }
}

void Timer0Init()
{
  T0CON = 0b11000001;
  TMR0 = 0x00;
  TMR0IE = 1;
  TMR0IF = 0;
}

void PortInit()
{
    TRISA = 0b11111111;
    TRISB = 0b10000000;
    TRISC = 0b10111000;
    TRISD = 0b00001100;
    ADCON0=0;			
    ADCON1=0x0F;	    
}

void InterruptInit()
{
    TMR0IE = 1;
    TMR0IF = 0;
    TMR0IP = 1;   
    
    PIR1bits.RCIF = 0; //reset RX pin flag
    IPR1bits.RCIP = 0; //high priority
    PIE1bits.RCIE = 1; //Enable RX interrupt
    INTCONbits.PEIE = 0; //Enable pheripheral interrupt (serial port is a pheripheral)
    RCONbits.IPEN = 1;  // for LOW and HIGH interrupt levels !!!
    GIEL = 1;
    GIEH = 1;
}

void init_UART(void)    // 9600bps @ 40MHz
{
    SPBRG=86;
    TXEN=1;
    BRGH=1;
    SPEN=1;
    CREN=1;
    BRG16=1;
    SYNC=0;    
}

void USARTWriteByte(char ch)
{
    while(!TXIF);
    TXREG=ch;
}

void USARTWriteString(const char *str)
{
    while((*str)!='\0')
    {
	while(!TXIF);
	TXREG=(*str);
	str++;
    }
}

void ADCInit()
{
    ADCON0 = 0b00000001;
    ADCON1 = 0b00000000;
    ADCON2 = 0b00010001;    // 4*Tad sample,  speed: Fosc/8
    ANSEL0 = 0b11111111;
    ADCHS =  0b00000000;    // Grup A, select AN0 for conversion
//    ADIE = 1;
    ADIF = 0;
    GODONE = 1;
}

unsigned int ADCRead1()
{
    ADCON0 = 0b00000001;
    ADON=1;
    GODONE=1;
    while(GODONE);
    ADON=0;
    return ADRESH;
}

void main()
{
    PortInit();
    Timer0Init();
    PWMInit();
    init_UART();    
    InterruptInit();
    ADCInit();
    
    mode = Manual;
    status = Start;
    speed = 0;
    speed_desire = 0;
//    speed_desire = 250;    // for test purpose; delete after that!
    direction = 2;  
//    power = 350;
//    DC0pcpwm(power);
//    DC1pcpwm(power);
//    DC2pcpwm(power);    
    
    while(1)
    {
        while(status == EmergencyStop)
        {
            DC0pcpwm(0); DC1pcpwm(0); DC2pcpwm(0); DC3pcpwm(0);
        }

        while(mode == Manual)
        {
           if(speed_desire==0) 
           {
                DC0pcpwm(0);
                DC1pcpwm(0);
                DC2pcpwm(0);            
           }
           else 
               {
                    DC0pcpwm(power);
                    DC1pcpwm(power);
                    DC2pcpwm(power);   
//                    for(bin=0; bin<50000; bin++);                    
                    adc_var = ADCRead1();
//                    USARTWriteByte(adc_var);
//                    USARTWriteByte(speed_desire);
                    if(read_adc_com==1) {read_adc_com=0; USARTWriteByte(adc_var);}
                                      
                }
        }
    }	
}