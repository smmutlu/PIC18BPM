#include <stdlib.h>
#include <xc.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#pragma config OSC = INTIO1              //Oscillator (HS Oscillator)
#pragma config FCMEN = OFF           // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF            // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

#pragma config PWRT = OFF            //Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF              //Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3              //Brown-out Reset Voltage bits (Minimum Setting)

#pragma config WDT = OFF             //Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config  WDTPS = 1         // Watchdog Timer Postscale Select bits (1:32768)

#pragma config CCP2MX = RC1          //CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ANA          // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF         // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON            //MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

#pragma config STVREN = ON           //Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF             //Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = BB256         //Boot Block Size Select bits ( 256 Word)
#pragma config  XINST = OFF           // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

#pragma config CP0 = OFF             //Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF             //Code Protection bit (Block 1 not code-protected)

#pragma config   CPB = OFF             // Boot Block Code Protection bitProtect Boot (Boot block not code-protected)
#pragma config   CPD = OFF             //Data EEPROM Code Protection bit (Data EEPROM not code-protected)

#pragma config WRT0 = OFF            //Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF            //Write Protection bit (Block 1 not write-protected)


#pragma config WRTC = OFF            // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config  WRTB= OFF            // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config   WRTD = OFF            // Data EEPROM Write Protection bit (Data EEPROM not write-protected)


#pragma config EBTR0 = OFF         // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF           // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

#pragma config  EBTRB = OFF           // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)

#define _XTAL_FREQ 8000000          // Internal Oscillator Frequency macro for delay functions

void UART_Open(const long int baudrate);
void UART_Close(void);
char UART_TX_Idle(void);
char UART_TX_Busy(void);
char UART_RX_Ready(void);
void UART_Write(char info[]);
void UART_Read(char info[]);
int adcReadChannel(char channelNumber);
void adcConfig();
void timerConfig ();
void printData (float data);
void printNewLine(void);
void appendUnits(void);
void __interrupt(high_priority) YourHighPriorityISR(void);

#define sysThreshold 0x30C
#define bpmThreshold 0x200
#define diasThreshold 0x2B2
#define offset 0x14F
#define acPin 0
#define dcPin 1

#define numSamplesBPM   8
#define numSamplesSYS   8
#define numSamplesDIAS  8

float prevMeasurement = 0;
float systolicPressure = 0,heartFloat=0,diastolicPressure=0 ,MAP = 0;
int adcData = 0,adcPrev = 1000,avgData= 0,maxData=0;
int pressure=0;
uint16_t timerInt =0;
int i = 0;
int state = 0;
int sysMeasure = 0;
int count=0; 
int avgCount = 0;
uint32_t timerComp = 0; 
uint32_t bpm = 0; 
uint32_t sysComp = 0; 
//********* Strings ********//
unsigned char sysDisplay[] = "Systolic Pressure: ";
unsigned char measuringDisplay[] = "Measuring...";
unsigned char bpmDisplay[] = "Heart Beats per Minute: ";
unsigned char diasDisplay[] = "Diastolic Pressure: ";
//********* PTRS *******//
unsigned char *diasPointer = diasDisplay; 
unsigned char *sysPointer = sysDisplay; 
unsigned char *bpmPointer = bpmDisplay;
unsigned char *measurePointer = measuringDisplay; 

void main() {

     //OSCCON: OSCILLATOR CONTROL REGISTER
     OSCCONbits.IRCF0 = 1; //8 MHz (INTOSC drives clock directly)
     OSCCONbits.IRCF1 = 1; //8 MHz (INTOSC drives clock directly)
     OSCCONbits.IRCF2 = 1; //8 MHz (INTOSC drives clock directly)
     OSCCONbits.SCS0 = 1; //Internal oscillator block
     OSCCONbits.SCS1 = 1; //Internal oscillator block

     //RD2 and RD3 CONFIGURATION (for multiplexing UART PIC output)
     TRISCbits.RC6 = 1; //Set RC6  as output
     TRISCbits.RC7 = 1; //Set RC7 as output
     LATCbits.LATC6 = 1; //Set RC6 as low
     LATCbits.LATC7 = 1; //Set RC7 as low
     TRISBbits.RB0 = 0;
     //RA0 I/O CONFIGURATION  
     TRISAbits.RA0 = 0; //Set RD2 as output
     LATAbits.LATA0 = 0; //Set RD2 as low

     //MAIN ROUTINE
      UART_Close();
     UART_Open(9600);

     adcConfig();
     timerConfig();
 
        while (1) {
         if(state == 0)
         {
            UART_Write(measurePointer);
            printNewLine();            
            state =1;
         }
         if(state == 1)
         {
         adcData = adcReadChannel(acPin);
         if (maxData> adcData)
         {
              sysMeasure++; 
              maxData = 0;
              sysComp += (unsigned)adcReadChannel(dcPin);
         }   
         else if(adcData>adcPrev)
         {
             maxData = adcData;
         }
         adcPrev = adcData; 
         if (sysMeasure >= numSamplesSYS )
         {
             state = 2;
             MAP = (sysComp/numSamplesSYS);
             
         }    
         __delay_ms(40);
         
         }
         if (state == 2)
         {
                systolicPressure = (float)(MAP-offset)*0.25925; //(((float)(avgData)/4)-offset)*0.305;
                UART_Write(sysPointer);
                printData(systolicPressure);
                appendUnits();
                printNewLine();
                adcPrev = 1000;
                count = 0;
                state = 3;
             
         }
         if (state ==3)
         {


            /*** HEART RATE MEASURE*/
            while(avgCount < numSamplesBPM)
            {
                adcData = adcReadChannel(acPin);
                if (adcPrev < bpmThreshold && adcData > bpmThreshold && count ==0)
                {
                    count=1;        //step into timed BPM algorithm
                    TMR0H = 0x00;       //reset timer
                    TMR0L = 0x00;       
                    T0CONbits.TMR0ON = 1;       //start Timer0
                }   
                if (adcPrev < bpmThreshold && adcData > bpmThreshold && count ==1)
                {
                    T0CONbits.TMR0ON = 0;       //once peak is found momentarily stop timer0
                    timerInt = (TMR0H*256)+TMR0L;       //compute 16-bit timer count
                    timerComp = timerInt + timerComp;     //'long' compounded count
                    TMR0H = 0x00;       //restart timer0
                    TMR0L = 0x00;
                    avgCount++;     //increment # of readings for average BPM computation
                    if(avgCount<numSamplesBPM)
                    {
                    T0CONbits.TMR0ON = 1; //start for next measurement or if finished do nothing
                    }
                }   
                    
                adcPrev = adcData; 
            }
            bpm = ((timerComp*0.000032)/numSamplesBPM)*60; //timer 0 w/prescaler period = 32us macro for number of samples in agolrithm 
            
            UART_Write(bpmPointer);
            heartFloat = (float)bpm; //cast to float for 
            printData(heartFloat);
            printNewLine();
            state = 4;
         }
         if(state ==4)
         {
                __delay_ms(2593);
                diastolicPressure = ((float)MAP-offset)*0.16775;
                UART_Write(diasPointer); 
                printData(diastolicPressure);
                appendUnits();
                printNewLine();                
             UART_Write("Finish!\r\n");
             UART_Write("Press 1 to start over!\r\n");
             state =5;
         }
         if(state ==5)
         {
             sysMeasure = 0;
             maxData = 0;
             sysComp = 0;
             MAP = 0;
            count = 0;
            avgCount = 0;
            timerComp = 0;
            state = 6;
         }
       
    }
}
void appendUnits(void)
{
    while (UART_TX_Busy());
    unsigned char data2[] = "mmHg";
    for (i = 0; i<strlen(data2);i++)
    {
        TXREG = data2[i]; 
        while (UART_TX_Busy());
    }    
}
void printNewLine (void)
{
        while (UART_TX_Busy());
        TXREG = 0x0A;
        while (UART_TX_Busy());
        TXREG = 0x0D;
}
void printData (float data)
{
    int sender = 0;
        if (data>100 && data<200)
        {
        sender = ((int)data/100);    
        while (UART_TX_Busy());
        TXREG = (int)sender + 0x30;
        sender = (int)(((int)data -100)/10);
        while (UART_TX_Busy());
        TXREG =(int)sender + 0x30;
        sender = ((int)data - 100 -(sender*10));
        TXREG = (int)sender + 0x30;
        }
        else if(data<100 && data>0)
        {
            sender = ((int)data/10);
            while (UART_TX_Busy());
            TXREG =(int)sender + 0x30;
            sender = (int)data-(sender*10);
            while (UART_TX_Busy());
            TXREG = (int)sender + 0x30;
        }

}
int adcReadChannel(char channelNumber)
{
      //Set the ADC to sample desired channel
  ADCON0bits.CHS3 =(channelNumber>>3)&0x01;
  ADCON0bits.CHS2 =(channelNumber>>2)&0x01;
  ADCON0bits.CHS1 = (channelNumber>>1)&0x01;
  ADCON0bits.CHS0 = channelNumber&0x01;

  //Start conversion
  ADCON0bits.ADON = 1;
  ADCON0bits.GO = 1;
  _delay(50000);
  while (ADCON0bits.GO)
     ;
  return ADRES;
  
}
void adcConfig()
{
      //Make PORTA0 input for A/D converter
  TRISAbits.TRISA0 = 1;
  TRISAbits.TRISA1 = 1;
  TRISAbits.TRISA2 = 1;
  TRISAbits.TRISA3 = 1;

  //Set the ADC for internal RC clock
  ADCON2bits.ADCS2 = 1;
  ADCON2bits.ADCS1 = 1;
  ADCON2bits.ADCS0 = 1;

  ADCON0 = 0;

  //Set the ADC to use VDD and GND as references and AN0 - AN4 as A/D inputs
  ADCON1 = 0b00001010;
  ADCON2bits.ADFM = 1;
 
}
void timerConfig ()
{
        T0CON = 0; //Sets time to 16 bit mode and transistion on internal clock
	T0CONbits.PSA = 1; //Ignore prescaler for now
	TMR0H = 0x00;
	TMR0L = 0x00;
	//Enable Timer 0 interrupt
    INTCONbits.TMR0IE =1;                                      
	INTCONbits.TMR0IF = 0; //Clear Interrupt Flag
	//INTCONbits.TMR0IE = 1; //Enable Interrupt
	RCONbits.IPEN = 1;            //enable priority levels
	T0CONbits.T08BIT = 0; // set as 8 bit (0 means 16 bit)
	T0CONbits.T0CS = 0;   //Use the system clock
	//T0CON.T0SE = 0;   //Don't Care
	T0CONbits.PSA = 0;   //No Pre scaler
	T0CONbits.T0PS2 = 0;
	T0CONbits.T0PS1 = 1;
	T0CONbits.T0PS0 = 0;
	//T0CON.T0PS = 0b001;   //Don't Care (pre scaler)
//	INTCONbits.GIEH = 1;          //enable interrupts
//	T0CONbits.TMR0ON = 1; //Turn on timer

}
void __interrupt(high_priority) YourHighPriorityISR(void)
{
     if (TMR0IE && TMR0IF)
    {
    TMR0 -= 250;                    // Reload the timer - 250uS per interrupt
        TMR0IF=0;
   //     ++tick_count;
       // TRISC=0;
    //    if (tick_count%10000)
       // LATCbits.LATC4 = 0x01;
     }   
    if (PIR1bits.RCIF )
    {
        PIR1bits.RCIF = 0;
         if(RCREG == 0x31)
        state = 0;
     }
    
}
void UART_Open(const long int baudrate) {

            BAUDCONbits.BRG16 = 0;
          SPBRG = 12; //Writing SPBRG Register
          TXSTAbits.SYNC = 0; //Setting Asynchronous Mode, ie UART
        TRISCbits.RC7 = 1; //As Prescribed in Datasheet
          TRISCbits.RC6 = 1; //As Prescribed in Datasheet
          RCSTAbits.CREN = 1; //Enables Continuous Reception
          TXSTAbits.TXEN = 1; //Enables Transmission
                    RCSTAbits.SPEN = 1; //Enables Serial Port

        // TXIE = 0; // Enable tx interrupts 
         PIE1bits.RCIE = 1; // Enable rx interrupts
         INTCONbits.GIE = 1;
         INTCONbits.PEIE = 1;
}

void UART_Close(void) {
     RCSTAbits.SPEN = 0;
}

char UART_TX_Idle(void) {
     return (char) TXSTAbits.TRMT;
}

char UART_TX_Busy(void) {
     return (char) !TXSTAbits.TRMT;
}

char UART_RX_Ready(void) {
     return (char) RCIF;
}

void UART_Write(char info[]) {
     int i;
     for (i = 0; i < strlen(info); i++) {
          TXREG = info[i];
          while (UART_TX_Busy());
     }
}

void UART_Read(char info[]) {
     unsigned int i = 0;
     while (1) {
          while (!UART_RX_Ready());
          info[i] = RCREG;
          if (info[i] != '\r') i++;
          else {
               info[i+1]='\0';
               break;
          }
     }
}