/**************************************************************
* File Name:    Light Controlled Sun Visor
* Description:  Automated sun Visor
* Author:  	    Resu Nikhil Reddy
* Employee ID:  137997
* Course:       Embedded C
* TinkerCADLink:
* Created Date: 14/9/2018
***************************************************************/

/**************************************************************
                            Header Files
***************************************************************/
#include<stdint.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
/**************************************************************
                      Preprocessor Definitions
***************************************************************/
#define SET_BIT(PORT,BIT) PORT|= (1<<BIT)
#define CLR_BIT(PORT,BIT) PORT&= ~(1<<BIT)
#define SW1 PD1
#define LED1 PB5    // engine on

#define motorPin1 PB4
#define motorPin2 PB3

/**************************************************************
    				Global Variables And Bit Fields
***************************************************************/
struct
{
    volatile unsigned int STATUS_ONE:1;
    volatile unsigned int windowStatus:1;
    volatile unsigned int timer:1;
}FLAG;

volatile uint16_t value;
volatile uint16_t count;


void PIN_func()
{
   /*..............PIN DECLARATION AND INITIALIZATION.............*/
  	CLR_BIT(DDRD,SW1);			// SW1 as input
  	SET_BIT(DDRB,LED1);         // LED1 as output
  	CLR_BIT(PORTB,LED1);        // Initially LED1 is OFF
}

int ADC_READ(void)
{
    ADMUX|=(1<<REFS0);		   //AVCC WithExternalCapacitorAtAREF
    ADCSRA|=(1<<ADEN);		   //ADC enable
    ADCSRA|=(1<<ADSC);		   //Start Conversion
    while(!(ADCSRA&(1<<ADIF)))
    value=ADC;
}


void Interrupt_func() // External Intrrupt of Ignition
{
/*................INTERRUPT CONFIGURATION SECTION..............*/
  	//Interrupt 1 as rising edge triggesred
    SREG  |= (1<<7);
 	EICRA|=(1<<ISC10);
    EICRA|=(1<<ISC11);
 	EIMSK|=(1<<INT1);			//Local INT enable
 	//Interrupt 0 as rising edge triggesred
  	EICRA|=(1<<ISC00);
    EICRA|=(1<<ISC01);
    EIMSK|=(1<<INT0);			//Local INT enable
}

void TIMER_func()
{
  /*.......................TIMER SECTION......................*/
  	TCCR0A|=0x00;				//timer0 in normal mode of operation
    TCCR0B|=((1<<CS00)|(1<<CS02));//clkI/O/1024 prescalar selection
  	TCCR0B&=~(1<<CS01);
  	TCNT0=0x00;					//start counting from zero
}



 /**************************************************************
* Name: TIMER_ON
***************************************************************/

void TIMER_ON()
{
    TIMSK0|=(1<< TOIE0);//Local Timer Overflow INT enable
}

/**************************************************************
* Name: Timer Disable
***************************************************************/

void TIMER_OFF(void)
{
    TIMSK0=0x00;
}

/***************************************************************
					Internal Service Routines
****************************************************************/
/***************************************************************
* Name:        SW1 interrupt
* Description: It switches on the system
****************************************************************/


ISR(INT0_vect)
{
   	cli();
    PORTB ^=(1<<PB5);
    FLAG.STATUS_ONE=~FLAG.STATUS_ONE;
    sei();
}


/***************************************************************
* Name:        Timer0 Overflow interrupt
* Description:
* Arguments:
****************************************************************/


ISR(TIMER0_OVF_vect)
{
    cli();
    count++;
    if(count==5)
    {
    	count=0;
        FLAG.timer=1;
    }
    sei();
}

/*****************************************************************
                            Main Program
*****************************************************************/
int main()
{
    Serial.begin(9600);
    PIN_func();
    Interrupt_func();
    TIMER_func();
  	TIMER_ON();
   	sei();// Global interrupt
/*............................SUPER LOOP........................*/
    while(1)

    {
      if(FLAG.STATUS_ONE==1)
      {

        ADC_READ();
        Serial.println(value);
        if(value >= 980) //50
        {
                if(FLAG.windowStatus == 1) //already close,do nothing
                {
                 PORTB |=(1<<PB4);
                 PORTB |= (1<<PB3);
                }
                else
                {
                  count=0;
                  FLAG.timer=0;
                  while(FLAG.timer==0)
                  {
                   PORTB &= ~(1<<PB4); // Anti, close sun visor
                   PORTB |= (1<<PB3);
                  }

                 FLAG.windowStatus = 1;
                }
         }
       else
        {
               if(FLAG.windowStatus == 0) //already open,do nothing
                {
                 PORTB |=(1<<PB4);
                 PORTB |= (1<<PB3);
                }
               else
                {
                 count=0;
                 FLAG.timer=0;
                 while(FLAG.timer==0)
                 {
                 PORTB |= (1<<PB4);  // clockwise,open sun visor
                 PORTB &= ~(1<<PB3);
                 }

                 FLAG.windowStatus = 0;
                }
         }
    }
}

}
