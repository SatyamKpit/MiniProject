

/**************************************************************
* File Name     : Body control module
* Course        : Embedded C
* Created Date  : 24/9/2018
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
#define SW1 PD2
#define LED1 PB5    // engine on

#define motorPin1 PB4
#define motorPin2 PB3

//Satyam
#define HighBeam PD0
#define LowBeam PD1
#define Switch PD5

//Lavanya
#define LockSwitch PB2
#define door_motorPin1 PD4
#define door_motorPin2 PD3

//Aarthi
#define LeftLedBlue PB1
#define LeftLedGreen PB0
#define RightLedGreen PD6
#define RightLedBlue PD7

/**************************************************************
    				Global Variables And Bit Fields
***************************************************************/

struct {
  volatile unsigned int Timer:1;
  volatile unsigned int STATUS_ONE:1;
  volatile unsigned int windowStatus:1;
  volatile unsigned int StartFLAG:1;
  volatile unsigned int AssistanceSwitch:1;

  volatile unsigned int timer:1;


  volatile unsigned int DoorLock:1;

  volatile unsigned int c:1;
}FLAG;

volatile uint16_t sun = 0, lights = 0, door=0, value=0, adapt=0;
volatile uint16_t count=0x00;
volatile uint16_t count_Timer = 0x00;

void PIN_func()
{
  SET_BIT(DDRD,HighBeam);//Set PD0 as Output
  SET_BIT(DDRD,LowBeam);//Set PD1 as Output
  CLR_BIT(DDRD,Switch);//Set PD5 as Input
  CLR_BIT(PORTD,HighBeam);//Output Value 0 in beginning
  CLR_BIT(PORTD,LowBeam);//Output Value 0 in beginning

  CLR_BIT(DDRD,SW1);			// SW1 as input
  SET_BIT(DDRB,LED1);         // LED1 as output
  CLR_BIT(PORTB,LED1);        // Initially LED1 is OFF

  //Lavanya
    CLR_BIT(DDRB,LockSwitch);  //Digital Pin 10
  	SET_BIT(DDRD,door_motorPin1);  //Digital Pin 4
    SET_BIT(DDRD,door_motorPin2);  //Digital Pin 3

  //Aarthi
    SET_BIT(DDRD,RightLedGreen);  //DIGITAL PIN 6
    CLR_BIT(PORTD,RightLedGreen);

    SET_BIT(DDRD,RightLedBlue); //DIGITAL PIN 7
    CLR_BIT(PORTD,RightLedBlue);

    SET_BIT(DDRB,LeftLedGreen);  //DIGITAL PIN 8
    CLR_BIT(PORTB,LeftLedGreen);

    SET_BIT(DDRB,LeftLedBlue); //DIGITAL PIN 9
    CLR_BIT(PORTB,LeftLedBlue);

}

uint16_t ADC_READ(uint8_t analog)
{
  ADMUX = analog; // ADC 3
  ADMUX |= (1<<REFS0);// Vref = 5V
  ADCSRA |= (1<<ADEN);//ADC Enable
  ADCSRA |= (1<<ADSC);//Single Conversion
  while(ADCSRA&(1<<ADSC));
  return(ADC);
}

void Interrupt_func()
{
  //Interrupt 1 as rising edge triggesred
  EICRA|=(1<<ISC10);
  EICRA|=(1<<ISC11);
  EIMSK|=(1<<INT1);			//Local INT enable
  //Interrupt 0 as rising edge triggesred
  EICRA|=(1<<ISC00);
  EICRA|=(1<<ISC01);
  EIMSK|=(1<<INT0);			//Local INT enable
  //Setting Switch 2 as PCINT21
  PCMSK2 |= (1<<PCINT21);	//Enabling Local Interrupt
  PCICR |= (1 <<PCIE2);		//Pin change interrupt for pin group [23:16]

}
void PinchangeL()
{
    //PCINT2 Interrupt
    PCICR |= (1<<PCIE0);   //Pin change interrupt control register
    PCMSK0 |= (1<<PCINT2); //PCINT2 interrupt
}

void TIMER_func()
{
  TCCR0A = 0X00; //NORMAL MODE
  TCCR0B |= (1<<CS02 ) | (1<<CS00)  ; //PRESCALAR = 1024
  TCCR0B &= ~(1<<CS01);
  TIMSK0 |= (1<<TOIE0); //LOCAL INTERRUPT ENABLE
  TCNT0 = 0x00; // INITIALIZE TIMER COUNT TO 0
}

void DelayFunction()
{
  TIMER_func();
  while(FLAG.Timer == 0)
  {
    //Waste time
  }
  FLAG.Timer=0;
}

void StartIndication()
{
  SET_BIT(PORTD,HighBeam);
  SET_BIT(PORTD,LowBeam);
  DelayFunction();
  CLR_BIT(PORTD,HighBeam);
  CLR_BIT(PORTD,LowBeam);
  DelayFunction();
  SET_BIT(PORTD,HighBeam);
  SET_BIT(PORTD,LowBeam);
  DelayFunction();
  CLR_BIT(PORTD,HighBeam);
  CLR_BIT(PORTD,LowBeam);
  DelayFunction();
  SET_BIT(PORTD,HighBeam);
  SET_BIT(PORTD,LowBeam);
  DelayFunction();
  CLR_BIT(PORTD,HighBeam);
  CLR_BIT(PORTD,LowBeam);
}

void MIL()
{
  SET_BIT(PORTD,HighBeam);
  SET_BIT(PORTD,LowBeam);
  DelayFunction();
  CLR_BIT(PORTD,HighBeam);
  CLR_BIT(PORTD,LowBeam);
  DelayFunction();
  SET_BIT(PORTD,HighBeam);
  SET_BIT(PORTD,LowBeam);
  DelayFunction();
  CLR_BIT(PORTD,HighBeam);
  CLR_BIT(PORTD,LowBeam);
  DelayFunction();
}


/***************************************************************
					Internal Service Routines
****************************************************************/

/***************************************************************
* Name:        SW1 interrupt
* Description: It switches on the system
****************************************************************/
/*ISR(INT0_vect)
{
   	cli();
    PORTB ^=(1<<LED1);
    FLAG.STATUS_ONE=~FLAG.STATUS_ONE;
    sei();
}*/


/***************************************************************
* Name:        Timer0 Overflow interrupt
* Description: Does the initial LED blinking when Assistance switch is turned on
* Arguments:
****************************************************************/
//ISR for Assistance switch On/Off
ISR(PCINT2_vect)
{
  FLAG.AssistanceSwitch = ~FLAG.AssistanceSwitch;
}

//Interrupt vector
ISR(PCINT0_vect)
{
    FLAG.DoorLock =! FLAG.DoorLock;
}


ISR(INT0_vect)
{
   cli();
    PORTB ^=(1<<PB5);
    FLAG.STATUS_ONE=~FLAG.STATUS_ONE;
   sei();

}

ISR(TIMER0_OVF_vect	)
{
  cli();
  count_Timer++;
  count++;
  if(count_Timer==10)
  {
    FLAG.Timer = 1;
    count_Timer=0;
    FLAG.timer=1;
    count=0;
  }
  sei();
}

void adaptive()
{

}
void visor1()
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

void visor2()
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

void powerdoor()
{
  if(FLAG.DoorLock == 1) //Interrupts when switch is pressed
        {
         door =  ADC_READ(0x42);
          //ADCSRA&= 0x00;


          if(door>0)  //Motor rotates clockwise
          {
             PORTD |= (1<<PD4);
             PORTD &= ~(1<<PD3);
          }
          else if(door==0) //Motor rotates anti-clockwise and stops
          {
             PORTD &= ~(1<<PD4);
             PORTD &= ~(1<<PD3);
          }

         }

      else //Motor doesnot rotate
      {
        PORTD &= ~(1<<PD4);
        PORTD &= ~(1<<PD3);
      }
}
void TurnLight()
{
  if(value>0 && value<250)  //LEFT LED BLINKING BLUE
    {
     SET_BIT(PORTB,PB1);
     _delay_ms(500);
     CLR_BIT(PORTB,PB1);
     _delay_ms(500);
    }

   if(value>250 && value<=400)  //LEFT LED BLINKING GREEN
    {
     SET_BIT(PORTB,PB0);
     _delay_ms(500);
     CLR_BIT(PORTB,PB0);
     _delay_ms(500);
    }

    if(value>=600 && value<=800)  //RIGHT LED BLINKING GREEN
    {
     SET_BIT(PORTD,PD6);
     _delay_ms(500);
     CLR_BIT(PORTD,PD6);
     _delay_ms(500);

    }
    if(value<=1023 && value>800)  //RIGHT LED BLINKING BLUE
    {
     SET_BIT(PORTD,PD7);
     _delay_ms(500);
     CLR_BIT(PORTD,PD7);
     _delay_ms(500);
    }
}



/*****************************************************************
                            Main Program
*****************************************************************/
int main()
{

  PIN_func();
  Interrupt_func();
  TIMER_func();
  PinchangeL();
  sei();

  while(1)
  {
    adapt = ADC_READ(0x43);
      if (FLAG.AssistanceSwitch == 0)
  {
    CLR_BIT(PORTD,HighBeam);
    CLR_BIT(PORTD,LowBeam);
    FLAG.StartFLAG=0;
  }
  else
  {
    if(FLAG.StartFLAG==0)
    {
      StartIndication();
      FLAG.StartFLAG=1;
    }
    else
    {
      if (adapt > 50) //Go into Low Beam mode
      {
        CLR_BIT(PORTD,HighBeam);
        SET_BIT(PORTD,LowBeam);
      }
      else  //Go into High Beam mode
      {
        SET_BIT(PORTD,HighBeam);
        CLR_BIT(PORTD,LowBeam);
      }
      if (HighBeam==0 && LowBeam==0)
      {
        MIL();
      }
    }
  }
    if(FLAG.STATUS_ONE==1)
       {

         sun = ADC_READ(0x40);
         //ADCSRA&= 0x00;
         value=ADC_READ(0x41);
/*****************************************************************
                            Sun visor Program
*****************************************************************/
       if(sun >= 980) //50
        {
          visor1();
        }
       else
        {
          visor2();
        }
 /**************************/
        TurnLight();
      }
      else
      {
        PORTB |=(1<<PB4);
        PORTB |= (1<<PB3);
        CLR_BIT(PORTD,RightLedGreen);
        CLR_BIT(PORTD,RightLedBlue);
        CLR_BIT(PORTB,LeftLedGreen);
        CLR_BIT(PORTB,LeftLedBlue);
      }


/*****************************************************************
                            Automatic sliding
*****************************************************************/

      powerdoor();
  }
}
