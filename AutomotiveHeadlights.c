#include<avr/io.h>
#include<avr/interrupt.h>
#include<stdint.h>
#define SET(PORT,PIN) PORT |= (1<<PIN);
#define CLR(PORT,PIN) PORT &= ~(1<<PIN);
#define HighBeam PD0
#define LowBeam PD1
#define Switch PD5

/*Switch-PD5-PCINT21
  HighBeam-PD0
  LowBeam-PD1*/

struct FLAG{
  volatile unsigned int Timer:1;
  volatile unsigned int StartFLAG:1;
  volatile unsigned int AssistanceSwitch:1;
}FLAGbit;


int count=0x00;

void InitializeTimer()
{
  TCCR0A = 0X00; //NORMAL MODE
  TCCR0B |= (1<<CS02 ) | (1<<CS00)  ; //PRESCALAR = 1024
  TCCR0B &= ~(1<<CS01);
  TIMSK0 |= (1<<TOIE0); //LOCAL INTERRUPT ENABLE
  TCNT0 = 0x00; // INITIALIZE TIMER COUNT TO 0
}

void InitializeSwitch()
{
  //Setting Switch 2 as PCINT19
  PCMSK2 |= (1<<PCINT21);	//Enabling Local Interrupt
  PCICR |= (1 <<PCIE2);		//Pin change interrupt for pin group [23:16]

}

void InitializeADC()
{
  ADMUX |= (1<<MUX0) |(1<<MUX1); // ADC 3
  PRR &= ~(1<<PRADC);//Power Reduction off
  ADMUX |= (1<<REFS0);// Vref = 5V
  ADCSRA |= (1<<ADEN);//ADC Enable
}

void InputOutput()
{
  SET(DDRD,HighBeam);//Set PD0 as Output
  SET(DDRD,LowBeam);//Set PD1 as Output
  CLR(DDRD,Switch);//Set PD5 as Input
  CLR(PORTD,HighBeam);//Output Value 0 in beginning
  CLR(PORTD,LowBeam);//Output Value 0 in beginning
}

void DelayFunction()
{
  InitializeTimer();
  while(FLAGbit.Timer == 0)
  {
    //Waste time
  }
  FLAGbit.Timer=0;
}

void StartIndication()
{
  SET(PORTD,HighBeam);
  SET(PORTD,LowBeam);
  DelayFunction();
  CLR(PORTD,HighBeam);
  CLR(PORTD,LowBeam);
  DelayFunction();
  SET(PORTD,HighBeam);
  SET(PORTD,LowBeam);
  DelayFunction();
  CLR(PORTD,HighBeam);
  CLR(PORTD,LowBeam);
  DelayFunction();
  SET(PORTD,HighBeam);
  SET(PORTD,LowBeam);
  DelayFunction();
  CLR(PORTD,HighBeam);
  CLR(PORTD,LowBeam);
}

void MIL()
{
  SET(PORTD,HighBeam);
  SET(PORTD,LowBeam);
  DelayFunction();
  CLR(PORTD,HighBeam);
  CLR(PORTD,LowBeam);
  DelayFunction();
  SET(PORTD,HighBeam);
  SET(PORTD,LowBeam);
  DelayFunction();
  CLR(PORTD,HighBeam);
  CLR(PORTD,LowBeam);
  DelayFunction();
}

int main()
{

  InputOutput();
  SREG |= (1<<7);//Enable Global Interrupt
  InitializeSwitch();
  InitializeADC();

  while(1)
  {
    ADCSRA |= (1<<ADSC);//Single Conversion

    if (FLAGbit.AssistanceSwitch == 0)
    {
      CLR(PORTD,HighBeam);
      CLR(PORTD,LowBeam);
      FLAGbit.StartFLAG=0;
    }
    else
    {
      if(FLAGbit.StartFLAG==0)
      {
        StartIndication();
        FLAGbit.StartFLAG=1;
      }
      else
      {
        if (ADC > 50) //Go into Low Beam mode
        {
          CLR(PORTD,HighBeam);
          SET(PORTD,LowBeam);
        }
        else  //Go into High Beam mode
        {
          SET(PORTD,HighBeam);
          CLR(PORTD,LowBeam);
        }
        if (HighBeam==0 && LowBeam==0)
        {
          MIL();
        }
      }
    }
  }
}

//ISR for Assistance switch On/Off
ISR(PCINT2_vect)
{
  FLAGbit.AssistanceSwitch = ~FLAGbit.AssistanceSwitch;
}

ISR(TIMER0_OVF_vect	)
{
  cli();
  count++;

  if(count>=10)
  {
    FLAGbit.Timer = 1;
    count=0;
  }
}
