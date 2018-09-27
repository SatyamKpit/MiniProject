/*
NAME   : AARTHI PARAMASIVAM
EMP ID : 138011
BATCH  : ADAS
NAME   : INTELLIGENT TURN LIGHTS
*/


/////////HEADER FILES/////
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>


////////PREPROCESSOR DIRECTIVES/////
#define SET_BIT(PORT,BIT) PORT|= (1<<BIT)
#define CLR_BIT(PORT,BIT) PORT&= ~(1<<BIT)
#define TOGGLE_BIT(PORT, BIT) PORT^=(1<<BIT)

////////VARIABLE DECLARATION////////
struct
{
  volatile unsigned int flag:1;
  volatile unsigned int c:1;

}FLAG;

int main()
{
  Serial.begin(9600);


  ////////OUTPUT DECLARATION/////

  SET_BIT(DDRD,PD6);  //DIGITAL PIN 6
  CLR_BIT(PORTD,PD6);

  SET_BIT(DDRD,PD7); //DIGITAL PIN 7
  CLR_BIT(PORTD,PD7);

  SET_BIT(DDRB,PB0);  //DIGITAL PIN 8
  CLR_BIT(PORTB,PB0);

  SET_BIT(DDRB,PB1); //DIGITAL PIN 9
  CLR_BIT(PORTB,PB1);

  ////////INPUT DECLARATION/////
  CLR_BIT (DDRD,PD2);  //switch

  ///////INTERRUPT REGISTER FOR INT0///////
  EICRA|=(1<<ISC00);
  EICRA&=~(1<<ISC01);
  EIMSK|=(1<<INT0);


  sei();//GLOBAL INTERRUPT

while(1)
  {

  ADMUX =0x01;
  ADC_INIT();
  Serial.println(ADC);
  if(FLAG.flag==1)     //ENGINE ON CONDITION
  {
    if(ADC>0 && ADC<250)  //LEFT LED BLINKING BLUE
    {
     SET_BIT(PORTB,PB1);
     _delay_ms(500);
     CLR_BIT(PORTB,PB1);
     _delay_ms(500);
    }

   if(ADC>250 && ADC<=400)  //LEFT LED BLINKING GREEN
    {
     SET_BIT(PORTB,PB0);
     _delay_ms(500);
     CLR_BIT(PORTB,PB0);
     _delay_ms(500);
    }

    if(ADC>=600 && ADC<=800)  //RIGHT LED BLINKING GREEN
    {
     SET_BIT(PORTD,PD6);
     _delay_ms(500);
     CLR_BIT(PORTD,PD6);
     _delay_ms(500);

    }
    if(ADC<=1023 && ADC>800)  //RIGHT LED BLINKING BLUE
    {
     SET_BIT(PORTD,PD7);
     _delay_ms(500);
     CLR_BIT(PORTD,PD7);
     _delay_ms(500);
    }
  }

  }
}


////////ADC_INIT FUNCTION/////////
 int ADC_INIT()
  {
  ADMUX |=(1<<REFS0);
  ADCSRA |=(1<<ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2);
  ADCSRA |=(1<<ADSC);
  }

////////INTERRUPT SERVICE ROUTINE-INT0//////
 ISR(INT0_vect)
  {
   //
   cli();

   FLAG.c++;
   if((FLAG.c)%2==1)
   {
     FLAG.flag=1;
   }
   else
   {
     FLAG.flag=0;
   }
   sei();
  }




