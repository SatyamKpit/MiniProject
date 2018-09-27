#include<avr/io.h>
#include<avr/interrupt.h>

#define SET_BIT(PORT,BIT) PORT|= (1<<BIT)
#define CLR_BIT(PORT,BIT) PORT&= ~(1<<BIT)

struct{
  volatile unsigned int FLAG_ISR:1;
}FLAG_BIT;

uint16_t Value=0; //Global variable

int ADC_Read(void)
{

  ADMUX  = (1<<MUX1);  //Analog pin 2
  PRR &= ~(1<<PRADC);  //Power reduction off
  ADMUX |= (1<<REFS0); //Reference voltage
  ADCSRA |= (1<<ADEN); //ADC Enable
  ADCSRA |= (1<<ADSC); //Single Conversion
  Value = ADC;         //ADC Level

}

int main()
{
    Serial.begin(9600);

    CLR_BIT(DDRB,PB2);  //Digital Pin 10
  	SET_BIT(DDRD,PD4);  //Digital Pin 4
    SET_BIT(DDRD,PD3);  //Digital Pin 3

    //PCINT2 Interrupt
    PCICR = (1<<PCIE0);   //Pin change interrupt control register
    PCMSK0 = (1<<PCINT2); //PCINT2 interrupt
    sei();                //Enable global interrupts

    while(1)
    {

      if(FLAG_BIT.FLAG_ISR == 1) //Interrupts when switch is pressed
        {
          ADC_Read();
   		  Serial.println(Value);

          if(Value>0)  //Motor rotates clockwise
          {
             PORTD |= (1<<PD4);
             PORTD &= ~(1<<PD3);
          }
          else if(Value==0) //Motor rotates anti-clockwise and stops
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
}

//Interrupt vector
ISR(PCINT0_vect)
{
    FLAG_BIT.FLAG_ISR =! FLAG_BIT.FLAG_ISR;
}

