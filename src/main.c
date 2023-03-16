#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define DIR_PINX PD2
#define STEP_PINX PD3
#define DIR_PINY PD4
#define STEP_PINY PD5
#define SEL PD6
#define JOY_X PC0
#define JOY_Y PC1
#define POT PC2

#define STEPS_PER_REV 200   // Change this to match your stepper motor
#define MICROSTEP 1         // Change this to match your A4988 driver

volatile uint16_t joy_x = 512;
volatile uint16_t joy_y = 512;
volatile uint16_t speed = 512;

ISR(ADC_vect) {
  static uint8_t mux = 0;
  
  if (mux == 0) {
    joy_x = ADC;
    mux = 1;
    ADMUX = (1<<REFS0) | (1<<MUX0);
  } else {
    joy_y = ADC;
    mux = 0;
    ADMUX = (1<<REFS0) | (0<<MUX0);
  }
  
  ADCSRA |= (1<<ADSC);
}

 void delay(unsigned int ms)
 {
  while(ms > 0)
  {
    _delay_ms(1);
    ms--;
  }
  return;
 }


void init_adc() {
  ADMUX = (1<<REFS0) | (0<<MUX0);
  ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRB = 0;
  ADCSRA |= (1<<ADSC);
}

void init_timer(){
  TCCR0A |= (0<<COM0A1) | (0<<COM0A0) | (0<<COM0B0) | (0<<COM0B1) | (1<<WGM01) | (0<<WGM00); // Prescaler /64 normal mode, CTC
  OCR0A = 0xF9; // OCR = ((clockfreq/prescaler)*time_in_seconds)-1 
  TCCR0B |= (0<<CS02) | (1<<CS01) | (1<<CS00) | (0<<WGM02);
}

  int speeds[]={16,8,4,2,1};
  char oneshot = 0;
  int i = 0;
  int delay_time = 1;
int main(void){
  
  //PORTD |= (1 << PD6);
  DDRD &= ~(1 << PD6);
  DDRD = DDRD | (1<<DIR_PINX) | (1<<STEP_PINX) | (1<<DIR_PINY) | (1<<STEP_PINY);   // Set the direction and step pins as outputs
  init_adc();
  init_timer();
  sei();  // Enable interrupts

  while(1){

  if (!(PIND & (1 << PD6)) && oneshot == 0) {
    oneshot = 1;
    delay_time = speeds[i];
    i = (i + 1) % 5;
    oneshot = 0;
  }


    // Control X motor based on joystick X input
    if (joy_x < 300) {
      PORTD = PORTD & ~(1<<DIR_PINX);  // Set direction counterclockwise
      while( (TIFR0 & (1 << OCF0A) ) == 0){
        PORTD = PORTD | (1<<STEP_PINX);
      }
      PORTD = PORTD & ~(1<<STEP_PINX);
      TIFR0 = (1 << OCF0A);
    } else if (joy_x > 550) {
      PORTD = PORTD | (1<<DIR_PINX);  // Set direction clockwise
      while( (TIFR0 & (1 << OCF0A) ) == 0){
        PORTD = PORTD | (1<<STEP_PINX);
      }
      PORTD = PORTD & ~(1<<STEP_PINX);
      TIFR0 = (1 << OCF0A);
    }

    // Control Y motor based on joystick Y input
    if (joy_y < 450) {
      PORTD = PORTD & ~(1<<DIR_PINY);  // Set direction counterclockwise
      while( (TIFR0 & (1 << OCF0A) ) == 0){
        PORTD = PORTD | (1<<STEP_PINY);
      }
      PORTD = PORTD & ~(1<<STEP_PINY);
      TIFR0 = (1 << OCF0A);
    } else if (joy_y > 550) {
      PORTD = PORTD | (1<<DIR_PINY);  // Set direction clockwise
      while( (TIFR0 & (1 << OCF0A) ) == 0){
        PORTD = PORTD | (1<<STEP_PINY);
      }
      PORTD = PORTD & ~(1<<STEP_PINY);
      TIFR0 = (1 << OCF0A);
    }
  }
return 0;
}

