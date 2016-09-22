#include <Arduino.h>

#define adc_disable()  (ADCSRA &= ~(1<<ADEN))
#define adc_enable() (ADCSRA |= (1<<ADEN) )
#define BUTTON_PIN PB2
#define HD1 PB1
#define HD2 PB4
#define DELAY_CNT 11

#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "wav.h"
#include <avr/sleep.h>

unsigned long p = 0;
volatile unsigned int song = 0;
unsigned long lastInteruptTime = 0;
volatile unsigned long timerCount = 0;

volatile bool buttonEdge = false;

void setup() {

  setupTimer();
  setOutputAndInterupts();
}

void loop() {
  if( buttonEdge ){
    buttonEdge = false;
    nextSong();
    /*
    if( digitalRead(BUTTON_PIN)==HIGH ) {

    }
    */
  }
}

void nextSong(){
  if( ((timerCount-lastInteruptTime)>400) ){
    song++;
    lastInteruptTime = timerCount;
    if( song > 1 ){
      song = 0;
      p = 0;
      sleep();
      return;
    }
  }
}

// Sample interrupt
ISR(TIMER0_COMPA_vect) {
  unsigned long t = p;
  //unsigned long anHour = 28800000;
  timerCount++;

  //char sample = pgm_read_byte(&wav[p]);
  //char sample = ((t>>6)|t|t>>(t>>16))*10+((t>>11)&7);
  //char sample = ((1-(((t+10)>>((t>>9)&15))&2))*2)*((((t)>>10)^((t+20)>>10))&1)*32+(((t&4095)-2047)*((t/((t>>10&3)+1))&((t>>10&7)+5))+(t>>(((t>>12)+16)&25)&1)*t%512*(t%256-128)/2)/1024+128;
  //char sample = t>>6^t&0x25|t+(t^t>>11)-t* ((t%24?2:6)&t>>11)^t<<1&(t&0x256?t>>4:t>>10);
  //char sample = ( t* (( t>>9| t>>13 ) & 15)) & 129;
  //char sample = (int)(((t>>4)|(t%10))+3.3) | (((t%101)|(t>>14))&((t>>7)|(t*t%17)));
  char sample = 0;
  switch( song ){
    case 0:
    sample = (t&(t>>6)+(t<<((t>>11)^((t>>13)+3L))|((t>>14%64L)+(t>>14)))&(-t>>5));
    break;
    case 1:
    sample = ((t>>9)&(t^(-t>>8)+t)+20L*(t>>17)%256L)>>1;
    break;
    case 2:
    sample = t;
    break;
  }
  p++;
  OCR1A = sample; OCR1B = sample ^ 255;

}

ISR(INT0_vect)
{
    //(PINB&(1<<BUTTON_PIN))!=0
    buttonEdge = true;

}

void setupTimer(){

  lastInteruptTime= 0;
  timerCount = 0;

  // Enable 64 MHz PLL and use as source for Timer1
  PLLCSR = 1<<PCKE | 1<<PLLE;
  // Set up Timer/Counter1 for PWM output
  TIMSK = 0;                              // Timer interrupts OFF
  TCCR1 = 1<<PWM1A | 2<<COM1A0 | 1<<CS10; // PWM A, clear on match, 1:1 prescale
  GTCCR = 1<<PWM1B | 2<<COM1B0;           // PWM B, clear on match
  OCR1A = 128; OCR1B = 128;               // 50% duty at start

  // Set up Timer/Counter0 for 8kHz interrupt to output samples.
  TCCR0A = 3<<WGM00;                      // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00;            // 1/8 prescale
  TIMSK = 1<<OCIE0A;                      // Enable compare match
  OCR0A = 124;                            // Divide by 1000
}

void setOutputAndInterupts(){
  pinMode(HD1, OUTPUT);
  pinMode(HD2, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  //DDRB |= B010010;
  adc_disable();

  //DDRB |= B000100;
  //PORTB &= ~B000100;

  DDRB &= ~_BV(BUTTON_PIN);
  PORTB |= _BV(BUTTON_PIN);
  MCUCR = _BV(ISC01) | _BV(ISC00);

  GIMSK = 0b01000000;    // turns on pin change interrupts
  PCMSK = 0b00000000;    // turn on interrupts on pins PB3

  sei();                 // enables interrupts
}

void sleep()
{
  TIMSK = 0;
  digitalWrite(1,LOW);
  digitalWrite(4,LOW);
  PORTB = _BV(BUTTON_PIN);
  //set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  GIMSK = 0b00100000;
  PCMSK = 0b00000100;
  sleep_enable();
  sei();
  wdt_disable();
  delay_ten_us(1000);
  MCUCR = _BV(SM1) |  _BV(SE);
  sleep_cpu();  // 1uA

  //sleep_disable();

  setupTimer();
  setOutputAndInterupts();
}

// This function delays the specified number of 10 microseconds
// it is 'hardcoded' and is calibrated by adjusting DELAY_CNT
// in main.h Unless you are changing the crystal from 8mhz, dont
// mess with this.
void delay_ten_us(uint16_t us) {
  uint8_t timer;
  while (us != 0) {
    // for 8MHz we want to delay 80 cycles per 10 microseconds
    // this code is tweaked to give about that amount.
    for (timer=0; timer <= DELAY_CNT; timer++) {
      __asm__("nop\n\t");
      __asm__("nop\n\t");
    }
    __asm__("nop\n\t");
    us--;
  }
}
