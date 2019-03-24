#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
//#include <util/twi.h>

#define FALSE 0
#define TRUE 1

uint8_t times = 0;


void init(void) {
    // set all pins as output
    DDRB = 0xFF; //sets all bits of port B for output
    DDRC = 0xFF;
    DDRD = 0xFF;
    PORTB = 0x00; //sets a output of port B to LOW
    PORTC = 0x00;
    PORTD = 0x00;

    // set buttons as input
    DDRD &= ~(1 << PD2);
    PORTD |= (1 << PD2);
    // enable pull up resistors
    MCUCR &= ~(1 << PUD);

    // Enable external button interrupt
    EIMSK |= (1 << INT0);
    EICRA &= ~((1 << ISC01) | (1 << ISC00));
    asm volatile ("nop");

    // timer configuration
    TCCR1A = 0;
    TCCR1B = 0;
    TIMSK1 = 0;
    TCNT1 = 0;
    TCCR1A |= (0 << WGM11) | (0 << WGM10);
    TCCR1B |= (0 << WGM13) | (1 << WGM12) | (1 << CS12) | (0 << CS11) | (1 << CS10);
    OCR1A = 15625;
//    OCR1AL = (uint8_t)(15625 & 0x00FF);
    TIMSK1 |= (1 << OCIE1A);
    asm volatile ("nop");

    //Enable global interrupts
    sei();
    // Relax after exhausting work
    asm volatile ("nop");
}

void showLights(void) {
    uint8_t tmp, cache[9] = {
            0b00000000,
            0b00000001,
            0b00000011,
            0b00000111,
            0b00001111,
            0b00011111,
            0b00111111,
            0b01111111,
            0b11111111
    };

    tmp = cache[times];
    tmp &= 0x0F;
    PORTC &= 0xF0;
    PORTC |= tmp;

    tmp = cache[times];
    tmp &= 0xF0;
    PORTD &= 0x0F;
    PORTD |= tmp;
}

ISR(TIMER1_COMPA_vect)
        {
                cli();
        times++;
        if (times>8){
            times = 0;
        }
        showLights();
        TCNT1 = 0;
        sei();
        }

ISR(INT0_vect)
        {
                cli();
        times++;
        if (times>8){
            times = 0;
        }
        showLights();
        _delay_ms(900);
        sei();
        }

int main(void) {
    init();
    //turn off relay
//    PORTB &= ~(1 << PB5);

    // Repeat indefinitely
    for (;;) {
        _delay_ms(50);
    }
}
