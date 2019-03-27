#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>

#define FALSE 0
#define TRUE 1
#define TTL_SECONDS 15 * 60

uint8_t times = 0, timerOn = FALSE;
uint16_t threshold = 0;

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
    TCCR1B |= (0 << WGM13) | (1 << WGM12);
    OCR1A = 15627;
    TIMSK1 |= (1 << OCIE1A);
    asm volatile ("nop");

    //Enable global interrupts
    sei();
    // Relax after exhausting work
    asm volatile ("nop");
}

void startTimer(void) {
    TCCR1B |= (1 << CS12) | (0 << CS11) | (1 << CS10);
    timerOn = TRUE;
}

void stopTimer(void) {
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    timerOn = FALSE;
}

void resetTimer(void) {
    TCNT1 = 0;
    threshold = 0;
}

void onRelay(void) {
    PORTB |= (1 << PB5);
}

void offRelay(void) {
    PORTB &= ~(1 << PB5);
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
        threshold++;
        if (threshold >= TTL_SECONDS ) {
            times--;
            resetTimer();
        }
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
        _delay_ms(600);
        sei();
        }

int main(void) {
    init();

    times = 4;

    // Repeat indefinitely
    for (;;) {
        if (times > 0 && timerOn == FALSE) {
            resetTimer();
            offRelay();
            startTimer();
        }
        if (times == 0 && timerOn == TRUE) {
            stopTimer();
            onRelay();
        }
        showLights();
    }
}
