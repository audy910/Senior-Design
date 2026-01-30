#include <avr/io.h>
#include <util/delay.h>

// --- Pulse widths (tune these for your servo) ---
#define SERVO_FORWARD_US 1500   // center
#define SERVO_LEFT_US    1000   // left turn
#define SERVO_RIGHT_US   2000   // right turn

void servo_pulse(uint16_t pulse_us)
{
    // Send one 20ms servo frame
    PORTB |= (1 << PB1);          // HIGH
    _delay_us(pulse_us);          // pulse width
    PORTB &= ~(1 << PB1);         // LOW
    _delay_ms(20 - (pulse_us / 1000));
}

void servo_forward()
{
    servo_pulse(SERVO_FORWARD_US);
}

void servo_left()
{
    servo_pulse(SERVO_LEFT_US);
}

void servo_right()
{
    servo_pulse(SERVO_RIGHT_US);
}

int main(void)
{
    DDRB |= (1 << PB1);   // PB1 = D9 output

    while (1) {
        // Forward
        servo_forward();
        _delay_ms(1000);

        // Left
        servo_left();
        _delay_ms(1000);

        // Right
        servo_right();
        _delay_ms(1000);
    }
}