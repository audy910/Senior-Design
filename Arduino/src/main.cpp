#define F_CPU 16000000UL //  frequency  delay
#include <avr/io.h>
#include <util/delay.h>

// --- SERVO CONFIGURATION  ---
#define SERVO_PIN PB1   // D9 on Nano

#define SERVO_FORWARD_US 1500   // center
#define SERVO_LEFT_US    1000   // left turn
#define SERVO_RIGHT_US   2000   // right turn

// --- MOTOR CONFIGURATION ---
// Motor A (Right)
#define AIN1   PD2
#define AIN2   PD4
#define PWMA   PD3   // OC2B

// Motor B (Left)
#define BIN1   PD7
#define BIN2   PD6
#define PWMB   PD5   // OC0B

// Standby Pin
#define STBY   PB0   // D8

// --- SERVO FUNCTIONS ---
void servo_pulse(uint16_t pulse_us)
{
    // Send one ~20ms servo frame
    PORTB |= (1 << SERVO_PIN);        // HIGH
    _delay_us(pulse_us);              // pulse width
    PORTB &= ~(1 << SERVO_PIN);       // LOW
    
    // Wait the rest of the 20ms period
    _delay_ms(20 - (pulse_us / 1000)); 
}

// Helper to hold servo position for a specific time
void servo_hold(uint16_t pulse_us, uint16_t duration_ms)
{
    uint16_t loops = duration_ms / 20; // Each pulse is ~20ms
    for(uint16_t i=0; i<loops; i++) {
        servo_pulse(pulse_us);
    }
}

void servo_forward() { servo_pulse(SERVO_FORWARD_US); }
void servo_left()    { servo_pulse(SERVO_LEFT_US); }
void servo_right()   { servo_pulse(SERVO_RIGHT_US); }


// --- MOTOR FUNCTIONS ---

void pwm_init(void)
{
    // Configure Timer0 for Motor B (PD5 / OC0B)
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0B1); // Fast PWM
    TCCR0B = (1 << CS01);   // Prescaler 8
    OCR0B  = 0;

    // Configure Timer2 for Motor A (PD3 / OC2B)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2B1); // Fast PWM
    TCCR2B = (1 << CS21);   // Prescaler 8
    OCR2B  = 0;
}

void motor_init(void)
{
    // Motor Pins Output
    DDRD |= (1 << AIN1) | (1 << AIN2) | (1 << BIN1) | (1 << BIN2) | (1 << PWMA) | (1 << PWMB);
    // Standby Pin Output
    DDRB |= (1 << STBY);
    // Servo Pin Output
    DDRB |= (1 << SERVO_PIN);

    // Enable Motors
    PORTB |= (1 << STBY);

    pwm_init();
}

void setMotorA(int16_t speed)
{
    if (speed > 0) {
        PORTD |=  (1 << AIN1);
        PORTD &= ~(1 << AIN2);
        if (speed > 255) speed = 255;
        OCR2B = speed;
    }
    else if (speed < 0) {
        PORTD |=  (1 << AIN2);
        PORTD &= ~(1 << AIN1);
        speed = -speed;
        if (speed > 255) speed = 255;
        OCR2B = speed;
    }
    else {
        PORTD &= ~((1 << AIN1) | (1 << AIN2));
        OCR2B = 0;
    }
}

void setMotorB(int16_t speed)
{
    if (speed > 0) {
        PORTD |=  (1 << BIN1);
        PORTD &= ~(1 << BIN2);
        if (speed > 255) speed = 255;
        OCR0B = speed;
    }
    else if (speed < 0) {
        PORTD |=  (1 << BIN2);
        PORTD &= ~(1 << BIN1);
        speed = -speed;
        if (speed > 255) speed = 255;
        OCR0B = speed;
    }
    else {
        PORTD &= ~((1 << BIN1) | (1 << BIN2));
        OCR0B = 0;
    }
}

// --- MAIN ---

int main(void)
{
    motor_init(); // Sets up motors AND servo pin direction

    while (1) {
        
        // 1. Drive Forward Fast, Servo Center
        // setMotorA(255);
        // setMotorB(255);
        
        // Hold servo center for 1 second (motors keep running)
        servo_hold(SERVO_FORWARD_US, 1000); 

        // 2. Slow Down, Turn Servo Left
        // setMotorA(100);
        // setMotorB(100);
        
        // Hold servo left for 1 second
         servo_hold(SERVO_LEFT_US, 1000);

        // 3. Reverse, Turn Servo Right
        // setMotorA(-150);
        // setMotorB(-150);
        
        // Hold servo right for 1 second
        servo_hold(SERVO_RIGHT_US, 1000);
    }

    return 0;
}