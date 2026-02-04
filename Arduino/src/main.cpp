#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>

// =======================
// PIN & CONSTANT DEFINITIONS
// =======================

// Servo (PB1 / D9) - Uses Timer1 OC1A
#define STEERING   PB1 
#define PAN_ULTRA  PB2  


// Motor A (Right) - Uses Timer2
#define AIN1   PD2
#define AIN2   PD4
#define PWMA   PD3  // OC2B

// Motor B (Left) - Uses Timer0
#define BIN1   PD6
#define BIN2   PD7
#define PWMB   PD5  // OC0B

// Standby Pin
#define STBY   PB0  // D8

// Commands
#define CMD_OFF              0
#define CMD_AI               1  // (Unused in this version, maps to Stop)
#define CMD_STOP             2
#define CMD_FORWARD_STRAIGHT 3
#define CMD_FORWARD_RIGHT    4
#define CMD_FORWARD_LEFT     5
#define CMD_REVERSE_STRAIGHT 6
#define CMD_REVERSE_RIGHT    7
#define CMD_REVERSE_LEFT     8
#define CMD_RIGHT            9
#define CMD_LEFT             10

// Servo Pulse Widths (Timer1 Ticks, 0.5us per tick)
#define SERVO_CENTER 3000 // 1500us
#define SERVO_RIGHT   2000 // 1000us
#define SERVO_LEFT  4000 // 2000us

// Globals
volatile uint8_t cmd_received_flag = 0;
volatile uint16_t safety_ticks = 0;
#define SAFETY_TIMEOUT 50 // ~1 second

// =======================
// HARDWARE DRIVERS
// =======================

// --- SERVO ---
void set_servo_steering(uint16_t ticks) {
    OCR1A = ticks;
}

void set_servo_ultrasonic(uint16_t ticks) {
    OCR1B = ticks;
}

// --- MOTORS ---
void setMotorA(int16_t speed) { // Right Motor
    if (speed > 0) {
        PORTD |= (1 << AIN1);
        PORTD &= ~(1 << AIN2);
    } else if (speed < 0) {
        PORTD |= (1 << AIN2);
        PORTD &= ~(1 << AIN1);
        speed = -speed;
    } else {
        PORTD &= ~((1 << AIN1) | (1 << AIN2));
    }
    if (speed > 255) speed = 255;
    OCR2B = speed;
}

void setMotorB(int16_t speed) { // Left Motor
    if (speed > 0) {
        PORTD |= (1 << BIN1);
        PORTD &= ~(1 << BIN2);
    } else if (speed < 0) {
        PORTD |= (1 << BIN2);
        PORTD &= ~(1 << BIN1);
        speed = -speed;
    } else {
        PORTD &= ~((1 << BIN1) | (1 << BIN2));
    }
    if (speed > 255) speed = 255;
    OCR0B = speed;
}

// =======================
// MOVEMENT HELPERS
// =======================

void stop_all() {
    setMotorA(0);
    setMotorB(0);
}

void forward() {
    // Mirrored motors: Left (+), Right (-)
    setMotorB(255);
    setMotorA(-255);
}

void reverse() {
    setMotorB(-255);
    setMotorA(255);
}

void turn_straight() { set_servo_steering(SERVO_CENTER); set_servo_ultrasonic(SERVO_CENTER); }
void turn_right()    { set_servo_steering(SERVO_RIGHT); set_servo_ultrasonic(SERVO_LEFT); }
void turn_left()     { set_servo_steering(SERVO_LEFT); set_servo_ultrasonic(SERVO_RIGHT); }

// =======================
// COMMAND PROCESSOR
// =======================

void process_command(uint8_t cmd) {
    switch (cmd) {
        case CMD_OFF: 
        case CMD_STOP:
        case CMD_AI:         // AI command treated as STOP since no BBOX logic
            turn_straight(); 
            stop_all(); 
            break;

        case CMD_FORWARD_STRAIGHT: 
            turn_straight(); 
            forward(); 
            break;

        case CMD_FORWARD_RIGHT: 
            turn_right(); 
            forward(); 
            break;

        case CMD_FORWARD_LEFT: 
            turn_left(); 
            forward(); 
            break;

        case CMD_REVERSE_STRAIGHT: 
            turn_straight(); 
            reverse(); 
            break;

        case CMD_REVERSE_RIGHT: 
            turn_right(); 
            reverse(); 
            break;

        case CMD_REVERSE_LEFT: 
            turn_left(); 
            reverse(); 
            break;

        case CMD_LEFT: 
            stop_all(); 
            turn_left(); 
            break;

        case CMD_RIGHT: 
            stop_all(); 
            turn_right(); 
            break;

        default: 
            turn_straight(); 
            stop_all(); 
            break;
    }
}

// =======================
// INTERRUPTS
// =======================

// UART Receive Interrupt
ISR(USART_RX_vect) {
    uint8_t c = UDR0;
    
    // Only process valid command bytes (0-10)
    if (c <= 10) {
        process_command(c);
        cmd_received_flag = 1; // Reset safety watchdog
    }
}

// Timer1 Overflow (Safety Watchdog - Runs approx 50Hz)
ISR(TIMER1_OVF_vect) {
    if (cmd_received_flag) {
        safety_ticks = 0;
        cmd_received_flag = 0;
    } else {
        safety_ticks++;
        if (safety_ticks > SAFETY_TIMEOUT) {
            stop_all(); // Kill motors if no command for 1s
        }
    }
}

// =======================
// INIT & MAIN
// =======================

void init_all() {
    // 1. GPIO
    DDRD |= (1 << AIN1) | (1 << AIN2) | (1 << BIN1) | (1 << BIN2) | (1 << PWMA) | (1 << PWMB);
    DDRB |= (1 << STBY) | (1 << STEERING) | (1 << PAN_ULTRA);
    PORTB |= (1 << STBY); // Enable Motor Driver
    

    // 2. PWM Setup
    // Timer0 (Motor B)
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0B1);
    TCCR0B = (1 << CS01); // Prescaler 8
    
    // Timer2 (Motor A)
    TCCR2A = (1 << WGM20) | (1 << WGM21) | (1 << COM2B1);
    TCCR2B = (1 << CS21); // Prescaler 8

    // Timer1 (Servo & Safety) - Mode 14 Fast PWM
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); 
    ICR1 = 40000;  
    OCR1A = SERVO_CENTER;
    OCR1B = SERVO_CENTER; // Initialize second servo to center
    
    TIMSK1 |= (1 << TOIE1);

    // 3. UART Setup (115200 Baud)
    UBRR0H = 0;
    UBRR0L = 8; // (16M / (16*115200)) - 1
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    sei(); // Enable Interrupts
}

int main(void) {
    init_all();

    while (1) {

    }
    return 0;
}