#include "fsl_device_registers.h"
#include <stdio.h>
#include <string.h>

/*
PINOUT

Motor 1
IN1 -> Motor 1 Red -> D0
IN2 -> Motor 1 Black -> D1
ENA -> D2

Motor 2
IN3 -> Motor 2 Red -> D3
IN4 -> Motor 2 Black -> D4
ENB -> D5

Servo
C10
*/

// COMMAND DEFINITIONS
#define CMD_OFF                 0
#define CMD_AI                  1
#define CMD_STOP                2
#define CMD_FORWARD_STRAIGHT    3
#define CMD_FORWARD_RIGHT       4
#define CMD_FORWARD_LEFT        5
#define CMD_REVERSE_STRAIGHT    6
#define CMD_REVERSE_RIGHT       7
#define CMD_REVERSE_LEFT        8
#define CMD_RIGHT               9
#define CMD_LEFT                10

// STATE DEFINITIONS
#define STRAIGHT 0
#define RIGHT    1
#define LEFT     2
#define STOP     0
#define FORWARD  1
#define REVERSE  2
#define MANUAL   0
#define AI       1

#define AI_SPEED 70
#define FRAME_WIDTH_PIXELS 640

// GLOBAL STATE
volatile int cur_lat   = STRAIGHT;
volatile int cur_lon   = STOP;
volatile int cur_mode  = MANUAL;

volatile int state     = CMD_STOP;    // current command
volatile int cmd_recv  = 0;           // used by PIT safety

volatile char bbox_buffer[64];
volatile int bbox_index = 0;
volatile int bbox_mode  = 0;          // 1 = reading inside <BBOX,...>
int last_turn = STRAIGHT;
int hysteresis_frames = 0;

// Helper Functions

void set_speed_right(uint8_t duty) {
    if (duty > 100) duty = 100;
    uint32_t cnv = (FTM3->MOD * duty) / 100;
    FTM3->CONTROLS[2].CnV = cnv;
}

void set_speed_left(uint8_t duty) {
    if (duty > 100) duty = 100;
    uint32_t cnv = (FTM0->MOD * duty) / 100;
    FTM0->CONTROLS[5].CnV = cnv;
}

void right_forward_ai(void) {
    set_speed_right(AI_SPEED);
    GPIOD_PDOR &= ~0x03;
    GPIOD_PDOR |= 0x01;
}

void left_forward_ai(void) {
    set_speed_left(AI_SPEED);
    GPIOD_PDOR &= ~0x18;
    GPIOD_PDOR |= 0x08;
}

void right_reverse_ai(void) {
    set_speed_right(AI_SPEED);
    GPIOD_PDOR &= ~0x03;
    GPIOD_PDOR |= 0x02;
}

void left_reverse_ai(void) {
    set_speed_left(AI_SPEED);
    GPIOD_PDOR &= ~0x18;
    GPIOD_PDOR |= 0x10;
}

void forward_ai(void) {
    if (cur_lon != FORWARD) {
        cur_lon = FORWARD;
        left_forward_ai();
        right_reverse_ai();
    }
}

void right_forward(void) {
    set_speed_right(100);
    GPIOD_PDOR &= ~0x03;
    GPIOD_PDOR |= 0x01;
}

void right_reverse(void) {
    set_speed_right(100);
    GPIOD_PDOR &= ~0x03;
    GPIOD_PDOR |= 0x02;
}

void right_stop(void) {
    GPIOD_PDOR &= ~0x03;
}

void left_forward(void) {
    set_speed_left(100);
    GPIOD_PDOR &= ~0x18;
    GPIOD_PDOR |= 0x08;
}

void left_reverse(void) {
    set_speed_left(100);
    GPIOD_PDOR &= ~0x18;
    GPIOD_PDOR |= 0x10;
}

void left_stop(void) {
    GPIOD_PDOR &= ~0x18;
}

void straight(void) {
    if (cur_lat != STRAIGHT) {
        FTM3->CONTROLS[6].CnV = 2500;
        cur_lat = STRAIGHT;
    }
}

void right(void) {
    if (cur_lat != RIGHT) {
        FTM3->CONTROLS[6].CnV = 3000;
        cur_lat = RIGHT;
    }
}

void left(void) {
    if (cur_lat != LEFT) {
        FTM3->CONTROLS[6].CnV = 2000;
        cur_lat = LEFT;
    }
}

void forward(void) {
    if (cur_lon != FORWARD) {
        cur_lon = FORWARD;
        left_forward();
        right_reverse();
    }
}

void reverse(void) {
    if (cur_lon != REVERSE) {
        cur_lon = REVERSE;
        left_reverse();
        right_forward();
    }
}

void stop(void) {
    left_stop();
    right_stop();
    cur_lon = STOP;
}

// Debug print
void uart0_putchar(char c) {
    while (!(UART0->S1 & UART_S1_TDRE_MASK));
    UART0->D = c;
}
void print_command(void) {
    uart0_putchar(state + '0');
    uart0_putchar('\r');
    uart0_putchar('\n');
}

// process bounding boxes
static void process_bbox(void) {
    int x = 0, y = 0, w = 0, h = 0;
    if (sscanf((char*)bbox_buffer, "BBOX,%d,%d,%d,%d", &x, &y, &w, &h) != 4)
        return;

    if (w <= 0 || h <= 0) {
        straight();
        forward_ai();
        return;
    }

    int center_x = x + (w / 2);
    int mid = FRAME_WIDTH_PIXELS / 2;
    int deadband = 50;
    int stable_required = 4;

    int desired_turn =
        (center_x < (mid - deadband)) ? RIGHT :
        (center_x > (mid + deadband)) ? LEFT :
        STRAIGHT;

    if (desired_turn != last_turn) {
        hysteresis_frames++;
        if (hysteresis_frames < stable_required)
            desired_turn = last_turn;
        else {
            last_turn = desired_turn;
            hysteresis_frames = 0;
        }
    }

    if (desired_turn == RIGHT) {
        right();
        forward_ai();
    } else if (desired_turn == LEFT) {
        left();
        forward_ai();
    } else {
        straight();
        forward_ai();
    }
}

// UART1 ISR — process uart signals
void UART1_RX_TX_IRQHandler(void) {
    uint8_t status = UART1->S1;
    if (!(status & UART_S1_RDRF_MASK)) return;

    uint8_t c = UART1->D;
    cmd_recv = 1;

    // NOT IN BBOX MODE
    if (!bbox_mode) {
        if (c == '<') {
            bbox_mode = 1;
            bbox_index = 0;
            return;
        }

        // Command byte (0–10)
        if (c <= 10) {
            state = c;

            // ENTER AI MODE
            if (state == CMD_AI) {
                cur_mode = AI;
                stop();
                straight();
                return;
            }

            // EXIT AI MODE
            if (cur_mode == AI && state != CMD_AI) {
                cur_mode = MANUAL;
                stop();
                straight();
            }

            // MANUAL COMMANDS
            if (cur_mode == MANUAL) {
                switch (state) {
                    case CMD_OFF: stop(); break;
                    case CMD_STOP: straight(); stop(); break;
                    case CMD_FORWARD_STRAIGHT: straight(); forward(); break;
                    case CMD_FORWARD_RIGHT: right(); forward(); break;
                    case CMD_FORWARD_LEFT: left(); forward(); break;
                    case CMD_REVERSE_STRAIGHT: straight(); reverse(); break;
                    case CMD_REVERSE_RIGHT: right(); reverse(); break;
                    case CMD_REVERSE_LEFT: left(); reverse(); break;
                    case CMD_LEFT: stop(); left(); break;
                    case CMD_RIGHT: stop(); right(); break;
                    default: straight(); stop(); break;
                }
            }
            return;
        }

        // ignore other ASCII outside BBOX
        return;
    }

    // INSIDE BBOX MODE
    if (c == '>') {
        bbox_buffer[bbox_index] = '\0';
        bbox_mode = 0;

        if (cur_mode == AI)
            process_bbox();

        return;
    }

    if (bbox_index < sizeof(bbox_buffer) - 1)
        bbox_buffer[bbox_index++] = (char)c;
}

// SAFETY TIMER
void PIT0_IRQHandler(void) {
    PIT->CHANNEL[0].TFLG = PIT_TFLG_TIF_MASK;

    if (!cmd_recv) stop();
    cmd_recv = 0;
}

// MAIN
int main(void)
{
	 // Setup Code

	    SIM_SCGC6 |= SIM_SCGC6_FTM0_MASK;      // FTM0 for left motor
	    SIM_SCGC3 |= SIM_SCGC3_FTM3_MASK;      // FTM3 for right motor + servo
	    SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;     // Port D
	    SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;     // Port C
	    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;    // Port B
	    SIM_SCGC6 |= SIM_SCGC6_PIT_MASK;       // PIT
	    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;    // UART0
	    SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;    // UART1

	    // UART0: PTB16 = RX, PTB17 = TX
	    PORTB->PCR[16] = PORT_PCR_MUX(3);
	    PORTB->PCR[17] = PORT_PCR_MUX(3);

	    // UART1: PTC3 = RX, PTC4 = TX
	    PORTC->PCR[3] = PORT_PCR_MUX(3);
	    PORTC->PCR[4] = PORT_PCR_MUX(3);

	    // Disable UART0/1 before configuration
	    UART0->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
	    UART1->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);

	    // Motor pins
	    PORTD_PCR0  = PORT_PCR_MUX(1);   // PTD0 = IN1
	    PORTD_PCR1  = PORT_PCR_MUX(1);   // PTD1 = IN2
	    PORTD_PCR2  = PORT_PCR_MUX(4);   // PTD2 = ENA (PWM FTM3 CH2)
	    PORTD_PCR3  = PORT_PCR_MUX(1);   // PTD3 = IN3
	    PORTD_PCR4  = PORT_PCR_MUX(1);   // PTD4 = IN4
	    PORTD_PCR5  = PORT_PCR_MUX(4);   // PTD5 = ENB (PWM FTM0 CH5)
	    PORTC_PCR10 = PORT_PCR_MUX(3);   // PTC10 = Servo (FTM3 CH6)

	    // Motor direction pins as outputs
	    GPIOD_PDDR |= (1 << 0);   // IN1
	    GPIOD_PDDR |= (1 << 1);   // IN2
	    GPIOD_PDDR |= (1 << 3);   // IN3
	    GPIOD_PDDR |= (1 << 4);   // IN4

	    // --- PWM setup for FTM0 (LEFT motor) ---
	    FTM0->MODE |= FTM_MODE_WPDIS_MASK;
	    FTM0->MODE &= ~FTM_MODE_FTMEN_MASK;

	    // Timer clock source to MCGFLLCLK
	    SIM->SOPT2 &= ~(0x3 << 24);
	    SIM->SOPT2 |=  (0x1 << 24);   // TPMSRC = 01

	    FTM0->SC = 0;
	    FTM0->SC = FTM_SC_PS(0);      // prescaler = 1
	    FTM0->MOD = 2000 - 1;         // ~24 kHz

	    FTM0->CONTROLS[5].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;
	    FTM0->SC |= FTM_SC_CLKS(1);   // start timer

	    // --- PWM setup for FTM3 (RIGHT motor + SERVO) ---
	    FTM3->MODE |= FTM_MODE_WPDIS_MASK;
	    FTM3->MODE &= ~FTM_MODE_FTMEN_MASK;

	    FTM3->SC = 0;
	    FTM3->SC = FTM_SC_PS(4);      // prescaler = 16
	    FTM3->MOD = 6000 - 1;
	    FTM3->OUTMASK &= ~(1 << 6);

	    FTM3->CONTROLS[2].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK; // Right motor speed
	    FTM3->CONTROLS[6].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK; // Servo

	    // Set servo to straight by default
	    FTM3->CONTROLS[6].CnV = 2500;
	    FTM3->SC |= FTM_SC_CLKS(1);   // start timer

	    // UART0: 9600 baud (debug)
	    uint16_t sbr = 137;           // for ~9600 at 21 MHz
	    UART0->BDH = (sbr >> 8) & UART_BDH_SBR_MASK;
	    UART0->BDL = sbr & UART_BDL_SBR_MASK;

	    // UART1: 115200 baud (NavQ)
	    sbr = 11;                     // for ~115200 at 21 MHz
	    UART1->BDH = (sbr >> 8) & UART_BDH_SBR_MASK;
	    UART1->BDL = sbr & UART_BDL_SBR_MASK;

	    UART0->C1 = 0;
	    UART1->C1 = 0;

	    // Enable RX interrupt
	    UART0->C2 |= UART_C2_RIE_MASK;
	    UART1->C2 |= UART_C2_RIE_MASK;

	    // Enable TX and RX
	    UART0->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;
	    UART1->C2 |= UART_C2_TE_MASK | UART_C2_RE_MASK;

	    // PIT Timer
	    PIT->MCR = 0x00;	// Enable PIT module

	    PIT->CHANNEL[0].LDVAL = 60000000 - 1;

	    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TIE_MASK | PIT_TCTRL_TEN_MASK;

	    // Enable interrupts in NVIC
	    NVIC_EnableIRQ(PIT0_IRQn);
	    NVIC_EnableIRQ(UART0_RX_TX_IRQn);
	    NVIC_EnableIRQ(UART1_RX_TX_IRQn);


    for (;;) {}
}
