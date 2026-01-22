#include <SoftwareSerial.h>

// Bluetooth Device 00:14:03:05:0C:EE DSD TECH HC-05

// Command Table
#define     CMD_OFF                 0
#define     CMD_AI                  1
#define     CMD_STOP                2
#define     CMD_FORWARD_STRAIGHT    3
#define     CMD_FORWARD_RIGHT       4
#define     CMD_FORWARD_LEFT        5
#define     CMD_BACKWARD_STRAIGHT   6
#define     CMD_BACKWARD_RIGHT      7
#define     CMD_BACKWARD_LEFT       8
#define     CMD_RIGHT               9
#define     CMD_LEFT                10

// Joystick Pins
#define JOY_X A0
#define JOY_Y A1            
#define JOY_BTN 2

// Joystick positioning
#define CENTER 512      // 0 - 1023
#define THRESHOLD 300
#define LEFT  (CENTER + THRESHOLD)
#define RIGHT (CENTER - THRESHOLD)
#define FORWARD (CENTER - THRESHOLD)
#define BACKWARD (CENTER + THRESHOLD)

// Bluetooth Module (HC-05)
SoftwareSerial BTSerial(10, 11); // RX, TX

// Mode tracking
bool manual_mode = true;

// Command tracking
int last_command = -1;
unsigned long last_command_time = 0;
const unsigned long command_interval = 100; // ms

volatile bool button_pressed = false;

void buttonISR() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if (interrupt_time - last_interrupt_time > 200) {
    button_pressed = true;
  }
  last_interrupt_time = interrupt_time;
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  // Setup joystick button
  pinMode(JOY_BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(JOY_BTN), buttonISR, FALLING);
  
  Serial.println("Arduino Joystick Controller Ready");
  
  // Manual mode sending CMD_OFF
  sendCommand(0);
}

void loop() {
  if(button_pressed){
    button_pressed = false;
    manual_mode = !manual_mode;
    if(manual_mode){

      sendCommand(CMD_STOP);
    }
    else{
      sendCommand(CMD_AI);
    }
  }
  
  // Read joystick and send commands only in manual mode
  if (manual_mode) {
    readJoystickAndSendCommand();
  }
  else{
    sendCommandWithTiming(CMD_AI);
  }
}

void readJoystickAndSendCommand() {

  int x = analogRead(JOY_X);
  int y = analogRead(JOY_Y);
  
  int command;

  if(x < FORWARD && y < RIGHT){
    command = CMD_FORWARD_RIGHT;
  }
  else if(x < FORWARD && y > LEFT){
    command = CMD_FORWARD_LEFT;
  }
  else if(x < FORWARD){
    command = CMD_FORWARD_STRAIGHT;
  }
  else if(x > BACKWARD && y < RIGHT){
    command = CMD_BACKWARD_RIGHT;
  }
  else if(x > BACKWARD && y > LEFT){
    command = CMD_BACKWARD_LEFT;
  }
  else if(x > BACKWARD){
    command = CMD_BACKWARD_STRAIGHT;
  }
  else if(y < RIGHT){
    command = CMD_RIGHT;
  }
  else if(y > LEFT){
    command = CMD_LEFT;
  }
  else{
    command = CMD_STOP;
  }
  
  sendCommandWithTiming(command);
}

void sendCommandWithTiming(int command){
  unsigned long currentTime = millis();

  if (command != last_command || currentTime - last_command_time > command_interval) {
    sendCommand(command);
    last_command = command;
    last_command_time = currentTime;
  }
}

void sendCommand(int cmd) {
  BTSerial.println(cmd);

  Serial.print("Sent: ");
  switch(cmd){
    case CMD_OFF:
      Serial.println("OFF");
      break;
    case CMD_AI:
      Serial.println("AI");
      break;
    case CMD_STOP:
      Serial.println("STOP");
      break;
    case CMD_FORWARD_STRAIGHT:
      Serial.println("FORWARD STRAIGHT");
      break;
    case CMD_FORWARD_RIGHT:
      Serial.println("FORWARD RIGHT");
      break;
    case CMD_FORWARD_LEFT:
      Serial.println("FORWARD LEFT");
      break;
    case CMD_BACKWARD_STRAIGHT:
      Serial.println("BACKWARD STRAIGHT");
      break;
    case CMD_BACKWARD_RIGHT:
      Serial.println("BACKWARD RIGHT");
      break;
    case CMD_BACKWARD_LEFT:
      Serial.println("BACKWARD LEFT");
      break;
    case CMD_LEFT:
      Serial.println("LEFT");
      break;
    case CMD_RIGHT:
      Serial.println("RIGHT");
    default:
      Serial.println("ERROR: Incorrect CMD");
  }
}
