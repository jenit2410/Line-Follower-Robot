# Line-Follower-Robot

## Basic Concept

The primary goal of this project is to minimize CPU usage by leveraging other hardware components, such as timers. Here’s a breakdown of the basic concepts:

- **Utilize CPU as little as possible** by relying on hardware components like timers.
- **Timer2’s OC2B on PD3 in CTC mode** could be an ideal solution for toggling the LED.
- **1 Hz toggle frequency** would require loading 7812 into OCR2B (with prescaler 1024).
- **Problem:** 7812 is too large for an 8-bit register, so this approach isn't feasible.
- **Alternative Solution:** Use Timer0 in Normal mode with interrupts.

## Code Snippets

```c
#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void InitTimerCounter1(); // for PWM frequency
void InitADC(uint8_t channel); // consider free running mode
ISR(ADC_vect); // ISR triggered by ADC, set OCR1A here for duty cycle
ISR(TIMER0_OVF_vect); // for LED blinking (1 Hz)

int main(void) {
    const uint8_t kInChannel = 0; // input channel for potentiometer
    DDRD |= (1 << PD3); // output pin for LED
    TIMSK0 |= (1 << TOIE0); // enable Timer0 overflow interrupt
    sei(); // enable global interrupts
    InitTimerCounter1(); // set PWM frequency to 50 Hz
    InitADC(kInChannel); // set free running mode with interrupts
    while(1) {
        if (ADC >= 511) { // depending on ADC value, turn Timer0 on or off
            TCCR0B |= (1 << CS02) | (1 << CS00); // turn on Timer0 with f_T0 = f_CPU/1024
        } else {
            TCCR0B &= ~( (1 << CS02) | (1<<CS00) ); // stop Timer0
            PORTD &= ~(1 << PD3); // turn off LED
        }
    } // while
} // main
`````

## Components
### Infrared Sensors
- **IR Transmitters and Receivers**: Used to detect the line by sending and receiving infrared light.
  - **White Surface**: Reflects IR rays, resulting in lower analog input voltage to the MCU.
  - **Black Surface**: Absorbs IR rays, resulting in higher analog input voltage to the MCU.

### Optical Sensor Circuits
- **Sensor Output**:
  - **Dark Surface**: Less light reflected, higher voltage at Line_1 (or Line_2).
  - **Bright Surface**: More light reflected, lower voltage at Line_1 (or Line_2).
- Ensure both left and right sensors yield the same output for the same surface (tune using potentiometers RV1 and RV2).

### Motor Control Operating Logic
- **Motor Speed Control**: 
  - `M1_REV` and `M2_REV`: 1 for forward, 0 for backward.
  - `M1_EN` and `M2_EN`: PWM Speed Control.

### Switch and Potentiometer
- **Switch BTN**: Connected to GPIO pin D2. Used to enter setup mode for tuning line detection thresholds.
- **Potentiometer POT1**: Connected to analog pin ADC7. Used to set the normal speed of the line follower.

## Line Follower Logic

### Basic Control Logic (Version 1)
- **Straight**: Both sensors detect the line (i.e., `sl ≥ T & sr ≥ T`).
- **Left Turn**: Left sensor detects off the line (`sl ≤ T & sr > T`).
- **Right Turn**: Right sensor detects off the line (`sl > T & sr ≤ T`).
- **Reverse**: Both sensors lose the line (`sl ≤ T & sr ≤ T`).

### Improved Control Logic  
- **Proportional (P) Control**: 
  - Divides the operational range into multiple intervals for more precise control.
  - Reduces wobble and increases speed by applying weaker turns when closer to the line and stronger turns when further off track.

### Code Snippet of :- Timer ,  H-Bridge
```cpp

// Timer

void initTimer1(){
// Clear OC1A and OC1B on Compare Match / Set OC1A and OC1B at Bottom;
// Wave Form Generator: Fast PWM, Mode 14, Top = ICR1
TCCR1A |= (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);
TCCR1B |= (1<<WGM13) | (1<<WGM12) | (1<<CS11); // prescaler = 8
ICR1 = 15999; // Top = 16000000/8/125 -1 = 15999 // 125 Hz
// Time to count up to ICR1+1 = 16000/(f_count)
// f_count = (16 MHz)/prescaler = (16 MHz)/8 = 2 MHz
 // Time to count up to ICR1+1 = 16000/(f_count) = 8 10^{-3} s = 8 ms
// PWM frequency = 1/(Time to count up to ICR1+1) = 1/8 kHz = 125 Hz
// ICR1 = 15999; this and OCR1A and OCR1B determine the duty cycles on OC1A and OC1B
// OCR1A = 3999; // duty cycle = 4000/16000 = 25%, assuming ICR1 = 15999
// OCR1B = 11999; // duty cycle = 12000/16000 = 75%, assuming ICR1 = 15999
// OCR1A and OCR1B could be set in setM1Speed(int16_t speed) and setM2Speed(int16_t)
// You can use the sign of the argument speed to decide on forward vs. backwards
// DDRB |= (1 << PB0) | (1<<PB1)|(1<<PB2) | (1<< PB3); // output pins, set above
}

// H-Bridge

// both motors forward
PORTB |= (1 << PB0); // forward right
PORTB |= (1 << PB3); // forward left
// both motors backward
PORTB &= ~(1 << PB0); // backward right
PORTB &= ~(1 << PB3); // backward left
// left turn
PORTB |= (1 << PB0); // forward right
PORTB &= ~(1 << PB3); // backward left
// right turn for 3 sec
PORTB &= ~(1 << PB0); // backward right
PORTB |= (1 << PB3); // forward left


`````


### Line-Follower with two Sensors – Improved :-

- **Goals**: 
  - reduce wobble
  - increase speed
- **Idea**:
  - Weaker turns when more on track
  - Stronger turns when wider off track
 
### Code Snippet of :-  Improved-(Version 2)
```cpp
// Improved

if ( left_sensor > 0.99*1024 && right_sensor > 0.99*1024 ) {
setM1Speed(set_speed_right); // straight
setM2Speed(set_speed_left);
} else if ( left_sensor < 0.96*1024 && right_sensor > 0.99*1024 ) {
setM1Speed(0.67*set_speed_right); // slight left
setM2Speed(1.33*set_speed_left);
} else if ( left_sensor < 0.93*1024 && right_sensor > 0.99*1024 ) {
setM1Speed(0.33*set_speed_right); // stronger left
setM2Speed(1.67*set_speed_left);
} else if ( left_sensor < 0.9*1024 && right_sensor > 0.99*1024) {
setM1Speed(0); // sharp left
setM2Speed(2.0*set_speed_left);
} else if ( right_sensor < 0.96*1024 && left_sensor > 0.99*1024) {
setM1Speed(1.33*set_speed_right); // slight right
setM2Speed(0.67*set_speed_left);
} else if ( right_sensor < 0.93*1024 && left_sensor > 0.99*1024) {
setM1Speed(1.67*set_speed_right); // stronger right
setM2Speed(0.33*set_speed_left);
} else if ( right_sensor < 0.9*1024 && left_sensor > 0.99*1024) {
setM1Speed(2.0*set_speed_right); // sharp right
setM2Speed(0);
} else { // reverse … }

// Note:
//   • The greater the deviation from the target value (0.99*1024), the greater offset, the greater the turn.
//   • This is the basic idea of proportional control
//   • It is however difficult to find the proper factors.

if ( left_sensor > 0.99*1024 && right_sensor > 0.99*1024 ) {
setM1Speed(set_speed_right);
setM2Speed(set_speed_left);
} else if ( left_sensor < 0.96*1024 && right_sensor > 0.99*1024 ) {
offset = 0.33;
setM1Speed((1-offset)*set_speed_right);
setM2Speed((1+offset)*set_speed_left);
} else if ( left_sensor < 0.93*1024 && right_sensor > 0.99*1024 ) {
offset = 0.66;
setM1Speed((1-offset)*set_speed_right);
setM2Speed((1+offset)*set_speed_left);
} else if ( left_sensor < 0.9*1024 && right_sensor > 0.99*1024) {
 offset = 1.0;
setM1Speed((1-offset)*set_speed_right);
setM2Speed((1+offset)*set_speed_left);
} else if ( right_sensor < 0.96*1024 && left_sensor > 0.99*1024) {
offset = 0.33;
setM1Speed((1+offset)*set_speed_right);
setM2Speed((1-offset)*set_speed_left);
}

`````

## Proportional (P) Control

Proportional (P) control is a technique used in control systems to maintain a desired output by adjusting the input based on the error between the desired and actual values. In this project, the error is the difference between the readings from two brightness sensors.

### Error Calculation

1. **Error Definitions**:
   - **Left Sensor Error**: `el = sl - ŝl` where `ŝl` is the desired value for the left sensor.
   - **Right Sensor Error**: `er = sr - ŝr` where `ŝr` is the desired value for the right sensor.
   - Simplified: `er = sr - sl` and `el = sl - sr`, thus `er = -el`.

2. **Control Logic**:
   - Calculate the error between the right and left sensor readings.
   - Apply a proportional gain `kKp` to determine the adjustment needed for the motor speeds.

### Implementation

```cpp
const float kKp = 1.0 / 500.0; // Proportional gain factor
const uint16_t kT = 500; // Threshold for sensor readings
float offset = 0.0;
int16_t error = 0;

// Motor speed adjustment based on sensor readings
if (right_sensor >= kT || left_sensor >= kT) {
    // On black line, apply P-control
    error = right_sensor - left_sensor;
    offset = kKp * static_cast<float>(error);
    setM1Speed((1.0 - offset) * set_speed_right); // Adjust right motor speed
    setM2Speed((1.0 + offset) * set_speed_left);  // Adjust left motor speed
} else if (right_sensor < kT && left_sensor < kT) {
    // Lost black line, move backward
    setM1Speed(-set_speed_right);
    setM2Speed(-set_speed_left);
} else {
    // Unexpected situation, stop motors
    setM1Speed(0);
    setM2Speed(0);
}
`````

## Proportional (P) Control

The Proportional (P) control algorithm is used to adjust the robot's movement to keep it aligned with a line. The control logic is based on the difference between the sensor readings.

### Control Logic

- **Error Calculation**:
  - `e = sr - sl`
  - Where `sr` is the reading from the right sensor, and `sl` is the reading from the left sensor.

- **Behavior Based on Error (`e`)**:
  - **e ≈ 0**: Both `sr` and `sl` are high, indicating the robot is on the line. The robot should move straight.
  - **e ≈ 0**: Both sensors are high, but slight differences cause a turn. The right motor runs faster than the left motor.
  - **e < 0**: `sr` is lower than `sl`, so the robot should turn left by making the left motor faster than the right motor.
  - **e > 0**: `sr` is higher than `sl`, so the robot should turn right by making the right motor faster than the left motor.
  - **Both sensors low**: The robot has lost the line, so both motors should move backwards.

### Main Ideas of P-Controller

1. **Sensor Measurement**:
   - Sensors measure the robot’s position relative to the line using brightness values.

2. **Error Calculation**:
   - Compute the error by subtracting the left sensor reading from the right sensor reading. A small error means the robot is close to the line, while a large error means a significant deviation.

3. **Scaling Factor**:
   - Multiply the error by a proportional gain factor `Kp` to determine the adjustment needed for the motors.

4. **Apply Correction**:
   - Adjust the motor speeds based on the scaled error to correct the robot's path.

5. **Tuning `Kp`**:
   - The scaling factor `Kp` is determined through trial and error, simulation, or calculation. Proper tuning is essential for effective control.



