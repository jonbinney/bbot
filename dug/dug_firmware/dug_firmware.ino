// Needs encoder library: https://www.pjrc.com/teensy/td_libs_Encoder.html

#include <Encoder.h>

// On the Mega 2560, pins 2, 3, 18, 19, 20, 21 can do interrupts.
Encoder right_drum_encoder(2, 3);

// A is the clockwise input, B is the counterclockwise input. To move, set the appropriate
// one to HIGH and the other to LOW. Setting both to HIGH brakes to V_s. Setting both to low
// brakes to GND.
const int right_motor_pin_a = 5;
const int right_motor_pin_b = 6;
const int right_motor_pin_pwm = 7;

unsigned long time;
unsigned long motor_start_time;

bool motor_on = false;

void setup() {
  Serial.begin(57600);

  pinMode(right_motor_pin_a, OUTPUT);
  pinMode(right_motor_pin_b, OUTPUT);
  pinMode(right_motor_pin_pwm, OUTPUT);
}

long right_drum_position  = -999;

void loop() {
  time = micros();

  // Start the motor after 5 seconds
  if(time > 5000000 && time < 6000000) {
    if(!motor_on) {
        digitalWrite(right_motor_pin_a, HIGH);
        digitalWrite(right_motor_pin_b, LOW);
        digitalWrite(right_motor_pin_pwm, HIGH);
        motor_start_time = time;
        motor_on = true;
    }

    // Read encoders
    right_drum_position = right_drum_encoder.read();

    Serial.print(time - motor_start_time);
    Serial.print(",");
    Serial.print(right_drum_position);
    Serial.println();
  }
  // Stop the motor after another second
  else if(time >= 6000000) {
      digitalWrite(right_motor_pin_pwm, LOW);
  }

}

