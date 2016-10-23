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


void setup() {
  Serial.begin(9600);
  Serial.println("Dug starting up");

  pinMode(right_motor_pin_a, OUTPUT);
  pinMode(right_motor_pin_b, OUTPUT);
  pinMode(right_motor_pin_pwm, OUTPUT);
}

long right_drum_position  = -999;

void loop() {
  long new_right_drum_position;
  new_right_drum_position = right_drum_encoder.read();

  digitalWrite(right_motor_pin_a, HIGH);
  digitalWrite(right_motor_pin_b, LOW);
  digitalWrite(right_motor_pin_pwm, LOW);

  if (new_right_drum_position != right_drum_position) {
    Serial.print("Right = ");
    Serial.print(new_right_drum_position);
    Serial.println();
    right_drum_position = new_right_drum_position;
  }
}

