#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

#include "pitches.h"

// Uses the default address 0x40.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// These are the minimum and maximum pulse length counts (out of 4096) for our servos.
#define BIG_SERVO_MIN 160
#define BIG_SERVO_MAX 584
#define MICRO_SERVO_MIN 160
#define MICRO_SERVO_MAX 584

#define LEFT_ARM_OPEN    372
#define LEFT_ARM_CLOSED  266
#define RIGHT_ARM_CLOSED 478
#define RIGHT_ARM_OPEN   372

#define LEFT_EAR_BACK 475
#define LEFT_EAR_FORWARD 265
#define RIGHT_EAR_BACK 265
#define RIGHT_EAR_FORWARD 475
#define MIDDLE_EAR 370

uint8_t left_arm = 0;
uint8_t right_arm = 1;
uint8_t left_ear = 2;
uint8_t right_ear = 3;

int kissed_flag = 0;
int normal_light_flag = 0;

int photocell_pin = A5;
int photocell_reading;

const int button_pin = 2;
const int button_state = 0;

int speakerPin = 12;
int mary_had_a_little_lamb_notes[] = {
  NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4
};
int mary_had_a_little_lamb_lengths[] = {
  4, 4, 4, 4, 4, 4, 2, 4, 4, 2, 4, 4, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2
};

void setup()
{
  // Initialize serial port at 9600 baud.
  Serial.begin(9600);

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz.
  
  // Initialize the pushbutton pin as an input.
  pinMode(button_pin, INPUT);
  
  // Put arms and ears at normal position and wait for that to happen.
  reset_arms();
  reset_ears();
  delay(1000);
}

void loop()
{
  photocell_reading = analogRead(photocell_pin);
  Serial.print("Analog reading = ");
  Serial.print(photocell_reading);
  // Commented out because it's unneeded debugging information.
  // Serial.print(", ");
  // Serial.println(normal_light_flag);

  // When it gets bright or dark, make a sound.
  // Stop playing the sound until it's normal lighting out again.
  if ( photocell_reading >= 700 && photocell_reading <= 710 )
  {
    normal_light_flag = 1;
  }

  if ( photocell_reading < 700 && normal_light_flag )
  {
    play_song(0);
    normal_light_flag = 0;
  }
  else if ( photocell_reading > 710 && normal_light_flag )
  {
    for ( int i = 0; i < 4; i++ )
    {
      wiggle_ears();
    }
    
    reset_ears();
    normal_light_flag = 0;
  }
  
  button_state = digitalRead(button_pin);
  
  if ( button_state == HIGH )
  {
    hug();
    reset_arms();
  }
  
  // TODO: check if manually moved
}

void wiggle_ears()
{
  pwm.setPWM(left_ear, 0, LEFT_EAR_FORWARD);
  pwm.setPWM(right_ear, 0, RIGHT_EAR_BACK);
  delay(500);
  pwm.setPWM(left_ear, 0, LEFT_EAR_BACK);
  pwm.setPWM(right_ear, 0, RIGHT_EAR_FORWARD);
  delay(500);
}

void hug()
{
  pwm.setPWM(left_arm, 0, LEFT_ARM_CLOSED);
  pwm.setPWM(right_arm, 0, RIGHT_ARM_CLOSED);
  delay(1000);
  pwm.setPWM(right_arm, 0, RIGHT_ARM_OPEN);
  pwm.setPWM(left_arm, 0, LEFT_ARM_OPEN);
  delay(500);
}

void reset_arms()
{
  // Sets left_arm and right_arm to their min and max values.
  pwm.setPWM(left_arm, 0, LEFT_ARM_OPEN);
  pwm.setPWM(right_arm, 0, RIGHT_ARM_OPEN);
}

void reset_ears()
{
  // Sets left_ear and right_ear to the middle.
  pwm.setPWM(left_ear, 0, MIDDLE_EAR);
  pwm.setPWM(right_ear, 0, MIDDLE_EAR);
}

void play_song(int song_number)
{
  // 26 notes long
  for ( int i = 0; i < 26; i++ )
  {
    if ( song_number == 0 )
    {
      int note_length = 1000 / mary_had_a_little_lamb_lengths[i];
      tone(speakerPin, mary_had_a_little_lamb_notes[i], note_length);
    }
    
    int pause_between_notes = note_length * 1.30;
    delay(pause_between_notes);
    noTone(speakerPin);
  }
}
