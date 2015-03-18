#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Uses the default address 0x40.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define BIG_SERVO_MIN 160 // this is the 'minimum' pulse length count (out of 4096)
#define BIG_SERVO_MAX 584 // this is the 'maximum' pulse length count (out of 4096)
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

int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider
int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;

int speakerPin = 12;
int numTones = 10;
int tones[] = {261, 277, 294, 311, 330, 349, 370, 392, 415, 440};
//            mid C  C#   D    D#   E    F    F#   G    G#   A

const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
int buttonState = 0;         // variable for reading the pushbutton status

void setup()
{
  // Initialize serial port.
  Serial.begin(9600);

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz.

  reset_arms();
  reset_ears();
  delay(1000);
  
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);   
}

void loop()
{
  photocellReading = analogRead(photocellPin);
  Serial.print("Analog reading = ");
  Serial.println(photocellReading);

  // When kissed or hugged (light sensor), wiggle several times and stop.
  if ( kissed_flag > 0 )
  {
    while ( kissed_flag > 4 )
    {
      wiggle_ears();
      kissed_flag++;
    }
    
    reset_ears();
  }

  // When it gets bright or dark, make a sound.
  // Stop playing the sound until it's normal lighting out again.
  if ( photocellReading > 800 && photocellReading < 950 )
  {
    normal_light_flag = 1;
  }

  if ( photocellReading < 300 && normal_light_flag ) //it is dark
  {
    tone(speakerPin, tones[0]);//play mid C
    delay(500);
    noTone(speakerPin);
    normal_light_flag = 0;
  }

  if ( photocellReading > 950 && normal_light_flag ) // it is bright
  {
    tone(speakerPin, tones[9]); //play A
    delay(500);
    noTone(speakerPin);
    normal_light_flag = 0;
  }
  
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  
  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if ( buttonState == HIGH )
  {
    digitalWrite(ledPin, HIGH);
    hug();
    reset_arms();
  }
  else
  {
    digitalWrite(ledPin, LOW);
  }
  
  // check if manually moved
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
  // Sets left_arm and right_arm at their min and max values.
  pwm.setPWM(left_arm, 0, LEFT_ARM_OPEN);
  pwm.setPWM(right_arm, 0, RIGHT_ARM_OPEN);
}

void reset_ears()
{
  // Sets left_ear and right_ear at the middle.
  pwm.setPWM(left_ear, 0, MIDDLE_EAR);
  pwm.setPWM(right_ear, 0, MIDDLE_EAR);
}
