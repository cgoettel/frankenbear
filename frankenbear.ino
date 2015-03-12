// blocking out ideas and code that needs to be written

// wiggle ears and play sound when button in hand (foot?) is pressed:
//     detect button click (debounce?)
//     signal for servos in ears to oscillate
//     oscillate for three(?) cycles

// accelerometer(?): when picked up (how to detect just being picked up? change in only y-axis?)
//     have bear say hello (or whatever)
//     wiggle ears?

// ****************************
// **** TEMPERATURE SENSOR ****
// ****************************
// hug when kissed or hugged
//     Detect change in relative temperature
//     Wiggle ears.
// NOTE: I fear that this isn't going to work. We're going to probably have to use an IR proximity sensor.






/*************************************************** 
 * This is an example for our Adafruit 16-channel PWM & Servo driver
 * Servo test - this will drive 16 servos, one after the other
 * 
 * Pick one up today in the adafruit shop!
 * ------> http://www.adafruit.com/products/815
 * 
 * These displays use I2C to communicate, 2 pins are required to  
 * interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4
 * 
 * Adafruit invests time and resources providing this open source code, 
 * please support Adafruit and open-source hardware by purchasing 
 * products from Adafruit!
 * 
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 * BSD license, all text above must be included in any redistribution
 ****************************************************/

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

int hug_flag = 0;
int kissed_flag = 0;
int test_flag = 0;

int normal_light_flag = 0;

int photocellPin = 0;     // the cell and 10K pulldown are connected to a0
int photocellReading;     // the analog reading from the sensor divider
int LEDpin = 11;          // connect Red LED to pin 11 (PWM pin)
int LEDbrightness;

int speakerPin = 12;
int numTones = 10;
int tones[] = {261, 277, 294, 311, 330, 349, 370, 392, 415, 440};
//            mid C  C#   D    D#   E    F    F#   G    G#   A

void setup()
{
  // Initialize serial port.
  Serial.begin(9600);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // Sets left_arm and right_arm at their min and max values.
  pwm.setPWM(left_arm, 0, LEFT_ARM_OPEN);
  pwm.setPWM(right_arm, 0, RIGHT_ARM_OPEN);
  
  // Sets left_arm and right_arm at their min and max values.
  pwm.setPWM(left_ear, 0, MIDDLE_EAR);
  pwm.setPWM(right_ear, 0, MIDDLE_EAR);
  delay(1000);
}

void loop()
{
  photocellReading = analogRead(photocellPin);
  Serial.print("Analog reading = ");
  Serial.println(photocellReading);     // the raw analog reading
  // invert the reading from 0-1023 back to 1023-0
//  photocellReading = 1023 - photocellReading;
  
  // When kissed or hugged (proximity sensor)
  if ( kissed_flag > 0 )
  {
    wiggle_ears();
    kissed_flag++;
    
    if ( kissed_flag > 4 )
    {
      kissed_flag = 0;
    }
  }
  
  // On button press?
  if ( hug_flag )
  {
    hug();
    hug_flag = 0;
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
  
  if ( photocellReading > 950 && normal_light_flag ) //it is bright
  {
    tone(speakerPin, tones[9]); //play A
    delay(500);
    noTone(speakerPin);
    normal_light_flag = 0;
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
