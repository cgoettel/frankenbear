// blocking out ideas and code that needs to be written

// wiggle ears and play sound when button in hand (foot?) is pressed:
//     detect button click (debounce?)
//     signal for servos in ears to oscillate
//     oscillate for three(?) cycles

// hug when kissed or hugged
//     Detect change in relative temperature
//     have servos in arms bend in, hold for a couple seconds, release
//     need to check if arm has been manually bent already and don't hug if so (will this already be taken care of with the PWM signal sent?)
//     
//     

// accelerometer(?): when picked up (how to detect just being picked up? change in only y-axis?)
//     have bear say hello (or whatever)
//     wiggle ears?

/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN 160 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 585 // this is the 'maximum' pulse length count (out of 4096)

// our servo # counter
//uint8_t servonum = 0;
uint8_t left_arm = 0;
uint8_t right_arm = 1;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // Sets left_arm and right_arm at their min and max values.
  pwm.setPWM(left_arm, 0, SERVOMIN);
  pwm.setPWM(right_arm, 0, SERVOMAX);
}

// you can use this function if you'd like to set the pulse length in seconds
// e.g. setServoPulse(0, 0.001) is a ~1 millisecond pulse width. its not precise!
void setServoPulse(uint8_t n, double pulse) {
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() {
  // Drive each servo one at a time
//  Serial.println(servonum);
  uint16_t pulselen_right = SERVOMAX;
  for (uint16_t pulselen_left = SERVOMIN; pulselen_left < SERVOMAX && pulselen_right > SERVOMIN; pulselen_left++) {
    pwm.setPWM(left_arm, 0, pulselen_left);
    pwm.setPWM(right_arm, 0, pulselen_right);
    pulselen_right--;
  }
  delay(500);
  uint16_t pulselen_left = SERVOMAX;
  for (uint16_t pulselen_right = SERVOMIN; pulselen_right < SERVOMAX && pulselen_left > SERVOMIN; pulselen_right++) {
    pwm.setPWM(right_arm, 0, pulselen_right);
    pwm.setPWM(left_arm, 0, pulselen_left);
    pulselen_left--;
  }
  delay(500);
//
//  servonum ++;
//  if (servonum > 3) servonum = 0;
}
