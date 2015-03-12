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
//TMP36 Pin Variables
//int sensorPin = 0; // the analog pin the TMP36's Vout (sense) pin is connected to
// the resolution is 10 mV / degree centigrade with a
// 500 mV offset to allow for negative temperatures
// This code assumes the temperature sensor is plugged into A0, 5V, and GND. Looking at the flat side of the temperature sensor, the left pin is power, middle is analog input, and right is GND.
//void setup()
//{
//  Serial.begin(9600);  // Start the serial connection with the computer to view the result open the serial monitor.
//}
//
//void loop()
//{
//  //getting the voltage reading from the temperature sensor
//  int reading = analogRead(sensorPin);  
//  
//  // converting that reading to voltage, for 3.3v arduino use 3.3
//  float voltage = reading * 5.0;
//  voltage /= 1024.0; 
//  
//  // print out the voltage
//  Serial.print(voltage); 
//  Serial.println(" volts");
//  
//  // now print out the temperature
//  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree with 500 mV offset
//  //to degrees ((voltage - 500mV) times 100)
//  Serial.print(temperatureC); 
//  Serial.println(" degrees C");
//  
//  // now convert to Fahrenheit
//  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;
//  Serial.print(temperatureF); 
//  Serial.println(" degrees F");
//  
//  delay(1000);                                     // waiting 1 second
//}





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

#define LEFT_ARM_OPEN    266
#define LEFT_ARM_CLOSED  372
#define RIGHT_ARM_CLOSED 372
#define RIGHT_ARM_OPEN   478

#define LEFT_EAR_BACK 475
#define LEFT_EAR_FORWARD 265
#define RIGHT_EAR_BACK 265
#define RIGHT_EAR_FORWARD 475
#define MIDDLE_EAR 370

uint8_t left_arm = 0;
uint8_t right_arm = 1;
uint8_t left_ear = 2;
uint8_t right_ear = 3;

int flag = 0;
int kissed_flag = 1;

void setup()
{
  // Initialize serial port.
  Serial.begin(9600);

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  
  // Sets left_arm and right_arm at their min and max values.
//  pwm.setPWM(left_arm, 0, LEFT_ARM_OPEN);
//  delay(1000);
//  pwm.setPWM(right_arm, 0, RIGHT_ARM_OPEN);
//  delay(1000);
  
  // Sets left_arm and right_arm at their min and max values.
  pwm.setPWM(left_ear, 0, MIDDLE_EAR);
//  delay(1000);
  pwm.setPWM(right_ear, 0, MIDDLE_EAR);
//  delay(1000);
}

void loop()
{
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
  if ( flag )
  {
    hug();
    flag = 0;
  }
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
  // Bring arms in together.
  uint16_t pulselen_right = RIGHT_ARM_OPEN;
  for (uint16_t pulselen_left = LEFT_ARM_OPEN; pulselen_left < LEFT_ARM_CLOSED && pulselen_right > RIGHT_ARM_CLOSED; pulselen_left++) {
    pwm.setPWM(left_arm, 0, pulselen_left);
    pwm.setPWM(right_arm, 0, pulselen_right);
    pulselen_right--;
    delay(1);
  }
  delay(1000);
  
  // Release arms.
  uint16_t pulselen_left = LEFT_ARM_CLOSED;
  for (uint16_t pulselen_right = RIGHT_ARM_CLOSED; pulselen_right < RIGHT_ARM_OPEN && pulselen_left > LEFT_ARM_OPEN; pulselen_right++) {
    pwm.setPWM(right_arm, 0, pulselen_right);
    pwm.setPWM(left_arm, 0, pulselen_left);
    pulselen_left--;
    delay(1);
  }
  delay(1000);
}
