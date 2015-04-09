#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "pitches.h"

// Wiring information can be found in the project's README: https://github.com/cgoettel/frankenbear/blob/master/README.md

#define DEBUG 0

// **************************
// **** PROXIMITY SENSOR ****
// **************************
// i2c address
#define VCNL4000_ADDRESS 0x13

// commands and constants
#define VCNL4000_COMMAND 0x80
#define VCNL4000_PRODUCTID 0x81
#define VCNL4000_IRLED 0x83
#define VCNL4000_AMBIENTPARAMETER 0x84
#define VCNL4000_AMBIENTDATA 0x85
#define VCNL4000_PROXIMITYDATA 0x87
#define VCNL4000_SIGNALFREQ 0x89
#define VCNL4000_PROXIMITYADJUST 0x8A

#define VCNL4000_3M125 0
#define VCNL4000_1M5625 1
#define VCNL4000_781K25 2
#define VCNL4000_390K625 3

#define VCNL4000_MEASUREAMBIENT 0x10
#define VCNL4000_MEASUREPROXIMITY 0x08
#define VCNL4000_AMBIENTREADY 0x40
#define VCNL4000_PROXIMITYREADY 0x20

uint16_t ambient_reading = 0;
uint16_t proximity_reading = 0;

int normal_proximity_flag = 0;

// **********************
// **** SERVO SHIELD ****
// **********************
// Uses the default address 0x40.
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// These are the minimum and maximum pulse length counts (out of 4096).
#define BIG_SERVO_MIN   160
#define BIG_SERVO_MAX   584
#define MICRO_SERVO_MIN 160
#define MICRO_SERVO_MAX 584

#define LEFT_ARM_OPEN    372
#define LEFT_ARM_CLOSED  266
#define RIGHT_ARM_CLOSED 478
#define RIGHT_ARM_OPEN   372

#define LEFT_EAR_BACK     475
#define LEFT_EAR_FORWARD  265
#define RIGHT_EAR_BACK    265
#define RIGHT_EAR_FORWARD 475
#define MIDDLE_EAR        370

const uint8_t left_arm  = 0;
const uint8_t right_arm = 1;
const uint8_t left_ear  = 2;
const uint8_t right_ear = 3;

// *******************
// **** PHOTOCELL ****
// *******************
int normal_light_flag = 0;

const int photocell_pin = A5;
int photocell_reading;

// ****************
// **** BUTTON ****
// ****************
const int button_pin = 2;
int button_state = 0;

// *****************
// **** SPEAKER ****
// *****************
const int speakerPin = 12;
int mary_had_a_little_lamb_notes[] = {
  NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_D4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_E4, NOTE_D4, NOTE_C4
};
int mary_had_a_little_lamb_lengths[] = {
  4, 4, 4, 4, 4, 4, 2, 4, 4, 2, 4, 4, 2, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 2
};

int twinkle_twinkle_notes[] = {
  NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4
};
int twinkle_twinkle_lengths[] = {
  4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2, 4, 4, 4, 4, 4, 4, 2
};

void setup()
{
  // Initialize serial port at 9600 baud.
  Serial.begin(9600);
  
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz.
  
  // Check if proximity sensor is connected.
  uint8_t rev = read8(VCNL4000_PRODUCTID);
  if ((rev & 0xF0) != 0x10) {
    Serial.println("Sensor not found :(");
    while (1);
  }
  
  write8(VCNL4000_IRLED, 20);        // set to 20 * 10mA = 200mA
#if DEBUG
  Serial.println("VCNL");
  Serial.print("IR LED current = ");
  Serial.print(read8(VCNL4000_IRLED) * 10, DEC);
  Serial.println(" mA");
  Serial.print("Proximity measurement frequency = ");
#endif
  uint8_t freq = read8(VCNL4000_SIGNALFREQ);
#if DEBUG
  if (freq == VCNL4000_3M125) Serial.println("3.125 MHz");
  if (freq == VCNL4000_1M5625) Serial.println("1.5625 MHz");
  if (freq == VCNL4000_781K25) Serial.println("781.25 KHz");
  if (freq == VCNL4000_390K625) Serial.println("390.625 KHz");
#endif
  
  write8(VCNL4000_PROXIMITYADJUST, 0x81);
#if DEBUG
  Serial.print("Proximity adjustment register = ");
  Serial.println(read8(VCNL4000_PROXIMITYADJUST), HEX);
#endif
  
  // Initialize the pushbutton pin as an input.
  pinMode(button_pin, INPUT);
  
  // Put arms and ears at normal position and wait for that to happen.
  reset_arms();
  reset_ears();
  delay(1000);
}

void loop()
{
  // PHOTOCELL
  // When it gets bright or dark, make a sound.
  // Don't play the sound again until it's normal lighting out.
  photocell_reading = analogRead(photocell_pin);
  
  if ( photocell_reading >= 700 && photocell_reading <= 710 )
  {
    normal_light_flag = 1;
  }
  
  if ( photocell_reading < 650 && normal_light_flag )
  {
    play_song(0);
    normal_light_flag = 0;
  }
  
  // BUTTON
  button_state = digitalRead(button_pin);
  
  if ( button_state == HIGH )
  {
    hug();
    reset_arms();
  }
  
  // PROXIMITY SENSOR
  // read ambient light
  write8(VCNL4000_COMMAND, VCNL4000_MEASUREAMBIENT | VCNL4000_MEASUREPROXIMITY);
  
  // read proximity sensor
  while (1)
  {
    uint8_t result = read8(VCNL4000_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if ( (result & VCNL4000_AMBIENTREADY) && (result & VCNL4000_PROXIMITYREADY) )
    {
      ambient_reading = read16(VCNL4000_AMBIENTDATA);
      proximity_reading = read16(VCNL4000_PROXIMITYDATA);
      
#if DEBUG == 1
      Serial.print("Analog reading = ");
      Serial.print(photocell_reading);
      Serial.print("\tAmbient = ");
      Serial.print(ambient_reading);
      Serial.print("\tProximity = ");
      Serial.println(proximity_reading);
#endif
      break;
    }
    
    delay(10);
  }
  
  // Don't wiggle again until it's normal proximity again.
  if ( ambient_reading > 45 )
  {
    normal_proximity_flag = 1;
  }
  
  if ( ambient_reading < 30 && normal_proximity_flag ) // proximity_reading doesn't work well throu gh fabric.
  {
    for ( int i = 0; i < 4; i++ )
    {
      wiggle_ears();
    }
    
    reset_ears();
    normal_proximity_flag = 0;
  }
  
  if ( Serial.available() > 0 )
  {
    // Read the incoming byte.
    int incoming_byte = Serial.read();
    incoming_byte -= 48;
    
    play_song(incoming_byte);
    
    // Echo it back to serial.
#if DEGBUG   
    Serial.println(incoming_byte, DEC);
#endif
    
    incoming_byte = -1;
  }
  
  // TODO: check if manually moved
  // is this as simple as just resetting the ears and arms after every loop?
  // This would work, I think, but could also draw a lot more power than otherwise.
  
  // TODO: Change clock speed to save battery.
}

uint16_t readProximity() {
  write8(VCNL4000_COMMAND, VCNL4000_MEASUREPROXIMITY);
  while (1) {
    uint8_t result = read8(VCNL4000_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & VCNL4000_PROXIMITYREADY) {
      return read16(VCNL4000_PROXIMITYDATA);
    }
    delay(1);
  }
}

// Read 1 byte from the VCNL4000 at 'address'
uint8_t read8(uint8_t address)
{
  uint8_t data;

  Wire.beginTransmission(VCNL4000_ADDRESS);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();

  delayMicroseconds(170);  // delay required

  Wire.requestFrom(VCNL4000_ADDRESS, 1);
  while(!Wire.available());

#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}


// Read 2 bytes from the VCNL4000 at 'address'
uint16_t read16(uint8_t address)
{
  uint16_t data;

  Wire.beginTransmission(VCNL4000_ADDRESS);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();

  Wire.requestFrom(VCNL4000_ADDRESS, 2);
  while(!Wire.available());
#if ARDUINO >= 100
  data = Wire.read();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.read();
#else
  data = Wire.receive();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.receive();
#endif
  
  return data;
}

// write 1 byte
void write8(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(VCNL4000_ADDRESS);
#if ARDUINO >= 100
  Wire.write(address);
  Wire.write(data);  
#else
  Wire.send(address);
  Wire.send(data);  
#endif
  Wire.endTransmission();
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
  if ( song_number < 0 )
  {
    return;
  }
  
  int song_length = 0;
  if ( song_number == 0 )
  {
    song_length = sizeof(mary_had_a_little_lamb_lengths)/sizeof(mary_had_a_little_lamb_lengths[0]);
  }
  else if ( song_number == 1 )
  {
    song_length = sizeof(twinkle_twinkle_lengths)/sizeof(twinkle_twinkle_lengths[0]);
  }
  
  int note_length = 0;
  
  for ( int i = 0; i < song_length; i++ )
  {
    if ( song_number == 0 || song_number > 1 )
    {
      note_length = 1000 / mary_had_a_little_lamb_lengths[i];
      tone(speakerPin, mary_had_a_little_lamb_notes[i], note_length);
    }
    else if ( song_number == 1 )
    {
      note_length = 1000 / twinkle_twinkle_lengths[i];
      tone(speakerPin, twinkle_twinkle_notes[i], note_length);
    }
    
    int pause_between_notes = note_length * 1.30;
    delay(pause_between_notes);
    noTone(speakerPin);
  }
}
