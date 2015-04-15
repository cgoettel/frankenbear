#!/usr/bin/python

import threading, time, serial, wiringpi2

stopFlag = False #Flag to determine when to end program

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=None)
ser.open()
wiringpi2.wiringPiSetup()
led_red = 5
led_yellow = 4
led_green = 3

wiringpi2.pinMode(led_red, 1)
wiringpi2.pinMode(led_yellow, 1)
wiringpi2.pinMode(led_green, 1)

wiringpi2.digitalWrite(led_yellow, 0)

def serialWrite():
  global stopFlag
  while True:
    dataToSend = raw_input("Command>>> ")  
    if dataToSend == "":
      continue      
    ser.write(dataToSend + "\n") 
    print dataToSend 
    ser.flush()   
    if dataToSend == "exit":
      stopFlag = True
      print "Exiting serialWrite"
      return

def serialRead():
  while stopFlag==False:
    dataToRead = ser.readline()
    print dataToRead,
    if dataToRead == "LED_YELLOW":
      wiringpi2.digitalWrite(led_yellow, 1) 
      print "RCVD: "+dataToRead
    ser.flush()
  print "Exiting serialRead"

try:
  print "Usage:"
  print "    0 - play Mary Had A Little Lamb"
  print "    1 - play Twinkle, Twinkle Little Star"
  print "Type 'exit' to close program"
  writeLoop = threading.Thread(target=serialWrite) #create a separate thread for writing to serial port
  readLoop = threading.Thread(target=serialRead) #create a separate thread for reading from serial port
  writeLoop.start()
  readLoop.start()
  writeLoop.join() #wait for this thread to finish
  readLoop.join() #wait for this thread to finish
  print "Exiting main thread"
  ser.close()
    
except KeyboardInterrupt:
  ser.close()
