import time
import serial
import sys

MILISECOND = 0.001
SECOND = 1

print('Initiating UART...')
ser = serial.Serial(            
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
print('UART initiated!')


##   mySerial.write(0x5A);
##   delay(1);
##   mySerial.write(1);
##   delay(200);
##  
##   mySerial.write(0x20);
##   delay(1);
##   mySerial.write(100);
##   delay(200);  
##
##   mySerial.write(0x39);
##   delay(1);
##   mySerial.write(200);
##   delay(2000);  
##
##   mySerial.write(0x35);
##   delay(1);
##   mySerial.write(100);
##   delay(2000);       

def stop():
    print('Stop')
    
    ser.write([0x39])
    time.sleep(MILISECOND)
    ser.write([0])
    time.sleep(MILISECOND)

def fwd(speed=100):
    if speed < 0:
        speed = 0
    if speed > 255:
        speed = 255
    print('FWD: {}'.format(speed))
    ser.write([0x35])
    time.sleep(MILISECOND)
    ser.write([speed])
    time.sleep(5 * MILISECOND)

def bkwd(speed=100):
    if speed < 0:
        speed = 0
    if speed > 255:
        speed = 255
    print('FWD: {}'.format(speed))
    ser.write([0x39])
    time.sleep(MILISECOND)
    ser.write([speed])
    time.sleep(5 * MILISECOND)
    

def send_test_data():
    print('Sending data...')
    ser.write([0x5A])
    time.sleep(MILISECOND)
    ser.write([1])
    time.sleep(200 * MILISECOND)
    
    ser.write([0x20])
    time.sleep(MILISECOND)
    ser.write([100])
    time.sleep(200 * MILISECOND)
    
    ser.write([0x39])
    time.sleep(MILISECOND)
    ser.write([200])
    time.sleep(2 * SECOND)
    
    ser.write([0x35])
    time.sleep(MILISECOND)
    ser.write([100])
    print('Data sent!')
    time.sleep(5 * SECOND)

