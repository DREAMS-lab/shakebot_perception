from binascii import hexlify
from numpy import empty
import serial

SerialObj = serial.Serial("/dev/ttyUSB0")

SerialObj.baudrate = 9600  # set Baud rate to 9600
SerialObj.bytesize = 8     # Number of data bits = 8
SerialObj.parity   ='N'    # No parity
SerialObj.stopbits = 1     # Number of Stop bits = 1
SerialObj.timeout = 3

print("Reading data.......")
ReceivedString = SerialObj.read(8)
if ReceivedString is not empty:
    print(int(str(ReceivedString, 'utf-8'),16))
else:
    print("Nothing received")