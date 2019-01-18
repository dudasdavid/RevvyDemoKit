import serial
import time

def sendCommand(command):
    global ser
    ser.write(b"%s\r\n" % command.encode('ascii','ignore'))
    time.sleep(1)

ser = serial.Serial(
  port='/dev/ttyS0',
  baudrate = 115200,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)

ser.close()
ser.open()
ser.isOpen()

command = "Kaki"
sendCommand(command)