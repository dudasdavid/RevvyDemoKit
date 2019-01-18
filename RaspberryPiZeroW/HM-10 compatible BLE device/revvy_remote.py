from gi.repository import GLib
import time

import serial

# Bluezero modules
from bluezero import peripheral

# constants
HM_10_UART_SERIVCE = '0000FFE0-0000-1000-8000-00805F9B34FB'
HM_10_UART_CHARACTERISTIC = '0000FFE1-0000-1000-8000-00805F9B34FB'

# connected = False

ser = serial.Serial(
  port='/dev/ttyS0',
  baudrate = 115200,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)

def sendCommand(command):
    global ser
    # ser.write(b"kaka\n")
    ser.write(command)
    # time.sleep(1)

bleTransferBytes = bytearray([0xFF,0x01,0x02,0x03,0xFF])
newMessage = False
    
class UartService:
    def __init__(self):
        self.app = peripheral.Application()
        self.ble_uart = peripheral.Service(HM_10_UART_SERIVCE,
                                           True)
        self.rxtx_uart = peripheral.Characteristic(HM_10_UART_CHARACTERISTIC,
                                                 ['write',
                                                  'write-without-response',
                                                  'notify'],
                                                 self.ble_uart)

        self.rxtx_uart.add_write_event(self.uart_print)
        self.rxtx_uart.add_notify_event(print)
        self.rxtx_uart.StartNotify()
        self.ble_uart.add_characteristic(self.rxtx_uart)
        self.app.add_service(self.ble_uart)

        self.other_service = peripheral.Service('0000180A-0000-1000-8000-00805F9B34FB', True)
        self.app.add_service(self.other_service)

    @staticmethod
    def _to_bytes(word):
        return [ord(x) for x in word]

    def uart_print(self, value):
        global bleTransferBytes
        global newMessage
        # print(value)
        # print("%d;%d;%d;%d;%d" % (int(value[0]),int(value[1]),int(value[2]),int(value[3]),int(value[4])))
        bleTransferBytes[0] = value[0]
        bleTransferBytes[1] = value[1]
        bleTransferBytes[2] = value[2]
        bleTransferBytes[3] = value[3]
        bleTransferBytes[4] = value[4]

        newMessage = True
        # if (connected == False):
            # print('connected')
            # connected = True

    def start(self):
        self.app.start()

    def stop(self):
        self.app.stop()


def sender(dev_loop):
    global bleTransferBytes
    global newMessage
    # time.sleep(1)
    # if (connected):
        # dev_loop.rxtx_uart.send_notify_event('buzi')
    # print("Message sent")
    # return True
    if (newMessage == True):
        # print(bleTransferBytes)
        sendCommand(bleTransferBytes)
        newMessage = False
        
    # time.sleep(1)
    # sendCommand(bleTransferBytes)
    return True



if __name__ == '__main__':
    # dev = InputDevice('/dev/input/event0')
    ser.close()
    ser.open()
    ser.isOpen()
    
    # while (1):
        # sendCommand("")
        # time.sleep(1)
    
    uart = UartService()
    uart.app.dongle.alias = 'Revvy_RPi'
    # uart.app.dongle.tx_power = 10
    # uart.app.dongle.icon = "folder-new"
    # print(uart.app.dongle.name)
    # print("***")
    GLib.idle_add(sender, uart)
    uart.start()

