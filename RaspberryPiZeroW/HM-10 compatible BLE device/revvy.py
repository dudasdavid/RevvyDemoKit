from gi.repository import GLib
from evdev import InputDevice, categorize, ecodes
import time

# Bluezero modules
from bluezero import peripheral

# constants
HM_10_UART_SERIVCE = '0000FFE0-0000-1000-8000-00805F9B34FB'
HM_10_UART_CHARACTERISTIC = '0000FFE1-0000-1000-8000-00805F9B34FB'

connected = False

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

    @staticmethod
    def _to_bytes(word):
        return [ord(x) for x in word]

    def uart_print(self, value):
        global connected
        print("%d;%d;%d;%d;%d" % (int(value[0]),int(value[1]),int(value[2]),int(value[3]),int(value[4])))
        # if (connected == False):
            # print('connected')
            # connected = True

    def start(self):
        self.app.start()

    def stop(self):
        self.app.stop()


def sender(dev_loop):
    global connected
    time.sleep(1)
    if (connected):
        dev_loop.rxtx_uart.send_notify_event('buzi')
    # print("Message sent")
    return True


if __name__ == '__main__':
    dev = InputDevice('/dev/input/event0')
    uart = UartService()
    uart.app.dongle.alias = 'Revvy_RPi'
    # GLib.idle_add(sender, uart)
    uart.start()

