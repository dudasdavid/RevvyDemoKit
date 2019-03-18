# Demo of revvy BLE peripheral using python port of bleno, pybleno
#
# Setup:
# sudo setcap 'cap_net_raw,cap_net_admin+eip' $(readlink -f $(which python3))
# # Enables python3 to open raw sockets. Required by bleno to talk to BT via HCI

import pybleno
import sys
import serial
import urllib.parse
import json

# constants
HM_10_UART_SERIVCE = '0000FFE0-0000-1000-8000-00805F9B34FB'.replace("-", "")
HM_10_UART_CHARACTERISTIC = '0000FFE1-0000-1000-8000-00805F9B34FB'.replace("-", "")

class UartCharacteristic(pybleno.Characteristic):
    def __init__(self, port):
        pybleno.Characteristic.__init__(self, {
            'uuid': HM_10_UART_CHARACTERISTIC,
            'properties': ['write', 'write-without-response', 'notify'],
            'value': None,
          })
        self._port = port
        self._rawData = bytearray()
        self._blockyList = []
        self._value = bytearray()
        self._updateValueCallback = None
    def onWriteRequest(self, data, offset, withoutResponse, callback):
      print(repr(data))  # DEBUG
      head = data[0]
      if head == 0xff:
        self._port.write(bytes(data))
      elif head == 0xfe:
        self.readSyncedPacket(data[1:])
      else:
        # TODO warning/error
        print("Error: Unknown header")
      callback(pybleno.Characteristic.RESULT_SUCCESS)
    def readSyncedPacket(self, data):
      isFinalPacket = bool(data[0])
      self._rawData += data[1:]
      if isFinalPacket:
        decoded = urllib.parse.unquote(self._rawData.decode("utf-8"))  # TODO: Is encoding correct?
        try:
          self._blocklyList = json.loads(decoded)
          self._rawData = bytearray()
          print(repr(self._blocklyList))
        except json.decoder.JSONDecodeError:
          print("Error: Invalid JSON payload")

class UartService(pybleno.BlenoPrimaryService):
    def __init__(self, port):
        pybleno.BlenoPrimaryService.__init__(self, {
          'uuid': HM_10_UART_SERIVCE,
          'characteristics': [
              UartCharacteristic(port)
          ]})

ser = serial.Serial(
  port='/dev/ttyS0',
  baudrate = 115200,
  parity=serial.PARITY_NONE,
  stopbits=serial.STOPBITS_ONE,
  bytesize=serial.EIGHTBITS,
  timeout=1
)

def main():
  uartService = UartService(ser)
  serviceName = "Mucsacsa"
  bleno = pybleno.Bleno()

  def onStateChange(state):
      if (state == 'poweredOn'):
          def on_startAdvertising(err):
              if err:
                  print(err)

          bleno.startAdvertising(serviceName, [uartService.uuid], on_startAdvertising)
      else:
          bleno.stopAdvertising()
  bleno.on('stateChange', onStateChange)
      
  def onAdvertisingStart(error):
      if not error:
          print('advertising...')
          bleno.setServices([
              uartService
          ])
  bleno.on('advertisingStart', onAdvertisingStart)

  bleno.start()

  print ('Hit <ENTER> to disconnect')

  if (sys.version_info > (3, 0)):
      input()
  else:
      raw_input()

  bleno.stopAdvertising()
  bleno.disconnect()

  print ('terminated.')
  sys.exit(1)

if __name__ == "__main__":
  main()
