
import os
import time, platform
if "arm" in platform.machine():
   import RPi.GPIO as GPIO
else:
   from gpioStub import stubGPIO as GPIO
from loraSPI import loraSPI
from sx127xRegs import sx127xRegs as regs
from sx127xConsts import sx127xConsts as consts


class sx127x(object):

   def __init__(self, spi: loraSPI, rst_pin: int):
      self.spi: loraSPI = spi
      self.rst_pin: int = rst_pin
      self._modem: int = 0

   def init(self):
      GPIO.setwarnings(False)
      GPIO.setmode(GPIO.BCM)
      GPIO.setup(self.rst_pin, GPIO.OUT)
      GPIO.setup(self.spi.cs_pin, GPIO.OUT)
      self.reset()
      self.setModem(consts.LORA_MODEM)

   def reset(self):
      GPIO.output(self.rst_pin, GPIO.LOW)
      time.sleep(0.001)
      GPIO.output(self.rst_pin, GPIO.HIGH)
      time.sleep(0.005)
      t = time.time()
      ver = 0x00
      while ver != 0x12 and ver != 0x22:
         ver = self.readRegister(regs.REG_VERSION)
         if time.time() - t > 4:
            return False
         print(f"[ ver: {ver}]")
      return True

   def setModem(self, modem: int):
      if modem != consts.LORA_MODEM:
         raise f"BadModemSetting: {modem}"
      self._modem = consts.LONG_RANGE_MODE
      time.sleep(0.001)
      self.writeRegister(regs.REG_OP_MODE, self._modem | consts.MODE_STDBY)

   def writeRegister(self, address: int, data: int):
      self._transfer(address | 0x80, data)

   def readRegister(self, address: int) -> int:
      return self._transfer(address & 0x7F, 0x00)

   def _transfer(self, address: int, data: int) -> int:
      buf = [address, data]
      self.__set_cs(GPIO.LOW)
      rval = self.spi.transfer(buf)
      print(f"[ rval: {rval}]")
      self.__set_cs(GPIO.HIGH)
      if len(rval) == 2:
         return int(rval[1])
      return -1

   def __set_cs(self, val: bool):
      GPIO.output(self.spi.cs_pin, val)
