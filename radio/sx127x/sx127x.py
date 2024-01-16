
import os
import time, platform
if "arm" in platform.machine():
   import RPi.GPIO as GPIO
else:
   from gpioStub import stubGPIO as GPIO
# -- -- -- --
from .sxBase import sxBase
from .loraSPI import loraSPI
from .sx127xRegs import sx127xRegs as regs
from .sx127xConsts import sx127xConsts as consts


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


class sx127x(sxBase):

   def __init__(self, spi: loraSPI, rst_pin: int, cs_pin: int):
      self.spi: loraSPI = spi
      self.rst_pin: int = rst_pin
      self.cs_pin: int = cs_pin
      self._modem: int = 0
      GPIO.setup([self.rst_pin, self.cs_pin], GPIO.OUT)

   def init(self):
      self.reset()

   def begin(self) -> bool:
      try:
         self.reset()
         self.setModem(consts.LORA_MODEM)
         self.setTxPower(17, consts.TX_POWER_PA_BOOST)
         self.setRxGain(consts.RX_GAIN_BOOSTED, consts.RX_GAIN_AUTO)
         return True
      except Exception as e:
         print(e)
         return False
      finally:
         pass

   def reset(self):
      GPIO.output(self.rst_pin, GPIO.LOW)
      time.sleep(0.004)
      GPIO.output(self.rst_pin, GPIO.HIGH)
      time.sleep(0.0016)

   def chip_ver(self):
      ver = 0x00
      t = time.time()
      while ver != 0x12 and ver != 0x22:
         ver = self.readRegister(regs.REG_VERSION)
         if time.time() - t > 8:
            return False
         print(f"[ ver: {ver} | hex: 0x{ver:02X} ]")
      return True

   def setModem(self, modem: int):
      print("[ setModem ]")
      if modem != consts.LORA_MODEM:
         raise f"BadModemSetting: {modem}"
      self._modem = consts.LONG_RANGE_MODE
      time.sleep(0.002)
      val = self.writeRegister(regs.REG_OP_MODE, self._modem | consts.MODE_STDBY)
      print(val)

   def setTxPower(self, txPower: int, paPin: int):
      # maximum TX power is 20 dBm and 14 dBm for RFO pin
      if txPower > 20:
         txPower = 20
      elif txPower > 14 and paPin == consts.TX_POWER_RFO:
         txPower = 14
      # -- -- -- --
      paConfig = 0x00
      outputPower = 0x00
      if paPin == consts.TX_POWER_RFO:
         # txPower = Pmax - (15 - outputPower)
         if txPower == 14:
            # max power (Pmax) 14.4 dBm
            paConfig = 0x60
            outputPower = txPower + 1
         else:
            # max power (Pmax) 13.2 dBm
            paConfig = 0x40
            outputPower = txPower + 2
      else:
         paDac = 0x04
         paConfig = 0xC0
         # txPower = 17 - (15 - outputPower)
         if txPower > 17:
            outputPower = 15
            paDac = 0x07
            self.setCurrentProtection(100)  # max current 100 mA
         else:
            if txPower < 2: txPower = 2
            outputPower = txPower - 2
            self.setCurrentProtection(140)  # max current 140 mA
         # enable or disable +20 dBm option on PA_BOOST pin
         self.writeRegister(regs.REG_PA_DAC, paDac)
      # set PA config
      self.writeRegister(regs.REG_PA_CONFIG, paConfig | outputPower)

   def writeRegister(self, address: int, data: int) -> int:
      return self._transfer(address | 0x80, data)

   def readRegister(self, address: int) -> int:
      return self._transfer(address & 0x7F, 0x00)

   def _transfer(self, address: int, data: int) -> int:
      buff_arr = [address, data]
      print(f"[ spi sending: {buff_arr} ]")
      self.__set_cs(GPIO.LOW)
      rval: () = self.spi.xtfr2(buff_arr)
      self.__set_cs(GPIO.HIGH)
      print(f"[ rval: {rval}]")
      if len(rval) == 2:
         return int(rval[1])
      return -1

   def __set_cs(self, val: bool):
      GPIO.output(self.cs_pin, val)

   def end(self):
      pass

   def beginPacket(self):
      pass

   def endPacket(self, timeout: int) -> bool:
      pass

   def write(self, data, length: int):
      pass

   def request(self, timeout: int) -> bool:
      pass

   def available(self):
      pass

   def read(self, length: int):
      pass

   def wait(self, timeout: int) -> bool:
      pass

   def status(self):
      pass

   def setCurrentProtection(self, current: int):
      # calculate ocp trim
      ocpTrim = 27
      if current <= 120:
         ocpTrim = int((current - 45) / 5)
      elif current <= 240:
         ocpTrim = int((current + 30) / 10)
      # set overcurrent protection config
      self.writeRegister(regs.REG_OCP, 0x20 | ocpTrim)

   def setOscillator(self, option: int):
      cfg = consts.OSC_CRYSTAL
      if option == consts.OSC_TCXO:
         cfg = consts.OSC_TCXO
      self.writeRegister(regs.REG_TCXO, cfg)

   def setRxGain(self, boost: int, level: int):
      # valid RX gain level 0 - 6 (0 -> AGC on)
      level = 6 if level > 6 else level
      # boost LNA and automatic gain controller config
      LnaBoostHf = 0x00
      if boost: LnaBoostHf = 0x03
      AgcOn = 0x00
      if level == consts.RX_GAIN_AUTO:
         AgcOn = 0x01
      # set gain and boost LNA config
      self.writeRegister(regs.REG_LNA, LnaBoostHf | (level << 5))
      # enable or disable AGC
      self.writeBits(regs.REG_MODEM_CONFIG_3, AgcOn, 2, 1)

   def writeBits(self, address: int, data: int, position: int, length: int):
      read = self._transfer(address & 0x7F, 0x00)
      mask = (0xFF >> (8 - length)) << position
      write = (data << position) | (read & ~mask)
      self._transfer(address | 0x80, write)
