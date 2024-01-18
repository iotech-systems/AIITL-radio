
import time
from .sx127xRegs import sx127xRegs
from .sx127xConsts import sx127xConsts as xc
from .sx127xRecOps import sx127xRecOps


class sx127xConfOps(object):

   def __init__(self, regs: sx127xRegs):
      self.regs: sx127xRegs = regs
      self._modem: int = xc.LORA_MODEM

   def init(self):
      if self.regs is None:
         raise ValueError("regs_is_none")

   def setCurrentProtection(self, current: int):
      # calculate ocp trim
      ocpTrim = 27
      if current <= 120:
         ocpTrim = int((current - 45) / 5)
      elif current <= 240:
         ocpTrim = int((current + 30) / 10)
      # set over-current protection config
      self.regs.writeReg(self.regs.REG_OCP, 0x20 | ocpTrim)

   def setOscillator(self, option: int):
      cfg = xc.OSC_CRYSTAL
      if option == xc.OSC_TCXO:
         cfg = xc.OSC_TCXO
      self.regs.writeReg(self.regs.REG_TCXO, cfg)

   def setModem(self, modem: int) -> bool:
      print("[ setModem ]")
      if modem != xc.LORA_MODEM:
         raise f"BadModemSetting: {modem}"
      self._modem = xc.LONG_RANGE_MODE
      time.sleep(0.002)
      self.regs.writeReg(self.regs.REG_OP_MODE, self._modem | xc.MODE_STDBY)
      return True

   # maximum TX power is 20 dBm and 14 dBm for RFO pin
   def setTxPower(self, txPower: int, paPin: int):
      max_power: int = 14 if paPin == xc.TX_POWER_RFO else 20
      txPower = max_power if txPower > max_power else txPower
      # -- -- -- --
      # paConfig = 0x00; outputPower = 0x00
      if paPin == xc.TX_POWER_RFO:
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
         paDac = 0x04; paConfig = 0xC0
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
         self.regs.writeReg(self.regs.REG_PA_DAC, paDac)
      # -- set PA config --
      self.regs.writeReg(self.regs.REG_PA_CONFIG, paConfig | outputPower)
