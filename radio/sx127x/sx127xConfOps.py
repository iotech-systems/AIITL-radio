
import time
from core.utils import utils
from .sx127xRegs import sx127xRegs
from .sx127xConsts import sx127xConsts as xc
from .sx127xRecOps import sx127xRecOps
from .sx127xEnums import sx127xRegEnum


class sx127xConfOps(object):

   def __init__(self, regs: sx127xRegs):
      self.regs: sx127xRegs = regs
      self._modem: int = xc.LORA_MODEM
      self._bw: int = 0
      self._rf_freq: int = 0
      self._headerType: int = 0
      self._payloadLength: int = 0

   def init(self):
      if self.regs is None:
         raise ValueError("regs_is_none")

   """ set calls """

   def setCurrentProtection(self, current: int):
      utils.trace_dbg("[ setCurrentProtection ]")
      # calculate ocp trim
      ocpTrim = 27
      if current <= 120:
         ocpTrim = int((current - 45) / 5)
      elif current <= 240:
         ocpTrim = int((current + 30) / 10)
      # set over-current protection config
      self.regs.set_reg(self.regs.REG_OCP, 0x20 | ocpTrim)

   def setOscillator(self, option: int):
      cfg = xc.OSC_CRYSTAL
      if option == xc.OSC_TCXO:
         cfg = xc.OSC_TCXO
      self.regs.set_reg(self.regs.REG_TCXO, cfg)

   def setModem(self, modem: int) -> bool:
      utils.trace_dbg("[ setModem ]")
      if modem != xc.LORA_MODEM:
         raise f"BadModemSetting: {modem}"
      self._modem = xc.LONG_RANGE_MODE
      time.sleep(0.002)
      self.regs.set_reg(sx127xRegEnum.REG_OP_MODE, self._modem | xc.MODE_STDBY)
      return True

   # maximum TX power is 20 dBm and 14 dBm for RFO pin
   def setTxPower(self, txPower: int, paPin: int):
      utils.trace_dbg("[ setTxPower ]")
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
            if txPower < 2:
               txPower = 2
            outputPower = txPower - 2
            self.setCurrentProtection(140)  # max current 140 mA
         # enable or disable +20 dBm option on PA_BOOST pin
         self.regs.set_reg(sx127xRegEnum.REG_PA_DAC, paDac)
      # -- set PA config --
      self.regs.set_reg(sx127xRegEnum.REG_PA_CONFIG, paConfig | outputPower)

   def setBandwidth(self, bw: int):
      self._bw = bw
      bwCfg = 9         # 500 kHz
      if bw < 9100:
         bwCfg = 0      # 7.8 kHz
      elif bw < 13000:
         bwCfg = 1      # 10.4 kHz
      elif bw < 18200:
         bwCfg = 2      # 15.6 kHz
      elif bw < 26000:
         bwCfg = 3      # 20.8 kHz
      elif bw < 36500:
         bwCfg = 4      # 31.25 kHz
      elif bw < 52100:
         bwCfg = 5      # 41.7 kHz
      elif bw < 93800:
         bwCfg = 6      # 62.5 kHz
      elif bw < 187500:
         bwCfg = 7      # 125 kHz
      elif bw < 375000:
         bwCfg = 8      # 250 kHz
      self.regs.set_bits(sx127xRegEnum.REG_MODEM_CONFIG_1, bwCfg, 4, 4)

   def setRxGain(self, boost: int, level: int):
      utils.trace_dbg("[ setRxGain ]")
      # valid RX gain level 0 - 6 (0 -> AGC on)
      level = 6 if level > 6 else level
      # boost LNA and automatic gain controller config
      # LnaBoostHf = 0x00 # if boost:
      LnaBoostHf = 0x03 if boost else 0x00
      # AgcOn = 0x00
      AgcOn = 0x01 if level == xc.RX_GAIN_AUTO else 0x00
      # AgcOn = 0x01
      # set gain and boost LNA config
      self.regs.set_reg(sx127xRegEnum.REG_LNA, LnaBoostHf | (level << 5))
      # enable or disable AGC
      self.regs.set_bits(sx127xRegEnum.REG_MODEM_CONFIG_3, AgcOn, 2, 1)

   def setLoRaModulation(self, sf: int, bw: int
         , cr: int, ldro: bool = False):
      # -- -- -- --
      self.setSpreadingFactor(sf)
      self.setBandwidth(bw)
      self.setCodeRate(cr)
      self.setLdroEnable(ldro)

   def setCrcEnable(self, crcType: bool):
      # crcTypeCfg = 0x00
      # if crcType:
      #    crcTypeCfg = 0x01
      crcTypeCfg: int = 0x00 if crcType else 0x00
      self.regs.set_bits(sx127xRegEnum.REG_MODEM_CONFIG_2, crcTypeCfg, 2, 1)

   def setLoRaPacket(self, headerType: int, preambleLength: int
         , payloadLength: int, crcType: bool = False
         , invertIq: bool = False):
      # -- -- -- --
      self.setHeaderType(headerType)
      self.setPreambleLength(preambleLength)
      self.setPayloadLength(payloadLength)
      self.setCrcEnable(crcType)
      # self.setInvertIq(invertIq)

   def setSpreadingFactor(self, sf: int):
      utils.trace_dbg("[ setSpreadingFactor ]")
      self.sf = sf
      # valid spreading factor is 6 - 12
      if sf < 6:
         sf = 6
      elif sf > 12:
         sf = 12
      # set appropriate signal detection optimize and threshold
      # -- -- -- --
      optimize = 0x03
      threshold = 0x0A
      if sf == 6:
         optimize = 0x05
         threshold = 0x0C
      # -- -- -- --
      self.regs.set_reg(sx127xRegEnum.REG_DETECTION_OPTIMIZE, optimize)
      self.regs.set_reg(sx127xRegEnum.REG_DETECTION_THRESHOLD, threshold)
      # set spreading factor config
      self.regs.set_bits(sx127xRegEnum.REG_MODEM_CONFIG_2, sf, 4, 4)

   def setFrequency(self, freq: int):
      utils.trace_dbg("[ setFrequency ]")
      self._rf_freq = freq
      # calculate frequency
      frf = int((freq << 19) / 32000000)
      self.regs.set_reg(sx127xRegEnum.REG_FRF_MSB, (frf >> 16) & 0xFF)
      self.regs.set_reg(sx127xRegEnum.REG_FRF_MID, (frf >> 8) & 0xFF)
      self.regs.set_reg(sx127xRegEnum.REG_FRF_LSB, frf & 0xFF)

   # valid code rate denominator is 5 - 8
   def setCodeRate(self, cr: int):
      utils.trace_dbg("[ setCodeRate ]")
      if cr < 5:
         cr = 4
      elif cr > 8:
         cr = 8
      # crCfg = (cr - 4)
      self.regs.set_bits(sx127xRegEnum.REG_MODEM_CONFIG_1, (cr - 4), 1, 3)

   def setLdroEnable(self, ldro: bool):
      # ldroCfg = 0x00
      # if ldro:
      #    ldroCfg = 0x01
      ldroCfg = 0x01 if ldro else 0x00
      self.regs.set_bits(self.regs.REG_MODEM_CONFIG_3, ldroCfg, 3, 1)

   def setHeaderType(self, headerType: int):
      self._headerType = headerType
      headerTypeCfg = xc.HEADER_EXPLICIT
      if headerType == xc.HEADER_IMPLICIT:
         headerTypeCfg = xc.HEADER_IMPLICIT
      self.regs.set_bits(self.regs.REG_MODEM_CONFIG_1, headerTypeCfg, 0, 1)

   def setPreambleLength(self, preambleLength: int):
      self.regs.set_reg(self.regs.REG_PREAMBLE_MSB, (preambleLength >> 8) & 0xFF)
      self.regs.set_reg(self.regs.REG_PREAMBLE_LSB, preambleLength & 0xFF)

   def setPayloadLength(self, payloadLength: int):
      self._payloadLength = payloadLength
      self.regs.set_reg(self.regs.REG_PAYLOAD_LENGTH, payloadLength)

   # keep compatibility between 1 and 2 bytes synchronize word
   def setSyncWord(self, syncWord: int):
      sw = syncWord
      if syncWord > 0xFF:
         sw = ((syncWord >> 8) & 0xF0) | (syncWord & 0x0F)
      self.regs.set_reg(self.regs.REG_SYNC_WORD, sw)

   def standby(self):
      self.regs.set_reg(sx127xRegEnum.REG_OP_MODE, self._modem | xc.MODE_STDBY)
      time.sleep(0.002)
      reg_val: int = self.regs.get_reg(sx127xRegEnum.REG_OP_MODE)
      utils.trace_dbg(sx127xRegEnum(reg_val).name)
