
import os
import typing as t
import time, platform
if "arm" in platform.machine():
   import RPi.GPIO as GPIO
else:
   from gpioStub import stubGPIO as GPIO
# -- -- -- --
from .sxBase import sxBase
from .loraSPI import loraSPI
from .sx127xRegs import sx127xRegs
from .sx127xConsts import sx127xConsts as xc
from .sx127xRecOps import sx127xRecOps
from .sx127xConfOps import sx127xConfOps


RF_FREQ: int = 433_000_000
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


class sx127x(sxBase, sx127xRecOps):

   def __init__(self, spi: loraSPI, rst_pin: int, cs_pin: int):
      # -- -- call supers -- --
      super(sx127xRecOps, self).__init__()
      # regs.__init__(self)
      self.spi: loraSPI = spi
      self.rst_pin: int = rst_pin
      self.cs_pin: int = cs_pin
      # -- pins in org. code --
      self._txen = None
      self._rxen = None
      self.sf: int = 7
      self._bw: int = 125_000
      self._irq: int = 0
      self._modem: int = xc.LORA_MODEM
      self._headerType: int = 0
      self._payloadTxRx: int = 0
      self._transmitTime: int = 0
      self._payloadLength: int = 0
      self._rf_freq: int = RF_FREQ
      self._onReceive: callable = None
      self._onTransmit: callable = None
      # SPI and GPIO pin setting
      # _irqTimeout = 10000
      # _txState = LoRaGpio.LOW
      # _rxState = LoRaGpio.LOW
      # # LoRa setting
      # _dio = 1
      # _cr = 5
      # _ldro = False
      # _headerType = HEADER_EXPLICIT
      # _preambleLength = 12
      # _payloadLength = 32
      # _crcType = False
      # _invertIq = False
      # # Operation properties
      # _monitoring = None
      # _payloadTxRx = 32
      self.conf: sx127xConfOps = t.Any
      self._statusWait = xc.STATUS_DEFAULT
      self._statusIrq = xc.STATUS_DEFAULT
      # _transmitTime = 0.0
      # -- -- -- --
      self.regs: sx127xRegs = sx127xRegs()
      self.conf: sx127xConfOps = sx127xConfOps(regs=self.regs)

   def init(self):
      GPIO.setup([self.rst_pin, self.cs_pin], GPIO.OUT)
      self.reset()

   def begin(self) -> bool:
      try:
         self.reset()
         self.conf.setModem(xc.LORA_MODEM)
         self.setTxPower(17, xc.TX_POWER_PA_BOOST)
         self.setRxGain(xc.RX_GAIN_BOOSTED, xc.RX_GAIN_AUTO)
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
      _t = time.time()
      # while ver != 0x12 and ver != 0x22:
      while ver not in [xc.CHIP_VER_0x12, xc.CHIP_VER_0x22]:
         ver = self.regs.readReg(self.regs.REG_VERSION)
         if time.time() - _t > 8:
            return False
         print(f"[ ver: {ver} | hex: 0x{ver:02X} ]")
      return True

   def __set_cs(self, val: bool):
      GPIO.output(self.cs_pin, val)

   def __set_cs_low(self):
      GPIO.output(self.cs_pin, 0)

   def __set_cs_high(self):
      GPIO.output(self.cs_pin, 1)

   def end(self):
      pass

   # reset TX buffer base address, FIFO address pointer and payload length
   def beginPacket(self):
      self._payloadTxRx = 0
      self.writeRegister(regs.REG_FIFO_TX_BASE_ADDR, self.readRegister(regs.REG_FIFO_ADDR_PTR))
      # save current txen and rxen pin state and set txen pin to high and rxen pin to low
      if self._txen is not None and self._rxen is not None:
         # self._txState = self._txen.input()
         # self._rxState = self._rxen.input()
         # self._txen.output(LoRaGpio.HIGH)
         # self._rxen.output(LoRaGpio.LOW)
         pass

   def endPacket(self, timeout: int) -> bool:
      # skip to enter TX mode when previous TX operation incomplete
      if self.readRegister(regs.REG_OP_MODE) & 0x07 == consts.MODE_TX:
         return False
      # clear IRQ flag from last TX or RX operation
      self.writeRegister(regs.REG_IRQ_FLAGS, 0xFF)
      # set packet payload length
      self.writeRegister(regs.REG_PAYLOAD_LENGTH, self._payloadTxRx)
      # set status to TX wait
      self._statusWait = consts.STATUS_TX_WAIT
      self._statusIrq = 0x00
      # set device to transmit mode
      self.writeRegister(regs.REG_OP_MODE, self._modem | consts.MODE_TX)
      self._transmitTime = time.time()
      # set TX done interrupt on DIO0 and attach TX interrupt handler
      if self._irq is not None:
         self.writeRegister(regs.REG_DIO_MAPPING_1, consts.DIO0_TX_DONE)
         # if isinstance(self._monitoring, Thread):
         #    self._monitoring.join()
         # to = self._irqTimeout / 1000 if timeout == 0 else timeout / 1000
         # self._monitoring = Thread(target=self._irq.monitor, args=(self._interruptTx, to))
         # self._monitoring.start()
      return True

   def write(self, data, length: int):
      pass

   ###
   # WAIT, OPERATION STATUS, AND PACKET STATUS METHODS
   # ###

   def wait(self, timeout: int) -> bool:
      # immediately return when currently not waiting
      # transmit or receive process
      if self._statusIrq:
         return True
      # wait transmit or receive process finish by checking
      # interrupt status or IRQ status
      irqFlag = 0x00
      irqFlagMask = consts.IRQ_RX_DONE | consts.IRQ_RX_TIMEOUT | consts.IRQ_CRC_ERR
      if self._statusWait == consts.STATUS_TX_WAIT:
         irqFlagMask = consts.IRQ_TX_DONE
      t = time.time()
      # -- -- -- --
      while not (irqFlag & irqFlagMask) and self._statusIrq == 0x00:
         # only check IRQ status register for non interrupt operation
         if self._irq is None:
            irqFlag = self.readRegister(regs.REG_IRQ_FLAGS)
         # return when timeout reached
         if time.time() - t > timeout > 0:
            return False
      # -- -- -- --
      if self._statusIrq:
         # immediately return when interrupt signal hit
         return True
      elif self._statusWait == consts.STATUS_TX_WAIT:
         # calculate transmit time and set back txen and rxen pin to previous state
         self._transmitTime = time.time() - self._transmitTime
         # if self._txen != None and self._rxen != None:
         #    self._txen.output(self._txState)
         #    self._rxen.output(self._rxState)
      elif self._statusWait == consts.STATUS_RX_WAIT:
         # terminate receive mode by setting mode to standby
         self.standby()
         # set pointer to RX buffer base address and get packet payload length
         self.writeRegister(regs.REG_FIFO_ADDR_PTR, self.readRegister(regs.REG_FIFO_RX_CURRENT_ADDR))
         self._payloadTxRx = self.readRegister(regs.REG_RX_NB_BYTES)
         # set back txen and rxen pin to previous state
         if self._txen is not None and self._rxen is not None:
            # self._txen.output(self._txState)
            # self._rxen.output(self._rxState)
            pass
      elif self._statusWait == consts.STATUS_RX_CONTINUOUS:
         # set pointer to RX buffer base address and get packet payload length
         self.writeRegister(regs.REG_FIFO_ADDR_PTR, self.readRegister(regs.REG_FIFO_RX_CURRENT_ADDR))
         self._payloadTxRx = self.readRegister(regs.REG_RX_NB_BYTES)
         # clear IRQ flag
         self.writeRegister(regs.REG_IRQ_FLAGS, 0xFF)
      # store IRQ status
      self._statusIrq = irqFlag
      return True

   def standby(self):
      self.writeRegister(regs.REG_OP_MODE, self._modem | consts.MODE_STDBY)

   def status(self):
      # set back status IRQ for RX continuous operation
      # statusIrq = self._statusIrq
      # if self._statusWait == self.STATUS_RX_CONTINUOUS:
      #    self._statusIrq = 0x0000
      # # get status for transmit and receive operation based on status IRQ
      # if statusIrq & self.IRQ_RX_TIMEOUT:
      #    return self.STATUS_RX_TIMEOUT
      # elif statusIrq & self.IRQ_CRC_ERR:
      #    return self.STATUS_CRC_ERR
      # elif statusIrq & self.IRQ_TX_DONE:
      #    return self.STATUS_TX_DONE
      # elif statusIrq & self.IRQ_RX_DONE:
      #    return self.STATUS_RX_DONE
      # # return TX or RX wait status
      # return self._statusWait
      pass

   def onTransmit(self, callback: callable):
      # register onTransmit function to call every transmit done
      self._onTransmit = callback

   def onReceive(self, callback: callable):
      # register onReceive function to call every receive done
      self._onReceive = callback



   def setRxGain(self, boost: int, level: int):
      # valid RX gain level 0 - 6 (0 -> AGC on)
      level = 6 if level > 6 else level
      # boost LNA and automatic gain controller config
      # LnaBoostHf = 0x00 # if boost:
      LnaBoostHf = 0x03 if boost else 0x00
      AgcOn = 0x00
      if level == consts.RX_GAIN_AUTO:
         AgcOn = 0x01
      # set gain and boost LNA config
      self.writeRegister(regs.REG_LNA, LnaBoostHf | (level << 5))
      # enable or disable AGC
      self.writeBits(regs.REG_MODEM_CONFIG_3, AgcOn, 2, 1)

   def setLoRaModulation(self, sf: int, bw: int, cr: int, ldro: bool = False):
      self.setSpreadingFactor(sf)
      self.setBandwidth(bw)
      self.setCodeRate(cr)
      self.setLdroEnable(ldro)

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
      self.sf = sf
      # valid spreading factor is 6 - 12
      if sf < 6:
         sf = 6
      elif sf > 12:
         sf = 12
      # set appropriate signal detection optimize and threshold
      optimize = 0x03; threshold = 0x0A
      if sf == 6:
         optimize = 0x05
         threshold = 0x0C
      self.writeRegister(regs.REG_DETECTION_OPTIMIZE, optimize)
      self.writeRegister(regs.REG_DETECTION_THRESHOLD, threshold)
      # set spreading factor config
      self.writeBits(regs.REG_MODEM_CONFIG_2, sf, 4, 4)

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
      self.writeBits(regs.REG_MODEM_CONFIG_1, bwCfg, 4, 4)

   def setFrequency(self, freq: int):
      self._rf_freq = freq
      # calculate frequency
      frf = int((freq << 19) / 32000000)
      self.writeRegister(regs.REG_FRF_MSB, (frf >> 16) & 0xFF)
      self.writeRegister(regs.REG_FRF_MID, (frf >> 8) & 0xFF)
      self.writeRegister(regs.REG_FRF_LSB, frf & 0xFF)

   # valid code rate denominator is 5 - 8
   def setCodeRate(self, cr: int):
      if cr < 5:
         cr = 4
      elif cr > 8:
         cr = 8
      crCfg = cr - 4
      self.writeBits(regs.REG_MODEM_CONFIG_1, crCfg, 1, 3)

   def setLdroEnable(self, ldro: bool):
      # ldroCfg = 0x00
      # if ldro:
      #    ldroCfg = 0x01
      ldroCfg = 0x01 if ldro else 0x00
      self.writeBits(regs.REG_MODEM_CONFIG_3, ldroCfg, 3, 1)

   def setHeaderType(self, headerType: int):
      self._headerType = headerType
      headerTypeCfg = consts.HEADER_EXPLICIT
      if headerType == consts.HEADER_IMPLICIT:
         headerTypeCfg = consts.HEADER_IMPLICIT
      self.writeBits(regs.REG_MODEM_CONFIG_1, headerTypeCfg, 0, 1)

   def setPreambleLength(self, preambleLength: int):
      self.writeRegister(regs.REG_PREAMBLE_MSB, (preambleLength >> 8) & 0xFF)
      self.writeRegister(regs.REG_PREAMBLE_LSB, preambleLength & 0xFF)

   def setPayloadLength(self, payloadLength: int):
      self._payloadLength = payloadLength
      self.writeRegister(regs.REG_PAYLOAD_LENGTH, payloadLength)

   def setCrcEnable(self, crcType: bool):
      # crcTypeCfg = 0x00
      # if crcType:
      #    crcTypeCfg = 0x01
      crcTypeCfg: int = 0x00 if crcType else 0x00
      self.writeBits(regs.REG_MODEM_CONFIG_2, crcTypeCfg, 2, 1)

   def transmitTime(self) -> float:
      # get transmit time in millisecond (ms)
      return self._transmitTime * 1000

   # get data rate last transmitted package in kbps
   def dataRate(self) -> float:
      return self._payloadTxRx / self._transmitTime

   # get relative signal strength index (RSSI) of last incoming package
   def packetRssi(self) -> float:
      offset = consts.RSSI_OFFSET_HF
      if self._rf_freq < consts.BAND_THRESHOLD:
         offset = consts.RSSI_OFFSET_LF
      if self.readRegister(regs.REG_VERSION) == 0x22:
         offset = consts.RSSI_OFFSET
      return self.readRegister(regs.REG_PKT_RSSI_VALUE) - offset

   def rssi(self) -> float:
      offset = consts.RSSI_OFFSET_HF
      if self._rf_freq < consts.BAND_THRESHOLD:
         offset = consts.RSSI_OFFSET_LF
      if self.readRegister(regs.REG_VERSION) == 0x22:
         offset = consts.RSSI_OFFSET
      return self.readRegister(regs.REG_RSSI_VALUE) - offset

   # get signal-to-noise ratio (SNR) of last incoming package
   def snr(self) -> float:
      return self.readRegister(regs.REG_PKT_SNR_VALUE) / 4.0

   def writeBits(self, address: int, data: int, position: int, length: int):
      read = self._transfer(address & 0x7F, 0x00)
      mask = (0xFF >> (8 - length)) << position
      write = (data << position) | (read & ~mask)
      self._transfer(address | 0x80, write)

   # keep compatibility between 1 and 2 bytes synchronize word
   def setSyncWord(self, syncWord: int):
      sw = syncWord
      if syncWord > 0xFF:y
         sw = ((syncWord >> 8) & 0xF0) | (syncWord & 0x0F)
      self.writeRegister(regs.REG_SYNC_WORD, sw)
