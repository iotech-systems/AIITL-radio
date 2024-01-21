import spidev
import spidev as sd
import time, platform
import os, typing as t
import threading as th
# -- -- -- --
if "arm" in platform.machine():
   import RPi.GPIO as GPIO
else:
   from gpioStub import stubGPIO as GPIO
# -- -- -- --
from .pinX import pinX
from .sxBase import sxBase
from .sx127xRegs import sx127xRegs
from .sx127xRecOps import sx127xRecOps
from .sx127xEnums import sx127xRegEnum
from .sx127xConfOps import sx127xConfOps
from .sx127xConsts import sx127xConsts as xc


RF_FREQ: int = 433_000_000
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)


class sx127x(sxBase):

   def __init__(self, chip_id: str, spi: sd.SpiDev
         , spiinfo: (int, int), rst_pin: int, cs_pin: int):
      # -- -- -- --
      self.chip_id: str = chip_id
      self.spidev: sd.SpiDev = spi
      self.bus_id, self.bus_dev = spiinfo
      self.rst_pin: pinX = pinX("RST_PIN", rst_pin, GPIO.OUT)
      self.rst_pin.init()
      self.cs_pin: pinX = pinX("CS_PIN", cs_pin, GPIO.OUT)
      self.cs_pin.init()
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
      self._ver: int = 0x00
      self.conf: sx127xConfOps = t.Any
      self._statusWait = xc.STATUS_DEFAULT
      self._statusIrq = xc.STATUS_DEFAULT
      # _transmitTime = 0.0
      # -- -- -- --
      self.regs: sx127xRegs = sx127xRegs(spi=self.spidev)
      self.recv: sx127xRecOps = sx127xRecOps(regs=self.regs)
      self.conf: sx127xConfOps = sx127xConfOps(regs=self.regs)

   def init(self):
      self.reset()

   def begin(self) -> bool:
      try:
         self.reset()
         self.cs_pin.on()
         self.spidev.open(self.bus_id, self.bus_dev)
         self.conf.setModem(xc.LORA_MODEM)
         self.conf.setTxPower(17, xc.TX_POWER_PA_BOOST)
         self.conf.setRxGain(xc.RX_GAIN_BOOSTED, xc.RX_GAIN_AUTO)
         self.spidev.close()
         self.cs_pin.off()
         return True
      except Exception as e:
         print(e)
         return False
      finally:
         pass

   def reset(self):
      self.rst_pin.off()
      time.sleep(0.004)
      self.rst_pin.on()
      time.sleep(0.008)

   @property
   def ver(self) -> t.Optional[int]:
      _t = time.time()
      while self._ver not in [xc.CHIP_VER_0x12, xc.CHIP_VER_0x22]:
         self.cs_pin.on()
         self.regs.spi.open(self.bus_id, self.bus_dev)
         self._ver = self.regs.get_reg(sx127xRegEnum.REG_VERSION)
         self.regs.spi.close()
         self.cs_pin.off()
         if time.time() - _t > 4:
            return None
         # print(f"[ ver: {self._ver} | hex: 0x{self._ver:02X} ]")
      return self._ver

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
      # self._payloadTxRx = 0
      # self.writeRegister(regs.REG_FIFO_TX_BASE_ADDR, self.readRegister(regs.REG_FIFO_ADDR_PTR))
      # # save current txen and rxen pin state and set txen pin to high and rxen pin to low
      # if self._txen is not None and self._rxen is not None:
      #    # self._txState = self._txen.input()
      #    # self._rxState = self._rxen.input()
      #    # self._txen.output(LoRaGpio.HIGH)
      #    # self._rxen.output(LoRaGpio.LOW)
      pass

   def endPacket(self, timeout: int) -> bool:
      # # skip to enter TX mode when previous TX operation incomplete
      # if self.readRegister(regs.REG_OP_MODE) & 0x07 == consts.MODE_TX:
      #    return False
      # # clear IRQ flag from last TX or RX operation
      # self.writeRegister(regs.REG_IRQ_FLAGS, 0xFF)
      # # set packet payload length
      # self.writeRegister(regs.REG_PAYLOAD_LENGTH, self._payloadTxRx)
      # # set status to TX wait
      # self._statusWait = consts.STATUS_TX_WAIT
      # self._statusIrq = 0x00
      # # set device to transmit mode
      # self.writeRegister(regs.REG_OP_MODE, self._modem | consts.MODE_TX)
      # self._transmitTime = time.time()
      # # set TX done interrupt on DIO0 and attach TX interrupt handler
      # if self._irq is not None:
      #    self.writeRegister(regs.REG_DIO_MAPPING_1, consts.DIO0_TX_DONE)
      #    # if isinstance(self._monitoring, Thread):
      #    #    self._monitoring.join()
      #    # to = self._irqTimeout / 1000 if timeout == 0 else timeout / 1000
      #    # self._monitoring = Thread(target=self._irq.monitor, args=(self._interruptTx, to))
      #    # self._monitoring.start()
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
      irqFlagMask = (xc.IRQ_RX_DONE | xc.IRQ_RX_TIMEOUT | xc.IRQ_CRC_ERR)
      if self._statusWait == xc.STATUS_TX_WAIT:
         irqFlagMask = xc.IRQ_TX_DONE
      # -- -- -- --
      tt = time.time()
      while not (irqFlag & irqFlagMask) and self._statusIrq == 0x00:
         # only check IRQ status register for non interrupt operation
         if self._irq is None:
            irqFlag = self.regs.get_reg(self.regs.REG_IRQ_FLAGS)
         # return when timeout reached
         if time.time() - tt > timeout > 0:
            return False
      # -- -- -- --
      if self._statusIrq:
         # immediately return when interrupt signal hit
         return True
      elif self._statusWait == xc.STATUS_TX_WAIT:
         # calculate transmit time and set back txen and rxen pin to previous state
         self._transmitTime = time.time() - self._transmitTime
         # if self._txen != None and self._rxen != None:
         #    self._txen.output(self._txState)
         #    self._rxen.output(self._rxState)
      elif self._statusWait == xc.STATUS_RX_WAIT:
         # terminate receive mode by setting mode to standby
         self.standby()
         # set pointer to RX buffer base address and get packet payload length
         self.regs.set_reg(self.regs.REG_FIFO_ADDR_PTR
            , self.regs.get_reg(self.regs.REG_FIFO_RX_CURRENT_ADDR))
         self._payloadTxRx = self.regs.get_reg(self.regs.REG_RX_NB_BYTES)
         # set back txen and rxen pin to previous state
         if self._txen is not None and self._rxen is not None:
            # self._txen.output(self._txState)
            # self._rxen.output(self._rxState)
            pass
      elif self._statusWait == xc.STATUS_RX_CONTINUOUS:
         # set pointer to RX buffer base address and get packet payload length
         self.regs.set_reg(self.regs.REG_FIFO_ADDR_PTR
            , self.regs.get_reg(self.regs.REG_FIFO_RX_CURRENT_ADDR))
         self._payloadTxRx = self.regs.get_reg(self.regs.REG_RX_NB_BYTES)
         # clear IRQ flag
         self.regs.set_reg(self.regs.REG_IRQ_FLAGS, 0xFF)
      # store IRQ status
      self._statusIrq = irqFlag
      return True

   def standby(self):
      self.regs.set_reg(self.regs.REG_OP_MODE, self._modem | xc.MODE_STDBY)

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

   def onTransmit(self, callback=None):
      # register onTransmit function to call every transmit done
      if callable(callback):
         self._onTransmit = callback

   def onReceive(self, callback=None):
      # register onReceive function to call every receive done
      if callable(callback):
         self._onReceive = callback

   def transmitTime(self) -> float:
      # get transmit time in millisecond (ms)
      return self._transmitTime * 1000

   # get data rate last transmitted package in kbps
   def dataRate(self) -> float:
      return self._payloadTxRx / self._transmitTime

   # get relative signal strength index (RSSI) of last incoming package
   def lastMsgInRssi(self) -> float:
      offset = xc.RSSI_OFFSET_HF
      if self._rf_freq < xc.BAND_THRESHOLD:
         offset = xc.RSSI_OFFSET_LF
      if self.regs.get_reg(self.regs.REG_VERSION) == 0x22:
         offset = xc.RSSI_OFFSET
      return self.regs.get_reg(self.regs.REG_PKT_RSSI_VALUE) - offset

   def rssi(self) -> float:
      offset = xc.RSSI_OFFSET_HF
      if self._rf_freq < xc.BAND_THRESHOLD:
         offset = xc.RSSI_OFFSET_LF
      if self.regs.get_reg(self.regs.REG_VERSION) == 0x22:
         offset = xc.RSSI_OFFSET
      return self.regs.get_reg(self.regs.REG_RSSI_VALUE) - offset

   # get signal-to-noise ratio (SNR) of last incoming package
   def snr(self) -> float:
      return self.regs.get_reg(self.regs.REG_PKT_SNR_VALUE) / 4.0
