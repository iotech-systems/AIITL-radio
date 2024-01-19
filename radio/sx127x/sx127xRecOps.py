
from .sx127xRegs import sx127xRegs
from .sx127xConsts import sx127xConsts as consts


class sx127xRecOps(object):

   def __init__(self, regs: sx127xRegs):
      self.regs: sx127xRegs = regs
      self._payloadTxRx = None
      self._statusWait: int = 0
      self._statusIrq: int = 0x00

   def request(self, timeout: int) -> bool:
      # skip to enter RX mode when previous RX operation incomplete
      rxMode = self.regs.readS(self.regs.REG_OP_MODE) & 0x07
      if rxMode == consts.MODE_RX_SINGLE or rxMode == consts.MODE_RX_CONTINUOUS:
         return False
      # clear IRQ flag from last TX or RX operation
      self.regs.writeS(self.regs.REG_IRQ_FLAGS, 0xFF)
      # save current txen and rxen pin state and set txen pin to low and rxen pin to high
      # if self._txen != None and self._rxen != None:
      #    self._txState = self._txen.input()
      #    self._rxState = self._rxen.input()
      #    self._txen.output(LoRaGpio.LOW)
      #    self._rxen.output(LoRaGpio.HIGH)
      # set status to RX wait
      self._statusIrq = 0x00
      self._statusWait = consts.STATUS_RX_WAIT
      # select RX mode to RX continuous mode for RX single and continuos operation
      rxMode = consts.MODE_RX_CONTINUOUS
      if timeout == consts.RX_CONTINUOUS:
         self._statusWait = consts.STATUS_RX_CONTINUOUS
      elif timeout > 0:
         # Select RX mode to single mode for RX operation with timeout
         rxMode = consts.MODE_RX_SINGLE
         # calculate and set symbol timeout
         symbTimeout = int(timeout * self._bw / 1000) >> self.sf  # devided by 1000, ms to s
         self.writeBits(regs.REG_MODEM_CONFIG_2, (symbTimeout >> 8) & 0x03, 0, 2)
         self.writeRegister(regs.REG_SYMB_TIMEOUT_LSB, symbTimeout & 0xFF)
      # set device to receive mode
      self.writeRegister(regs.REG_OP_MODE, self._modem | rxMode)
      # set RX done interrupt on DIO0 and attach RX interrupt handler
      if self._irq is not None:
         self.writeRegister(regs.REG_DIO_MAPPING_1, consts.DIO0_RX_DONE)
         if isinstance(self._monitoring, Thread):
            self._monitoring.join()
         to = self._irqTimeout / 1000 if timeout == 0 else timeout / 1000
         if timeout == self.RX_CONTINUOUS:
            self._monitoring = Thread(target=self._irq.monitor_continuous, args=(self._interruptRxContinuous, to))
            self._monitoring.setDaemon(True)
         else:
            self._monitoring = Thread(target=self._irq.monitor, args=(self._interruptRx, to))
         self._monitoring.start()
      return True

   # get size of package still available to read
   def available(self):
      return self._payloadTxRx

   # single or multiple bytes read
   def read(self, length: int):
      single = False
      if length == 0:
         length = 1
         single = True
      # calculate actual read length and remaining payload length
      if self._payloadTxRx > length:
         self._payloadTxRx -= length
      else:
         self._payloadTxRx = 0
      # read multiple bytes of received package in FIFO buffer
      data = tuple()
      for i in range(length):
         data = data + (self.regs.readS(self.regs.REG_FIFO),)
      # return single byte or tuple
      if single:
         return data[0]
      else:
         return data

   def get(self, length: int = 1) -> bytes:
      # calculate actual read length and remaining payload length
      if self._payloadTxRx > length:
         self._payloadTxRx -= length
      else:
         self._payloadTxRx = 0
      # read data from FIFO buffer and update payload length
      data = tuple()
      for i in range(length):
         data = data + (self.readRegister(regs.REG_FIFO),)
      # return array of bytes
      return bytes(data)

   def purge(self, length: int = 0):
      # subtract or reset received payload length
      if (self._payloadTxRx > length) and length:
         self._payloadTxRx = self._payloadTxRx - length
      else:
         self._payloadTxRx = 0
