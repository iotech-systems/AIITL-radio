
import spidev, time
import threading as th
from collections import deque
# -- -- -- --
from core.utils import utils


R_MASK: int = 0x7f
W_MASK: int = 0x80


class sx127xRegs(object):
   #
   # SX127X LoRa Mode Register Map
   #
   REG_FIFO = 0x00
   REG_OP_MODE = 0x01
   REG_FRF_MSB = 0x06
   REG_FRF_MID = 0x07
   REG_FRF_LSB = 0x08
   REG_PA_CONFIG = 0x09
   REG_PA_RAMP = 0x0A
   REG_OCP = 0x0B
   REG_LNA = 0x0C
   REG_FIFO_ADDR_PTR = 0x0D
   REG_FIFO_TX_BASE_ADDR = 0x0E
   REG_FIFO_RX_BASE_ADDR = 0x0F
   REG_FIFO_RX_CURRENT_ADDR = 0x10
   REG_IRQ_FLAGS_MASK = 0x11
   REG_IRQ_FLAGS = 0x12
   REG_RX_NB_BYTES = 0x13
   REG_RX_HEADR_CNT_VALUE_MSB = 0x14
   REG_RX_HEADR_CNT_VALUE_LSB = 0x15
   REG_RX_PKT_CNT_VALUE_MSB = 0x16
   REG_RX_PKT_CNT_VALUE_LSB = 0x17
   REG_MODEB_STAT = 0x18
   REG_PKT_SNR_VALUE = 0x19
   REG_PKT_RSSI_VALUE = 0x1A
   REG_RSSI_VALUE = 0x1B
   REG_HOP_CHANNEL = 0x1C
   REG_MODEM_CONFIG_1 = 0x1D
   REG_MODEM_CONFIG_2 = 0x1E
   REG_SYMB_TIMEOUT_LSB = 0x1F
   REG_PREAMBLE_MSB = 0x20
   REG_PREAMBLE_LSB = 0x21
   REG_PAYLOAD_LENGTH = 0x22
   REG_MAX_PAYLOAD_LENGTH = 0x23
   REG_HOP_PERIOD = 0x24
   REG_FIFO_RX_BYTE_ADDR = 0x25
   REG_MODEM_CONFIG_3 = 0x26
   REG_FREQ_ERROR_MSB = 0x28
   REG_FREQ_ERROR_MID = 0x29
   REG_FREQ_ERROR_LSB = 0x2A
   REG_RSSI_WIDEBAND = 0x2C
   REG_FREQ1 = 0x2F
   REG_FREQ2 = 0x30
   REG_DETECTION_OPTIMIZE = 0x31
   REG_INVERTIQ = 0x33
   REG_HIGH_BW_OPTIMIZE_1 = 0x36
   REG_DETECTION_THRESHOLD = 0x37
   REG_SYNC_WORD = 0x39
   REG_HIGH_BW_OPTIMIZE_2 = 0x3A
   REG_INVERTIQ2 = 0x3B
   REG_DIO_MAPPING_1 = 0x40
   REG_DIO_MAPPING_2 = 0x41
   REG_VERSION = 0x42
   REG_TCXO = 0x4B
   REG_PA_DAC = 0x4D
   REG_FORMER_TEMP = 0x5B
   REG_AGC_REF = 0x61
   REG_AGC_THRESH_1 = 0x62
   REG_AGC_THRESH_2 = 0x63
   REG_AGC_THRESH_3 = 0x64
   REG_PLL = 0x70
   CMD_FRQ_HZ: int = 200

   def __init__(self, spi: spidev.SpiDev):
      self.spi: spidev.SpiDev = spi
      self.rd_thrd: th.Thread = th.Thread(target=self._rd_thread)
      self.wr_thrd: th.Thread = th.Thread(target=self._wr_thread)
      self.cmds_in: deque = deque()
      self.cmds_out: deque = deque()
      self.rd_wr_sleep: float = (1 / sx127xRegs.CMD_FRQ_HZ)

   def init(self):
      try:
         self.rd_thrd.start()
         self.wr_thrd.start()
      except Exception as e:
         utils.log_err(e)
      finally:
         pass

   def set_bits(self, address: int, data: int
         , position: int, length: int):
      # -- -- -- --
      _tup: () = self._tfer(address & R_MASK, 0x00)
      read = sx127xRegs._val(_tup)
      mask = (0xFF >> (8 - length)) << position
      write = (data << position) | (read & ~mask)
      _tup: () = self._tfer(address | W_MASK, write)

   def set_reg(self, address: int, data: int) -> int:
      _tup: () = self._tfer(address | W_MASK, data)
      return sx127xRegs._val(_tup)

   def get_reg(self, address: int) -> int:
      _tup: () = self._tfer(address & R_MASK, 0x00)
      return sx127xRegs._val(_tup)

   # -- private -- #

   def _tfer(self, address: int, data: int) -> tuple:
      utils.trace_dbg(f"[ spi sending: {[address, data]} ]")
      buff_arr = [address, data]
      rval: () = self.spi.xfer2(buff_arr)
      utils.trace_dbg(f"[ rval: {rval}]")
      return rval

   @staticmethod
   def _val(_tup: ()):
      if len(_tup) == 2:
         return _tup[1]
      return -1

   def _rd_thread(self):
      print("[ _rd_thread ]")
      def __th_thick():
         pass
      # -- -- -- --
      while True:
         __th_thick()
         time.sleep(self.rd_wr_sleep)

   def _wr_thread(self):
      print("[ _wr_thread ]")
      def __th_thick():
         if len(self.cmds_in) == 0:
            return
         cmd_arr = self.cmds_in.popleft()
         print(cmd_arr)
      # -- -- -- --
      while True:
         __th_thick()
         time.sleep(self.rd_wr_sleep)
