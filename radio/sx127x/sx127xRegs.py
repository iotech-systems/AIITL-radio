
import spidev

R_MASK: int = 0x7f
W_MASK: int = 0x80


class sx127xRegs(object):

   # SX127X LoRa Mode Register Map
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

   def __init__(self, spi: spidev.SpiDev, cs_pin: int = None
         ,  with_cs: bool = False):
      # -- -- -- --
      self.spi: spidev.SpiDev = spi
      # cs should be fixed with hardware ???
      self.cs_pin: int = cs_pin
      self.with_cs: bool = with_cs

   def writeBits(self, address: int, data: int
         , position: int, length: int):
      # -- -- -- --
      read = self._transfer(address & R_MASK, 0x00)
      mask = (0xFF >> (8 - length)) << position
      write = (data << position) | (read & ~mask)
      self._transfer(address | W_MASK, write)

   def writeS(self, address: int, data: int) -> int:
      return self._transfer(address | W_MASK, data)

   def writeM(self, address: int, data: bytes) -> int:
      pass

   def readS(self, address: int) -> int:
      return self._transfer(address & R_MASK, 0x00)

   def _transfer(self, address: int, data: int, with_cs: bool = False) -> int:
      print(f"[ spi sending: {[address, data]} ]")
      buff_arr = [address, data]
      if with_cs:
         self.__set_cs_low()
      rval: () = self.spi.xtfr2(buff_arr)
      if with_cs:
         self.__set_cs_high()
      print(f"[ rval: {rval}]")
      if len(rval) == 2:
         return int(rval[1])
      return -1
