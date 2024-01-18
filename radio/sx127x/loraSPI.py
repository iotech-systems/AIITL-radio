
import spidev, typing as t
from core.utils import utils


# -- bus_dev / 6 --
SPI_SPEED: int = 3_900_000


class loraSPI(spidev.SpiDev):

   def __init__(self, bus: int = 0, bus_dev: int = 0
         , bus_hz: int = SPI_SPEED, keep_open: bool = False):
      try:
         super().__init__()
         self.bus = bus
         self.bus_dev = bus_dev
         self.bus_hz = bus_hz
         self.mode: int = 0
         self.lsbfst: bool = False
         # -- not used yet --
         self.keep_open: bool = keep_open
         self.is_opened: bool = False
      except Exception as e:
         utils.log_err(e)
      finally:
         pass

   def dump(self):
      self.open(bus=self.bus, device=self.bus_dev)
      self.max_speed_hz = self.bus_hz
      print(f"lsbfst: {self.lsbfirst} | mode: {self.mode} | hz: {self.max_speed_hz}")
      self.close()

   def init(self):
      self.open(bus=self.bus, device=self.bus_dev)
      self.max_speed_hz = self.bus_hz
      print((f"lsbfst: {self.lsbfirst} | mode: {self.mode} "
         f"| hz: {self.max_speed_hz}"))
      self.close()

   def xtfr2(self, buff: t.Iterable) -> tuple:
      try:
         self.open(bus=self.bus, device=self.bus_dev)
         self.max_speed_hz = self.bus_hz
         self.lsbfirst = self.lsbfst
         self.mode = self.mode
         return self.xfer2(buff)
      except Exception as e:
         utils.log_err(e)
      finally:
         try:
            self.close()
         finally:
            pass
