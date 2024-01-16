
import spidev, typing as t


# -- dev / 6 --
SPI_SPEED: int = 3900000


class loraSPI(object):

   def __init__(self, bus: int = 0, bus_dev: int = 0, bus_hz: int = SPI_SPEED):
      try:
         self.bus = bus
         self.dev = bus_dev
         self.spd_hz = bus_hz
         self.mode: int = 0
         self.lsbfst: bool = False
         self.spi: spidev.SpiDev = spidev.SpiDev()
      except Exception as e:
         print(e)
      finally:
         pass

   def dump(self):
      self.spi.open(bus=self.bus, device=self.dev)
      self.spi.max_speed_hz = self.spd_hz
      print(f"lsbfst: {self.spi.lsbfirst} | mode: {self.spi.mode} | hz: {self.spi.max_speed_hz}")
      self.spi.close()

   def init(self):
      self.spi.open(bus=self.bus, device=self.dev)
      self.spi.max_speed_hz = self.spd_hz
      print(f"lsbfst: {self.spi.lsbfirst} | mode: {self.spi.mode} | hz: {self.spi.max_speed_hz}")
      self.spi.close()

   def xtfr2(self, buff: t.Iterable) -> tuple:
      try:
         self.spi.open(self.bus, self.dev)
         self.spi.max_speed_hz = self.spd_hz
         self.spi.lsbfirst = self.lsbfst
         self.spi.mode = self.mode
         return self.spi.xfer2(buff)
      except Exception as e:
         print(e)
      finally:
         self.spi.close()
