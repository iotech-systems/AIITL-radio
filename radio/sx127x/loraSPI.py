import time

import spidev, typing as t


# dev / 64
SPI_SPEED: int = 500000
# SPI_SPEED: int = 3900000


class loraSPI(object):

   def __init__(self, bus: int = 0, bus_dev: int = 0, speed: int = SPI_SPEED):
      self.bus = bus
      self.dev = bus_dev
      self.speed = speed
      self.mode: int = 0
      self.lsbfst: bool = False

   def dump(self):
      spi: spidev.SpiDev = spidev.SpiDev()
      spi.max_speed_hz = self.speed
      spi.open(bus=self.bus, device=self.dev)
      print(f"lsbfst: {spi.lsbfirst} | mode: {spi.mode} | hz: {spi.max_speed_hz}")
      spi.close()

   def init(self):
      spi: spidev.SpiDev = spidev.SpiDev()
      spi.open(bus=self.bus, device=self.dev)
      spi.max_speed_hz = self.speed
      print(f"lsbfst: {spi.lsbfirst} | mode: {spi.mode} | hz: {spi.max_speed_hz}")
      spi.close()

   def transfer(self, buff: t.Iterable) -> tuple:
      spi = spidev.SpiDev()
      try:
         spi.open(self.bus, self.dev)
         spi.lsbfirst = self.lsbfst
         spi.mode = self.mode
         spi.max_speed_hz = self.speed
         ret = spi.xfer2(buff)
         return ret
      except Exception as e:
         print(e)
      finally:
         spi.close()
