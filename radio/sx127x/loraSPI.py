import time

import spidev, typing as t


MHZ_1: int = 1000000
# SPI_SPEED: int = 5 * MHZ_1
SPI_SPEED: int = 1 * MHZ_1


class loraSPI(object):

   def __init__(self, bus: int, bus_dev: int, cs_pin: int = 0, speed: int = SPI_SPEED):
      self.bus = bus
      self.dev = bus_dev
      self.cs_pin: int = cs_pin
      self.speed = speed
      self.mode: int = 0
      self.lsbfst: bool = False

   def dump(self):
      spi: spidev.SpiDev = spidev.SpiDev()
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
      try:
         spi = spidev.SpiDev()
         spi.open(self.bus, self.dev)
         print(spi)
         spi.lsbfirst = self.lsbfst
         spi.mode = self.mode
         spi.max_speed_hz = self.speed
         time.sleep(0.001)
         ret = spi.xfer2(buff)
         time.sleep(0.001)
         ret = spi.xfer2(buff)
         spi.close()
         return ret
      except Exception as e:
         print(e)
      finally:
         pass
