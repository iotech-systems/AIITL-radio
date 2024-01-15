
import spidev, typing as t


MHZ_1: int = 1000000
SPI_SPEED_5Mhz: int = 5 * MHZ_1


class loraSPI(object):

   def __init__(self, bus: int, cs: int, speed: int = SPI_SPEED_5Mhz):
      self.bus = bus
      self.cs = cs
      self.speed = speed

   def init(self):
      spi: spidev.SpiDev = spidev.SpiDev()
      spi.open(bus=self.bus, device=self.cs)
      print(f"lsbfst: {spi.lsbfirst} | mode: {spi.mode} | hz: {spi.max_speed_hz}")
      spi.close()

   def transfer(self, buf: t.Iterable) -> tuple:
      spi = spidev.SpiDev()
      spi.open(self.bus, self.cs)
      spi.lsbfirst = False
      spi.mode = 0
      spi.max_speed_hz = self.speed
      ret = spi.xfer2(buf)
      spi.close()
      return ret
