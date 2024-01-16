
import spidev, typing as t
import time, RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
# -- dev / 6 --
SPI_SPEED: int = 3900000


class loraSPI(object):

   def __init__(self, pin_rst: int = 22, pin_cs: int = 8
         , bus: int = 0, bus_dev: int = 0, speed: int = SPI_SPEED):
      # -- -- -- --
      try:
         self.pin_rst: int = pin_rst
         self.pin_cs: int = pin_cs
         self.bus = bus
         self.dev = bus_dev
         self.speed = speed
         self.mode: int = 0
         self.lsbfst: bool = False
         # -- init pins --
         GPIO.setup([self.pin_rst, self.pin_cs], GPIO.OUT)
         GPIO.output([self.pin_rst, self.pin_cs], GPIO.LOW)
      except Exception as e:
         print(e)
      finally:
         pass

   def dump(self):
      spi: spidev.SpiDev = spidev.SpiDev()
      spi.open(bus=self.bus, device=self.dev)
      spi.max_speed_hz = self.speed
      print(f"lsbfst: {spi.lsbfirst} | mode: {spi.mode} | hz: {spi.max_speed_hz}")
      spi.close()

   def rst(self):
      pass

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
         spi.max_speed_hz = self.speed
         spi.lsbfirst = self.lsbfst
         spi.mode = self.mode
         ret = spi.xfer2(buff)
         return ret
      except Exception as e:
         print(e)
      finally:
         spi.close()
