
import platform
# -- -- -- --
if "arm" in platform.machine():
   import RPi.GPIO as GPIO
else:
   from gpioStub import stubGPIO as GPIO


class pinX(object):

   def __init__(self, name: str, pin: int
         , DIR: int, actLevel: int = 1):
      # -- -- -- --
      self.name: str = name
      self.pin: int = pin
      self.pin_dir: int = DIR
      self.act_lvl: int = actLevel

   def init(self):
      GPIO.setup(self.pin, self.pin_dir)

   def on(self) -> bool:
      GPIO.output(self.pin, self.act_lvl)
      return GPIO.input(self.pin) == self.act_lvl

   def off(self) -> bool:
      GPIO.output(self.pin, (not self.act_lvl))
      return GPIO.input(self.pin) == (not self.act_lvl)

   def val(self) -> int:
      return GPIO.input(self.pin)
