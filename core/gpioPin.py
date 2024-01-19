
import RPi.GPIO as GPIO


MODE_IN: int = 1
MODE_OUT: int = 0


# class gpioPin(object):
#
#    def __init__(self, pin_id: int = 0, mode: int = MODE_OUT, act_level: int = 1):
#       self.pin_id: int = pin_id
#       self.pin_mode: int = mode
#       self.act_level: int = act_level
#       # -- -- -- --
#       if self.pin_mode == MODE_OUT:
#          GPIO.setmode(self.pin_mode, GPIO.OUT)
#       elif self.pin_mode == MODE_IN:
#          GPIO.setmode(self.pin_mode, GPIO.INPUT)
#
#    def on(self) -> int:
