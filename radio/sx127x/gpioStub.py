


class stubGPIO(object):

   LOW = 0
   HIGH = 1
   BCM = 0
   OUT = 0
   IN = 1

   @staticmethod
   def setmode(mode: int):
      pass

   @staticmethod
   def output(pin, val):
      pass

   @staticmethod
   def setup(pin, _dir):
      pass

   @staticmethod
   def setwarnings(val: bool):
      pass
