

from .loraSPI import loraSPI



class sx127x(object):

   def __init__(self, spi: loraSPI):
      self.spi: loraSPI = spi

   def init(self):
      pass
