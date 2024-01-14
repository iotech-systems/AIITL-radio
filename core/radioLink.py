
# import LoRaRF as lora
from LoRaRF import SX127x


class radioLink(object):

   RST_PIN: int = 22

   def __init__(self):
      self.lnk: SX127x = SX127x()
      self.lnk.setPins(reset=radioLink.RST_PIN)

   def init(self):
      begin_rval = self.lnk.begin()
      print(f"begin_rval: {begin_rval}")
