
import RPi.GPIO


class loraPin(object):

   LOW = 0
   HIGH = 1

   def __init__(self, chip: int, offset: int):
      self.chip = "gpiochip" + str(chip)
      self.offset = offset
      self.bcm_id: int = 0

   def output(self, value: int):
      chip = gpiod.Chip(self.chip)
      line = chip.get_line(self.offset)
      try:
         line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_DIR_OUT)
         line.set_value(value)
      except:
         return
      finally:
         line.release()
         chip.close()

   def set_val(self, val: int = 0):



   def input(self) -> int:
      chip = gpiod.Chip(self.chip)
      line = chip.get_line(self.offset)
      try:
         line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_DIR_IN)
         value = line.get_value()
      except: return -1
      finally:
         line.release()
         chip.close()
      return value

   def monitor(self, callback, timeout: float):
      seconds = int(timeout)
      chip = gpiod.Chip(self.chip)
      line = chip.get_line(self.offset)
      try:
         line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_EV_RISING_EDGE)
         if line.event_wait(seconds, int((timeout - seconds) * 1000000000)):
            callback()
      except: return
      finally:
         line.release()
         chip.close()

   def monitor_continuous(self, callback, timeout: float):
      seconds = int(timeout)
      while True:
         chip = gpiod.Chip(self.chip)
         line = chip.get_line(self.offset)
         try:
            line.request(consumer="LoRaGpio", type=gpiod.LINE_REQ_EV_RISING_EDGE)
            if line.event_wait(seconds, int((timeout - seconds) * 1000000000)):
               callback()
         except: continue
         finally:
            line.release()
            chip.close()
