
class sx127xConsts(object):
   # Modem options
   FSK_MODEM = 0x00  # GFSK packet type
   LORA_MODEM = 0x01  # LoRa packet type
   OOK_MODEM = 0x02  # OOK packet type
   # Long range mode and modulation type
   LONG_RANGE_MODE = 0x80  # GFSK packet type
   MODULATION_OOK = 0x20  # OOK packet type
   MODULATION_FSK = 0x00  # LoRa packet type
   # Devices modes
   MODE_SLEEP = 0x00  # sleep
   MODE_STDBY = 0x01  # standby
   MODE_TX = 0x03  # transmit
   MODE_RX_CONTINUOUS = 0x05  # continuous receive
   MODE_RX_SINGLE = 0x06  # single receive
   MODE_CAD = 0x07  # channel activity detection (CAD)
