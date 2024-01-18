
class sx127xConsts(object):
   # Modem options
   FSK_MODEM = 0x00              # GFSK packet type
   LORA_MODEM = 0x01             # LoRa packet type
   OOK_MODEM = 0x02              # OOK packet type
   # Long range mode and modulation type
   LONG_RANGE_MODE = 0x80        # GFSK packet type
   MODULATION_OOK = 0x20         # OOK packet type
   MODULATION_FSK = 0x00         # LoRa packet type
   # Devices modes
   MODE_SLEEP = 0x00             # sleep
   MODE_STDBY = 0x01             # standby
   MODE_TX = 0x03                # transmit
   MODE_RX_CONTINUOUS = 0x05     # continuous receive
   MODE_RX_SINGLE = 0x06         # single receive
   MODE_CAD = 0x07               # channel activity detection (CAD)
   # Rx operation mode
   RX_SINGLE = 0x000000          # Rx timeout duration: no timeout (Rx single mode)
   RX_CONTINUOUS = 0xFFFFFF      # infinite (Rx continuous mode)
   # TX power options
   TX_POWER_RFO = 0x00           # output power is limited to +14 dBm
   TX_POWER_PA_BOOST = 0x80      # output power is limited to +20 dBm
   # RX gain options
   RX_GAIN_POWER_SAVING = 0x00   # gain used in Rx mode: power saving gain (default)
   RX_GAIN_BOOSTED = 0x01        # boosted gain
   RX_GAIN_AUTO = 0x00
   # Oscillator options
   OSC_CRYSTAL = 0x00            # crystal oscillator with external crystal
   OSC_TCXO = 0x10               # external clipped sine TCXO AC-connected to XTA pin
   # DIO mapping
   DIO0_RX_DONE = 0x00           # set DIO0 interrupt for: RX done
   DIO0_TX_DONE = 0x40           # TX done
   DIO0_CAD_DONE = 0x80          # CAD done
   # IRQ flags
   IRQ_CAD_DETECTED = 0x01       # Valid Lora signal detected during CAD operation
   IRQ_FHSS_CHANGE = 0x02        # FHSS change channel interrupt
   IRQ_CAD_DONE = 0x04           # channel activity detection finished
   IRQ_TX_DONE = 0x08            # packet transmission completed
   IRQ_HEADER_VALID = 0x10       # valid LoRa header received
   IRQ_CRC_ERR = 0x20            # wrong CRC received
   IRQ_RX_DONE = 0x40            # packet received
   IRQ_RX_TIMEOUT = 0x80         # waiting packet received timeout
   # Header type
   HEADER_EXPLICIT = 0x00        # explicit header mode
   HEADER_IMPLICIT = 0x01        # implicit header mode
   # Rssi offset
   RSSI_OFFSET_LF = 164          # low band frequency RSSI offset
   RSSI_OFFSET_HF = 157          # high band frequency RSSI offset
   RSSI_OFFSET = 139             # frequency RSSI offset for SX1272
   BAND_THRESHOLD = 525E6        # threshold between low and high band frequency
   # TX and RX operation status
   STATUS_DEFAULT = 0            # default status (false)
   STATUS_TX_WAIT = 1
   STATUS_TX_TIMEOUT = 2
   STATUS_TX_DONE = 3
   STATUS_RX_WAIT = 4
   STATUS_RX_CONTINUOUS = 5
   STATUS_RX_TIMEOUT = 6
   STATUS_RX_DONE = 7
   STATUS_HEADER_ERR = 8
   STATUS_CRC_ERR = 9
   STATUS_CAD_WAIT = 10
   STATUS_CAD_DETECTED = 11
   STATUS_CAD_DONE = 12
   # -- chip ver --
   CHIP_VER_0x12 = 0x12
   CHIP_VER_0x22 = 0x22
