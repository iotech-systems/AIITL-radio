#!/usr/bin/env python3

import os, sys, spidev
from radio.sx127x.loraSPI import loraSPI


loraspi: loraSPI = loraSPI()
dir(loraspi)
