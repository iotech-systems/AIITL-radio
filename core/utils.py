
from termcolor import colored
from core.enums import TCOLORS


class utils(object):

   """ [ basic logging calls ] """

   @staticmethod
   def log_err(err: object):
      print(err)

   @staticmethod
   def log_warn(msg: str):
      print(msg)

   @staticmethod
   def log_info(msg: str):
      print(msg)

   @staticmethod
   def trace_dbg(msg: str):
      print(colored(f"{msg}", color=TCOLORS.light_grey.name))
