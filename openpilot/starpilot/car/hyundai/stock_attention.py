import time

from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, is_blended


MAX_SAMPLE_AGE_NS = 500_000_000
MAX_CAN_BATCHES = 32
ALERT_TYPE = 'stockDriverAttention'
CLOCK_BOOTTIME = getattr(time, 'CLOCK_BOOTTIME', time.CLOCK_MONOTONIC)


def attention_bus(CP):
  if not CP.passive and not CP.dashcamOnly and is_blended(CP) and CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG:
    return CanBus(CP).ECAN
  return None


class StockAttention:
  def __init__(self, bus):
    self.bus = bus
    self.stamp_ns = 0
    self.level = 0

  def poll(self, receive, *, active, clock_ns=lambda: time.clock_gettime_ns(CLOCK_BOOTTIME)):
    previous = self.level
    for _ in range(MAX_CAN_BATCHES):
      event = receive()
      if event is None:
        break
      stamp_ns = int(event.logMonoTime)
      if not event.valid or stamp_ns <= self.stamp_ns or not 0 <= clock_ns() - stamp_ns <= MAX_SAMPLE_AGE_NS:
        continue
      for frame in event.can:
        if frame.src == self.bus and frame.address == 0x364 and len(frame.dat) == 8:
          # ALERTS_364.DAW_Status occupies bits 35 through 37.
          self.level = (frame.dat[4] >> 3) & 7
          self.stamp_ns = stamp_ns
    if active and self.level != previous and (self.level == 1 or self.level == 2 and previous != 1):
      return self.level
    return None
