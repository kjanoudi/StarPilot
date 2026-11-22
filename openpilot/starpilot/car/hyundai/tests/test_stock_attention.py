from types import SimpleNamespace

from opendbc.can import CANPacker
from opendbc.car.hyundai.tests.test_palisade_2023 import params
from openpilot.starpilot.car.hyundai.stock_attention import StockAttention, attention_bus


def sample(stamp, level, bus=1, valid=True):
  address, data, source = CANPacker('hyundai_palisade_2023_generated').make_can_msg(
    'ALERTS_364', bus, {'DAW_Status': level})
  frame = SimpleNamespace(address=address, dat=data, src=source)
  return SimpleNamespace(can=[frame], logMonoTime=stamp, valid=valid)


def poll(observer, now, *samples, active=True):
  pending = iter(samples)
  return observer.poll(lambda: next(pending, None), active=active, clock_ns=lambda: now)


def test_advisories_on_the_mixed_hda2_powertrain_bus():
  assert attention_bus(params('hdai')) is None
  observer = StockAttention(attention_bus(params('hdaii')))
  start = 1_000_000_000
  assert poll(observer, start, sample(start, 5)) is None
  assert poll(observer, start + 1, sample(start + 1, 2)) == 2
  assert poll(observer, start + 2, sample(start + 2, 2)) is None
  assert poll(observer, start + 3, sample(start + 3, 1)) == 1
  assert poll(observer, start + 4, sample(start + 4, 2)) is None


def test_untrusted_samples_and_inactive_assistance_do_not_alert():
  observer = StockAttention(1)
  now = 1_000_000_000
  assert poll(observer, now, sample(now, 1, bus=2), sample(now, 1, valid=False), sample(now - 600_000_000, 1)) is None
  assert poll(observer, now, sample(now, 1), active=False) is None
  assert poll(observer, now + 1, sample(now + 1, 1)) is None
  assert poll(observer, now + 2, sample(now + 2, 5)) is None
  assert poll(observer, now + 3, sample(now + 3, 1)) == 1
