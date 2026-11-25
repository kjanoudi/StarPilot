import numpy as np

from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.starpilot.lateral import palisade_shaping as shaping
from openpilot.starpilot.lateral.hkg_torque_policy import HKGShapedTorquePolicy, supported_torque_cp


PALISADE_CARS = frozenset((CAR.HYUNDAI_PALISADE, CAR.HYUNDAI_PALISADE_2023,))
FRICTION_TAPER_ACCEL = (.08, .30)


def supported_cp(cp):
  return supported_torque_cp(cp, PALISADE_CARS)


class PalisadeTorquePolicy(HKGShapedTorquePolicy):
  FACTOR_MULT = shaping.PALISADE_BASE_LAT_ACCEL_FACTOR_MULT
  LOG_PALISADE_DIAGNOSTIC = True
  supported_cp = staticmethod(supported_cp)

  def __init__(self, parent, cp):
    super().__init__(parent, cp)
    self.taper_center_friction = (cp.carFingerprint == CAR.HYUNDAI_PALISADE_2023 and
                                  bool(cp.flags & HyundaiFlags.CANFD_LKA_STEER_MSG))

  def feedforward(self, value, setpoint, jerk, speed):
    return value * shaping.get_palisade_ff_scale(setpoint, jerk, speed) * shaping.get_palisade_center_taper_scale(setpoint, speed)

  def friction(self, setpoint, jerk, speed):
    taper = shaping.get_palisade_center_taper_scale(setpoint, speed)
    return (shaping.get_palisade_friction_threshold(speed, setpoint, jerk),
            1.0 + (shaping.get_palisade_friction_scale(speed, setpoint, jerk) - 1.0) * taper)

  def shape_friction_compensation(self, value, setpoint):
    if not self.taper_center_friction:
      return value
    return value * float(np.interp(abs(setpoint), FRICTION_TAPER_ACCEL, (0., 1.)))

  def output(self, value, setpoint, speed):
    return value * shaping.get_palisade_center_output_scale(setpoint, speed)
