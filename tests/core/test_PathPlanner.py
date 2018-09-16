from __future__ import absolute_import

import unittest
import mock    # use mock.Mock etc
import sys

sys.modules['evdev'] = mock.Mock()
sys.modules['spidev'] = mock.MagicMock()
sys.modules['redeem.RotaryEncoder'] = mock.Mock()
sys.modules['redeem.Watchdog'] = mock.Mock()
sys.modules['redeem.GPIO'] = mock.Mock()
sys.modules['redeem.Enable'] = mock.Mock()
sys.modules['redeem.Key_pin'] = mock.Mock()
sys.modules['redeem.DAC'] = mock.Mock()
sys.modules['redeem.ShiftRegister.py'] = mock.Mock()
sys.modules['Adafruit_BBIO'] = mock.Mock()
sys.modules['Adafruit_BBIO.GPIO'] = mock.Mock()
sys.modules['Adafruit_GPIO'] = mock.Mock()
sys.modules['Adafruit_GPIO.I2C'] = mock.MagicMock()
sys.modules['Adafruit_BBIO.PWM'] = mock.MagicMock()
sys.modules['redeem.StepperWatchdog'] = mock.Mock()
sys.modules['redeem.StepperWatchdog.GPIO'] = mock.Mock()
sys.modules['redeem.PruInterface'] = mock.Mock()
sys.modules['redeem.PruInterface'].PruInterface = mock.MagicMock()
sys.modules['redeem.PruFirmware'] = mock.Mock()
sys.modules['redeem.HBD'] = mock.MagicMock()
sys.modules['redeem.RotaryEncoder'] = mock.Mock()
sys.modules['JoinableQueue'] = mock.Mock()
sys.modules['redeem.USB'] = mock.Mock()
sys.modules['redeem.Ethernet'] = mock.Mock()
sys.modules['redeem.Pipe'] = mock.Mock()
sys.modules['redeem.Fan'] = mock.Mock()
sys.modules['redeem.Mosfet'] = mock.Mock()
sys.modules['redeem.PWM'] = mock.Mock()

sys.modules['_PathPlannerNative'] = mock.Mock()

from redeem.PathPlanner import PathPlanner
from redeem.Printer import Printer
from redeem.Path import *

class PathPlanner_TestCase(unittest.TestCase):
  @classmethod
  @mock.patch.object(PathPlanner, "_init_path_planner", new=None)
  def setUpClass(cls):
    cls.printer = Printer()
    Path.printer = cls.printer
    cls.path_planner = PathPlanner(cls.printer, None)
    cls.path_planner.native_planner = mock.Mock()
    cls.printer.ensure_steppers_enabled = mock.Mock()
    cls.path_planner.native_planner.getLastQueueMoveStatus = mock.MagicMock(return_value=None)
    cls.path_planner.native_planner.getState = mock.Mock(return_value=np.zeros(cls.printer.MAX_AXES, dtype=Path.DTYPE))

class PathPlanner_AbsolutePathTests(PathPlanner_TestCase):
  def test_add_absolute_path(self):
    speed = 123
    accel = 456
    path = AbsolutePath({
        "X": 10.0,
        "Y": 20.0,
        "Z": 30.0,
        "E": 40.0,
    }, speed, accel)

    self.path_planner.add_path(path)
    self.path_planner.native_planner.queueMove.assert_called_with((10, 20, 30, 40, 0, 0, 0, 0), speed, accel, 0b1111)

class PathPlanner_RelativePathTests(PathPlanner_TestCase):
  def test_add_two_relative_paths(self):
    speed = 123
    accel = 456
    first_path = RelativePath({
        "X": 10.0,
        "Y": 20.0,
        "E": 40.0,
    }, speed, accel)

    second_path = RelativePath({
        "X": 20.0,
        "Y": 10.0,
        "E": -10.0,
    }, speed, accel)

    self.path_planner.add_path(first_path)
    self.path_planner.native_planner.queueMove.assert_called_with((10, 20, 0, 40, 0, 0, 0, 0), speed, accel, 0)
    self.path_planner.add_path(second_path)
    self.path_planner.native_planner.queueMove.assert_called_with((20, 10, 0, -10, 0, 0, 0, 0), speed, accel, 0)

class PathPlanner_ProbeTests(PathPlanner_TestCase):
  def test_basic_probe(self):

    speed = 321
    accel = 654
    self.path_planner.probe(10, speed, accel)
    self.path_planner.native_planner.queueMove.assert_called_with((0, 0, -10, 0, 0, 0, 0, 0), speed, accel, 0b1, 0)