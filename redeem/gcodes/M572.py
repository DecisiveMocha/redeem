"""
GCode M572 - set pressure advance factor
"""

from GCodeCommand import GCodeCommand
import logging


class M572(GCodeCommand):
  def execute(self, g):
    axis = 3    # E axis by default
    factor = 0    # no pressure advance by default

    if g.has_letter("D"):    # target extruder
      # convert from extruder index to axis index
      axis = int(g.get_value_by_letter("D")) + 3
      g.remove_token_by_letter("D")

    factor = g.get_float_by_letter("S")

    self.printer.pressure_advance_factors[axis] = factor

    self.printer.path_planner.native_planner.setPressureAdvanceFactors(
        self.printer.pressure_advance_factors)

    logging.debug("M572: pressure advance for " + self.printer.index_to_axis(axis) + " set to " +
                  str(factor))

  def get_description(self):
    return "Set extruder pressure advance factor"

  def get_long_description(self):
    return ("Sets the pressure advance factors for the extruder axes.")

  def is_buffered(self):
    return True

  def is_async(self):
    return True
