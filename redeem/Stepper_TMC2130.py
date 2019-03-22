import ctypes
import logging
import math
from Stepper import Stepper


def int_to_bytes(value):
  return [
      int((value & 0xff000000) >> 24),
      int((value & 0xff0000) >> 16),
      int((value & 0xff00) >> 8),
      int((value & 0xff))
  ]


def bytes_to_int(bytes):
  return bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3]


def format_transaction(bytes):
  result = ""
  for index in range(len(bytes) / 5):
    result += "{:02x}:{:08x} ".format(bytes[index * 5],
                                      bytes_to_int(bytes[index * 5 + 1:index * 5 + 5]))
  return result


class StepperBankSpi(object):
  def __init__(self, spi_bus, stepper_count):
    self.steppers = [None] * stepper_count
    self.spi_bus = spi_bus

  def add_stepper(self, index, stepper):
    self.steppers[index] = stepper

  def _update_status_from_transfer(self, data):
    for index, stepper in enumerate(self.steppers):
      if stepper is not None:
        reverse_index = len(self.steppers) - index - 1
        stepper.update_status(data[reverse_index * 5] & 0xf)

  def _single_register_transfer(self, index, addr, value, write):
    # 40 bit messages, first byte is the address
    # first build a no-op message to all steppers on the bus
    to_send = [addr, 0, 0, 0, 0] * len(self.steppers)

    # now fill in the message for the index we care about
    reverse_index = len(self.steppers) - index - 1
    # the first byte we write is going to end up at the last driver on the bus - indices are reversed
    if write:
      to_send[reverse_index * 5:(reverse_index + 1) * 5] = [addr | 0x80] + int_to_bytes(value)
    else:
      to_send[reverse_index * 5:(reverse_index + 1) * 5] = [addr, 0, 0, 0, 0]

    # first result is actually the stale result of whatever happened previously
    # logging.debug("send %s: %s", "write" if write else "read1", format_transaction(to_send))
    stale_data = self.spi_bus.xfer(to_send)
    # logging.debug("recv stale: %s", format_transaction(stale_data))
    self._update_status_from_transfer(stale_data)

    # if this is a write, that's all we need
    if write:
      return

    # the second time we get the results of the first command
    data = self.spi_bus.xfer(to_send)
    self._update_status_from_transfer(data)
    #logging.debug("send read2: %s", format_transaction(to_send))
    #logging.debug("recv read2: %s", format_transaction(data))

    # the output ordering is also reversed - the last stepper on the bus gets the first result in the list
    self.steppers[index].update_register(
        addr, bytes_to_int(data[reverse_index * 5 + 1:(reverse_index + 1) * 5]))

  def read_single_register(self, index, addr):
    self._single_register_transfer(index, addr, 0, False)

  def write_single_register(self, index, addr):
    self._single_register_transfer(index, addr, self.steppers[index].get_register_value(addr), True)

  def _bulk_register_transfer(self, addr, values, write):
    if write and len(values) != len(self.steppers):
      raise RuntimeError("SPI bulk write failed - value count must match stepper count")

    if write:
      to_send = []
      for index in reversed(range(len(self.steppers))):
        value = values[index]
        if value is not None:
          to_send += [addr | 0x80] + int_to_bytes(value)
        else:
          to_send += [0] * 5
    else:
      to_send = [addr, 0, 0, 0, 0] * len(self.steppers)

    stale_data = self.spi_bus.xfer(to_send)
    self._update_status_from_transfer(stale_data)

    if write:
      return

    data = self.spi_bus.xfer(to_send)
    self._update_status_from_transfer(data)

    for index in range(len(self.steppers)):
      if self.steppers[index] is not None:
        reverse_index = len(self.steppers) - index - 1
        stepper_data = bytes_to_int(data[reverse_index * 5 + 1:(reverse_index + 1) * 5])
        self.steppers[index].update_register(addr, stepper_data)

  def read_all_registers(self, addr):
    self._bulk_register_transfer(addr, None, False)

  def write_all_registers(self, addr):
    values = [
        stepper.get_register_value(addr) if stepper is not None else None
        for stepper in self.steppers
    ]
    self._bulk_register_transfer(addr, values, True)


c_uint = ctypes.c_uint32

# don't let the formatter mangle these
# yapf: disable

class SPI_STATUS_bits(ctypes.Structure):
  _fields_ = [
      ("reset_flag", c_uint, 1),
      ("driver_error", c_uint, 1),
      ("sg2", c_uint, 1),
      ("standstill", c_uint, 1),
  ]


class SPI_STATUS_data(ctypes.Union):
  _fields_ = [("bits", SPI_STATUS_bits), ("register", ctypes.c_uint8)]


class GCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("I_scale_analog", c_uint, 1),
      ("internal_Rsense", c_uint, 1),
      ("en_pwm_mode", c_uint, 1),
      ("enc_commutation", c_uint, 1),
      ("shaft", c_uint, 1),
      ("diag0_error", c_uint, 1),
      ("diag0_otpw", c_uint, 1),
      ("diag0_stall", c_uint, 1),
      ("diag1_stall", c_uint, 1),
      ("diag1_index", c_uint, 1),
      ("diag1_onstate", c_uint, 1),
      ("diag1_steps_skipped", c_uint, 1),
      ("diag0_int_pushpull", c_uint, 1),
      ("diag1_pushpull", c_uint, 1),
      ("small_hysteresis", c_uint, 1),
      ("stop_enable", c_uint, 1),
      ("direct_mode", c_uint, 1),
      ("test_mode", c_uint, 1),
  ]


class GCONF_Register_data(ctypes.Union):
  _fields_ = [("bits", GCONF_Register_bits), ("register", c_uint)]


class GSTAT_Register_bits(ctypes.Structure):
  _fields_ = [
      ("reset", c_uint, 1),
      ("drv_err", c_uint, 1),
      ("uv_cp", c_uint, 1)
  ]


class GSTAT_Register_data(ctypes.Union):
  _fields_ = [("bits", GSTAT_Register_bits), ("register", c_uint)]


class IOIN_Register_bits(ctypes.Structure):
  _fields_ = [
      ("STEP", c_uint, 1),
      ("DIR", c_uint, 1),
      ("DCEN_CFG4", c_uint, 1),
      ("DCIN_CFG5", c_uint, 1),
      ("DRV_ENN_CFG6", c_uint, 1),
      ("DCO", c_uint, 1),
      ("always_1", c_uint, 1),
      ("dont_care", c_uint, 1),
      ("unused", c_uint, 16),
      ("VERSION", c_uint, 8),
  ]


class IOIN_Register_data(ctypes.Union):
  _fields_ = [("bits", IOIN_Register_bits), ("register", c_uint)]


class IHOLD_IRUN_Register_bits(ctypes.Structure):
  _fields_ = [
      ("IHOLD", c_uint, 5),
      ("unused1", c_uint, 3),
      ("IRUN", c_uint, 5),
      ("unused2", c_uint, 3),
      ("IHOLDDELAY", c_uint, 4),
      ("unused3", c_uint, 12)
  ]


class IHOLD_IRUN_Register_data(ctypes.Union):
  _fields_ = [("bits", IHOLD_IRUN_Register_bits), ("register", c_uint)]


class CHOPCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("toff", c_uint, 4),
      ("hstrt", c_uint, 3),
      ("hend", c_uint, 4),
      ("fd3", c_uint, 1),
      ("disfdcc", c_uint, 1),
      ("rndtf", c_uint, 1),
      ("chm", c_uint, 1),
      ("tbl", c_uint, 2),
      ("vsense", c_uint, 1),
      ("vhighfs", c_uint, 1),
      ("vhighchm", c_uint, 1),
      ("sync", c_uint, 4),
      ("mres", c_uint, 4),
      ("intpol", c_uint, 1),
      ("dedge", c_uint, 1),
      ("diss2g", c_uint, 1),
      ("reserved", c_uint, 1)
  ]


class CHOPCONF_Register_data(ctypes.Union):
  _fields_ = [("bits", CHOPCONF_Register_bits), ("register", c_uint)]


class COOLCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("semin", c_uint, 4),
      ("reserved", c_uint, 1),
      ("seup", c_uint, 2),
      ("reserved2", c_uint, 1),
      ("semax", c_uint, 4),
      ("reserved3", c_uint, 1),
      ("sedn", c_uint, 2),
      ("seimin", c_uint, 1),
      ("sgt", c_uint, 7),
      ("reserved4", c_uint, 1),
      ("sfilt", c_uint, 1),
      ("reserved5", c_uint, 7),
  ]


class COOLCONF_Register_data(ctypes.Union):
  _fields_ = [("bits", COOLCONF_Register_bits), ("register", c_uint)]


class PWMCONF_Register_bits(ctypes.Structure):
  _fields_ = [
      ("pwm_ampl", c_uint, 8),
      ("pwm_grad", c_uint, 8),
      ("pwm_freq", c_uint, 2),
      ("pwm_autoscale", c_uint, 1),
      ("pwm_symmetric", c_uint, 1),
      ("freewheel", c_uint, 2),
      ("reserved", c_uint, 10),
  ]


class PWMCONF_Register_data(ctypes.Structure):
  _fields_ = [("bits", PWMCONF_Register_bits), ("register", c_uint)]


class DRV_STATUS_Register_bits(ctypes.Structure):
  _fields_ = [
      ("SG_RESULT", c_uint, 10),
      ("reserved", c_uint, 5),
      ("fsactive", c_uint, 1),
      ("CS_ACTUAL", c_uint, 5),
      ("reserved", c_uint, 3),
      ("stallGuard", c_uint, 1),
      ("ot", c_uint, 1),
      ("otpw", c_uint, 1),
      ("s2ga", c_uint, 1),
      ("s2gb", c_uint, 1),
      ("ola", c_uint, 1),
      ("olb", c_uint, 1),
      ("stst", c_uint, 1),
  ]


class DRV_STATUS_Register_data(ctypes.Structure):
  _fields_ = [("bits", DRV_STATUS_Register_bits), ("register", c_uint)]


class Generic_Register_data(ctypes.Union):
  _fields_ = [("register", c_uint)]

# yapf: enable


class Register(object):
  def __init__(self, name, addr, mode, data):
    self.name = name
    self.addr = addr
    self.mode = mode
    self.data = data
    self.data.register = 0

  def can_readback(self):
    return 'r' in self.mode and not 'c' in self.mode


class Stepper_TMC2130(Stepper):
  def __init__(self, stepPin, dirPin, faultPin, name, bank, bank_index):
    Stepper.__init__(self, stepPin, dirPin, faultPin, -1, name)
    logging.debug("Adding stepper with step {}, dir {}".format(stepPin, dirPin))

    self.name = name
    self.bank = bank
    self.bank_index = bank_index
    self.bank.add_stepper(bank_index, self)

    self.status = SPI_STATUS_data()

    self.gconf = Register("gconf", 0x0, "rw", GCONF_Register_data())
    self.gstat = Register("gstat", 0x01, "rc", GSTAT_Register_data())
    self.ioin = Register("ioin", 0x04, "r", IOIN_Register_data())
    self.ihold_irun = Register("ihold_irun", 0x10, "w", IHOLD_IRUN_Register_data())
    self.tstep = Register("tstep", 0x12, "r", Generic_Register_data())
    self.xdirect = Register("xdirect", 0x2D, "rw", Generic_Register_data())
    self.chopconf = Register("chopconf", 0x6C, "rw", CHOPCONF_Register_data())
    self.coolconf = Register("coolconf", 0x6D, "w", COOLCONF_Register_data())
    self.drv_status = Register("drv_status", 0x6F, "r", DRV_STATUS_Register_data())
    self.pwmconf = Register("pwmconf", 0x70, "w", PWMCONF_Register_data())
    self.pwm_scale = Register("pwm_scale", 0x71, "r", Generic_Register_data())
    self.lost_steps = Register("lost_steps", 0x73, "r", Generic_Register_data())

    self.registers = [
        self.gconf, self.gstat, self.ioin, self.ihold_irun, self.tstep, self.xdirect, self.chopconf,
        self.coolconf, self.drv_status, self.pwmconf, self.pwm_scale, self.lost_steps
    ]
    self.registers_by_addr = {}
    for register in self.registers:
      self.registers_by_addr[register.addr] = register

  def initialize_registers(self):
    self.gconf.data.bits.diag0_error = 1
    self.gconf.data.bits.diag0_otpw = 1
    # Elias observed that these are inverted from the datasheet?
    self.gconf.data.bits.diag0_stall = 1
    self.gconf.data.bits.diag1_stall = 1

    self.chopconf.data.bits.diss2g = 1
    self.chopconf.data.bits.vsense = 1 # note that this affects the current calculations below
    self.chopconf.data.bits.tbl = 2
    self.chopconf.data.bits.toff = 0 # this is changed below

    self.set_microstepping(6)    # writes gconf and chopconf

    self.coolconf.data.bits.sgt = 0xf
    self.coolconf.data.bits.semax = 2
    self.coolconf.data.bits.semin = 1

    self.chopconf.data.bits.toff = 3 # set but don't write this so we start out "disabled"

    self._write_register(self.coolconf)

    self.ihold_irun.data.bits.IHOLD = 3
    self.ihold_irun.data.bits.IRUN = 8
    self.ihold_irun.data.bits.IHOLDDELAY = 10

    self._write_register(self.ihold_irun)

    self.current_enabled = False
    self._read_register(self.gstat)

  def sanity_test(self):
    errors = 0

    self._read_register(self.ioin)
    if self.ioin.data.bits.VERSION != 0x11:
      logging.error("sanity test failed for stepper %s - expected version %x, ioin was actually %x",
                    self.name, 0x11, self.ioin.data.register)
      errors += 1
    else:
      # logging.debug("version check succeeded for stepper %s", self.name)
      pass

    magic = 0x00550021 + self.bank_index
    self.xdirect.data.register = magic
    self._write_register(self.xdirect)
    self._read_register(self.xdirect)
    if self.xdirect.data.register != magic:
      logging.error("sanity test failed for stepper %s: %x instead of %x", self.name,
                    self.xdirect.data.register, magic)
      errors += 1
      if self.xdirect.data.register != 0:
        logging.error("SANITY TEST MISMATCH FOR STEPPER %s - my index is %d, magic index is %d",
                      self.name, self.bank_index, self.xdirect.data.register - 0x12345678)
    else:
      # logging.debug("sanity test succeeded for stepper %s", self.name)
      pass

    self.xdirect.data.register = 0
    self._write_register(self.xdirect)

    self.step_pin.disable()
    self.dir_pin.disable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.STEP != 0 or self.ioin.data.bits.DIR != 0:
      errors += 1
      logging.error("sanity test failed for stepper %s: ioin was %x but step and dir were zero",
                    self.name, self.ioin.data.register)

    self.step_pin.enable()
    self.dir_pin.enable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.STEP != 1 or self.ioin.data.bits.DIR != 1:
      logging.error("sanity test failed for stepper %s: ioin was %x but step and dir were one",
                    self.name, self.ioin.data.register)
      errors += 1

    self.step_pin.disable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.STEP != 0 or self.ioin.data.bits.DIR != 1:
      logging.error(
          "sanity test failed for stepper %s: ioin was %x but step was zero and dir was one",
          self.name, self.ioin.data.register)
      errors += 1

    self.dir_pin.disable()

    self._read_register(self.ioin)
    if self.ioin.data.bits.STEP != 0 or self.ioin.data.bits.DIR != 0:
      logging.error(
          "sanity test failed for stepper %s: ioin was %x but step and dir were zero (second time)",
          self.name, self.ioin.data.register)
      errors += 1

    logging.debug("sanity test complete for stepper %s, %d errors", self.name, errors)

  def _read_register(self, register):
    self.bank.read_single_register(self.bank_index, register.addr)

  def _write_register(self, register):
    logging.debug("stepper %s writing register %s as %x", self.name, register.name,
                  register.data.register)
    self.bank.write_single_register(self.bank_index, register.addr)

    #if register.can_readback():
    #  desired_value = register.data.register
    #  self._read_register(register)
    #  if desired_value != register.data.register:
    #    logging.error("%s write failed - wrote %x, read %x", register.name, desired_value, register.data.register)
    #  else:
    #    # logging.debug("%s write succeeded", register.name)
    #    pass

  def get_register_value(self, addr):
    return self.registers_by_addr[addr].data.register

  def update_register(self, addr, value):
    register = self.registers_by_addr[addr]
    logging.debug("stepper %s read register %s to be %x", self.name, register.name, value)
    register.data.register = value

  def update_status(self, value):
    if value != self.status.register:
      logging.debug("stepper %s status changed to %x", self.name, value)
      self.status.register = value

  def get_status(self):
    return self.status.register

  def set_microstepping(self, value, force_update=False):
    stealthchop = False
    interpolation = False

    if value == 0:
      # full step
      self.microsteps = 1
    elif value == 1:
      # half step
      self.microsteps = 2
    elif value == 2:
      # half step with 256 microstep interpolation
      interpolation = True
      self.microsteps = 2
    elif value == 3:
      # quarter step
      self.microsteps = 4
    elif value == 4:
      # 16th step
      self.microsteps = 16
    elif value == 5:
      # quarter step with 256 microstep interpolation
      self.microsteps = 4
      interpolation = True
    elif value == 6:
      # 16th step with 256 microstep interpolation
      self.microsteps = 16
      interpolation = True
    elif value == 7:
      # quarter step, stealthchop, 256 microstep interpolation
      self.microsteps = 4
      stealthchop = True
      interpolation = True
    elif value == 8:
      # 16th step, stealthchop, 256 microstep interpolation
      self.microsteps = 16
      stealthchop = True
      interpolation = True

    self.gconf.data.bits.en_pwm_mode = 1 if stealthchop else 0
    self.chopconf.data.bits.intpol = 1 if interpolation else 0

    # the actual formula is microsteps = 256/2^MRES, so log2(256/microsteps) = MRES
    self.chopconf.data.bits.mres = int(0.5 + math.log(256.0 / self.microsteps, 2))

    self._write_register(self.gconf)
    self._write_register(self.chopconf)

  def set_current_value(self, i_rms):
    # we have 0.1 ohm sense resistors with high sensitivity enabled (vsense = 1)
    # calculations here come from Trinamics TMC2130 spec sheet, page 55
    # our peak RMS current with IRUN==31 is 1.06A

    # I_rms = (current_scale + 1) / 32 * I_peak
    # which solves to...
    current_scale = int(0.5+ (32 * i_rms / 1.06 - 1))

    current_scale = max(0, min(31, current_scale))

    self.ihold_irun.data.bits.IRUN = current_scale
    self.ihold_irun.data.bits.IHOLD = max(0, int(current_scale * 0.33 + 0.5))
    self._write_register(self.ihold_irun)

  def set_disabled(self, force_update=False):
    # preserve the register value for when we re-enable
    original_value = self.chopconf.data.register

    self.chopconf.data.bits.toff = 0
    self._write_register(self.chopconf)
    self.current_enabled = False

    self.chopconf.data.register = original_value

  def set_enabled(self, force_update=False):
    self._write_register(self.chopconf)
    self.current_enabled = True

  def set_decay(self, value):
    self.decay = value
    # TODO stub

  def reset(self):
    self.set_disabled()
    # TODO this could make sense but needs the registers to be tagged with read/write/clear information
    #for register in self.registers:
    #  self._write_register(register)
    self.set_enabled()

  def set_current_disabled(self):
    self.set_disabled()

  def set_current_enabled(self):
    self.set_enabled()
