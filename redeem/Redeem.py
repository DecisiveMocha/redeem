#!/usr/bin/python
"""
Redeem main program. This should run on the BeagleBone.

Author: Elias Bakken
email: elias(dot)bakken(at)gmail(dot)com
Website: http://www.thing-printer.com
License: GNU GPL v3: http://www.gnu.org/copyleft/gpl.html

 Redeem is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Redeem is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Redeem.  If not, see <http://www.gnu.org/licenses/>.


Minor version tag is Arnold Schwarzenegger movies chronologically.
"""

import glob
import shutil
import logging
import os
import os.path
import signal
from threading import Thread
from multiprocessing import JoinableQueue
import Queue
import numpy as np
import sys

from Mosfet import Mosfet
from Stepper import Stepper
from Thermistor import Thermistor
from Fan import Fan
from Servo import Servo
from EndStop import EndStop
from USB import USB
from Pipe import Pipe
from Ethernet import Ethernet
from Extruder import Extruder, HBP
from Cooler import Cooler
from Path import Path
from PathPlanner import PathPlanner
from ColdEnd import ColdEnd
from PruFirmware import PruFirmware
from CascadingConfigParser import CascadingConfigParser
from Printer import Printer
from GCodeProcessor import GCodeProcessor
from PluginsController import PluginsController
from Delta import Delta


version = "0.16.8~The Terminator"

# Default logging level is set to debug
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M')


class Redeem:

    def __init__(self):
        """ Init """
        logging.info("Redeem initializing " + version)

        printer = Printer()
        self.printer = printer

        # Copy/create config files if not present
        if not os.path.exists("/etc/redeem/default.cfg"):
            dirname = os.path.dirname(os.path.realpath(__file__))
            logging.warning("/etc/redeem/default.cfg does not exist, copying it...")
            for f in glob.glob(dirname+"/../configs/*.cfg"):
                logging.warning(f)
                shutil.copy(f, "/etc/redeem")

        # Parse the config files.
        printer.config = CascadingConfigParser(
            ['/etc/redeem/default.cfg', '/etc/redeem/printer.cfg',
             '/etc/redeem/local.cfg'])

        # Find out which capes are connected
        self.printer.config.parse_capes()
        self.revision = self.printer.config.replicape_revision
        if self.revision:
            logging.info("Found Replicape rev. " + self.revision)
            Path.set_axes(5)
        else:
            logging.warning("Oh no! No Replicape present!")
            self.revision = "0A4A"
            # We set it to 5 axis by default
            Path.set_axes(5)
        if self.printer.config.reach_revision:
            Path.set_axes(8)
            logging.info("Found Reach rev. "+self.printer.config.reach_revision)

        # Get the revision and loglevel from the Config file
        level = self.printer.config.getint('System', 'loglevel')
        if level > 0:
            logging.getLogger().setLevel(level)

        # Init the Paths
        Path.axis_config = printer.config.getint('Geometry', 'axis_config')

        # Init the end stops
        EndStop.callback = self.end_stop_hit
        EndStop.inputdev = self.printer.config.get("Endstops", "inputdev")
        for es in ["X1", "X2", "Y1", "Y2", "Z1", "Z2"]:
            pin = self.printer.config.get("Endstops", "pin_"+es)
            keycode = self.printer.config.getint("Endstops", "keycode_"+es)
            invert = self.printer.config.getboolean("Endstops", "invert_"+es)
            self.printer.end_stops[es] = EndStop(pin, keycode, es, invert)

        # Backwards compatibility with A3
        if self.revision == "00A3":
            Stepper.revision = "A3"
            Stepper.ENABLED = 6
            Stepper.SLEEP = 5
            Stepper.RESET = 4
            Stepper.DECAY = 0

        # Init the 5 Stepper motors (step, dir, fault, DAC channel, name)
        printer.steppers["X"] = Stepper("GPIO0_27", "GPIO1_29", "GPIO2_4" , 0, "X", 0, 0)
        printer.steppers["Y"] = Stepper("GPIO1_12", "GPIO0_22", "GPIO2_5" , 1, "Y", 1, 1)
        printer.steppers["Z"] = Stepper("GPIO0_23", "GPIO0_26", "GPIO0_15", 2, "Z", 2, 2)
        printer.steppers["E"] = Stepper("GPIO1_28", "GPIO1_15", "GPIO2_1" , 3, "E", 3, 3)
        printer.steppers["H"] = Stepper("GPIO1_13", "GPIO1_14", "GPIO2_3" , 4, "H", 4, 4)

        if printer.config.reach_revision:
            printer.steppers["A"] = Stepper("GPIO2_2" , "GPIO1_18", "GPIO0_14", 5, "A", 5, 5)
            printer.steppers["B"] = Stepper("GPIO1_14", "GPIO0_5" , "GPIO0_14", 6, "B", 6, 6)
            printer.steppers["C"] = Stepper("GPIO0_3" , "GPIO3_19", "GPIO0_14", 7, "C", 7, 7)

        # Enable the steppers and set the current, steps pr mm and
        # microstepping
        for name, stepper in self.printer.steppers.iteritems():
            stepper.in_use = printer.config.getboolean('Steppers', 'in_use_' + name)
            stepper.direction = printer.config.getint('Steppers', 'direction_' + name)
            stepper.has_endstop = printer.config.getboolean('Endstops', 'has_' + name)
            stepper.set_current_value(printer.config.getfloat('Steppers', 'current_' + name))
            stepper.set_steps_pr_mm(printer.config.getfloat('Steppers', 'steps_pr_mm_' + name))
            stepper.set_microstepping(printer.config.getint('Steppers', 'microstepping_' + name))
            stepper.set_decay(printer.config.getboolean("Steppers", "slow_decay_" + name))
            # Add soft end stops
            Path.soft_min[Path.axis_to_index(name)] = printer.config.getfloat('Endstops', 'soft_end_stop_min_' + name)
            Path.soft_max[Path.axis_to_index(name)] = printer.config.getfloat('Endstops', 'soft_end_stop_max_' + name)

        # Commit changes for the Steppers
        Stepper.commit()

        # Delta printer setup
        if Path.axis_config == Path.AXIS_CONFIG_DELTA:
            opts = ["Hez", "L", "r", "Ae", "Be", "Ce", "Aco",
                    "Bco", "Cco", "Apxe", "Apye", "Bpxe", "Bpye",
                    "Cpxe", "Cpye"]

            for opt in opts:
                Delta.__dict__[opt] = printer.config.getfloat('Delta', opt)

            Delta.recalculate()

        # Set up cold ends
        path = self.printer.config.get('Cold-ends', 'path', 0)
        if os.path.exists(path):
            self.printer.cold_ends.append(ColdEnd(path, "Cold End 0"))
            logging.info("Found Cold end on " + path)
        else:
            logging.info("No cold end present in path: " + path)

        # Make Mosfets, thermistors and extruders
        heaters = ["E", "H", "HBP"]
        if self.printer.config.reach_revision:
            heaters.extend(["A", "B", "C"])
        for e in heaters:
            # Mosfets
            channel = self.printer.config.getint("Heaters", "mosfet_"+e)
            self.printer.mosfets[e] = Mosfet(channel)
            # Thermistors
            adc = self.printer.config.get("Heaters", "path_adc_"+e)
            chart = self.printer.config.get("Heaters", "temp_chart_"+e)
            self.printer.thermistors[e] = Thermistor(adc, "MOSFET "+e, chart)

            # Extruders
            onoff = self.printer.config.getboolean('Heaters', 'onoff_'+e)
            prefix =  self.printer.config.get('Heaters', 'prefix_'+e)
            if e != "HBP":
                self.printer.heaters[e] = Extruder(
                                        self.printer.steppers[e],
                                        self.printer.thermistors[e], 
                                        self.printer.mosfets[e], e, onoff)
            else:
                self.printer.heaters[e] = HBP(
                                        self.printer.thermistors[e],
                                        self.printer.mosfets[e], onoff)
            self.printer.heaters[e].prefix = prefix
            self.printer.heaters[e].P = self.printer.config.getfloat('Heaters', 'pid_p_'+e)
            self.printer.heaters[e].I = self.printer.config.getfloat('Heaters', 'pid_i_'+e)
            self.printer.heaters[e].D = self.printer.config.getfloat('Heaters', 'pid_d_'+e)

        # Init the three fans. Argument is PWM channel number
        self.printer.fans = []
        if self.revision == "00A3":
            self.printer.fans.append(Fan(0))
            self.printer.fans.append(Fan(1))
            self.printer.fans.append(Fan(2))
        else:
            self.printer.fans.append(Fan(8))
            self.printer.fans.append(Fan(9))
            self.printer.fans.append(Fan(10))

        Fan.set_PWM_frequency(100)

        for f in self.printer.fans:
            f.set_value(0)

        # Init the servos
        printer.servos = []
        servo_nr = 0
        while(printer.config.has_option("Servos", "servo_"+str(servo_nr)+"_enable")):
            if printer.config.getboolean("Servos", "servo_"+str(servo_nr)+"_enable"):
                channel = printer.config.getint("Servos", "servo_"+str(servo_nr)+"_channel")
                angle_off = printer.config.getint("Servos", "servo_"+str(servo_nr)+"_angle_off")
                s = Servo(channel, 500, 750, angle_off)
                s.angle_on = printer.config.getint("Servos", "servo_"+str(servo_nr)+"_angle_on")
                s.angle_off = angle_off
                printer.servos.append(s)
                logging.info("Added servo "+str(servo_nr))
            servo_nr += 1

        # Connect thermitors to fans
        for t, therm in self.printer.heaters.iteritems():
            for f, fan in enumerate(self.printer.fans):
                if self.printer.config.getboolean('Cold-ends', "connect-therm-{}-fan-{}".format(t, f)):
                    c = Cooler(therm, fan, "Cooler-{}-{}".format(t, f), False)
                    c.ok_range = 4
                    c.set_target_temperature(60)
                    c.enable()
                    self.printer.coolers.append(c)
                    logging.info("Cooler connects therm {} with fan {}".format(t, f))

        # Connect the cold end 0 to fan 2
        # This is very "Thing" specific, should be configurable somehow.
        if len(self.printer.cold_ends):
            self.printer.coolers.append(
                Cooler(self.printer.cold_ends[0], self.printer.fans[2],
                       "Cooler0", False))
            self.printer.coolers[0].ok_range = 4
            self.printer.coolers[0].set_target_temperature(60)
            self.printer.coolers[0].enable()

        # Make a queue of commands
        self.printer.commands = JoinableQueue(10)

        # Make a queue of commands that should not be buffered
        self.printer.sync_commands = JoinableQueue()
        self.printer.unbuffered_commands = JoinableQueue(10)

        # Bed compensation matrix
        Path.matrix_bed_comp = printer.load_bed_compensation_matrix()
        Path.matrix_bed_comp_inv = np.linalg.inv(Path.matrix_bed_comp)
        logging.debug("Loaded bed compensation matrix: \n"+str(Path.matrix_bed_comp))

        for axis in printer.steppers.keys():
            i = Path.axis_to_index(axis)
            Path.max_speeds[i] = printer.config.getfloat('Steppers', 'max_speed_'+axis.lower())
            Path.home_speed[i] = printer.config.getfloat('Steppers', 'home_speed_'+axis.lower())
            Path.steps_pr_meter[i] = printer.steppers[axis].get_steps_pr_meter()
            Path.backlash_compensation[i] = printer.config.getfloat('Steppers', 'backlash_'+axis.lower())

        dirname = os.path.dirname(os.path.realpath(__file__))

        # Create the firmware compiler
        pru_firmware = PruFirmware(
            dirname + "/firmware/firmware_runtime.p",
            dirname + "/firmware/firmware_runtime.bin",
            dirname + "/firmware/firmware_endstops.p",
            dirname + "/firmware/firmware_endstops.bin",
            self.revision, self.printer.config, "/usr/bin/pasm")

        printer.maxJerkXY = printer.config.getfloat('Steppers', 'maxJerk_xy')
        printer.maxJerkZ = printer.config.getfloat('Steppers', 'maxJerk_z')
        printer.maxJerkEH = printer.config.getfloat('Steppers', 'maxJerk_eh')

        self.printer.processor = GCodeProcessor(self.printer)
        self.printer.plugins = PluginsController(self.printer)

        # Path planner
        self.printer.path_planner = PathPlanner(self.printer, pru_firmware)
        for axis in printer.steppers.keys():
            i = Path.axis_to_index(axis)

            # Sometimes soft_end_stop aren't defined to be at the exact hardware boundary.
            # Adding 100mm for searching buffer.
            printer.path_planner.travel_length[axis] = \
                printer.config.getfloat('Geometry', 'travel_' + axis.lower()) \
                if printer.config.has_option('Geometry', 'travel_' + axis.lower()) \
                else (Path.soft_max[i] - Path.soft_min[i]) + .1            
            
            printer.path_planner.center_offset[axis] = \
                printer.config.getfloat('Geometry', 'offset_' + axis.lower()) \
                if printer.config.has_option('Geometry', 'offset_' + axis.lower()) \
                else (Path.soft_min[i] if Path.home_speed[i] > 0 else Path.soft_max[i])

            printer.path_planner.home_pos[axis] = \
                printer.config.getfloat('Geometry', 'home_' + axis.lower()) \
                if printer.config.has_option('Geometry', 'home_' + axis.lower()) \
                else printer.path_planner.center_offset[axis]

            printer.acceleration[Path.axis_to_index(axis)] = printer.config.getfloat(
                                                        'Steppers', 'acceleration_' + axis.lower())


        # Set up communication channels
        printer.comms["USB"] = USB(self.printer)
        printer.comms["Eth"] = Ethernet(self.printer)

        if Pipe.check_tty0tty() or Pipe.check_socat():
            printer.comms["octoprint"] = Pipe(printer, "octoprint")
            printer.comms["toggle"] = Pipe(printer, "toggle")
            printer.comms["testing"] = Pipe(printer, "testing")
            printer.comms["testing_noret"] = Pipe(printer, "testing_noret")
            # Does not send "ok"
            printer.comms["testing_noret"].send_response = False
        else:
            logging.warning("Neither tty0tty or socat is installed! No virtual tty pipes enabled")

    def start(self):
        """ Start the processes """
        self.running = True
        # Start the two processes
        p0 = Thread(target=self.loop,
                    args=(self.printer.commands, "buffered"))
        p1 = Thread(target=self.loop,
                    args=(self.printer.unbuffered_commands, "unbuffered"))
        p2 = Thread(target=self.eventloop,
                    args=(self.printer.sync_commands, "sync"))
        p0.daemon = True
        p1.daemon = True
        p2.daemon = True

        p0.start()
        p1.start()
        p2.start()

        # Signal everything ready
        logging.info("Redeem ready")

    def loop(self, queue, name):
        """ When a new gcode comes in, execute it """
        try:
            while self.running:
                try:
                    gcode = queue.get(block=True, timeout=1)
                except Queue.Empty:
                    continue

                logging.debug("Executing "+gcode.code()+" from "+name + " " + gcode.message)

                self._execute(gcode)

                self.printer.reply(gcode)

                queue.task_done()
        except Exception:
            logging.exception("Exception in {} loop: ".format(name))

    def eventloop(self, queue, name):
        """ When a new event comes in, execute the pending gcode """
        try:

            while self.running:
                # Returns False on timeout, else True
                if self.printer.path_planner.wait_until_sync_event():
                    try:
                        gcode = queue.get(block=True, timeout=1)
                    except Queue.Empty:
                        logging.error("Unsolicited Sync event occured.")
                        continue

                    self._synchronize(gcode)
                    logging.info("Event handled for " + gcode.code() + " from " + name + " " + gcode.message)
                    queue.task_done()
        except Exception:
            logging.exception("Exception in {} eventloop: ".format(name))

    def exit(self):
        logging.info("Redeem starting exit")
        self.running = False
        self.printer.path_planner.wait_until_done()
        self.printer.path_planner.force_exit()

        # Stops plugins
        self.printer.plugins.exit()

        for name, stepper in self.printer.steppers.iteritems():
            stepper.set_disabled()
        Stepper.commit()

        for name, heater in self.printer.heaters.iteritems():
            logging.debug("closing "+name)
            heater.disable()

        for name, comm in self.printer.comms.iteritems():
            logging.debug("closing "+name)
            comm.close()
        logging.info("Redeem exited")

    def _execute(self, g):
        """ Execute a G-code """
        if g.message == "ok" or g.code() == "ok" or g.code() == "No-Gcode":
            g.set_answer(None)
            return
        if g.is_info_command():
            desc = self.printer.processor.get_long_description(g)
            self.printer.send_message(g.prot, desc)
        else:
            self.printer.processor.execute(g)

    def _synchronize(self, g):
        """ Syncrhonized execution of a G-code """
        self.printer.processor.synchronize(g)

    def end_stop_hit(self, endstop):
        """ An endStop has been hit """
        logging.warning("End Stop " + endstop.name + " hit!")


def main():
    # Create Redeem
    r = Redeem()

    def signal_handler(signal, frame):
        r.exit()

    # Register signal handler to allow interrupt with CTRL-C
    signal.signal(signal.SIGINT, signal_handler)

    # Launch Redeem
    r.start()

    # Wait for end of process signal
    signal.pause()



def profile():
    import yappi
    yappi.start()
    main()
    yappi.get_func_stats().print_all()

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == "profile":
        profile()
    else:
        main()
