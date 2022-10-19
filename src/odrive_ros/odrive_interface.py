
import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *
from odrive.utils import dump_errors

import fibre
from fibre.protocol import ChannelDamagedException
from fibre.protocol import ObjectLostError as ChannelBrokenException
from tf2_ros.buffer_interface import NotImplementedException

default_logger = logging.getLogger(__name__)
default_logger.setLevel(logging.DEBUG)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)

default_logger.addHandler(ch)

class ODriveFailure(Exception):
    pass

class ODriveInterfaceAPI(object):
    
    def __init__(self, logger=None):
        self.logger = logger if logger else default_logger
        self.flip_left_direction = False
        self.flip_right_direction = False

        self.odrives = dict()

        self.left_axes = []
        self.right_axes = []
     
    def connect(self, odrive_sn=None, left_sn=None, right_sn=None, right_axis=0, flip_left_direction=False, flip_right_direction=False, 
        torque_control=False, timeout=30, attempts=4):
        # Setup a single odrive with each axis controlling one side
        if odrive_sn is not None:
            self.logger.info("Single odrive mode")
            try:
                self.odrives["odrive"] = odrive.find_any(serial_number=odrive_sn)
            except:
                self.logger.error("No ODrive found. Is device powered?")
                return False

            if right_axis == 0:
                self.left_axes = [self.odrives["odrive"].axis1]
                self.right_axes = [self.odrives["odrive"].axis0]
            else:
                self.left_axes = [self.odrives["odrive"].axis0]
                self.right_axes = [self.odrives["odrive"].axis1]

        # Setup two odrives with one odrive per side
        elif left_sn is not None and right_sn is not None:
            self.logger.info("Dual odrive mode")

            try:
                self.odrives["left"] = odrive.find_any(serial_number=left_sn)
                self.odrives["right"] = odrive.find_any(serial_number=right_sn)
            except:
                self.logger.error("No ODrive found. Is device powered?")
                return False

            self.left_axes = [self.odrives["left"].axis0, self.odrives["left"].axis1]
            self.right_axes = [self.odrives["right"].axis0, self.odrives["right"].axis1]

        else:
            self.logger.error("Specify odrive_sn or both left_sn and right_sn")
            return False

        # Log connected odrives 
        for odrv_name in self.odrives:
            self.logger.info("Connected to %s %s" %(odrv_name,  self.get_version_string(self.odrives[odrv_name])))

        self.flip_left_direction = flip_left_direction
        self.flip_right_direction = flip_right_direction   
        self.torque_control = torque_control     

        self.disable_watchdog()
        self.get_errors(True)
        self.clear_watchdog()

        # Any watchdog timeouts will have been cleared. For all other errors we will reboot the odrive.
        errors = False
        for axis in self.left_axes + self.right_axes:
            if axis.error != 0:
                error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x." % (axis.error, axis.motor.error, axis.encoder.error)
                self.logger.error(error_str)
                errors = True
        if errors:
            for odrv in self.odrives.values():
                try:
                    self.logger.error("Rebooting odrive")
                    odrv.reboot()
                except fibre.libfibre.ObjectLostError:
                    self.logger.error("Odrive has disconnected")

            if attempts >= 0:
                self.logger.error("Reconnecting: Attempts remaining %d" %attempts)
                return self.connect(
                    odrive_sn=odrive_sn,
                    left_sn=left_sn,
                    right_sn=right_sn,
                    right_axis=right_axis,
                    flip_left_direction=flip_left_direction,
                    flip_right_direction=flip_right_direction,
                    timeout=timeout,
                    attempts=attempts-1)
            else:
                return False
        else:
            return True
        
    def get_version_string(self, odrv):
        return "ODrive %s, hw v%d.%d-%d, fw v%d.%d.%d%s, sdk v%s" % (
            str(odrv.serial_number),
            odrv.hw_version_major, odrv.hw_version_minor, odrv.hw_version_variant,
            odrv.fw_version_major, odrv.fw_version_minor, odrv.fw_version_revision,
            "-dev" if odrv.fw_version_unreleased else "",
            odrive.version.get_version_str())

    @property
    def prerolled(self):
        # true if all motors are prerolled
        if not self.connected:
            self.logger.error("Not connected.")
            return False

        for axis in self.left_axes + self.right_axes:
            if not axis.encoder.is_ready:
                return False
        return True

    @property
    def prerolling(self):
        # true if any motors are prerolling
        if not self.connected:
            self.logger.error("Not connected.")
            return False

        for axis in self.left_axes + self.right_axes:
            if axis.current_state == AXIS_STATE_ENCODER_OFFSET_CALIBRATION:
                return True
        return False

    def preroll(self, wait=True):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        
        if self.prerolled:
            return True
            
        if self.prerolling:
            self.logger.warn("Already prerolling")
            return False
            
        for axis in self.left_axes + self.right_axes:
            axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        if wait:
            while self.prerolling:
                time.sleep(0.1)

            for axis in self.left_axes + self.right_axes:
                if axis.error != 0:
                    self.logger.error("Failed preroll with left_axis error 0x%x, motor error 0x%x" % (axis.error, axis.motor.error))
                    return False
            
            if self.prerolled:
                self.logger.info("Index search preroll complete.")
                return True
        
        return False

    @property   
    def engaged(self):
        # returns true if all axes are engaged
        if not self.connected:
            self.logger.error("Not connected.")
            return False

        for axis in self.left_axes + self.right_axes:
            if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                return False
        return True 
    
    def idle(self):
        # returns true if all axes are idle
        if not self.connected:
            self.logger.error("Not connected.")
            return False

        for axis in self.left_axes + self.right_axes:
            if axis.current_state != AXIS_STATE_IDLE:
                return False
        return True 
        
    def engage(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False

        for axis in self.left_axes + self.right_axes:
            axis.controller.input_vel = 0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            if self.torque_control:
                axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            else:
                axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                axis.controller.config.vel_ramp_rate = 0.5
                axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
        return True
        
    def release(self):
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        for axis in self.left_axes + self.right_axes:
            axis.requested_state = AXIS_STATE_IDLE
        return True
    
    def drive(self, left_motor_val, right_motor_val):
        if not self.connected:
            self.logger.error("Not connected.")
            return
        
        for axis in self.left_axes:
            if self.torque_control:
                axis.controller.input_torque = self.flip_l(left_motor_val)
            else:
                axis.controller.input_vel = self.flip_l(left_motor_val)
        for axis in self.right_axes:
            if self.torque_control:
                axis.controller.input_torque = self.flip_r(right_motor_val)
            else:
                axis.controller.input_vel = self.flip_r(right_motor_val)

    def enable_watchdog(self, timeout=1):
        for axis in self.left_axes + self.right_axes:
            axis.config.watchdog_timeout = timeout
            axis.config.enable_watchdog = True

    def disable_watchdog(self):
        for axis in self.left_axes + self.right_axes:
            axis.config.enable_watchdog = False
 
    
    def feed_watchdog(self):
        for axis in self.left_axes + self.right_axes:
            axis.watchdog_feed()

    def clear_watchdog(self):
        # Clears watchdog timeout error will preserving other errors
        for axis in self.left_axes + self.right_axes:
            axis.error = axis.error & ~AXIS_ERROR_WATCHDOG_TIMER_EXPIRED

    @property
    def connected(self):
        for odrv in self.odrives.values():
            if odrv is None:
                return False
            if isinstance(odrv, fibre.libfibre.EmptyInterface):
                return False
        return True
        
    def get_errors(self, clear=True):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        
        if not self.connected:
            return "disconnected"

        axis_error = 0
        for axis in self.left_axes + self.right_axes:
            axis_error |= axis.error
            axis_error |= axis.motor.error
            axis_error |= axis.encoder.error
            axis_error |= axis.controller.error
        
        if axis_error:
            for odrv in self.odrives.values():
                dump_errors(odrv, self.logger.warn)

            error_string = "Errors(hex): "
            for axis in self.left_axes + self.right_axes:
                error_string += "a%x m%x e%x c%x " %(axis.error,  axis.motor.error,  axis.encoder.error,  axis.controller.error)
        
        if clear:
            for axis in self.left_axes + self.right_axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0
        
        if axis_error:
            return error_string
        return ""

    def flip_l(self, value):
        return -value if self.flip_left_direction else value

    def flip_r(self, value):
        return -value if self.flip_right_direction else value

    @property
    def left_vel_estimate(self):
        return self.flip_l( min([axis.encoder.vel_estimate for axis in self.left_axes]) ) 
    
    @property
    def right_vel_estimate(self):
        return self.flip_r( min([axis.encoder.vel_estimate for axis in self.right_axes]) )
    
    @property
    def left_pos(self):
        return self.flip_l(self.left_axes[0].encoder.pos_estimate)
    
    @property
    def right_pos(self):
        return self.flip_r(self.right_axes[0].encoder.pos_estimate)
    
    @property
    def left_temperature(self):   
        return max([axis.motor.fet_thermistor.temperature for axis in self.left_axes])

    @property
    def right_temperature(self):
        return max([axis.motor.fet_thermistor.temperature for axis in self.left_axes])
    
    @property
    def left_current(self):
        return sum([axis.motor.I_bus for axis in self.left_axes])

    @property
    def right_current(self):
        return sum([axis.motor.I_bus for axis in self.right_axes])

    # from axis.hpp: https://github.com/madcowswe/ODrive/blob/767a2762f9b294b687d761029ef39e742bdf4539/Firmware/MotorControl/axis.hpp#L26
    MOTOR_STATES = [
        "UNDEFINED",                  #<! will fall through to idle
        "IDLE",                       #<! disable PWM and do nothing
        "STARTUP_SEQUENCE",           #<! the actual sequence is defined by the config.startup_... flags
        "FULL_CALIBRATION_SEQUENCE",  #<! run all calibration procedures, then idle
        "MOTOR_CALIBRATION",          #//<! run motor calibration
        "SENSORLESS_CONTROL",         #//<! run sensorless control
        "ENCODER_INDEX_SEARCH",       #//<! run encoder index search
        "ENCODER_OFFSET_CALIBRATION", #//<! run encoder offset calibration
        "CLOSED_LOOP_CONTROL",        #//<! run closed loop control
        "LOCKIN_SPIN",                #//<! run lockin spin
        "ENCODER_DIR_FIND",
        ]
        
    def left_state(self):       return self.MOTOR_STATES[self.left_axis.current_state] if self.left_axis else "NOT_CONNECTED"
    def right_state(self):      return self.MOTOR_STATES[self.right_axis.current_state] if self.right_axis else "NOT_CONNECTED"
    
    @property
    def bus_voltage(self):
        v_max = 0
        for odrv in self.odrives.values():
            v_max = max(odrv.vbus_voltage, v_max)
        return v_max
    
