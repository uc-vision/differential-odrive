
import serial
from serial.serialutil import SerialException

import sys
import time
import logging
import traceback

import odrive
from odrive.enums import *

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
    driver = None
    encoder_cpr = 10000
    right_axis = None
    left_axis = None
    
    def __init__(self, logger=None):
        self.logger = logger if logger else default_logger
        self.flip_left_direction = False
        self.flip_right_direction = False
        self.left_odrive = None
        self.right_odrive = None

        self.left_axes = []
        self.right_axes = []
              
    def update_time(self, curr_time):
        # provided so simulator can update position
        pass
                    
    def connect(self, port=None, left_sn=None, right_sn=None, flip_left_direction=False, flip_right_direction=False, timeout=30):
        self.flip_left_direction = flip_left_direction
        self.flip_right_direction = flip_right_direction
        if self.left_odrive:
            self.logger.info("Left already connected. Disconnecting and reconnecting.")
        if self.right_odrive:
            self.logger.info("Right already connected. Disconnecting and reconnecting.")
        
        try:
            self.left_odrive = odrive.find_any(serial_number=left_sn)
            self.right_odrive = odrive.find_any(serial_number=right_sn)
        except:
            self.logger.error("No ODrive found. Is device powered?")
            return False
        
        self.left_axes = [self.left_odrive.axis0, self.left_odrive.axis1]
        self.right_axes = [self.right_odrive.axis0, self.right_odrive.axis1]

        # check for no errors
        for axis in self.left_axes + self.right_axes:
            if axis.error != 0:
                error_str = "Had error on startup, rebooting. Axis error 0x%x, motor error 0x%x, encoder error 0x%x. Rebooting." % (axis.error, axis.motor.error, axis.encoder.error)
                self.logger.error(error_str)
                self.reboot()
                return False
        
        # Note: assumes all motors have the same CPR
        self.encoder_cpr = self.left_odrive.axis0.encoder.config.cpr
        
        self.logger.info("Connected to left odrive" + self.get_version_string(self.left_odrive))
        self.logger.info("Connected to right odrive. " + self.get_version_string(self.right_odrive))
               
        return True
        
    def disconnect(self):
        raise NotImplementedException()
        self.right_axis = None
        self.left_axis = None
        
        #self.engaged = False
        
        if not self.driver:
            self.logger.error("Not connected.")
            return False
        
        try:
            self.release()
        except:
            self.logger.error("Error in timer: " + traceback.format_exc())
            return False
        finally:
            self.driver = None
        return True
        
    def get_version_string(self, odrv):
        return "ODrive %s, hw v%d.%d-%d, fw v%d.%d.%d%s, sdk v%s" % (
            str(odrv.serial_number),
            odrv.hw_version_major, odrv.hw_version_minor, odrv.hw_version_variant,
            odrv.fw_version_major, odrv.fw_version_minor, odrv.fw_version_revision,
            "-dev" if odrv.fw_version_unreleased else "",
            odrive.version.get_version_str())
        
      
    def reboot(self):
        raise NotImplementedException()
        if not self.connected:
            self.logger.error("Not connected.")
            return False
        try:
            self.driver.reboot()
        except KeyError:
            self.logger.error("Rebooted ODrive.")
        except:
            self.logger.error("Failed to reboot: " + traceback.format_exc())
        finally:
            self.driver = None
        return True

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
        # returns true if all axes are iengaged
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
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        
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
            axis.controller.input_vel = self.flip_l(left_motor_val)
        for axis in self.right_axes:
            axis.controller.input_vel = self.flip_r(right_motor_val)
        
    def feed_watchdog(self):
        for axis in self.left_axes + self.right_axes:
            axis.watchdog_feed()

    @property
    def connected(self):
        for odrv in [self.left_odrive, self.right_odrive]:
            if odrv is None:
                return False
            if isinstance(odrv, fibre.libfibre.EmptyInterface):
                return False
        return True
        
    def get_errors(self, clear=True):
        # TODO: add error parsing, see: https://github.com/madcowswe/ODrive/blob/master/tools/odrive/utils.py#L34
        
        if not self.connected:
            return "disconnected"
            
        axis_error = self.axes[0].error or self.axes[1].error
        
        if axis_error:
            error_string = "Errors(hex): L: a%x m%x e%x c%x, R: a%x m%x e%x c%x" % (
                self.left_axis.error,  self.left_axis.motor.error,  self.left_axis.encoder.error,  self.left_axis.controller.error,
                self.right_axis.error, self.right_axis.motor.error, self.right_axis.encoder.error, self.right_axis.controller.error,
            )
        
        if clear:
            for axis in self.axes:
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0
        
        if axis_error:
            return error_string

    def flip_l(self, value):
        return -value if self.flip_left_direction else value

    def flip_r(self, value):
        return -value if self.flip_right_direction else value

    @property
    def left_vel_estimate(self):
        return self.flip_l(self.left_axes[0].encoder.vel_estimate) 
    
    @property
    def right_vel_estimate(self):
        return self.flip_r(self.right_axes[0].encoder.vel_estimate)
    
    @property
    def left_pos(self):
        return self.flip_l(self.left_axes[0].encoder.pos_estimate)
    
    @property
    def right_pos(self):
        return self.flip_r(self.right_axes[0].encoder.pos_estimate)
    
    # TODO check these match the right motors, but it doesn't matter for now
    def left_temperature(self):   
        return self.left_axis.motor.fet_thermistor.temperature  if self.left_axis  else 0.
    def right_temperature(self):  return self.right_axis.motor.fet_thermistor.temperature if self.right_axis else 0.
    
    def left_current(self):       return self.left_axis.motor.I_bus  if self.left_axis and self.left_axis.current_state > 1 else 0.
    def right_current(self):      return self.right_axis.motor.I_bus if self.right_axis and self.right_axis.current_state > 1 else 0.
    
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
    
    def bus_voltage(self):      return self.driver.vbus_voltage if self.left_axis else 0.
    
