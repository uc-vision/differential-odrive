#!/usr/bin/env python

import rospy
import tf.transformations
import tf_conversions
import tf2_ros

import std_msgs.msg
from std_msgs.msg import Float64, Int32
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import std_srvs.srv


import time
import math
import traceback

from odrive_ros.odrive_interface import ODriveInterfaceAPI, ODriveFailure
from odrive_ros.odrive_interface import ChannelDamagedException
from fibre.protocol import ObjectLostError as ChannelBrokenException
from fibre.libfibre import ObjectLostError

from odrive_ros.odrive_simulator import ODriveInterfaceSimulator

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #
    

def get_param(name, default):
    val = rospy.get_param(name, default)
    rospy.loginfo('  %s: %s', name, str(val))
    return val

class ODriveNode(object):
    last_speed = 0.0
    driver = None
    prerolling = False
    
    # Robot wheel_track params for velocity -> motor speed conversion
    wheel_track = None
    tyre_circumference = None
    axis_for_right = 0
    
    # Startup parameters
    calibrate_on_startup = False
    engage_on_startup = False
    

    def __init__(self):
        self.publish_temperatures = get_param('publish_temperatures', True)

        self.dual_odrive = get_param('~dual_odrive', False)
        self.odrive_sn = get_param('~odrive_sn', None)
        self.left_sn = get_param('~left_sn', None)
        self.right_sn = get_param('~right_sn', None)
        
        self.axis_for_right = float(get_param('~axis_for_right', 0)) # if right calibrates first, this should be 0, else 1
        self.flip_left_direction = float(get_param('~flip_left_direction', False))
        self.flip_right_direction = float(get_param('~flip_right_direction', False))
        self.wheel_track = float(get_param('~wheel_track', 0.285)) # m, distance between wheel centres
        self.tyre_circumference = float(get_param('~tyre_circumference', 0.341)) # used to translate velocity commands in m/s into motor rpm
        
        self.calibrate_on_startup = get_param('~calibrate_on_startup', False)
        self.engage_on_startup    = get_param('~engage_on_startup', False)
        
        self.has_preroll     = get_param('~use_preroll', True)
                
        self.publish_current = get_param('~publish_current', True)
        
        self.publish_odom    = get_param('~publish_odom', True)
        self.publish_tf      = get_param('~publish_odom_tf', False)
        self.odom_topic      = get_param('~odom_topic', "odom")
        self.odom_frame      = get_param('~odom_frame', "odom")
        self.base_frame      = get_param('~base_frame', "base_link")
        self.odom_calc_hz    = get_param('~odom_calc_hz', 10)
        
        self.fast_timer = None

        rospy.on_shutdown(self.terminate)

        rospy.Service('calibrate_motors',         std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',            std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',           std_srvs.srv.Trigger, self.release_motor)
                
        self.command = (0,0)
        self.vel_subscribe = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=2)
        
        if self.publish_temperatures:
            self.temperature_publisher_left  = rospy.Publisher('left/temperature', Float64, queue_size=2)
            self.temperature_publisher_right = rospy.Publisher('right/temperature', Float64, queue_size=2)
        
        if self.publish_current:
            self.current_publisher_left  = rospy.Publisher('left/current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('right/current', Float64, queue_size=2)
         
            rospy.logdebug("ODrive will publish motor currents.")

        self.last_cmd_vel_time = rospy.Time.now()
                                  
        if self.publish_odom:
            rospy.Service('reset_odometry',    std_srvs.srv.Trigger, self.reset_odometry)
            self.old_pos_l = 0
            self.old_pos_r = 0
            
            self.odom_publisher  = rospy.Publisher(self.odom_topic, Odometry, tcp_nodelay=True, queue_size=2)
            # setup message
            self.odom_msg = Odometry()
            self.odom_msg.header.frame_id = self.odom_frame
            self.odom_msg.child_frame_id = self.base_frame
            self.odom_msg.pose.pose.position.x = 0.0
            self.odom_msg.pose.pose.position.y = 0.0
            self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
            self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
            self.odom_msg.twist.twist.linear.x = 0.0
            self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
            self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
            self.odom_msg.twist.twist.angular.x = 0.0 # or roll
            self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
            self.odom_msg.twist.twist.angular.z = 0.0
            
            # store current location to be updated. 
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            
            # setup transform
            self.tf_publisher = tf2_ros.TransformBroadcaster()
            self.tf_msg = TransformStamped()
            self.tf_msg.header.frame_id = self.odom_frame
            self.tf_msg.child_frame_id  = self.base_frame
            self.tf_msg.transform.translation.x = 0.0
            self.tf_msg.transform.translation.y = 0.0
            self.tf_msg.transform.translation.z = 0.0
            self.tf_msg.transform.rotation.x = 0.0
            self.tf_msg.transform.rotation.y = 0.0
            self.tf_msg.transform.rotation.w = 0.0
            self.tf_msg.transform.rotation.z = 1.0
        

        if not self.connect_driver():
            rospy.logerr("Failed to connect.")
        else:
            self.old_pos_l = self.driver.left_pos
            self.old_pos_r = self.driver.right_pos

        
    def main_loop(self):
        main_rate = rospy.Rate(float(self.odom_calc_hz))

        if self.calibrate_on_startup:
            self.driver.preroll()

        if self.engage_on_startup:
            self.driver.engage()
        else:
            self.driver.release()

        self.driver.enable_watchdog(1.0)
               
        while not rospy.is_shutdown():
            try:
                main_rate.sleep()
            except rospy.ROSInterruptException: # shutdown / stop ODrive??
                break;

            time_now = rospy.Time.now()

            self.driver.feed_watchdog()

            if self.publish_odom:
                self.pub_odometry(time_now)
            if self.publish_temperatures:
                self.pub_temperatures()
            if self.publish_current:
                self.pub_current()

            if self.command is not None:
                if not self.driver.prerolled:
                    rospy.logwarn_throttle(5.0, "ODrive has not been prerolled, ignoring drive command.")
                    return
            
                elif not self.driver.engaged:
                    rospy.logwarn_throttle(5.0, "ODrive is not engaged, engaging now.")
                    self.driver.engage()

                else:      
                    left_linear_val, right_linear_val = self.command
    
                    self.driver.drive(left_linear_val, right_linear_val)
                    self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
                    self.last_cmd_vel_time = time_now
                    self.command = None
        

            if (time_now - self.last_cmd_vel_time).to_sec() > 0.5 and self.last_speed > 0:
                self.driver.drive(0,0)
                self.last_speed = 0
                self.last_cmd_vel_time = time_now

            if (time_now - self.last_cmd_vel_time).to_sec() > 10.0 and self.driver.engaged:
                rospy.logwarn("No cmd_vel received for %ds - disengaging motors" %10.0 )
                self.driver.release() # and release   

        self.driver.disable_watchdog()
    

    def terminate(self):
        if self.fast_timer:
          self.fast_timer.shutdown()
        
        if self.driver:
            self.driver.release()
    
    def connect_driver(self):
        if self.driver:
            raise Exception("Driver already connected.")
            return False
               
        self.driver = ODriveInterfaceAPI(logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")

        if self.dual_odrive:
            raise NotImplemented("Dual odrive setup")
        else:
            if not self.driver.connect(
                odrive_sn = self.odrive_sn,
                right_axis=self.axis_for_right):
            
                return False
            
        rospy.loginfo("ODrive connected.")
        
        return True
    
    
    def calibrate_motor(self, request):
         
        if self.has_preroll:
            if not self.driver.preroll(wait=True):
                return (False, "Failed preroll.")
            
                
        return (True, "Calibration success.")
    
    def engage_motor(self, request):
        if not self.driver.prerolled:
            return (False, "Not prerolled.")
        if not self.driver.engage():
            return (False, "Failed to engage motor.")
        return (True, "Engage motor success.")
    
    def release_motor(self, request):
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")
    
    def reset_odometry(self, request):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        return(True, "Odometry reset.")
    
    def convert(self, forward, ccw):
        angular_to_linear = ccw * (self.wheel_track/2.0) 
        left_linear_val  = (forward - angular_to_linear) / self.tyre_circumference
        right_linear_val = (forward + angular_to_linear) / self.tyre_circumference
        return left_linear_val, right_linear_val

    def cmd_vel_callback(self, msg):
        left_linear_val, right_linear_val = self.convert(msg.linear.x, msg.angular.z)
        self.command = (left_linear_val, right_linear_val)
        self.last_cmd_vel_time = rospy.Time.now()
        
    def pub_temperatures(self):
        self.temperature_publisher_left.publish(self.driver.left_temperature)
        self.temperature_publisher_right.publish(self.driver.right_temperature)
        
    # Current publishing     
    def pub_current(self):
        self.current_publisher_left.publish(self.driver.left_current)
        self.current_publisher_right.publish(self.driver.right_current)
        
    def pub_odometry(self, time_now):
        now = time_now
        self.odom_msg.header.stamp = now
        self.tf_msg.header.stamp = now
              
        vel_l = self.driver.left_vel_estimate
        vel_r = self.driver.right_vel_estimate
        # Twist/velocity: calculated from motor values only
        s = self.tyre_circumference * (vel_l+vel_r) / (2.0)
        w = self.tyre_circumference * (vel_r-vel_l) / (self.wheel_track) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w
    
        self.new_pos_l = self.driver.left_pos
        self.new_pos_r = self.driver.right_pos
        # Position
        delta_pos_l = self.new_pos_l - self.old_pos_l
        delta_pos_r = self.new_pos_r - self.old_pos_r
        
        self.old_pos_l = self.new_pos_l
        self.old_pos_r = self.new_pos_r
              
        # counts to metres
        delta_pos_l_m = delta_pos_l * self.tyre_circumference
        delta_pos_r_m = delta_pos_r * self.tyre_circumference
    
        # Distance travelled
        d = (delta_pos_l_m+delta_pos_r_m)/2.0  # delta_ps
        th = (delta_pos_r_m-delta_pos_l_m)/self.wheel_track # works for small angles
    
        xd = math.cos(th)*d
        yd = -math.sin(th)*d
    
        # elapsed time = event.last_real, event.current_real
        #elapsed = (event.current_real-event.last_real).to_sec()
        # calc_vel: d/elapsed, th/elapsed
    
        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)
        
        # fill odom message and publish
        
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2
    
        #self.odom_msg.pose.covariance
         # x y z
         # x y z
    
        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        #self.tf_msg.transform.rotation.x
        #self.tf_msg.transform.rotation.x
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]
        
        
        # ... and publish!
        self.odom_publisher.publish(self.odom_msg)
        if self.publish_tf:
            self.tf_publisher.sendTransform(self.tf_msg)            
    

def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()  
    odrive_node.main_loop()
    #rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass

