# Differential Odrive drive for ROS
A driver for differential drive robots using an ODrive motor controller.

### Key Features:
- Supports two motors on a single odrive
- Support four motors on two odrives
- Supports velocity control using odrives control loops
- Supports torque control using control loops in driver

## Getting Started

Install odrivetool
```
pip install --upgrade odrive
```

Build your catkin workspace

## Configuration

### Two motor - Single Odrive
```xml
<launch>  
    <node pkg="differential_odrive" type="odrive_node" name="odrive">
        <!-- Vehicle Parameters -->
        <param name="wheel_track" value="0.77"/>
        <param name="tyre_circumference" value="0.03852"/>

        <!-- Odrive Parameters -->
        <param name ="odrive_sn" value="205F37905753"/>
        <param name ="axis_for_right" value="1"/>
        <param name ="flip_left_direction" value="false"/>
        <param name ="flip_right_direction" value="false"/>

        <!-- Driver Parameters -->
        <param name ="loop_rate" value="50"/>

        <param name="odom_frame"   value="odom"/>
        <param name="base_frame" value="wheel_odom_link"/>
        
        <param name="connect_on_startup"   value="true"/>
        <param name="calibrate_on_startup" value="true"/>
        <param name="engage_on_startup"    value="true"/>
        
        <param name="publish_odom"         value="true"/>
        <param name="publish_odom_tf"      value="false"/>
        <param name="publish_current"      value="true"/>
        <param name="publish_voltage"      value="true"/>  
    </node>
</launch>
```

### Four motor - Dual Odrive
Dual odrive configuration is similar to the single. You need to specify you want dual odrive operation, along with the serial number of the odrive for each axis.
```xml
<launch>
    <node pkg="differential_odrive" type="odrive_node" name="odrive">
        <!-- Vehicle Parameters -->
        <param name="wheel_track" value="0.77"/>
        <param name="tyre_circumference" value="0.03852"/>

        <!-- Odrive Parameters -->
        <param name ="dual_odrive" value="true"/>
        <param name ="left_sn" type="string" value="206E35925748"/>
        <param name ="right_sn" type="string" value="205938915848"/>
        <param name ="flip_left_direction" value="true"/>
        <param name ="flip_right_direction" value="false"/>

        <!-- Driver Parameters -->
        <param name ="loop_rate" value="50"/>

        <param name="odom_frame"   value="odom"/>
        <param name="base_frame" value="wheel_odom_link"/>
        
        <param name="connect_on_startup"   value="true"/>
        <param name="calibrate_on_startup" value="true"/>
        <param name="engage_on_startup"    value="true"/>
        
        <param name="publish_odom"         value="true"/>
        <param name="publish_odom_tf"      value="false"/>
        <param name="publish_current"      value="true"/>
        <param name="publish_voltage"      value="true"/>
    </node>
</launch>
```

### Torque control
Controlling the torque directly can be useful if the velocity control loops on the odrive are not honouring the rotation component of the commanded velocity. This is can be the case on direct drive motors such as found on e-scooters. In the torque control mode we instead control the torque of each motor with a PID loop for turnning and a loop for velocity. 

The values of these control loops are exposed via dynamic reconfigure allowing for real-time tunning.
```xml
<launch>
    <arg name="torque_control" default="true"/>
    <arg name="velo_p" default="1"/>
    <arg name="velo_i" default="0"/>
    <arg name="velo_d" default="0"/>
    <arg name="turn_p" default="1"/>
    <arg name="turn_i" default="0"/>
    <arg name="turn_d" default="0"/>
    
    <node pkg="differential_odrive" type="odrive_node" name="odrive">
        <!-- Vehicle Parameters -->
        <param name="wheel_track" value="0.77"/>
        <param name="tyre_circumference" value="0.03852"/>

        <!-- Odrive Parameters -->
        <param name ="odrive_sn" value="205F37905753"/>
        <param name ="axis_for_right" value="1"/>
        <param name ="flip_left_direction" value="false"/>
        <param name ="flip_right_direction" value="false"/>

        <!-- Torque Control Parameters -->
        <param name="torque_control" value="$(arg torque_control)"/>
        <param name="velo_p" value="$(arg velo_p)"/>
        <param name="velo_i" value="$(arg velo_i)"/>
        <param name="velo_d" value="$(arg velo_d)"/>
        <param name="turn_p" value="$(arg turn_p)"/>
        <param name="turn_i" value="$(arg turn_i)"/>
        <param name="turn_d" value="$(arg turn_d)"/>
        
        <!-- Driver Parameters -->
        <param name ="loop_rate" value="50"/>

        <param name="odom_frame"   value="odom"/>
        <param name="base_frame" value="wheel_odom_link"/>
        
        <param name="connect_on_startup"   value="true"/>
        <param name="calibrate_on_startup" value="true"/>
        <param name="engage_on_startup"    value="true"/>
        
        <param name="publish_odom"         value="true"/>
        <param name="publish_odom_tf"      value="false"/>
        <param name="publish_current"      value="true"/>
        <param name="publish_voltage"      value="true"/>
    </node>
</launch>
```


## Authors
Josh McCulloch - [Github](https://github.com/joshmcculloch)

## Acknowledgments

Based on [odrive_ros](https://github.com/neomanic/odrive_ros) by Josh Marshall




