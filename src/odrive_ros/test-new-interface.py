import odrive_interface
import time

od = odrive_interface.ODriveInterfaceAPI()

#od.connect(left_sn='205938915848', right_sn='206E35925748')

od.connect(odrive_sn='205F37905753')

print("is prerolled:", od.prerolled)
print("is prerolling:", od.prerolling)

od.preroll()

print("engaged", od.engaged)
print("engaging")
od.engage()

print("engaged", od.engaged)

vel = 0
while vel < 50:
    vel += 0.1
    od.drive(vel,vel)
    print("vel cmd", vel, end="")
    time.sleep(0.01)
    print(" - left %f:%f, right %f:%f" %(od.left_vel_estimate, od.left_current, od.right_vel_estimate, od.right_current))


print("releasing")
od.release()

print("engaged", od.engaged)

print(od.get_errors())

print("voltage", od.bus_voltage)
