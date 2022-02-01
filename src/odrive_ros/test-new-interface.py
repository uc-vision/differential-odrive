import odrive_interface
import time

od = odrive_interface.ODriveInterfaceAPI()

od.connect('205938915848', '206E35925748')

print("is prerolled:", od.prerolled)
print("is prerolling:", od.prerolling)

od.preroll()

print("engaged", od.engaged)
print("engaging")
od.engage()

print("engaged", od.engaged)

vel = 0
while vel < 50:
    vel += 1
    od.drive(vel,vel)
    print("vel cmd", vel, end="")
    time.sleep(0.1)
    print(" - left %f, right %f" %(od.left_vel_estimate, od.right_vel_estimate))


print("releasing")
od.release()

print("engaged", od.engaged)