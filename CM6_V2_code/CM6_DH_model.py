from roboticstoolbox import DHRobot, RevoluteDH, ERobot, ELink, ETS
from math import pi, sin, cos
import numpy as np
#np.set_printoptions(linewidth=100, suppress=True)


# robot length values (metres)
l1 = 162.65 / 1000
l2 = 280 / 1000
l3 = 250 / 1000
l4 = 37.2 / 1000

alpha_DH = [pi/2, 0, pi/2, -pi/2, pi/2, 0]

robot = DHRobot(
    [
        RevoluteDH(d=l1, a=0, alpha=alpha_DH[0]),
        RevoluteDH(d=0, a=l2, alpha=alpha_DH[1]),
        RevoluteDH(d=0, a=0, alpha=alpha_DH[2]),
        RevoluteDH(d=l3, a=0, alpha=alpha_DH[3]),
        RevoluteDH(d=0, a=0, alpha=alpha_DH[4]),
        RevoluteDH(d=l4, a=0, alpha=alpha_DH[5]),
    ],
    name="CM6",
)

#print(robot.isspherical())

print(robot)
robot_plot = robot.plot([0,pi/2,0,0,0,0])
robot_plot.hold()