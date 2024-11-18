import kinematics as kin    
import numpy as np
from visualization import VizScene

pi = np.pi
dh = np.array([[pi,.27035,-.069,pi/2], #ADDED IN PI OFFSET FOR TH1 TO FLIP X AXIS TO MATCH BASE ORIENTATION
               [pi/2, 0, 0, -pi/2],
               [0,.36435,-.069,pi/2],
               [0,0,0,-pi/2],
               [0,.37429,-.01,pi/2],
               [0,0,0,-pi/2],
               [pi,.229525,0,0]])

base = np.array([[.7071,.7071,0,.06353],
                 [-.7071,.7071,0,-.2597],
                 [0,0,1,.119],
                 [0,0,0,1]])


arm = kin.SerialArm(dh,base=base)

q1 = np.array([.82763311,-.33632529,-2.32244691,-.04947088,2.2227386,1.84806335,3.03536448])
q2 = np.array([.93802925,-.76890787,-2.0915828,.33747577,3.05338876,1.81469927,3.03613147])
q3 = np.array([.28033499,-1.28739338,-2.18477214,.08973788,1.37866523,-.4072719,3.03574798])
q4 = np.array([-.81684477,-.55530105,-1.9320488,.05062137,1.90367016,1.72572839,3.03614147])

print(arm.fk(q1,base=True)[:3,:3])
print(arm.fk(q2,base=True)[:3,:3])
print(arm.fk(q3,base=True)[:3,:3])
print(arm.fk(q4,base=True)[:3,:3])

# viz = VizScene()
# viz.add_arm(arm)
# viz.update(qs=q1)
# viz.hold()