import numpy as np


b = np.array([[1,2,3,4],[5,6,7,8],[9,10,11,12],[13,14,15,16]])
R = b[:3, :3]
p = b[:3, -1:]

print(b, "\n")
print(R, "\n")
print(np.reshape(p,(-1,1)))
