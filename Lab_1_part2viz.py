import matplotlib.pyplot as plt
import numpy as np
import sympy as sp

# Define end-effector positions observed from all images (test cycles 00 to 09)
# x = np.array([0.13948352, 0.13994219, 0.13913635, 0.13933574, 0.13948289, 0.14078367, 0.13971862, 0.13980497, 0.13962537, 0.13987942])
# y = np.array([-0.79103174, -0.79091961, -0.79059913, -0.79142393, -0.79042225, -0.78952162, -0.79103811, -0.79074211, -0.79083475, -0.79109002])
# z = np.array([-0.2560909, -0.25620668, -0.25654688, -0.25609916, -0.25655822, -0.25672633, -0.25653018, -0.25653432, -0.25664166, -0.25626994])

x_recorded = np.array([.82763311, .67845583, .22350842, -.0229034])
y_recorded = np.array([-.24175168, -.13730141, -.39652527, -1.03379773])
z_recorded = np.array([.43224686, .88021117, 1.337201, .57452786])

x_calculated = np.array([0.82918387, 0.681818, 0.23032537, -0.04829173])
y_calculated = np.array([-0.28965065, -0.13614332, -0.39867142, -1.03478112])
z_calculated = np.array([0.42788204, 0.87671386, 1.33517501, 0.58670841])

orientation_pos1_reported = np.array([
    [0.97649607, 0.21128226, 0.0420655],
    [0.2146778, -0.9710342, -0.1049968],
    [0.01920586, 0.11159023, -0.9935687]
])

orientation_pos2_reported = np.array([
    [0.62797155, 0.70128906, 0.33740858],
    [0.06229841, -0.47793940, 0.87613442],
    [0.77568305, -0.52893466, -0.3443021]
])

orientation_pos3_reported = np.array([
    [-0.16682202, -0.98431448, 0.05740575],
    [0.96430641, -0.15073482, 0.2176974],
    [-0.20562966, 0.09167345, 0.97432665]
])

orientation_pos4_reported = np.array([
    [0.11179721, -0.99270156, 0.04522167],
    [-0.92304979, -0.0868821, -0.3747407],
    [0.36807673, 0.08363682, -0.92602614]
])

orientation_pos1_calculated = np.array([
    [0.98785238, 0.1520501, 0.03176897],
    [0.15458222, -0.98238511, -0.10490302],
    [0.01525914, 0.1085417, -0.99397478]
])

orientation_pos2_calculated = np.array([
    [0.63113646, 0.70134027, 0.33131466],
    [0.06458214, -0.47316621, 0.87859189],
    [0.7729736, -0.53312459, -0.3439331]
])

orientation_pos3_calculated = np.array([
    [-0.17272684, -0.9830885, 0.06068981],
    [0.96385735, -0.15601889, 0.21591188],
    [-0.20279562, 0.09579193, 0.97452442]
])

orientation_pos4_calculated = np.array([
    [0.02542699, -0.99909417, -0.03383986],
    [-0.9146507, -0.00959056, -0.40410758],
    [0.40342472, 0.04122768, -0.91408357]
])

orientation_pos1_difference = orientation_pos1_reported - orientation_pos1_calculated
orientation_pos2_difference = orientation_pos2_reported - orientation_pos2_calculated
orientation_pos3_difference = orientation_pos3_reported - orientation_pos3_calculated
orientation_pos4_difference = orientation_pos4_reported - orientation_pos4_calculated

sp.pprint( orientation_pos1_difference)
sp.pprint( orientation_pos2_difference)
sp.pprint( orientation_pos3_difference)
sp.pprint( orientation_pos4_difference)

fig, ax = plt.subplots(subplot_kw=dict(projection='3d'))
x_difference = x_recorded - x_calculated
y_difference = y_recorded - y_calculated
z_difference = z_recorded - z_calculated

print("X Difference: ", x_difference)
print("Y Difference: ", y_difference)
print("Z Difference: ", z_difference)

ax.stem(x_recorded, y_recorded, z_recorded, linefmt='C0-', markerfmt='C0o', basefmt=' ', label = "Recorded Positions")
ax.stem(x_calculated, y_calculated, z_calculated, linefmt='C1-', markerfmt='C1o', basefmt=' ', label = "Calculated Positions")
ax.set_title("Recorded Position vs FK Calculations with Recorded Joint Angles")
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xbound(0,1.2)
ax.set_ybound(0,-1.2)
ax.set_zbound(0, 1.5)
plt.style.use("seaborn-v0_8")
plt.legend()
plt.show()