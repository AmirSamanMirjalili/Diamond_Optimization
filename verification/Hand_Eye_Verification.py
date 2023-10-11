# Setup
import numpy as np
import math
import symforce

symforce.set_symbolic_api("symengine")
symforce.set_log_level("warning")

# Set epsilon to a symbol for safe code generation.  For more information, see the Epsilon tutorial:
# https://symforce.org/tutorials/epsilon_tutorial.html
symforce.set_epsilon_to_symbol()

from symforce import codegen
from symforce.codegen import codegen_util
from symforce import ops
import symforce.symbolic as sf
from symforce.values import Values
from symforce.notebook_util import display, display_code, display_code_file


from pyquaternion import Quaternion

def calculate_norm(vector):
    return sf.sqrt(sum(component**2 for component in vector))

theta1, theta2, theta11, theta12, theta21, theta22, alpha, beta = sf.symbols("theta1 theta2 theta11 theta12 theta21 theta22 alpha beta")

# inverse kinematic
phi = (theta1 + theta2) / 2
A_hat = (theta1 - theta2) / 2

gamma = 2 * sf.atan((sf.sin(alpha) * sf.cos(A_hat) + sf.sqrt(sf.sin(beta) ** 2 - sf.sin(alpha) ** 2 * sf.sin(A_hat) ** 2)) / (sf.cos(alpha) + sf.cos(beta)))

# special vector of robot in space
a = sf.V3.symbolic("")
a[0] = 0
a[1] = 0
a[2] = 1

b = sf.V3.symbolic("")
b[0] = sf.cos(theta2)*sf.sin(alpha)
b[1] = sf.sin(theta2)*sf.sin(alpha)
b[2] = sf.cos(alpha)

c = sf.V3.symbolic("")
c[0] = sf.cos(theta1)*sf.sin(alpha)
c[1] = sf.sin(theta1)*sf.sin(alpha)
c[2] = sf.cos(alpha)

d = sf.V3.symbolic("")
d[0] = sf.cos(phi)*sf.sin(gamma)
d[1] = sf.sin(phi)*sf.sin(gamma)
d[2] = sf.cos(gamma)

# create delta rotation matrix for kinematic in two pose 1(i) and 2(j)
RotationKin = sf.Matrix([np.cross(np.cross(c, d), c) / sf.sin(beta),np.cross(c, d) / sf.sin(beta),c])
RotationKin1 = RotationKin.subs((theta1, theta2), (theta11, theta12))
RotationKin2 = RotationKin.subs((theta1, theta2), (theta21, theta22))
DeltaRotKin = RotationKin1.inv() * RotationKin2

Data_of_Rotation_and_Encoder = np.load('Data_of_Rotation_and_Encoder.npy', allow_pickle=True)

# create delta rotation matrix for ground truth in two pose 1(i) and 2(j)
RotationGT1 = sf.M33.symbolic("RotGT1")
RotationGT2 = sf.M33.symbolic("RotGT2")
DeltaRotGT = RotationGT1.inv() * RotationGT2
RotHandEye = sf.M33.symbolic("RotHandEye")
DeltaRotGT_Tild = RotHandEye.inv() * DeltaRotGT * RotHandEye

R_mat_hand_eye = sf.M33(
    [
        [0.996926, 0.0656799, -0.0427278],
        [-0.0636131, 0.996817, 0.0480569],
        [0.0457481, -0.0451911, 0.99793],
    ]
)

k = 20
# calculate error for identified
DeltaRotGT_list = []
DeltaRotGT_Tild_list = []
DeltaRotKin_list = []
for j in range(1, (len(Data_of_Rotation_and_Encoder)//k)+1):  # j=(1,68)
    for i in range((j-1)*k, j*k):   # for k=30:  i=(0,29)   i=(30,59)  i=(60,89)  ...
        DeltaRotGT_list.append(DeltaRotGT.subs((RotationGT1, RotationGT2),(Data_of_Rotation_and_Encoder[i][2], Data_of_Rotation_and_Encoder[j][2])))
        DeltaRotKin_list.append(DeltaRotKin.subs((alpha, beta, theta11, theta12, theta21, theta22),(0.726492, 0.807529, Data_of_Rotation_and_Encoder[i][3][0], Data_of_Rotation_and_Encoder[i][3][1], Data_of_Rotation_and_Encoder[j][3][0], Data_of_Rotation_and_Encoder[j][3][1])))
        DeltaRotGT_Tild_list.append(DeltaRotGT_Tild.subs((RotationGT1, RotationGT2, RotHandEye),(Data_of_Rotation_and_Encoder[i][2], Data_of_Rotation_and_Encoder[j][2], R_mat_hand_eye)))
error_list = []
for i in range(len(DeltaRotGT_Tild_list)):
    error_rotation = (DeltaRotGT_Tild_list[i].inv() * DeltaRotKin_list[i])
    error_list.append((calculate_norm(sf.Rot3.from_rotation_matrix(error_rotation).to_tangent(sf.numeric_epsilon)))**2)

error_identified = math.sqrt(sum(error_list))/len(error_list) * 180/math.pi
print("identified error is: ", error_identified)


# calculate error for nonidentified
DeltaRotGT_list = []
DeltaRotGT_Tild_list = []
DeltaRotKin_list = []
for j in range(1, (len(Data_of_Rotation_and_Encoder)//k)+1):  # j=(1,68)
    for i in range((j-1)*k, j*k):   # for k=30:  i=(0,29)   i=(30,59)  i=(60,89)  ...
        DeltaRotGT_list.append(DeltaRotGT.subs((RotationGT1, RotationGT2),(Data_of_Rotation_and_Encoder[i][2], Data_of_Rotation_and_Encoder[j][2])))
        DeltaRotKin_list.append(DeltaRotKin.subs((alpha, beta, theta11, theta12, theta21, theta22),(3.14/4.0, 3.14/4.0, Data_of_Rotation_and_Encoder[i][3][0], Data_of_Rotation_and_Encoder[i][3][1], Data_of_Rotation_and_Encoder[j][3][0], Data_of_Rotation_and_Encoder[j][3][1])))
        DeltaRotGT_Tild_list.append(DeltaRotGT_Tild.subs((RotationGT1, RotationGT2, RotHandEye),(Data_of_Rotation_and_Encoder[i][2], Data_of_Rotation_and_Encoder[j][2], R_mat_hand_eye)))
error_list = []
for i in range(len(DeltaRotGT_Tild_list)):
    error_rotation = (DeltaRotGT_Tild_list[i].inv() * DeltaRotKin_list[i])
    error_list.append((calculate_norm(sf.Rot3.from_rotation_matrix(error_rotation).to_tangent(sf.numeric_epsilon)))**2)

error_nonidentified = math.sqrt(sum(error_list))/len(error_list) * 180/math.pi
print("nonidentified error is: ", error_nonidentified)

improvment = (error_nonidentified-error_identified)/error_nonidentified * 100
print("improvment in error is: ", improvment , "percent")
