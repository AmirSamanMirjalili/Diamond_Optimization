# Setup
import numpy as np
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
RotationKin = sf.Rot3.from_rotation_matrix(RotationKin)
RotationKin1 = RotationKin.subs((theta1, theta2), (theta11, theta12))
RotationKin2 = RotationKin.subs((theta1, theta2), (theta21, theta22))
DeltaRotKin = RotationKin1.inverse() * RotationKin2

# create delta rotation matrix for ground truth in two pose 1(i) and 2(j)
RotationGT1 = sf.Rot3.symbolic("RotGT1")
RotationGT2 = sf.Rot3.symbolic("RotGT2")
DeltaRotGT = RotationGT1.inverse() * RotationGT2
RotHandEye = sf.Rot3.symbolic("RotHandEye")
DeltaRotGT_Tild = RotHandEye.inverse() * DeltaRotGT * RotHandEye

Error_Model = DeltaRotGT_Tild.inverse() * DeltaRotKin
Error_Model_in_tangent_list = Error_Model.to_tangent()
Error_Model_in_tangent = sf.V3.symbolic("")
Error_Model_in_tangent[0] = Error_Model_in_tangent_list[0]
Error_Model_in_tangent[1] = Error_Model_in_tangent_list[1]
Error_Model_in_tangent[2] = Error_Model_in_tangent_list[2]


def error_model_func(theta11: sf.Symbol('theta11'), theta12: sf.Symbol('theta12'), theta21: sf.Symbol('theta21'), theta22: sf.Symbol('theta22'),
                     alpha: sf.Symbol('alpha'), beta: sf.Symbol('beta'), 
                     RotGT1: sf.Rot3.symbolic("RotGT1"),
                     RotGT2: sf.Rot3.symbolic("RotGT2"),
                     RotHandEye: sf.Rot3.symbolic("RotHandEye"),
                     epsilon: sf.Scalar = 0
                    ) -> sf.Vector3:
    model = Error_Model_in_tangent
    return sf.V3(model)
resedual_func_codegen = codegen.Codegen.function(func=error_model_func, config=codegen.CppConfig(),)
resedual_func_codegen_data = resedual_func_codegen.generate_function(output_dir="/home/mohammad/Diamond_Optimization")


def error_model_func_wrt_alpha(theta11: sf.Symbol('theta11'), theta12: sf.Symbol('theta12'), theta21: sf.Symbol('theta21'), theta22: sf.Symbol('theta22'),
                     alpha: sf.Symbol('alpha'), beta: sf.Symbol('beta'), 
                     RotGT1: sf.Rot3.symbolic("RotGT1"),
                     RotGT2: sf.Rot3.symbolic("RotGT2"),
                     RotHandEye: sf.Rot3.symbolic("RotHandEye"),
                     epsilon: sf.Scalar = 0
                    ) -> sf.Vector3:
    model_wrt_alpha = Error_Model_in_tangent.diff(alpha)
    return sf.V3(model_wrt_alpha)
resedual_func_codegen = codegen.Codegen.function(func=error_model_func_wrt_alpha, config=codegen.CppConfig(),)
resedual_func_codegen_data = resedual_func_codegen.generate_function(output_dir="/home/mohammad/Diamond_Optimization")


def error_model_func_wrt_beta(theta11: sf.Symbol('theta11'), theta12: sf.Symbol('theta12'), theta21: sf.Symbol('theta21'), theta22: sf.Symbol('theta22'),
                     alpha: sf.Symbol('alpha'), beta: sf.Symbol('beta'), 
                     RotGT1: sf.Rot3.symbolic("RotGT1"),
                     RotGT2: sf.Rot3.symbolic("RotGT2"),
                     RotHandEye: sf.Rot3.symbolic("RotHandEye"),
                     epsilon: sf.Scalar = 0
                    ) -> sf.Vector3:
    model_wrt_beta = Error_Model_in_tangent.diff(beta)
    return sf.V3(model_wrt_beta)
resedual_func_codegen = codegen.Codegen.function(func=error_model_func_wrt_beta, config=codegen.CppConfig(),)
resedual_func_codegen_data = resedual_func_codegen.generate_function(output_dir="/home/mohammad/Diamond_Optimization")


def error_model_func_wrt_hand_eye(theta11: sf.Symbol('theta11'), theta12: sf.Symbol('theta12'), theta21: sf.Symbol('theta21'), theta22: sf.Symbol('theta22'),
                     alpha: sf.Symbol('alpha'), beta: sf.Symbol('beta'), 
                     RotGT1: sf.Rot3.symbolic("RotGT1"),
                     RotGT2: sf.Rot3.symbolic("RotGT2"),
                     RotHandEye: sf.Rot3.symbolic("RotHandEye"),
                     epsilon: sf.Scalar = 0
                    ) -> sf.Vector3:
    model_wrt_hand_eye = Error_Model_in_tangent.jacobian(RotHandEye)
    return sf.Matrix33(model_wrt_hand_eye)
resedual_func_codegen = codegen.Codegen.function(func=error_model_func_wrt_hand_eye, config=codegen.CppConfig(),)
resedual_func_codegen_data = resedual_func_codegen.generate_function(output_dir="/home/mohammad/Diamond_Optimization")
