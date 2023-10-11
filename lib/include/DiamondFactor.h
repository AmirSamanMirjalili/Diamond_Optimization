#pragma once

#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>

#include "model.h"

using namespace gtsam;
using namespace std;

sym::Rot3<double> SymforceFromGtsam(const gtsam::Rot3& gtsam_rot3) {
  return sym::Rot3<double>(gtsam_rot3.toQuaternion());
}

namespace gtsam
{   // this factor says: second should be larger than the first
    class DiamondCalibrationFactor : public NoiseModelFactor3<double, double, gtsam::Rot3>
    {   
        private:
        double theta11;
        double theta12;
        double theta21;
        double theta22;
        gtsam::Rot3 RotGT1;
        gtsam::Rot3 RotGT2;

        public:
        // Constructor
        DiamondCalibrationFactor(Key key1, Key key2, Key key3, 
                                 double theta11_, double theta12_, double theta21_, double theta22_,
                                 gtsam::Rot3 RotGT1_, 
                                 gtsam::Rot3 RotGT2_, 
                                 const SharedNoiseModel &model) 
        : NoiseModelFactor3<double, double, gtsam::Rot3>(model, key1, key2, key3), theta11(theta11_), theta12(theta12_), theta21(theta21_), theta22(theta22_), RotGT1(RotGT1_), RotGT2(RotGT2_){}

        // Evaluate the error
        Vector evaluateError(const double &alpha, const double &beta, const gtsam::Rot3 &RotHandEye,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2,
                             OptionalMatrixType H3) const override
        {   
            Eigen::Matrix<double, 3, 1> model_func = sym::ErrorModelFunc(theta11, theta12, theta21, theta22,
                                                                         alpha, beta,
                                                                         SymforceFromGtsam(RotGT1), 
                                                                         SymforceFromGtsam(RotGT2), 
                                                                         SymforceFromGtsam(RotHandEye),
                                                                         sym::kDefaultEpsilon<double>);

            if (H1)
            {
                Eigen::Matrix<double, 3, 1> func_wrt_alpha = sym::ErrorModelFuncWrtAlpha(theta11, theta12, theta21, theta22,
                                                                                         alpha, beta,
                                                                                         SymforceFromGtsam(RotGT1), 
                                                                                         SymforceFromGtsam(RotGT2), 
                                                                                         SymforceFromGtsam(RotHandEye),
                                                                                         sym::kDefaultEpsilon<double>);
                *H1 = (Matrix(3, 1) << func_wrt_alpha).finished();
            }

            if (H2)
            {
                Eigen::Matrix<double, 3, 1> func_wrt_beta = sym::ErrorModelFuncWrtBeta(theta11, theta12, theta21, theta22,
                                                                                       alpha, beta,
                                                                                       SymforceFromGtsam(RotGT1), 
                                                                                       SymforceFromGtsam(RotGT2), 
                                                                                       SymforceFromGtsam(RotHandEye),
                                                                                       sym::kDefaultEpsilon<double>);
                *H2 = (Matrix(3, 1) << func_wrt_beta).finished();
            }

            if (H3)
            {
                Eigen::Matrix<double, 3, 3> func_wrt_hand_eye = sym::ErrorModelFuncWrtHandEye(theta11, theta12, theta21, theta22,
                                                                                              alpha, beta,
                                                                                              SymforceFromGtsam(RotGT1), 
                                                                                              SymforceFromGtsam(RotGT2), 
                                                                                              SymforceFromGtsam(RotHandEye),
                                                                                              sym::kDefaultEpsilon<double>);
                *H3 = (Matrix(3, 3) << func_wrt_hand_eye).finished();            
            }
            // std::cout << "\nError is: " << model_func;
            return (Vector(3) << model_func).finished();
        }
    };
}










