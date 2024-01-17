#pragma once

#include <iostream>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>



using namespace gtsam;
using namespace std;

/*
 //Diamond costriants factor//
 Measurements: dR_camera, theta11, theta12, theta21, theta22
 What we know: the forward kinematic of our robot
 Variables: beta, alpha
 E(q) = z - h(q) = dR_camera - dR_fk
*/

namespace gtsam
{   // this factor says: second should be larger than the first
    class Inequality : public NoiseModelFactor2<double, double>
    {


        public:
        // Constructor
        Inequality(Key key1, Key key2, const SharedNoiseModel &model) 
        : NoiseModelFactor2<double, double>(model, key1, key2){}

        // Evaluate the error
        Vector evaluateError(const double &first, const double &second,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2) const override
        {
            if (H1)
            {
                *H1 = (Matrix(1, 1) << 0).finished();
            }
            if (H2)
            {
                *H2 = (Matrix(1, 1) << 0).finished();
            }

            if (first > second)
            {
                return (Vector(1) << 1e9).finished();
            }
            return (Vector(1) << 0.0).finished();
        }
    };
}



namespace gtsam
{
    class Equality : public NoiseModelFactor2<double, double>
    {
        private:
            double measurement_;

        public:
        // Constructor
        Equality(Key key1, Key key2, double meaurement, const SharedNoiseModel &model) 
        : NoiseModelFactor2<double, double>(model, key1, key2), measurement_(meaurement) {}

        // Evaluate the error
        Vector evaluateError(const double &beta, const double &alpha,
                             OptionalMatrixType H1,
                             OptionalMatrixType H2) const override
        {
            if (H1)
            {
                *H1 = (Matrix(1, 1) << 1).finished();
            }
            if (H2)
            {
                *H2 = (Matrix(1, 1) << 1).finished();
            }

            double error = beta + alpha - measurement_;
            return (Vector(1) << error).finished();
        }
    };
}


namespace gtsam
{
    class BoundFactor : public NoiseModelFactor1<double>
    {
        private:

            double MinBoubd_;
            double MaxBoubd_;

        public:
        // Constructor
        BoundFactor(Key key, double MinBoubd, double MaxBoubd, const SharedNoiseModel &model) 
        : NoiseModelFactor1<double>(model, key), MinBoubd_(MinBoubd), MaxBoubd_(MaxBoubd) {}

        // Evaluate the error
        Vector evaluateError(const double &x_varable,
                             OptionalMatrixType H1) const override
        {
            if (H1)
            {
                *H1 = (Matrix(1, 1) << 0).finished();
            }

            if (MinBoubd_ > x_varable || x_varable > MaxBoubd_)
            {
                return (Vector(1) << 1e9).finished();
            }
            return (Vector(1) << 0.0).finished();
        }
    };
}