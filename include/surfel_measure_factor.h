#pragma once
#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/NoiseModel.h>
#include "surfel_node_new.h"
#include "math.h"
#include <iostream>

using namespace gtsam;
using namespace surfelnodenew;
class SurfelMeasureFactor: public NoiseModelFactor2<Pose3, Surfel>{

private:
    Vector6 measured_;

public:
    SurfelMeasureFactor(Key key1, Key key2, 
                       const Vector6& measured, const SharedNoiseModel& model = nullptr);

    Vector evaluateError(const Pose3& pose, const Surfel & surfel,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;

};