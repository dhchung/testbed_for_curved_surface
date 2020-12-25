#pragma once
#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Ordering.h>
#include <gtsam/linear/NoiseModel.h>
#include "surfel_node.h"
#include "math.h"
#include <iostream>
#include "calculate_transformations.h"

using namespace gtsam;
using namespace surfelnode;
class PlaneMeasureFactor: public NoiseModelFactor2<Pose3, Surfel>{

private:
    Vector4 measured_;
    CalTransform c_trans;

public:
    PlaneMeasureFactor(Key key1, Key key2, 
                       const Vector4& measured, const SharedNoiseModel& model = nullptr);

    Vector evaluateError(const Pose3& pose, const Surfel & surfel,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;

    gtsam::Matrix3 skewsym_matrix(gtsam::Vector3 & vec) const;


};