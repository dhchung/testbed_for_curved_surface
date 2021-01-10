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
#include "calculate_transformations.h"

using namespace gtsam;
using namespace surfelnodenew;
class CoplanarFactorNew: public NoiseModelFactor2<Surfel, Surfel>{

private:
    // Vector4 measured_;
    CalTransform c_trans;

public:
    CoplanarFactorNew(Key key1, Key key2, const SharedNoiseModel& model = nullptr);

    Vector evaluateError(const Surfel& surfel1, const Surfel & surfel2,
                         boost::optional<Matrix&> H1 = boost::none,
                         boost::optional<Matrix&> H2 = boost::none) const;

    gtsam::Matrix3 skewsym_matrix(gtsam::Vector3 & vec) const;


};