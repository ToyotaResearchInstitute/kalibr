#pragma once

#include <vector>

#include <Eigen/Core>

namespace kalibr2 {

namespace math {

double median(const std::vector<double>& vec);

Eigen::Vector3d median(const std::vector<Eigen::Vector3d>& vec);

}  // namespace math

}  // namespace kalibr2
