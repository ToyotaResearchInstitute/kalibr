#include <algorithm>
#include <stdexcept>

#include <kalibr2/BasicMathUtils.hpp>

namespace kalibr2 {

namespace math {

double median(const std::vector<double>& vec) {
  if (vec.empty()) {
    throw std::runtime_error("Cannot compute median of an empty vector.");
  }
  std::vector<double> copy = vec;
  std::nth_element(copy.begin(), copy.begin() + copy.size() / 2, copy.end());
  return copy.at(copy.size() / 2);
}

Eigen::Vector3d median(const std::vector<Eigen::Vector3d>& vec) {
  if (vec.empty()) {
    throw std::runtime_error("Cannot compute median of an empty vector.");
  }
  std::vector<double> x, y, z;
  for (const auto& v : vec) {
    x.push_back(v.x());
    y.push_back(v.y());
    z.push_back(v.z());
  }
  return Eigen::Vector3d(median(x), median(y), median(z));
}

}  // namespace math

}  // namespace kalibr2
