#pragma once

#include <map>
#include <vector>

#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <common_robotics_utilities/simple_graph.hpp>
#include <common_robotics_utilities/simple_graph_search.hpp>

namespace kalibr2 {

using aslam::cameras::GridCalibrationTargetObservation;
using SyncedSet = std::vector<std::optional<GridCalibrationTargetObservation>>;
using Corners = std::vector<unsigned int>;
using SyncedSetCorners = std::vector<Corners>;
using common_robotics_utilities::simple_graph::Graph;
using common_robotics_utilities::simple_graph_search::DijkstrasResult;
using common_robotics_utilities::simple_graph_search::PerformDijkstrasAlgorithm;

namespace detail {

// Convert a SyncedSet to a vector of corners indices
SyncedSetCorners ToCornersIdxs(const SyncedSet& set);

// Get common corners between two sets of corners
Corners GetCommonCorners(const Corners& corners1, const Corners& corners2);

// Get a map of common corners between all pairs of sets in SyncedSetCorners
std::map<std::pair<size_t, size_t>, Corners> GetCommonCornersMap(const SyncedSetCorners& corners_per_set);

// Convert a map to a size map, where each key maps to the size of its value
// ValueT should be a container type
template <typename KeyT, typename ValueT>
std::map<KeyT, size_t> ToSizeMap(const std::map<KeyT, ValueT>& input_map) {
  std::map<KeyT, size_t> size_map;
  for (const auto& [key, container] : input_map) {
    size_map[key] = container.size();
  }
  return size_map;
}
}  // namespace detail

Graph<size_t> BuildCameraGraph(const std::vector<SyncedSet> synced_sets);
sm::kinematics::Transformation GetTransform(
    const std::map<std::pair<size_t, size_t>, sm::kinematics::Transformation>& transformation_map,
    const common_robotics_utilities::simple_graph_search::DijkstrasResult& result, size_t left_node_index,
    size_t right_node_index);

}  // namespace kalibr2
