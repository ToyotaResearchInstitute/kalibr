#include <algorithm>
#include <numeric>

#include <kalibr2/CameraGraph.hpp>

namespace kalibr2 {

namespace detail {

Corners ToCornersIdxs(std::optional<GridCalibrationTargetObservation> observation) {
  Corners corner_idxs;
  if (observation.has_value()) {
    observation.value().getCornersIdx(corner_idxs);
  }
  return corner_idxs;
}

SyncedSetCorners ToCornersIdxs(const SyncedSet& set) {
  SyncedSetCorners corners;
  std::transform(set.begin(), set.end(), std::back_inserter(corners), [](const auto& observation) {
    return ToCornersIdxs(observation);
  });
  return corners;
}

Corners GetCommonCorners(const Corners& corners1, const Corners& corners2) {
  Corners common_corners;
  std::set_intersection(corners1.begin(), corners1.end(), corners2.begin(), corners2.end(),
                        std::back_inserter(common_corners));
  return common_corners;
}

std::map<std::pair<size_t, size_t>, Corners> GetCommonCornersMap(const SyncedSetCorners& corners_per_set) {
  std::map<std::pair<size_t, size_t>, Corners> common_corners_map;
  for (size_t i = 0; i < corners_per_set.size(); ++i) {
    for (size_t j = i + 1; j < corners_per_set.size(); ++j) {
      Corners common_corners = GetCommonCorners(corners_per_set[i], corners_per_set[j]);
      common_corners_map[{i, j}] = common_corners;
    }
  }
  return common_corners_map;
}

std::map<std::pair<size_t, size_t>, size_t> ComputeTotalCommonCorners(const std::vector<SyncedSet> synced_sets) {
  // Compute the edges as a function of the number of common corners
  // between the sets of synced observations. Across the whole dataset.

  // Convert the sets of synced observations to sets of corners indices
  std::vector<SyncedSetCorners> corners_per_set;
  std::transform(synced_sets.begin(), synced_sets.end(), std::back_inserter(corners_per_set), [](const auto& set) {
    return ToCornersIdxs(set);
  });

  // Get common corners between all pairs of sets a map of the form (i, j) -> Corners
  std::vector<std::map<std::pair<size_t, size_t>, Corners>> common_corners_map;
  std::transform(corners_per_set.begin(), corners_per_set.end(), std::back_inserter(common_corners_map),
                 [](const auto& corners) {
                   return GetCommonCornersMap(corners);
                 });

  // Convert the common corners map to a size map (i, j) -> length(common_corners)
  std::vector<std::map<std::pair<size_t, size_t>, size_t>> common_corners_size_map;
  std::transform(common_corners_map.begin(), common_corners_map.end(), std::back_inserter(common_corners_size_map),
                 [](const auto& map) {
                   return ToSizeMap(map);
                 });

  // Accumulate the sizes of common corners across all sets
  // This will give us a map of the form (i, j) -> total_common on the complete dataset
  std::map<std::pair<size_t, size_t>, size_t> common_corners_size_accumulated_map;
  for (const auto& map : common_corners_size_map) {
    for (const auto& [camera_pair, size] : map) {
      common_corners_size_accumulated_map[camera_pair] += size;
    }
  }

  return common_corners_size_accumulated_map;
}

}  // namespace detail

Graph<size_t> BuildCameraGraph(const std::vector<SyncedSet> synced_sets) {
  // Add the nodes being the cameras to the graph
  Graph<size_t> graph;

  size_t num_cameras = synced_sets.at(0).size();
  SM_ASSERT_TRUE(std::runtime_error,
                 std::all_of(synced_sets.begin(), synced_sets.end(),
                             [num_cameras](const SyncedSet& set) {
                               return set.size() == num_cameras;
                             }),
                 "All synced sets must have the same number of cameras");

  for (size_t i = 0; i < num_cameras; ++i) {
    graph.AddNode(i);
  }

  // Add edges between nodes based on the common corners size map
  auto common_corners_size_accumulated_map = detail::ComputeTotalCommonCorners(synced_sets);
  for (const auto& [camera_pair, n_shared_observations] : common_corners_size_accumulated_map) {
    if (n_shared_observations > 0) {
      // Abussing that idx is equal to node value.
      const auto& [cam_idx_1, cam_idx_2] = camera_pair;
      graph.AddEdgesBetweenNodes(cam_idx_1, cam_idx_2, 1.0 / n_shared_observations);
    }
  }

  return graph;
}

/**
 * @brief Computes the transformation between two nodes in a camera graph using Dijkstra's shortest path result.
 *
 * Given a map of optimal baseline transformations between node pairs and the result of a Dijkstra search,
 * this function finds the transformation from one node to another by following the shortest path between them.
 * The transformation is composed by accumulating the transformations along the path.
 * If the left node is closer to the Dijkstra start node than the right node, the composed transformation is inverted.
 *
 * @param optimal_baselines Map containing transformations between pairs of node indices.
 * @param result Result of Dijkstra's shortest path search, providing distances and previous indices.
 * @param left_node_index Index of the left node.
 * @param right_node_index Index of the right node.
 * @return sm::kinematics::Transformation The transformation from the left node to the right node along the shortest
 * path.
 */
sm::kinematics::Transformation GetTransform(
    const std::map<std::pair<size_t, size_t>, sm::kinematics::Transformation>& transformation_map,
    const common_robotics_utilities::simple_graph_search::DijkstrasResult& result, size_t left_node_index,
    size_t right_node_index) {
  auto distance_left_to_dijkstra_start = result.GetNodeDistance(left_node_index);
  auto distance_right_to_dijkstra_start = result.GetNodeDistance(right_node_index);

  bool is_left_further = distance_left_to_dijkstra_start > distance_right_to_dijkstra_start;
  // We always go from further to closer to the start of the dijkstra search.
  size_t start_index, end_index;
  if (is_left_further) {
    start_index = left_node_index;
    end_index = right_node_index;
  } else {
    start_index = right_node_index;
    end_index = left_node_index;
  }

  // Follow the dijkstra path until we reach the end index
  // and tore the transformations along the path.
  std::vector<sm::kinematics::Transformation> transformations;
  auto current_index = start_index;
  while (true) {
    auto previous_index = result.GetPreviousIndex(current_index);
    transformations.push_back(transformation_map.at({current_index, previous_index}));
    if (previous_index == end_index) {
      break;
    }
    current_index = previous_index;
  }

  auto composed_transform =
      std::accumulate(transformations.begin(), transformations.end(), sm::kinematics::Transformation{},
                      std::multiplies<sm::kinematics::Transformation>());

  // If the left node was further away, we need to invert the composed transform.
  // Since we computed T_right_t_left
  if (!is_left_further) {
    composed_transform = composed_transform.inverse();
  }

  return composed_transform;
}

}  // namespace kalibr2
