#include <cstdlib>

#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <common_robotics_utilities/simple_graph.hpp>
#include <common_robotics_utilities/simple_graph_search.hpp>
#include <gtest/gtest.h>
#include <kalibr2/CameraGraph.hpp>
#include <kalibr2/SyncedObservationView.hpp>

namespace {

using aslam::cameras::GridCalibrationTargetObservation;
using kalibr2::SyncedSet;
using kalibr2::SyncedSetCorners;

TEST(CameraGraphTest, DetailToCornersIdxs) {
  auto target_grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(5, 7, 0.01);
  std::vector<GridCalibrationTargetObservation> observations;

  // Build the fake observations
  constexpr size_t n_sources = 3;
  constexpr size_t n_observations = 2;
  std::array<std::vector<aslam::Time>, n_sources> timestamps;
  aslam::Time start_time(1, 0);
  for (size_t i = 0; i < n_observations; ++i) {
    timestamps.at(0).push_back(start_time + aslam::Duration(0.1 * i));
    timestamps.at(1).push_back(start_time + aslam::Duration(0.1 * i + 0.04));  // Offset by 0.04s
    timestamps.at(2).push_back(start_time + aslam::Duration(0.1 * i - 0.03));  // Offset by -0.03s
  }
  std::vector<std::vector<GridCalibrationTargetObservation>> observations_by_source{n_sources};
  for (size_t i = 0; i < n_sources; ++i) {
    for (size_t j = 0; j < n_observations; ++j) {
      // Create a dummy observation with the target grid.
      GridCalibrationTargetObservation observation(target_grid);
      observation.setTime(timestamps.at(i).at(j));
      observations_by_source[i].push_back(observation);
    }
  }

  aslam::Duration tolerance(0.07);
  std::vector<SyncedSet> synced_sets;
  for (const auto& sync_set : kalibr2::SyncedObservationView(observations_by_source, tolerance)) {
    // Store our synced sets.
    synced_sets.push_back(sync_set);
  }
  // Convert to corners.
  std::vector<SyncedSetCorners> corners_per_set;
  std::transform(synced_sets.begin(), synced_sets.end(), std::back_inserter(corners_per_set), [](const auto& set) {
    return kalibr2::detail::ToCornersIdxs(set);
  });
}

TEST(CameraGraphTest, FromCommonCornersToGraph) {
  // Define a set of known corners for each synced set
  // And build the graph from them.
  std::vector<SyncedSetCorners> corners_per_set;
  corners_per_set.push_back({{1, 2, 3}, {1, 3, 4, 6}, {2, 4, 6}});
  corners_per_set.push_back({{1, 3}, {2, 4, 6}, {1, 4, 6}});

  std::vector<std::map<std::pair<size_t, size_t>, kalibr2::Corners>> common_corners_map;
  std::transform(corners_per_set.begin(), corners_per_set.end(), std::back_inserter(common_corners_map),
                 [](const auto& corners) {
                   return kalibr2::detail::GetCommonCornersMap(corners);
                 });

  ASSERT_EQ(common_corners_map.size(), 2) << "Should have two sets of common corners";

  ASSERT_EQ(common_corners_map.at(0).at({0, 1}), (kalibr2::Corners{1, 3}));
  ASSERT_EQ(common_corners_map.at(0).at({0, 2}), (kalibr2::Corners{2}));
  ASSERT_EQ(common_corners_map.at(0).at({1, 2}), (kalibr2::Corners{4, 6}));

  ASSERT_EQ(common_corners_map.at(1).at({0, 1}), (kalibr2::Corners{}));
  ASSERT_EQ(common_corners_map.at(1).at({0, 2}), (kalibr2::Corners{1}));
  ASSERT_EQ(common_corners_map.at(1).at({1, 2}), (kalibr2::Corners{4, 6}));

  std::vector<std::map<std::pair<size_t, size_t>, size_t>> common_corners_size_map;
  std::transform(common_corners_map.begin(), common_corners_map.end(), std::back_inserter(common_corners_size_map),
                 [](const auto& map) {
                   return kalibr2::detail::ToSizeMap(map);
                 });

  ASSERT_EQ(common_corners_size_map.size(), 2) << "Should have two sets of common corners sizes";
  ASSERT_EQ(common_corners_size_map.at(0).at({0, 1}), 2);
  ASSERT_EQ(common_corners_size_map.at(0).at({0, 2}), 1);
  ASSERT_EQ(common_corners_size_map.at(0).at({1, 2}), 2);

  ASSERT_EQ(common_corners_size_map.at(1).at({0, 1}), 0);
  ASSERT_EQ(common_corners_size_map.at(1).at({0, 2}), 1);
  ASSERT_EQ(common_corners_size_map.at(1).at({1, 2}), 2);

  std::map<std::pair<size_t, size_t>, size_t> common_corners_size_accumulated_map;
  for (const auto& map : common_corners_size_map) {
    for (const auto& pair : map) {
      common_corners_size_accumulated_map[pair.first] += pair.second;
    }
  }

  ASSERT_EQ(common_corners_size_accumulated_map.size(), 3) << "Should have three unique pairs of sources";
  ASSERT_EQ(common_corners_size_accumulated_map.at({0, 1}), 2);
  ASSERT_EQ(common_corners_size_accumulated_map.at({0, 2}), 2);
  ASSERT_EQ(common_corners_size_accumulated_map.at({1, 2}), 4);

  common_robotics_utilities::simple_graph::Graph<size_t> graph{};
  for (size_t i = 0; i < corners_per_set.at(0).size(); ++i) {
    graph.AddNode(i);
  }

  for (const auto& pair : common_corners_size_accumulated_map) {
    if (pair.second > 0) {
      // Abussing that idx is equal to node value.
      graph.AddEdgesBetweenNodes(pair.first.first, pair.first.second, 1.0 / pair.second);
    }
  }
}

}  // namespace
