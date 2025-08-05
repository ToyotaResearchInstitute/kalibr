#include <cstdlib>

#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>
#include <gtest/gtest.h>
#include <kalibr2/SynchronizedObservationView.hpp>

namespace {

TEST(SynchronizedObservationViewTest, AllWithinTolerance) {
  auto target_grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(5, 7, 0.01);
  std::vector<aslam::cameras::GridCalibrationTargetObservation> observations;

  constexpr size_t n_sources = 3;
  constexpr size_t n_observations = 10;
  std::array<std::vector<aslam::Time>, n_sources> timestamps;
  aslam::Time start_time(1, 0);
  for (size_t i = 0; i < n_observations; ++i) {
    timestamps.at(0).push_back(start_time + aslam::Duration(0.1 * i));
    timestamps.at(1).push_back(start_time + aslam::Duration(0.1 * i + 0.04));  // Offset by 0.04s
    timestamps.at(2).push_back(start_time + aslam::Duration(0.1 * i - 0.03));  // Offset by -0.03s
  }

  std::vector<std::vector<aslam::cameras::GridCalibrationTargetObservation>> observations_by_source{n_sources};
  for (size_t i = 0; i < n_sources; ++i) {
    for (size_t j = 0; j < n_observations; ++j) {
      // Create a dummy observation with the target grid.
      aslam::cameras::GridCalibrationTargetObservation observation(target_grid);
      observation.setTime(timestamps.at(i).at(j));
      observations_by_source[i].push_back(observation);
    }
  }

  size_t round = 0;
  aslam::Duration tolerance(0.07);
  for (const auto& sync_set : kalibr2::SynchronizedObservationView(observations_by_source, tolerance)) {
    std::cout << "\n--- Round " << ++round << " ---" << std::endl;
    ASSERT_TRUE(std::all_of(sync_set.begin(), sync_set.end(),
                            [](const std::optional<aslam::cameras::GridCalibrationTargetObservation>& obs) {
                              return obs.has_value();
                            }))
        << "All sources should have observations in the sync set";
    for (size_t i = 0; i < sync_set.size(); ++i) {
      std::cout << "  Source " << i + 1 << ": ";
      if (sync_set[i].has_value()) {
        std::cout << sync_set[i].value().time();
      } else {
        std::cout << "---";
      }
      std::cout << std::endl;
    }
  }
  ASSERT_EQ(round, n_observations) << "Should have iterated through all observations";
}

TEST(SynchronizedObservationViewTest, TwoObservationsPerSyncedView) {
  auto target_grid = boost::make_shared<aslam::cameras::GridCalibrationTargetCirclegrid>(5, 7, 0.01);

  constexpr size_t n_sources = 3;
  constexpr size_t n_observations = 10;
  std::array<std::vector<aslam::Time>, n_sources> timestamps;
  aslam::Time start_time(1, 0);
  for (size_t i = 0; i < n_observations; ++i) {
    timestamps.at(0).push_back(start_time + aslam::Duration(0.1 * i));
    timestamps.at(1).push_back(start_time + aslam::Duration(0.1 * i + 0.04));  // Offset by 0.04s
    timestamps.at(2).push_back(start_time + aslam::Duration(0.1 * i - 0.03));  // Offset by -0.03s
  }

  std::vector<std::vector<aslam::cameras::GridCalibrationTargetObservation>> observations_by_source{n_sources};
  for (size_t i = 0; i < n_sources; ++i) {
    for (size_t j = 0; j < n_observations; ++j) {
      aslam::cameras::GridCalibrationTargetObservation observation(target_grid);
      observation.setTime(timestamps.at(i).at(j));
      observations_by_source[i].push_back(observation);
    }
  }

  size_t round = 0;
  aslam::Duration tolerance(0.04);
  for (const auto& sync_set : kalibr2::SynchronizedObservationView(observations_by_source, tolerance)) {
    ++round;
    // At least two sources should have observations in each sync set
    int count = 0;
    for (const auto& obs : sync_set) {
      if (obs.has_value()) ++count;
    }
    ASSERT_GE(count, 2) << "At least two sources should have observations in the sync set";
  }
  // With this tolerance and offset, we expect about twice as many sync sets as observations
  ASSERT_EQ(round, n_observations * 1.5) << "Should have two observations per synced view";
}

}  // namespace
