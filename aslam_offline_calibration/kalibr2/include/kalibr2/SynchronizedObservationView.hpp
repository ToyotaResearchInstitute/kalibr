
#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include <aslam/Time.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>

namespace kalibr2 {

using aslam::cameras::GridCalibrationTargetObservation;
using SyncedSet = std::vector<std::optional<GridCalibrationTargetObservation>>;

class SynchronizedObservationView {
  // A class to synchronize observations from multiple sources based on timestamps
  // and a tolerance window. Returns an iterator that yields synchronized sets of data,
  // one from each source at most, within the specified tolerance.
  // A set could not contain an observation from every source. In that case,
  // the missing observation will be represented by std::nullopt.
 public:
  class Iterator {
   public:
    // Required iterator traits
    using iterator_category = std::input_iterator_tag;
    // Returns a vector of optional GridCalibrationTargetObservation,
    // where each element corresponds to a source.
    using value_type = std::vector<std::optional<GridCalibrationTargetObservation>>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type*;
    using reference = const value_type&;

    Iterator();
    Iterator(const std::vector<std::vector<GridCalibrationTargetObservation>>& sources, aslam::Duration tolerance);

    reference operator*() const { return current_sync_set_; }
    Iterator& operator++() {
      find_next_set();
      return *this;
    }
    bool operator!=(const Iterator& other) const { return is_finished_ != other.is_finished_; }

   private:
    // Find the next synchronized set of observations across all sources
    // within the specified tolerance.
    void find_next_set();

    size_t n_sources_;
    aslam::Duration tolerance_;
    bool is_finished_;
    value_type current_sync_set_;
    std::vector<typename std::vector<GridCalibrationTargetObservation>::const_iterator> iterators_;
    std::vector<typename std::vector<GridCalibrationTargetObservation>::const_iterator> end_iterators_;
  };

  SynchronizedObservationView(const std::vector<std::vector<GridCalibrationTargetObservation>>& sources,
                              aslam::Duration tolerance);

  Iterator begin() const;
  Iterator end() const;

 private:
  // The sources of observations, each source is a vector of GridCalibrationTargetObservation.
  const std::vector<std::vector<GridCalibrationTargetObservation>>& sources_;
  // The tolerance for synchronization. Will be applied symmetrically.
  // timestamp difference must be within [-tolerance, +tolerance].
  aslam::Duration tolerance_;
};

/**
 * @brief Retrieves all observations from a specific source index across a collection of synchronized sets.
 *
 * This function iterates over a vector of synchronized observation sets and extracts the observation
 * at the specified source index from each set. The result is a vector containing the observations,
 * where each observation is represented as an optional value (it may be empty if the observation is missing).
 *
 * @param sets A vector of synchronized observation sets, where each set contains observations from multiple sources.
 * @param source_index The index of the source whose observations are to be retrieved from each set.
 * @return A vector of optional GridCalibrationTargetObservation objects, one for each set.
 */
std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>> GetAllObservationsFromSource(
    const std::vector<kalibr2::SyncedSet>& sets, size_t source_index);

}  // namespace kalibr2
