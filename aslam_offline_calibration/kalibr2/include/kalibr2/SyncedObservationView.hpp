
#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include <aslam/Time.hpp>
#include <aslam/cameras/GridCalibrationTargetObservation.hpp>

namespace kalibr2 {

using aslam::cameras::GridCalibrationTargetObservation;

class SyncedObservationView {
  // A class to synchronize observations from multiple sources based on timestamps
  // and a tolerance window. Returns an iterator that yields synchronized sets of data,
  // one from each source at most, within the specified tolerance.
  // A set could not contain an observation from every source. In that case,
  // the missing observation will be represented by std::nullopt.
 public:
  class SyncIterator {
   public:
    // Required iterator traits
    using iterator_category = std::input_iterator_tag;
    // Returns a vector of optional GridCalibrationTargetObservation,
    // where each element corresponds to a source.
    using value_type = std::vector<std::optional<GridCalibrationTargetObservation>>;
    using difference_type = std::ptrdiff_t;
    using pointer = const value_type*;
    using reference = const value_type&;

    SyncIterator();
    SyncIterator(const std::vector<std::vector<GridCalibrationTargetObservation>>& sources, aslam::Duration tolerance);

    reference operator*() const { return current_sync_set_; }
    SyncIterator& operator++() {
      find_next_set();
      return *this;
    }
    bool operator!=(const SyncIterator& other) const { return is_finished_ != other.is_finished_; }

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

  SyncedObservationView(const std::vector<std::vector<GridCalibrationTargetObservation>>& sources,
                        aslam::Duration tolerance);

  SyncIterator begin() const;
  SyncIterator end() const;

 private:
  // The sources of observations, each source is a vector of GridCalibrationTargetObservation.
  const std::vector<std::vector<GridCalibrationTargetObservation>>& sources_;
  // The tolerance for synchronization. Will be applied symmetrically.
  // timestamp difference must be within [-tolerance, +tolerance].
  aslam::Duration tolerance_;
};

}  // namespace kalibr2
