#include <kalibr2/SynchronizedObservationView.hpp>

namespace kalibr2 {

SynchronizedObservationView::SynchronizedObservationView(
    const std::vector<std::vector<GridCalibrationTargetObservation>>& sources, aslam::Duration tolerance)
    : sources_(sources), tolerance_(tolerance) {}

SynchronizedObservationView::Iterator SynchronizedObservationView::begin() const {
  return Iterator(sources_, tolerance_);
}

SynchronizedObservationView::Iterator SynchronizedObservationView::end() const {
  return Iterator();
}

SynchronizedObservationView::Iterator::Iterator() : is_finished_(true) {}

SynchronizedObservationView::Iterator::Iterator(
    const std::vector<std::vector<GridCalibrationTargetObservation>>& sources, aslam::Duration tolerance)
    : n_sources_(sources.size()), tolerance_(tolerance), is_finished_(false) {
  for (const auto& source : sources) {
    iterators_.push_back(source.begin());
    end_iterators_.push_back(source.end());
  }
  // Initialize the first sync set
  ++(*this);
}

// The logic to approximately synchronize observations across multiple sources is based on having a iterator for each
// source. We would take the top elements from each iterator, and choose the oldest one as a pivot. Then we would look
// for observations in the time window defined by the pivot and the tolerance. If we find an observation in the time
// window, we add it to the current sync set. If we do not find any observation in the time window, we discard the pivot
// and continue.
void SynchronizedObservationView::Iterator::find_next_set() {
  std::optional<size_t> pivot_index = get_pivot_index();
  if (!pivot_index.has_value()) {
    is_finished_ = true;
    return;
  }

  // Define the time window for synchronization.
  aslam::Time window_start = iterators_[pivot_index.value()]->time();
  aslam::Time window_end = window_start + tolerance_;

  // Get the next sync set. Since the sets are ordered by time, we can just iterate through the top
  // of the iterators and check if they fall within the time window.
  current_sync_set_.assign(n_sources_, std::nullopt);
  for (size_t i = 0; i < n_sources_; ++i) {
    if (iterators_[i] == end_iterators_[i]) {
      continue;
    }
    if (iterators_[i]->time() >= window_start && iterators_[i]->time() <= window_end) {
      current_sync_set_[i] = *iterators_[i];
      iterators_[i]++;
    }
  }
}

std::optional<size_t> SynchronizedObservationView::Iterator::get_pivot_index() const {
  std::optional<size_t> pivot_it = std::nullopt;
  for (size_t i = 0; i < n_sources_; ++i) {
    // If the iterator is exhausted, skip it.
    if (iterators_[i] == end_iterators_[i]) {
      continue;
    }
    // Find the iterator with the oldest observation.
    // If there's already a pivot, compare and update if necessary.
    // Else set the current iterator as the pivot.
    if (pivot_it.has_value()) {
      if (iterators_[i]->time() < iterators_[pivot_it.value()]->time()) {
        pivot_it = i;
      }
    } else {
      pivot_it = i;
    }
  }
  return pivot_it;
}

std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>> GetAllObservationsFromSource(
    const std::vector<kalibr2::SyncedSet>& sets, size_t source_index) {
  std::vector<std::optional<aslam::cameras::GridCalibrationTargetObservation>> observations;

  for (const auto& set : sets) {
    observations.push_back(set.at(source_index));
  }

  return observations;
}

}  // namespace kalibr2
