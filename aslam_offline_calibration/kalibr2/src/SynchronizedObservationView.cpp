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
  // Exclude exhausted iterators.
  std::vector<typename std::vector<GridCalibrationTargetObservation>::const_iterator> active_iterators;
  for (size_t i = 0; i < n_sources_; ++i) {
    if (iterators_[i] != end_iterators_[i]) {
      active_iterators.push_back(iterators_[i]);
    }
  }

  // If all are exhausted, we are done.
  if (active_iterators.empty()) {
    is_finished_ = true;
    return;
  }

  // Choose the pivot iterator as the one with oldest data.
  auto pivot_it_ptr =
      std::min_element(active_iterators.begin(), active_iterators.end(), [](const auto& a, const auto& b) {
        return a->time() < b->time();
      });
  auto pivot_timestamp = (*pivot_it_ptr)->time();

  // Define the time window for synchronization.
  aslam::Time window_start = pivot_timestamp;
  aslam::Time window_end = pivot_timestamp + tolerance_;

  // Get the next sync set. Since the sets are ordered by time, we can just iterate through the top
  // of the iterators and check if they fall within the time window.
  current_sync_set_.assign(n_sources_, std::nullopt);
  for (size_t i = 0; i < n_sources_; ++i) {
    if (iterators_[i]->time() >= window_start && iterators_[i]->time() <= window_end) {
      current_sync_set_[i] = *iterators_[i];
      iterators_[i]++;
    }
  }
}

}  // namespace kalibr2
