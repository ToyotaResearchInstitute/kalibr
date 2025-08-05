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
  size_t n_exhausted_iterators = 0;
  size_t pivot_index = 0;
  // Find the pivot index, which is the iterator with the oldest observation.
  for (size_t i = 0; i < n_sources_; ++i) {
    if (iterators_[i] == end_iterators_[i]) {
      n_exhausted_iterators++;
      continue;
    }
    if (iterators_[i]->time() < iterators_[pivot_index]->time()) {
      pivot_index = i;
    }
  }

  // If all iterators are exhausted, we are done.
  if (n_exhausted_iterators == n_sources_) {
    is_finished_ = true;
    return;
  }

  // Define the time window for synchronization.
  aslam::Time window_start = iterators_[pivot_index]->time();
  aslam::Time window_end = window_start + tolerance_;

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
