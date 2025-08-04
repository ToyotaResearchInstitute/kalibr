#include <kalibr2/SyncedObservationView.hpp>

SyncedObservationView::SyncedObservationView(const std::vector<std::vector<GridCalibrationTargetObservation>>& sources,
                                             aslam::Duration tolerance)
    : sources_(sources), tolerance_(tolerance) {}

SyncedObservationView::SyncIterator SyncedObservationView::begin() const {
  return SyncIterator(sources_, tolerance_);
}

SyncedObservationView::SyncIterator SyncedObservationView::end() const {
  return SyncIterator();
}

SyncedObservationView::SyncIterator::SyncIterator() : is_finished_(true) {}

SyncedObservationView::SyncIterator::SyncIterator(
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
void SyncedObservationView::SyncIterator::find_next_set() {
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
  aslam::Time window_start = std::max(aslam::Time(0), pivot_timestamp - tolerance_);
  aslam::Time window_end = pivot_timestamp + tolerance_;

  // Get the next sync set.
  current_sync_set_.assign(n_sources_, std::nullopt);
  for (size_t i = 0; i < n_sources_; ++i) {
    auto found_it = std::find_if(iterators_[i], end_iterators_[i],
                                 [window_start, window_end](const GridCalibrationTargetObservation& d) {
                                   return d.time() >= window_start && d.time() <= window_end;
                                 });

    if (found_it != end_iterators_[i]) {
      current_sync_set_[i] = *found_it;
      // Element added, advance the iterator to the next one.
      iterators_[i] = std::next(found_it);
    }
  }

  // If no matches were found for the current pivot discard it and continue.
  if (std::none_of(current_sync_set_.begin(), current_sync_set_.end(), [](const auto& opt) {
        return opt.has_value();
      })) {
    (*pivot_it_ptr)++;
  }
}
