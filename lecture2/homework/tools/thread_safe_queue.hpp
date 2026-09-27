#ifndef TOOLS__THREAD_SAFE_QUEUE_HPP
#define TOOLS__THREAD_SAFE_QUEUE_HPP

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <queue>
#include <utility>

namespace tools
{
template <typename T, bool PopWhenFull = false>
class ThreadSafeQueue
{
public:
  explicit ThreadSafeQueue(
    std::size_t max_size, std::function<void()> full_handler = [] {})
  : max_size_(max_size), full_handler_(std::move(full_handler))
  {
  }

  void push(const T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (queue_.size() >= max_size_) {
      if constexpr (PopWhenFull) {
        queue_.pop();
      } else {
        full_handler_();
        return;
      }
    }

    queue_.push(value);
    not_empty_condition_.notify_one();
  }

  void pop(T & value)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    not_empty_condition_.wait(lock, [this] { return !queue_.empty(); });
    value = std::move(queue_.front());
    queue_.pop();
  }

  bool try_pop_for(T & value, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!not_empty_condition_.wait_for(lock, timeout, [this] { return !queue_.empty(); })) {
      return false;
    }
    value = std::move(queue_.front());
    queue_.pop();
    return true;
  }

private:
  std::queue<T> queue_;
  std::size_t max_size_;
  std::mutex mutex_;
  std::condition_variable not_empty_condition_;
  std::function<void()> full_handler_;
};

}  // namespace tools

#endif  // TOOLS__THREAD_SAFE_QUEUE_HPP
