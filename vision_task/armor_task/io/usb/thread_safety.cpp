#include "thread_safety.hpp"

namespace tools
{
template <typename T>
ThreadSafeQueue<T>::ThreadSafeQueue(std::size_t capacity) : capacity_(capacity)
{
}

template <typename T>
void ThreadSafeQueue<T>::set_capacity(std::size_t capacity)
{
    std::lock_guard<std::mutex> lock(mutex_);
    capacity_ = capacity;
    trim_to_capacity_locked();
}

template <typename T>
bool ThreadSafeQueue<T>::push(const T &value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (capacity_ > 0 && queue_.size() >= capacity_)
    {
        queue_.pop_front();
    }
    queue_.push_back(value);
    cv_.notify_one();
    return true;
}

template <typename T>
bool ThreadSafeQueue<T>::push(T &&value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (capacity_ > 0 && queue_.size() >= capacity_)
    {
        queue_.pop_front();
    }
    queue_.push_back(std::move(value));
    cv_.notify_one();
    return true;
}

template <typename T>
bool ThreadSafeQueue<T>::pop(T &value)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty())
    {
        return false;
    }
    value = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

template <typename T>
bool ThreadSafeQueue<T>::front(T &value) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty())
    {
        return false;
    }
    value = queue_.front();
    return true;
}

template <typename T>
bool ThreadSafeQueue<T>::wait_pop(T &value, std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [this] { return !queue_.empty(); }))
    {
        return false;
    }
    value = std::move(queue_.front());
    queue_.pop_front();
    return true;
}

template <typename T>
bool ThreadSafeQueue<T>::peek_latest(T &value) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (queue_.empty())
    {
        return false;
    }
    value = queue_.back();
    return true;
}

template <typename T>
std::size_t ThreadSafeQueue<T>::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

template <typename T>
bool ThreadSafeQueue<T>::empty() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
}

template <typename T>
void ThreadSafeQueue<T>::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
}

template <typename T>
void ThreadSafeQueue<T>::trim_to_capacity_locked()
{
    if (capacity_ == 0)
    {
        return;
    }
    while (queue_.size() > capacity_)
    {
        queue_.pop_front();
    }
}

template class ThreadSafeQueue<io::Cboard2Vision>;
template class ThreadSafeQueue<QuatStamped>;
} // namespace tools
