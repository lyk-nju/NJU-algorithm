#pragma once

#include "../dataframe/base_cmd.hpp"
#include <Eigen/Geometry>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <tuple>

namespace tools
{
template <typename T>
class ThreadSafeQueue
{
public:
    explicit ThreadSafeQueue(std::size_t capacity = 0);

    void set_capacity(std::size_t capacity);
    bool push(const T &value);
    bool push(T &&value);
    bool pop(T &value);
    bool front(T &value) const;
    bool wait_pop(T &value, std::chrono::milliseconds timeout);
    bool peek_latest(T &value) const;
    std::size_t size() const;
    bool empty() const;
    void clear();

private:
    void trim_to_capacity_locked();

    std::size_t capacity_ = 0;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<T> queue_;
};

using QuatStamped = std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>;

extern template class ThreadSafeQueue<io::Cboard2Vision>;
extern template class ThreadSafeQueue<QuatStamped>;
} // namespace tools
