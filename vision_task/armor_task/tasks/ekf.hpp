#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace armor_task
{

class Ekf
{
public:
  Ekf() = default;

  Ekf(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  std::map<std::string, double> data;  //卡方检验数据
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 100;
  double last_nis;

  const Eigen::VectorXd &state() const { return x_; }
  const Eigen::MatrixXd &covariance() const { return P_; }

private:
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  int nees_count_ = 0;
  int nis_count_ = 0;
  int total_count_ = 0;
};

} // namespace armor_task

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP