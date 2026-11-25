#include "capstone_localization/estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Cholesky>

namespace {
using capstone_localization::Covariance;
using capstone_localization::Pose;
using capstone_localization::between;
using capstone_localization::wrap_angle;

constexpr double kPi = 3.14159265358979323846;

class Csv {
 public:
  explicit Csv(const char *path) : stream_(path) {
    if (!next()) throw std::runtime_error(std::string("missing CSV header: ") + path);
    for (std::size_t i = 0; i < fields_.size(); ++i) {
      if (!columns_.emplace(fields_[i], i).second) throw std::runtime_error("duplicate CSV field");
    }
  }

  bool next() {
    std::string line;
    if (!std::getline(stream_, line)) return false;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    fields_.clear();
    std::istringstream split(line);
    std::string value;
    while (std::getline(split, value, ',')) fields_.push_back(value);
    if (!columns_.empty() && fields_.size() != columns_.size()) {
      throw std::runtime_error("unexpected field count in CSV row");
    }
    return true;
  }

  const std::string &get(const std::string &key) const { return fields_.at(columns_.at(key)); }

  double number(const std::string &key) const {
    const auto &value = get(key);
    std::size_t end = 0;
    const auto decoded = std::stod(value, &end);
    if (end != value.size()) throw std::runtime_error("invalid numeric field " + key);
    return decoded;
  }

  bool flag(const std::string &key) const {
    const auto &value = get(key);
    if (value == "1") return true;
    if (value == "0") return false;
    throw std::runtime_error("invalid boolean field " + key);
  }

 private:
  std::ifstream stream_;
  std::map<std::string, std::size_t> columns_;
  std::vector<std::string> fields_;
};

struct Sample {
  double stamp;
  Pose pose;
};

Sample sample(const Csv &csv) {
  Sample row{csv.number("stamp"), {csv.number("x"), csv.number("y"), csv.number("yaw")}};
  if (!std::isfinite(row.stamp) || !row.pose.allFinite()) throw std::runtime_error("nonfinite pose");
  return row;
}

std::vector<Sample> truth_track(const char *path) {
  Csv csv(path);
  std::vector<Sample> rows;
  while (csv.next()) {
    auto row = sample(csv);
    if (!rows.empty() && row.stamp <= rows.back().stamp) {
      throw std::runtime_error("reference timestamps must increase strictly");
    }
    rows.push_back(row);
  }
  if (rows.size() < 2) throw std::runtime_error("too few reference poses");
  return rows;
}

std::optional<Pose> reference_at(const std::vector<Sample> &truth, double time) {
  if (time < truth.front().stamp || time > truth.back().stamp) return {};
  auto right = std::lower_bound(truth.begin() + 1, truth.end(), time,
                                [](const Sample &row, double t) { return row.stamp < t; });
  const auto &a = *(right - 1);
  const auto &b = *right;
  const double step = b.stamp - a.stamp;
  if (step > .2) return {};
  const double fraction = (time - a.stamp) / step;
  Pose delta = b.pose - a.pose;
  delta.z() = wrap_angle(delta.z());
  Pose result = a.pose + fraction * delta;
  result.z() = wrap_angle(result.z());
  return result;
}

struct Errors {
  double position_squared = 0;
  double yaw_squared = 0;
  std::size_t size = 0;

  void observe(const Pose &difference) {
    position_squared += difference.head<2>().squaredNorm();
    yaw_squared += std::pow(wrap_angle(difference.z()), 2);
    ++size;
  }

  std::optional<double> position_rmse() const {
    if (!size) return {};
    return std::sqrt(position_squared / size);
  }
  std::optional<double> yaw_rmse() const {
    if (!size) return {};
    return std::sqrt(yaw_squared / size);
  }
};

struct Consistency {
  double sum = 0;
  std::size_t size = 0;
  std::size_t outside = 0;

  void observe(double squared) {
    if (!std::isfinite(squared) || squared < 0) throw std::runtime_error("invalid normalized error");
    sum += squared;
    ++size;
    outside += squared < .216 || squared > 9.348;
  }
};

struct Aligned {
  Sample estimate;
  Pose reference;
  double distance;
  double yaw_error;
};

struct ScanStats {
  std::size_t count = 0;
  std::size_t accepted = 0;
  std::size_t fallback = 0;
  std::size_t failed_match = 0;
  Consistency nis;
};

struct Loss {
  std::vector<double> recovered;
  std::size_t episodes = 0;
  double seconds = 0;
  bool unrecovered = false;
};

using Window = std::optional<std::pair<std::int64_t, std::int64_t>>;

bool in_window(const Window &window, double stamp) {
  if (!window) return false;
  const long double ns = std::round(static_cast<long double>(stamp) * 1e9L);
  return window->first <= ns && ns < window->second;
}

class Evaluation {
 public:
  std::vector<Aligned> poses;
  std::vector<double> distances;
  std::vector<double> latency_ms;
  std::vector<double> scan_latency_ms;
  Errors full;
  Errors window;
  Consistency nees;
  ScanStats scans;
  std::size_t wheel_rows = 0;

  Evaluation(const std::vector<Sample> &truth, const char *path, const Window &time_window) {
    Csv csv(path);
    std::optional<double> previous_wheel;
    while (csv.next()) {
      const auto latency = csv.number("latency_us");
      if (!std::isfinite(latency) || latency < 0) throw std::runtime_error("invalid kernel latency");
      latency_ms.push_back(latency / 1000.0);

      const auto &kind = csv.get("kind");
      if (kind == "S") {
        scan_latency_ms.push_back(latency / 1000.0);
        ++scans.count;
        scans.accepted += csv.flag("accepted");
        scans.fallback += csv.flag("fallback");
        scans.failed_match += csv.get("reason") == "scan_match_failed";
        if (std::isfinite(csv.number("mx"))) scans.nis.observe(csv.number("nis"));
        continue;
      }
      if (kind != "W") throw std::runtime_error("unknown replay event kind");
      ++wheel_rows;
      const auto pose = sample(csv);
      if (previous_wheel && pose.stamp <= *previous_wheel) {
        throw std::runtime_error("prediction timestamps must increase strictly");
      }
      previous_wheel = pose.stamp;
      const bool in_gap = in_window(time_window, pose.stamp);
      const auto reference = reference_at(truth, pose.stamp);
      if (!reference) {
        if (in_gap) throw std::runtime_error("unaligned prediction in requested window");
        continue;
      }

      Pose error = pose.pose - *reference;
      error.z() = wrap_angle(error.z());
      Covariance covariance;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          covariance(i, j) = csv.number("p" + std::to_string(i) + std::to_string(j));
        }
      }
      if (!covariance.allFinite() || !covariance.isApprox(covariance.transpose(), 1e-10)) {
        throw std::runtime_error("invalid output covariance");
      }
      const Eigen::LLT<Covariance> factor(covariance);
      if (factor.info() != Eigen::Success) throw std::runtime_error("non-positive covariance");
      double squared = error.dot(factor.solve(error));
      nees.observe(squared);
      full.observe(error);
      if (in_gap) window.observe(error);
      const double distance = error.head<2>().norm();
      distances.push_back(distance);
      poses.push_back({pose, *reference, distance, error.z()});
    }
    if (poses.size() < 2 || (time_window && window.size < 10)) {
      throw std::runtime_error("too few synchronized predictions");
    }
  }

  Errors relative_error() const {
    Errors result;
    for (auto start = poses.begin(); start != poses.end(); ++start) {
      const double time = start->estimate.stamp + 1.0;
      auto end = std::lower_bound(start + 1, poses.end(), time, [](const Aligned &row, double t) {
        return row.estimate.stamp < t;
      });
      if (end == poses.end() || std::abs(end->estimate.stamp - time) >= .05) continue;
      result.observe(between(start->estimate.pose, end->estimate.pose) -
                     between(start->reference, end->reference));
    }
    return result;
  }

  Loss losses() const {
    Loss result;
    std::optional<double> start;
    for (auto it = poses.begin(); it != poses.end(); ++it) {
      const bool bad = it->distance > .5 || std::abs(it->yaw_error) > 15.0 * kPi / 180.0;
      if (bad && !start) start = it->estimate.stamp;
      if (it == poses.end() - 1) result.unrecovered = bad;
      if (start && (!bad || it == poses.end() - 1)) {
        const double duration = it->estimate.stamp - *start;
        if (duration > 1.0) {
          ++result.episodes;
          result.seconds += duration;
          if (!bad) result.recovered.push_back(duration);
        }
        start.reset();
      }
    }
    return result;
  }
};

double quantile(const std::vector<double> &sorted, double probability) {
  const double index = probability * (sorted.size() - 1);
  const auto lo = static_cast<std::size_t>(index);
  return sorted[lo] + (index - lo) * (sorted[std::min(lo + 1, sorted.size() - 1)] - sorted[lo]);
}

std::optional<double> fraction(double numerator, std::size_t n) {
  if (!n) return {};
  return numerator / n;
}

void number(const char *key, std::optional<double> value) {
  std::cout << "  \"" << key << "\": ";
  if (value) std::cout << *value;
  else std::cout << "null";
  std::cout << ",\n";
}

void consistency(const std::string &label, const Consistency &result) {
  number((label + "_mean").c_str(), fraction(result.sum, result.size));
  number((label + "_outside_95_fraction").c_str(), fraction(result.outside, result.size));
  number((label + "_samples").c_str(), result.size);
}

void print(Evaluation &evaluation, const Window &window) {
  const auto rpe = evaluation.relative_error();
  const auto loss = evaluation.losses();
  const auto &scan = evaluation.scans;
  std::sort(evaluation.distances.begin(), evaluation.distances.end());
  auto &latency = evaluation.latency_ms;
  std::sort(latency.begin(), latency.end());
  auto &scan_latency = evaluation.scan_latency_ms;
  std::sort(scan_latency.begin(), scan_latency.end());
  std::cout << std::setprecision(17) << "{\n";
  number("samples", evaluation.full.size);
  number("synchronized_fraction", fraction(evaluation.full.size, evaluation.wheel_rows));
  number("ate_rmse_m", evaluation.full.position_rmse());
  number("yaw_rmse_rad", evaluation.full.yaw_rmse());
  number("position_p95_m", quantile(evaluation.distances,.95));
  number("rpe_1s_translation_rmse_m", rpe.position_rmse());
  number("rpe_1s_yaw_rmse_rad", rpe.yaw_rmse());
  consistency("nees", evaluation.nees);
  consistency("nis", scan.nis);
  number("loss_episodes", loss.episodes);
  const double duration = evaluation.poses.back().estimate.stamp - evaluation.poses.front().estimate.stamp;
  number("loss_fraction", loss.seconds / duration);
  number("scan_acceptance_fraction", fraction(scan.accepted, scan.count));
  number("scan_match_failed_fraction", fraction(scan.failed_match, scan.count));
  number("fallback_fraction", fraction(scan.fallback, scan.count));
  const auto over = std::count_if(latency.begin(), latency.end(), [](double t) { return t > 20.0; });
  number("kernel_over_20ms_fraction", fraction(over, latency.size()));
  if (window) {
    number("samples_window", evaluation.window.size);
    number("ate_rmse_window_m", evaluation.window.position_rmse());
    number("yaw_rmse_window_deg", *evaluation.window.yaw_rmse() * 180.0 / kPi);
  }
  std::cout << "  \"kernel_latency_ms_p50_p95_p99\": [" << quantile(latency,.5) << ", "
            << quantile(latency,.95) << ", " << quantile(latency,.99) << "],\n";
  if (!scan_latency.empty()) {
    std::cout << "  \"scan_kernel_latency_ms_p50_p95_p99\": [" << quantile(scan_latency,.5) << ", "
              << quantile(scan_latency,.95) << ", " << quantile(scan_latency,.99) << "],\n";
  }
  std::cout << "  \"recovery_seconds\": [";
  for (std::size_t i = 0; i < loss.recovered.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << loss.recovered[i];
  }
  std::cout << "],\n  \"unrecovered_at_end\": " << (loss.unrecovered ? "true" : "false") << "\n}\n";
}

void paired(const char *path) {
  Csv csv(path);
  std::map<std::string, std::vector<std::pair<double, double>>> grouped;
  while (csv.next()) {
    const auto a = csv.number("A_m");
    const auto b = csv.number("B_m");
    if (!std::isfinite(a) || !std::isfinite(b)) throw std::runtime_error("nonfinite paired metric");
    grouped[csv.get("layout")].emplace_back(a, b);
  }
  if (grouped.size() < 2) throw std::runtime_error("paired comparison needs two layouts");

  std::mt19937_64 random(7);
  std::vector<double> bootstrap(10000, 0);
  std::size_t pairs = 0;
  double mean_a = 0, mean_b = 0;
  for (const auto &entry : grouped) {
    const auto &rows = entry.second;
    const double weight = 1.0 / (grouped.size() * rows.size());
    pairs += rows.size();
    for (const auto &row : rows) {
      mean_a += weight * row.first;
      mean_b += weight * row.second;
    }
    std::uniform_int_distribution<std::size_t> draw(0, rows.size() - 1);
    for (auto &estimate : bootstrap) {
      for (std::size_t j = 0; j < rows.size(); ++j) {
        const auto &row = rows[draw(random)];
        estimate += weight * (row.second - row.first);
      }
    }
  }
  std::sort(bootstrap.begin(), bootstrap.end());
  std::cout << std::setprecision(17) << "{\n";
  number("pairs", pairs);
  number("A_mean_m", mean_a);
  number("B_mean_m", mean_b);
  number("B_minus_A_m", mean_b - mean_a);
  std::cout << "  \"bootstrap_95_m\": [" << quantile(bootstrap, .025) << ", "
            << quantile(bootstrap, .975) << "]\n}\n";
}

struct Grid {
  std::size_t rows, columns;
  double resolution;
  Pose origin;
  std::vector<int> cells;
};

Grid load_grid(const char *path) {
  std::ifstream input(path);
  Grid grid;
  if (!(input >> grid.columns >> grid.rows >> grid.resolution >> grid.origin.x() >>
        grid.origin.y() >> grid.origin.z()) || grid.columns == 0 || grid.rows == 0 ||
      grid.columns > 100000000 / grid.rows || !std::isfinite(grid.resolution) ||
      grid.resolution <= 0 || !grid.origin.allFinite()) {
    throw std::runtime_error("invalid occupancy grid header");
  }
  grid.cells.resize(grid.rows * grid.columns);
  for (auto &cell : grid.cells) {
    if (!(input >> cell) || cell < -1 || cell > 100) throw std::runtime_error("invalid map cell");
  }
  std::string extra;
  if (input >> extra) throw std::runtime_error("extra map cells");
  return grid;
}

void map_overlap(const char *estimate_path, const char *reference_path) {
  auto estimate = load_grid(estimate_path);
  auto reference = load_grid(reference_path);
  if (estimate.rows != reference.rows || estimate.columns != reference.columns ||
      std::abs(estimate.resolution - reference.resolution) > 1e-9 ||
      (estimate.origin - reference.origin).cwiseAbs().maxCoeff() > 1e-9) {
    throw std::runtime_error("map lattices must be co-registered");
  }
  std::size_t known = 0, intersection = 0, total = 0;
  for (std::size_t i = 0; i < estimate.cells.size(); ++i) {
    if (estimate.cells[i] < 0 || reference.cells[i] < 0) continue;
    ++known;
    const bool a = estimate.cells[i] >= 65, b = reference.cells[i] >= 65;
    total += a || b;
    intersection += a && b;
  }
  std::cout << std::setprecision(17) << "{\n";
  number("occupied_iou", fraction(intersection,total));
  number("jointly_known_fraction", fraction(known,estimate.cells.size()));
  std::cout << "  \"evaluated_cells\": " << known << "\n}\n";
}

std::int64_t nanoseconds(const std::string &value) {
  std::size_t end = 0;
  auto parsed = std::stoll(value, &end);
  if (end != value.size() || parsed < 0) throw std::runtime_error("invalid window boundary");
  return parsed;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 4 && std::string(argv[1]) == "--map") {
      map_overlap(argv[2], argv[3]);
      return 0;
    }
    if (argc == 3 && std::string(argv[1]) == "--paired") {
      paired(argv[2]);
      return 0;
    }
    if (argc != 3 && argc != 5) {
      std::cerr << "usage: localization_metrics ground_truth.csv prediction.csv [start_ns end_ns]\n"
                   "       localization_metrics --paired scored_pairs.csv\n"
                   "       localization_metrics --map estimate.txt reference.txt\n";
      return 2;
    }
    Window window;
    if (argc == 5) {
      window = std::make_pair(nanoseconds(argv[3]), nanoseconds(argv[4]));
      if (window->second <= window->first) throw std::runtime_error("window must have positive duration");
    }
    Evaluation result(truth_track(argv[1]), argv[2], window);
    print(result, window);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "localization_metrics: " << error.what() << '\n';
    return 1;
  }
}
