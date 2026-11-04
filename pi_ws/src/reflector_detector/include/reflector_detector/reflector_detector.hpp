// Core thuật toán detect reflector từ LaserScan 2D (không phụ thuộc ROS => test độc lập được).
//
//  1. filterScan   : lọc điểm theo intensity / range / góc
//  2. cluster      : gom cụm theo thứ tự tia (khoảng cách + khoảng trống index)
//  3. shape filter : lọc theo chiều dài / chiều rộng hình chữ nhật bao
//  4. fitRectangle : tâm (x, y) + yaw bằng min-area-rect hoặc PCA
#ifndef REFLECTOR_DETECTOR_REFLECTOR_DETECTOR_HPP_
#define REFLECTOR_DETECTOR_REFLECTOR_DETECTOR_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace reflector_detector
{

struct Point2
{
  double x{0.0};
  double y{0.0};
  float intensity{0.0f};
  int index{0};  // chỉ số tia trong LaserScan
};

enum class FitMethod
{
  MIN_AREA_RECT,
  PCA
};

enum class YawMode
{
  MAJOR_AXIS,     // yaw = hướng cạnh dài
  FACING_SENSOR   // yaw = pháp tuyến chỉ về phía sensor
};

struct Params
{
  // --- 1. filter ---
  float intensity_min{1000.0f};
  float intensity_max{std::numeric_limits<float>::max()};
  double range_min{0.10};
  double range_max{15.0};
  double roi_angle_min{-M_PI};  // [rad]
  double roi_angle_max{M_PI};   // [rad]

  // --- 2. cluster ---
  double cluster_tolerance{0.10};
  int max_index_gap{2};
  int cluster_min_points{3};
  int cluster_max_points{500};

  // --- 3. shape (length >= width) ---
  double length_min{0.08};
  double length_max{0.30};
  double width_min{0.0};
  double width_max{0.10};

  // --- 4. pose ---
  FitMethod fit_method{FitMethod::MIN_AREA_RECT};
  YawMode yaw_mode{YawMode::MAJOR_AXIS};
  double rect_angle_step_deg{0.5};
  double closeness_d0{0.01};  // [m] nhiễu sàn của tiêu chí closeness
};

struct RectFit
{
  double cx{0.0};
  double cy{0.0};
  double yaw{0.0};     // hướng cạnh dài, trong (-pi/2, pi/2]
  double length{0.0};  // cạnh dài
  double width{0.0};   // cạnh ngắn
};

struct Detection
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};       // yaw xuất ra (theo yaw_mode)
  double rect_yaw{0.0};  // hướng cạnh dài của hình chữ nhật
  double length{0.0};
  double width{0.0};
  double mean_intensity{0.0};
  int num_points{0};
};

struct DetectionResult
{
  std::vector<Point2> filtered;        // điểm sau lọc intensity
  std::vector<Detection> detections;   // sắp xếp gần -> xa
  std::size_t num_clusters{0};
  std::size_t num_clusters_size_ok{0};
};

// Chuẩn hoá góc về (-pi, pi]
inline double normalizeAngle(double a)
{
  a = std::fmod(a + M_PI, 2.0 * M_PI);
  if (a <= 0.0) a += 2.0 * M_PI;
  return a - M_PI;
}

// Chuẩn hoá góc của 1 trục (không có chiều) về (-pi/2, pi/2]
inline double normalizeAxisAngle(double a)
{
  return 0.5 * normalizeAngle(2.0 * a);
}

class ReflectorDetector
{
public:
  ReflectorDetector() = default;
  explicit ReflectorDetector(const Params& p) : p_(p) {}

  void setParams(const Params& p) { p_ = p; }
  const Params& params() const { return p_; }

  // ------------------------------------------------------------------
  // Bước 1: lọc intensity / range / ROI góc, trả về điểm theo thứ tự tia
  // ------------------------------------------------------------------
  std::vector<Point2> filterScan(const std::vector<float>& ranges,
                                 const std::vector<float>& intensities,
                                 double angle_min, double angle_inc,
                                 double scan_range_min, double scan_range_max) const
  {
    std::vector<Point2> pts;
    const std::size_t n = std::min(ranges.size(), intensities.size());
    const double rmin = std::max(p_.range_min, scan_range_min);
    const double rmax = std::min(p_.range_max, scan_range_max);
    pts.reserve(n / 8 + 1);

    for (std::size_t i = 0; i < n; ++i) {
      const double r = ranges[i];
      if (!std::isfinite(r) || r < rmin || r > rmax) continue;

      const float inten = intensities[i];
      if (!std::isfinite(inten) || inten < p_.intensity_min || inten > p_.intensity_max) continue;

      const double a = angle_min + static_cast<double>(i) * angle_inc;
      if (!inRoi(normalizeAngle(a))) continue;

      Point2 pt;
      pt.x = r * std::cos(a);
      pt.y = r * std::sin(a);
      pt.intensity = inten;
      pt.index = static_cast<int>(i);
      pts.push_back(pt);
    }
    return pts;
  }

  // ------------------------------------------------------------------
  // Bước 2: clustering theo thứ tự tia. O(N), không cần KD-tree.
  // wrap = true nếu scan phủ 360 độ => ghép cụm cuối với cụm đầu.
  // ------------------------------------------------------------------
  std::vector<std::vector<Point2>> cluster(const std::vector<Point2>& pts,
                                           bool wrap, int num_beams) const
  {
    std::vector<std::vector<Point2>> out;
    if (pts.empty()) return out;

    std::vector<Point2> cur;
    cur.push_back(pts[0]);
    for (std::size_t i = 1; i < pts.size(); ++i) {
      if (isNeighbor(pts[i - 1], pts[i], pts[i].index - pts[i - 1].index)) {
        cur.push_back(pts[i]);
      } else {
        out.push_back(std::move(cur));
        cur.clear();
        cur.push_back(pts[i]);
      }
    }
    out.push_back(std::move(cur));

    if (wrap && out.size() > 1) {
      const Point2& last = out.back().back();
      const Point2& first = out.front().front();
      const int gap = first.index + num_beams - last.index;
      if (isNeighbor(last, first, gap)) {
        out.back().insert(out.back().end(), out.front().begin(), out.front().end());
        out.erase(out.begin());
      }
    }
    return out;
  }

  // ------------------------------------------------------------------
  // Bước 4: hình chữ nhật bao (tâm, yaw, length, width)
  // ------------------------------------------------------------------
  RectFit fitRectangle(const std::vector<Point2>& pts) const
  {
    const double angle =
        (p_.fit_method == FitMethod::PCA) ? pcaAngle(pts) : minAreaAngle(pts);

    const Box b = computeBox(pts, angle);
    const double du = b.umax - b.umin;
    const double dv = b.vmax - b.vmin;
    const double cu = 0.5 * (b.umin + b.umax);
    const double cv = 0.5 * (b.vmin + b.vmax);
    const double c = std::cos(angle);
    const double s = std::sin(angle);

    RectFit f;
    // u = (c, s), v = (-s, c)
    f.cx = cu * c - cv * s;
    f.cy = cu * s + cv * c;
    if (du >= dv) {
      f.length = du;
      f.width = dv;
      f.yaw = angle;
    } else {
      f.length = dv;
      f.width = du;
      f.yaw = angle + 0.5 * M_PI;
    }
    f.yaw = normalizeAxisAngle(f.yaw);
    return f;
  }

  // ------------------------------------------------------------------
  // Chạy toàn bộ pipeline
  // ------------------------------------------------------------------
  DetectionResult detect(const std::vector<float>& ranges,
                         const std::vector<float>& intensities,
                         double angle_min, double angle_inc,
                         double scan_range_min, double scan_range_max) const
  {
    DetectionResult res;
    res.filtered = filterScan(ranges, intensities, angle_min, angle_inc,
                              scan_range_min, scan_range_max);

    const std::size_t n = std::min(ranges.size(), intensities.size());
    const double inc = std::fabs(angle_inc);
    const bool full_circle =
        n > 1 && static_cast<double>(n - 1) * inc >= 2.0 * M_PI - 2.0 * inc;

    const auto clusters = cluster(res.filtered, full_circle, static_cast<int>(n));
    res.num_clusters = clusters.size();

    for (const auto& c : clusters) {
      const int np = static_cast<int>(c.size());
      if (np < p_.cluster_min_points || np > p_.cluster_max_points) continue;
      ++res.num_clusters_size_ok;

      const RectFit f = fitRectangle(c);
      if (f.length < p_.length_min || f.length > p_.length_max) continue;
      if (f.width < p_.width_min || f.width > p_.width_max) continue;

      res.detections.push_back(makeDetection(c, f));
    }

    std::sort(res.detections.begin(), res.detections.end(),
              [](const Detection& a, const Detection& b) {
                return std::hypot(a.x, a.y) < std::hypot(b.x, b.y);
              });
    return res;
  }

private:
  struct Box
  {
    double umin, umax, vmin, vmax;
  };

  bool inRoi(double a) const
  {
    if (p_.roi_angle_min <= p_.roi_angle_max) {
      return a >= p_.roi_angle_min && a <= p_.roi_angle_max;
    }
    return a >= p_.roi_angle_min || a <= p_.roi_angle_max;  // vùng vắt qua ±pi
  }

  bool isNeighbor(const Point2& a, const Point2& b, int index_gap) const
  {
    return index_gap <= p_.max_index_gap &&
           std::hypot(b.x - a.x, b.y - a.y) <= p_.cluster_tolerance;
  }

  // Hộp bao theo hệ trục quay góc `a`: u = (cos a, sin a), v = (-sin a, cos a)
  static Box computeBox(const std::vector<Point2>& pts, double a)
  {
    const double c = std::cos(a);
    const double s = std::sin(a);
    Box b{std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest(),
          std::numeric_limits<double>::max(), std::numeric_limits<double>::lowest()};
    for (const auto& p : pts) {
      const double u = p.x * c + p.y * s;
      const double v = -p.x * s + p.y * c;
      b.umin = std::min(b.umin, u);
      b.umax = std::max(b.umax, u);
      b.vmin = std::min(b.vmin, v);
      b.vmax = std::max(b.vmax, v);
    }
    return b;
  }

  static double pcaAngle(const std::vector<Point2>& pts)
  {
    double mx = 0.0, my = 0.0;
    for (const auto& p : pts) {
      mx += p.x;
      my += p.y;
    }
    mx /= static_cast<double>(pts.size());
    my /= static_cast<double>(pts.size());

    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const auto& p : pts) {
      const double dx = p.x - mx;
      const double dy = p.y - my;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }
    // góc trục chính của ma trận hiệp phương sai 2x2 (dạng đóng, không cần Eigen)
    return 0.5 * std::atan2(2.0 * sxy, sxx - syy);
  }

  // Tiêu chí "closeness" (Zhang et al., 2017): mỗi điểm càng sát cạnh hộp càng tốt.
  // Dùng để phá hoà khi diện tích bằng nhau (vd. hình chữ L => bao lồi là tam giác vuông,
  // 3 cạnh đều cho hộp bao cùng diện tích).
  double closeness(const std::vector<Point2>& pts, double a) const
  {
    const Box b = computeBox(pts, a);
    const double c = std::cos(a);
    const double s = std::sin(a);
    double sum = 0.0;
    for (const auto& p : pts) {
      const double u = p.x * c + p.y * s;
      const double v = -p.x * s + p.y * c;
      const double du = std::min(b.umax - u, u - b.umin);
      const double dv = std::min(b.vmax - v, v - b.vmin);
      sum += 1.0 / std::max(std::min(du, dv), p_.closeness_d0);
    }
    return sum;
  }

  // Quét góc [0, 90°): lấy các góc có diện tích hộp bao xấp xỉ nhỏ nhất (trong 10%),
  // rồi chọn góc có closeness lớn nhất (hoà -> diện tích nhỏ hơn).
  double minAreaAngle(const std::vector<Point2>& pts) const
  {
    const double step = std::max(p_.rect_angle_step_deg, 0.1) * M_PI / 180.0;

    std::vector<std::pair<double, double>> cand;  // (angle, area)
    double min_area = std::numeric_limits<double>::max();
    for (double a = 0.0; a < 0.5 * M_PI - 1e-9; a += step) {
      const Box b = computeBox(pts, a);
      const double area = (b.umax - b.umin) * (b.vmax - b.vmin);
      cand.emplace_back(a, area);
      min_area = std::min(min_area, area);
    }

    const double thr = min_area * 1.10 + 1e-6;
    double best_a = cand.front().first;
    double best_area = std::numeric_limits<double>::max();
    double best_close = -1.0;
    for (const auto& c : cand) {
      if (c.second > thr) continue;
      const double cl = closeness(pts, c.first);
      if (cl > best_close * (1.0 + 1e-9) ||
          (cl >= best_close * (1.0 - 1e-9) && c.second < best_area)) {
        best_a = c.first;
        best_area = c.second;
        best_close = cl;
      }
    }
    return best_a;
  }

  Detection makeDetection(const std::vector<Point2>& c, const RectFit& f) const
  {
    Detection d;
    d.x = f.cx;
    d.y = f.cy;
    d.rect_yaw = f.yaw;
    d.length = f.length;
    d.width = f.width;
    d.num_points = static_cast<int>(c.size());

    double sum = 0.0;
    for (const auto& p : c) sum += p.intensity;
    d.mean_intensity = sum / static_cast<double>(c.size());

    if (p_.yaw_mode == YawMode::FACING_SENSOR) {
      // pháp tuyến của cạnh dài, chọn chiều hướng về sensor (gốc toạ độ)
      double nx = -std::sin(f.yaw);
      double ny = std::cos(f.yaw);
      if (nx * (-f.cx) + ny * (-f.cy) < 0.0) {
        nx = -nx;
        ny = -ny;
      }
      d.yaw = std::atan2(ny, nx);
    } else {
      d.yaw = f.yaw;
    }
    return d;
  }

  Params p_;
};

// ----------------------------------------------------------------------
// Gộp nhiều mẫu pose (nhiều scan liên tiếp) thành 1 pose bằng median.
// ----------------------------------------------------------------------
struct PoseSample
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct AggregatedPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double std_xy{0.0};  // độ lệch chuẩn vị trí sqrt(var_x + var_y) [m]
  int num_samples{0};
};

inline double median(std::vector<double> v)
{
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return (n % 2 == 1) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// yaw_is_axis = true: yaw chỉ là 1 trục (chu kỳ pi, vd. yaw_mode major_axis) => nhân đôi góc
// khi lấy trung bình vòng tròn để không bị lỗi quanh ±90°.
inline AggregatedPose medianPose(const std::vector<PoseSample>& samples, bool yaw_is_axis)
{
  AggregatedPose r;
  r.num_samples = static_cast<int>(samples.size());
  if (samples.empty()) return r;

  const double k = yaw_is_axis ? 2.0 : 1.0;
  std::vector<double> xs, ys;
  double sc = 0.0, ss = 0.0, mx = 0.0, my = 0.0;
  for (const auto& s : samples) {
    xs.push_back(s.x);
    ys.push_back(s.y);
    sc += std::cos(k * s.yaw);
    ss += std::sin(k * s.yaw);
    mx += s.x;
    my += s.y;
  }
  const double n = static_cast<double>(samples.size());
  mx /= n;
  my /= n;

  const double ref = std::atan2(ss, sc) / k;  // hướng tham chiếu (trung bình vòng tròn)
  std::vector<double> dev;
  double var = 0.0;
  for (const auto& s : samples) {
    dev.push_back(normalizeAngle(k * (s.yaw - ref)) / k);
    var += (s.x - mx) * (s.x - mx) + (s.y - my) * (s.y - my);
  }

  r.x = median(xs);
  r.y = median(ys);
  const double yaw = ref + median(dev);
  r.yaw = yaw_is_axis ? normalizeAxisAngle(yaw) : normalizeAngle(yaw);
  r.std_xy = std::sqrt(var / n);
  return r;
}

// Offset tính trong hệ toạ độ của reflector (x dọc theo yaw, y sang trái, yaw cộng thêm).
struct PoseOffset
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};  // [rad]
};

inline PoseSample applyOffset(const PoseSample& p, const PoseOffset& o)
{
  const double c = std::cos(p.yaw);
  const double s = std::sin(p.yaw);
  PoseSample r;
  r.x = p.x + o.x * c - o.y * s;
  r.y = p.y + o.x * s + o.y * c;
  r.yaw = normalizeAngle(p.yaw + o.yaw);
  return r;
}

}  // namespace reflector_detector

#endif  // REFLECTOR_DETECTOR_REFLECTOR_DETECTOR_HPP_