// Unit test phần lõi: giả lập lidar 2D bằng ray-casting lên các hình chữ nhật.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "reflector_detector/reflector_detector.hpp"

namespace rd = reflector_detector;

namespace
{

struct Rect
{
  double cx, cy, yaw, len, wid;
  float intensity;
};

struct Scan
{
  std::vector<float> ranges, intensities;
  double angle_min, angle_inc, range_min, range_max;
};

double cross(double ax, double ay, double bx, double by) { return ax * by - ay * bx; }

Scan makeScan(const std::vector<Rect>& rects, int n = 720)
{
  Scan s;
  s.angle_min = -M_PI;
  s.angle_inc = 2.0 * M_PI / n;
  s.range_min = 0.05;
  s.range_max = 30.0;
  s.ranges.assign(n, std::numeric_limits<float>::infinity());
  s.intensities.assign(n, 0.0f);

  for (int i = 0; i < n; ++i) {
    const double a = s.angle_min + i * s.angle_inc;
    const double dx = std::cos(a), dy = std::sin(a);
    double best = std::numeric_limits<double>::max();
    float best_int = 0.0f;
    for (const auto& r : rects) {
      const double c = std::cos(r.yaw), sn = std::sin(r.yaw);
      const double hl = r.len / 2, hw = r.wid / 2;
      const double lx[4] = {hl, hl, -hl, -hl}, ly[4] = {hw, -hw, -hw, hw};
      double px[4], py[4];
      for (int k = 0; k < 4; ++k) {
        px[k] = r.cx + lx[k] * c - ly[k] * sn;
        py[k] = r.cy + lx[k] * sn + ly[k] * c;
      }
      for (int k = 0; k < 4; ++k) {
        const int m = (k + 1) % 4;
        const double ex = px[m] - px[k], ey = py[m] - py[k];
        const double den = cross(dx, dy, ex, ey);
        if (std::fabs(den) < 1e-9) continue;
        const double t = cross(px[k], py[k], ex, ey) / den;
        const double u = cross(px[k], py[k], dx, dy) / den;
        if (t > 0.0 && u >= 0.0 && u <= 1.0 && t < best) {
          best = t;
          best_int = r.intensity;
        }
      }
    }
    if (best < std::numeric_limits<double>::max()) {
      s.ranges[i] = static_cast<float>(best);
      s.intensities[i] = best_int;
    }
  }
  return s;
}

rd::Params testParams()
{
  rd::Params p;
  p.intensity_min = 1000.0f;
  p.cluster_tolerance = 0.10;
  p.max_index_gap = 2;
  p.cluster_min_points = 3;
  p.length_min = 0.20;
  p.length_max = 0.50;
  p.width_min = 0.0;
  p.width_max = 0.20;
  return p;
}

rd::DetectionResult run(const rd::Params& p, const Scan& s)
{
  rd::ReflectorDetector det(p);
  return det.detect(s.ranges, s.intensities, s.angle_min, s.angle_inc, s.range_min, s.range_max);
}

double axisDiff(double a, double b) { return std::fabs(rd::normalizeAxisAngle(a - b)); }

}  // namespace

TEST(ReflectorDetector, DetectsPoseAndRejectsDistractors)
{
  // cạnh dài quay về phía sensor, lệch ~0.4 rad => sensor thấy hình chữ L (2 cạnh)
  const Rect target{1.5, 0.4, std::atan2(0.4, 1.5) + M_PI / 2 - 0.4, 0.30, 0.10, 3000.0f};
  const std::vector<Rect> world = {
      target,
      {2.0, -1.0, 0.0, 0.04, 0.04, 3000.0f},   // quá nhỏ => loại bởi length_min
      {0.0, 3.0, 0.0, 2.00, 0.05, 3000.0f},    // tường sáng quá dài => loại bởi length_max
      {-1.5, -1.5, 0.5, 0.30, 0.10, 100.0f}};  // đúng hình dạng nhưng intensity thấp

  for (auto method : {rd::FitMethod::MIN_AREA_RECT, rd::FitMethod::PCA}) {
    rd::Params p = testParams();
    p.fit_method = method;
    const auto res = run(p, makeScan(world));
    ASSERT_EQ(res.detections.size(), 1u);
    const auto& d = res.detections[0];
    if (method == rd::FitMethod::MIN_AREA_RECT) {
      EXPECT_NEAR(d.x, target.cx, 0.03);
      EXPECT_NEAR(d.y, target.cy, 0.03);
      EXPECT_LT(axisDiff(d.yaw, target.yaw), 0.05);
    } else {
      // PCA bị lệch khi chỉ thấy hình chữ L, chỉ yêu cầu gần đúng
      EXPECT_NEAR(d.x, target.cx, 0.08);
      EXPECT_NEAR(d.y, target.cy, 0.08);
    }
  }
}

TEST(ReflectorDetector, FacingSensorYawPointsToSensor)
{
  // thanh mỏng đặt ở phía +x, vuông góc trục x => pháp tuyến hướng về sensor là yaw = pi
  const Rect target{2.0, 0.0, M_PI / 2, 0.30, 0.01, 3000.0f};
  rd::Params p = testParams();
  p.yaw_mode = rd::YawMode::FACING_SENSOR;
  const auto res = run(p, makeScan({target}));
  ASSERT_EQ(res.detections.size(), 1u);
  EXPECT_NEAR(std::fabs(res.detections[0].yaw), M_PI, 0.05);
}

TEST(ReflectorDetector, SortedNearToFar)
{
  const std::vector<Rect> world = {
      {3.0, 1.0, std::atan2(1.0, 3.0) + M_PI / 2 - 0.3, 0.30, 0.10, 3000.0f},
      {1.0, -1.0, std::atan2(-1.0, 1.0) + M_PI / 2 + 0.3, 0.30, 0.10, 3000.0f}};
  const auto res = run(testParams(), makeScan(world));
  ASSERT_EQ(res.detections.size(), 2u);
  EXPECT_LT(std::hypot(res.detections[0].x, res.detections[0].y),
            std::hypot(res.detections[1].x, res.detections[1].y));
}

TEST(ReflectorDetector, MergesClusterAcrossWraparound)
{
  // reflector nằm ngay trục -x (góc ±pi) => bị cắt đôi ở đầu/cuối mảng ranges
  const Rect target{-1.5, 0.0, M_PI / 2, 0.30, 0.01, 3000.0f};
  const auto res = run(testParams(), makeScan({target}));
  ASSERT_EQ(res.detections.size(), 1u);
  EXPECT_NEAR(res.detections[0].x, -1.5, 0.03);
  EXPECT_NEAR(res.detections[0].y, 0.0, 0.03);
}

TEST(ReflectorDetector, NoReflectorNoDetection)
{
  const auto res = run(testParams(), makeScan({{1.0, 1.0, 0.0, 0.3, 0.1, 50.0f}}));
  EXPECT_TRUE(res.detections.empty());
}

TEST(MedianPose, RobustToOutlierAndYawWraparound)
{
  // yaw dạng trục quanh ±90°: 1.55, -1.55 (= 1.59 + k*pi), 1.56 và 1 mẫu nhiễu hẳn
  std::vector<rd::PoseSample> s = {{1.00, 0.50, 1.55},  {1.01, 0.51, -1.55}, {0.99, 0.49, 1.56},
                                   {1.00, 0.50, 1.57},  {3.00, 2.00, 0.20}};  // mẫu cuối là outlier
  const auto a = rd::medianPose(s, true);
  EXPECT_NEAR(a.x, 1.00, 0.02);
  EXPECT_NEAR(a.y, 0.50, 0.02);
  EXPECT_LT(axisDiff(a.yaw, 1.56), 0.05);
  EXPECT_TRUE(a.num_samples == 5);

  const auto e = rd::medianPose({}, true);
  EXPECT_TRUE(e.num_samples == 0);
}

TEST(Offset, AppliedInPoseFrame)
{
  // pose hướng +y (yaw = 90°); offset x=1 (đi tới theo yaw), y=0.5 (sang trái), xoay thêm 180°
  const rd::PoseSample p{2.0, 3.0, M_PI / 2};
  const rd::PoseOffset off{1.0, 0.5, M_PI};
  const auto r = rd::applyOffset(p, off);
  EXPECT_NEAR(r.x, 2.0 - 0.5, 1e-9);
  EXPECT_NEAR(r.y, 3.0 + 1.0, 1e-9);
  EXPECT_NEAR(std::fabs(r.yaw), M_PI / 2, 1e-9);  // 90° + 180° = -90°

  const auto z = rd::applyOffset(p, rd::PoseOffset{});
  EXPECT_NEAR(z.x, 2.0, 1e-12);
  EXPECT_NEAR(z.y, 3.0, 1e-12);
  EXPECT_NEAR(z.yaw, M_PI / 2, 1e-12);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}