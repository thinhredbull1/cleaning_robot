// ROS1 node: detect reflector hình chữ nhật từ sensor_msgs/LaserScan (có intensities).
//
// Hai chế độ (param `mode`):
//   continuous : detect liên tục mỗi scan, publish ~reflector_poses.
//   service    : chỉ detect khi có request tới service DetectDock. Trong cửa sổ `detect_duration`
//                node thu các detection từng scan, gộp bằng median rồi trả kết quả. Không thấy
//                reflector trong cửa sổ => success=false.
//
// Sub : scan                      (sensor_msgs/LaserScan)
// Pub : ~reflector_poses          (geometry_msgs/PoseArray)   x, y, yaw (gần -> xa)
//       ~filtered_points          (sensor_msgs/PointCloud2)   điểm sau lọc intensity (debug)
//       ~markers                  (visualization_msgs/MarkerArray)  hộp + mũi tên yaw (debug, frame của scan)
// Srv : <service/name>            (moving_server_msgs/DetectDock)  chỉ ở mode service

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moving_server_msgs/DetectDock.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "reflector_detector/reflector_detector.hpp"

namespace rd = reflector_detector;

class ReflectorDetectorNode
{
public:
  ReflectorDetectorNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    loadParams(pnh);
    detector_.setParams(params_);

    if (!target_frame_.empty()) {
      tf_buffer_.reset(new tf2_ros::Buffer());
      tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
    }

    pose_pub_ = pnh.advertise<geometry_msgs::PoseArray>("reflector_poses", 1);
    if (publish_debug_) {
      cloud_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("filtered_points", 1);
      marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("markers", 1);
    }
    scan_sub_ = nh.subscribe("scan", 1, &ReflectorDetectorNode::scanCallback, this);

    if (service_mode_) {
      detect_srv_ = nh.advertiseService(service_name_, &ReflectorDetectorNode::handleDetectDock, this);
      ROS_INFO("reflector_detector: mode=service, service='%s', duration=%.2fs, min_samples=%d",
               detect_srv_.getService().c_str(), detect_duration_, min_samples_);
    } else {
      ROS_INFO("reflector_detector: mode=continuous");
    }
    ROS_INFO("intensity_min=%.1f, length=[%.3f, %.3f], width=[%.3f, %.3f], target_frame='%s'",
             params_.intensity_min, params_.length_min, params_.length_max, params_.width_min,
             params_.width_max, target_frame_.c_str());
  }

private:
  // ---------------------------------------------------------------- params
  void loadParams(ros::NodeHandle& pnh)
  {
    rd::Params& p = params_;

    pnh.param<std::string>("target_frame", target_frame_, "");
    pnh.param("tf_timeout", tf_timeout_, 0.1);
    pnh.param("publish_debug", publish_debug_, true);

    std::string mode;
    pnh.param<std::string>("mode", mode, "continuous");
    if (mode == "service") {
      service_mode_ = true;
    } else if (mode != "continuous") {
      ROS_WARN("mode '%s' không hợp lệ, dùng continuous", mode.c_str());
    }
    pnh.param<std::string>("service/name", service_name_, "detect_dock");
    pnh.param("service/detect_duration", detect_duration_, 1.5);
    pnh.param("service/min_samples", min_samples_, 5);
    pnh.param("service/max_std_xy", max_std_xy_, 0.05);

    double d = 0.0;
    pnh.param("filter/intensity_min", d, 1000.0);
    p.intensity_min = static_cast<float>(d);
    pnh.param("filter/intensity_max", d, 1.0e9);
    p.intensity_max = static_cast<float>(d);
    pnh.param("filter/range_min", p.range_min, 0.10);
    pnh.param("filter/range_max", p.range_max, 15.0);
    double roi_min_deg = -180.0, roi_max_deg = 180.0;
    pnh.param("filter/roi_angle_min_deg", roi_min_deg, -180.0);
    pnh.param("filter/roi_angle_max_deg", roi_max_deg, 180.0);
    p.roi_angle_min = rd::normalizeAngle(roi_min_deg * M_PI / 180.0);
    p.roi_angle_max = rd::normalizeAngle(roi_max_deg * M_PI / 180.0);
    // 360° đầy đủ: -180 và 180 cùng chuẩn hoá về pi => tự mở rộng lại
    if (roi_max_deg - roi_min_deg >= 360.0) {
      p.roi_angle_min = -M_PI;
      p.roi_angle_max = M_PI;
    }

    pnh.param("cluster/tolerance", p.cluster_tolerance, 0.10);
    pnh.param("cluster/max_index_gap", p.max_index_gap, 2);
    pnh.param("cluster/min_points", p.cluster_min_points, 3);
    pnh.param("cluster/max_points", p.cluster_max_points, 500);

    pnh.param("shape/length_min", p.length_min, 0.08);
    pnh.param("shape/length_max", p.length_max, 0.30);
    pnh.param("shape/width_min", p.width_min, 0.0);
    pnh.param("shape/width_max", p.width_max, 0.10);
    if (p.length_min > p.length_max || p.width_min > p.width_max) {
      ROS_WARN("shape: min > max, kiểm tra lại length/width trong config");
    }

    std::string method, yaw_mode;
    pnh.param<std::string>("pose/fit_method", method, "min_area_rect");
    pnh.param<std::string>("pose/yaw_mode", yaw_mode, "major_axis");
    pnh.param("pose/rect_angle_step_deg", p.rect_angle_step_deg, 0.5);
    pnh.param("pose/closeness_d0", p.closeness_d0, 0.01);

    // offset áp lên pose cuối (trong hệ toạ độ của reflector)
    double offset_yaw_deg = 0.0;
    pnh.param("pose/offset/x", offset_.x, 0.0);
    pnh.param("pose/offset/y", offset_.y, 0.0);
    pnh.param("pose/offset/yaw_deg", offset_yaw_deg, 0.0);
    offset_.yaw = offset_yaw_deg * M_PI / 180.0;

    if (method == "pca") {
      p.fit_method = rd::FitMethod::PCA;
    } else {
      if (method != "min_area_rect") {
        ROS_WARN("pose/fit_method '%s' không hợp lệ, dùng min_area_rect", method.c_str());
      }
      p.fit_method = rd::FitMethod::MIN_AREA_RECT;
    }
    if (yaw_mode == "facing_sensor") {
      p.yaw_mode = rd::YawMode::FACING_SENSOR;
    } else {
      if (yaw_mode != "major_axis") {
        ROS_WARN("pose/yaw_mode '%s' không hợp lệ, dùng major_axis", yaw_mode.c_str());
      }
      p.yaw_mode = rd::YawMode::MAJOR_AXIS;
    }
  }

  // ---------------------------------------------------------------- scan callback
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
  {
    // mode service: nằm yên (không tốn CPU) khi không có request
    if (service_mode_ && !collecting_.load()) return;

    if (scan->intensities.size() != scan->ranges.size()) {
      ROS_WARN_THROTTLE(2.0,
                        "LaserScan không có intensities (ranges=%zu, intensities=%zu). "
                        "Kiểm tra driver lidar.",
                        scan->ranges.size(), scan->intensities.size());
      return;
    }

    const rd::DetectionResult res =
        detector_.detect(scan->ranges, scan->intensities, scan->angle_min, scan->angle_increment,
                         scan->range_min, scan->range_max);

    ROS_DEBUG("filtered=%zu clusters=%zu size_ok=%zu detections=%zu", res.filtered.size(),
              res.num_clusters, res.num_clusters_size_ok, res.detections.size());

    if (publish_debug_) {
      publishFilteredCloud(scan->header, res.filtered);
      publishMarkers(scan->header, res.detections);
    }
    publishPoses(scan->header, res.detections);

    if (service_mode_ && collecting_.load()) recordSample(scan->header, res.detections);
  }

  // ---------------------------------------------------------------- service
  // Gom mẫu từ callback: mỗi scan lấy reflector gần nhất (detections đã sắp xếp gần -> xa).
  void recordSample(const std_msgs::Header& header, const std::vector<rd::Detection>& dets)
  {
    geometry_msgs::PoseStamped ps;
    const bool have = !dets.empty() && toOutputFrame(header, dets.front(), ps, false);

    std::lock_guard<std::mutex> lock(samples_mutex_);
    ++scans_total_;
    if (!have) return;
    sample_frame_ = ps.header.frame_id;
    rd::PoseSample s;
    s.x = ps.pose.position.x;
    s.y = ps.pose.position.y;
    s.yaw = yawOf(ps.pose.orientation);
    samples_.push_back(s);
  }

  bool handleDetectDock(moving_server_msgs::DetectDock::Request& req,
                        moving_server_msgs::DetectDock::Response& res)
  {
    std::unique_lock<std::mutex> busy(service_mutex_, std::try_to_lock);
    if (!busy.owns_lock()) {
      res.success = false;
      res.confidence = 0.0;
      res.message = "Reflector detection is already running";
      return true;
    }

    {
      std::lock_guard<std::mutex> lock(samples_mutex_);
      samples_.clear();
      scans_total_ = 0;
      sample_frame_.clear();
    }

    // thu mẫu trong cửa sổ detect_duration (callback scan chạy ở thread khác)
    collecting_.store(true);
    const ros::Time end = ros::Time::now() + ros::Duration(detect_duration_);
    while (ros::ok() && ros::Time::now() < end) ros::Duration(0.01).sleep();
    collecting_.store(false);

    std::vector<rd::PoseSample> samples;
    std::string frame;
    int scans_total = 0;
    {
      std::lock_guard<std::mutex> lock(samples_mutex_);
      samples = samples_;
      frame = sample_frame_;
      scans_total = scans_total_;
    }

    res.success = false;
    res.confidence = 0.0;
    res.dock_pose.header.stamp = ros::Time::now();

    if (scans_total == 0) {
      res.message = "No LaserScan received within " + std::to_string(detect_duration_) + " s";
      ROS_WARN("DetectDock(%s): %s", req.dock_id.c_str(), res.message.c_str());
      return true;
    }
    if (static_cast<int>(samples.size()) < min_samples_) {
      res.message = "Reflector not found (" + std::to_string(samples.size()) + "/" +
                    std::to_string(scans_total) + " scans, need " + std::to_string(min_samples_) + ")";
      ROS_WARN("DetectDock(%s): %s", req.dock_id.c_str(), res.message.c_str());
      return true;
    }

    const bool axis = params_.yaw_mode == rd::YawMode::MAJOR_AXIS;
    const rd::AggregatedPose agg = rd::medianPose(samples, axis);
    if (max_std_xy_ > 0.0 && agg.std_xy > max_std_xy_) {
      res.message = "Reflector position unstable (std " + std::to_string(agg.std_xy) + " m > " +
                    std::to_string(max_std_xy_) + " m)";
      ROS_WARN("DetectDock(%s): %s", req.dock_id.c_str(), res.message.c_str());
      return true;
    }

    // áp offset lên pose đã lấy median
    rd::PoseSample raw;
    raw.x = agg.x;
    raw.y = agg.y;
    raw.yaw = agg.yaw;
    const rd::PoseSample fin = rd::applyOffset(raw, offset_);

    res.dock_pose.header.frame_id = frame;
    res.dock_pose.pose = makePose(fin.x, fin.y, fin.yaw);

    // moving_server tự sinh approach path
    res.approach_path.header.frame_id = frame;
    res.approach_path.header.stamp = res.dock_pose.header.stamp;

    // confidence = tỉ lệ scan thấy reflector trong cửa sổ
    res.confidence = static_cast<double>(samples.size()) / static_cast<double>(scans_total);
    res.success = true;
    res.message = "Detected reflector for '" + req.dock_id + "' (" + std::to_string(samples.size()) +
                  "/" + std::to_string(scans_total) + " scans, std " + std::to_string(agg.std_xy) + " m)";
    ROS_INFO("DetectDock(%s): x=%.3f y=%.3f yaw=%.3f (offset applied) [%s] %s", req.dock_id.c_str(),
             fin.x, fin.y, fin.yaw, frame.c_str(), res.message.c_str());
    return true;
  }

  // ---------------------------------------------------------------- outputs
  static double yawOf(const geometry_msgs::Quaternion& q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  static geometry_msgs::Pose makePose(double x, double y, double yaw)
  {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    return pose;
  }

  // Detection (frame của scan) -> PoseStamped ở frame xuất (target_frame nếu có).
  // apply_offset = false: giữ pose thô (service mode áp offset sau khi lấy median).
  bool toOutputFrame(const std_msgs::Header& header, const rd::Detection& d,
                     geometry_msgs::PoseStamped& out, bool apply_offset = true)
  {
    rd::PoseSample s;
    s.x = d.x;
    s.y = d.y;
    s.yaw = d.yaw;
    if (apply_offset) s = rd::applyOffset(s, offset_);

    geometry_msgs::PoseStamped in;
    in.header = header;
    in.pose = makePose(s.x, s.y, s.yaw);

    if (!tf_buffer_ || target_frame_ == header.frame_id) {
      out = in;
      return true;
    }
    try {
      tf_buffer_->transform(in, out, target_frame_, ros::Duration(tf_timeout_));
      return true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "Không transform được %s -> %s: %s", header.frame_id.c_str(),
                        target_frame_.c_str(), ex.what());
      return false;
    }
  }

  void publishPoses(const std_msgs::Header& scan_header, const std::vector<rd::Detection>& dets)
  {
    geometry_msgs::PoseArray out;
    out.header = scan_header;
    if (tf_buffer_) out.header.frame_id = target_frame_;

    for (const auto& d : dets) {
      geometry_msgs::PoseStamped ps;
      if (!toOutputFrame(scan_header, d, ps)) return;
      out.poses.push_back(ps.pose);  // offset đã được áp trong toOutputFrame
    }
    pose_pub_.publish(out);  // publish cả khi rỗng để consumer biết là không thấy reflector
  }

  void publishFilteredCloud(const std_msgs::Header& header, const std::vector<rd::Point2>& pts)
  {
    if (cloud_pub_.getNumSubscribers() == 0) return;

    sensor_msgs::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(pts.size());
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                             sensor_msgs::PointField::FLOAT32, "z", 1,
                             sensor_msgs::PointField::FLOAT32, "intensity", 1,
                             sensor_msgs::PointField::FLOAT32);
    mod.resize(pts.size());

    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> ii(cloud, "intensity");
    for (const auto& p : pts) {
      *ix = static_cast<float>(p.x);
      *iy = static_cast<float>(p.y);
      *iz = 0.0f;
      *ii = p.intensity;
      ++ix;
      ++iy;
      ++iz;
      ++ii;
    }
    cloud_pub_.publish(cloud);
  }

  void publishMarkers(const std_msgs::Header& header, const std::vector<rd::Detection>& dets)
  {
    if (marker_pub_.getNumSubscribers() == 0) return;

    visualization_msgs::MarkerArray arr;

    visualization_msgs::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(clear);

    int id = 0;
    for (const auto& d : dets) {
      // hộp chữ nhật (hướng theo cạnh dài)
      visualization_msgs::Marker box;
      box.header = header;
      box.ns = "reflector_box";
      box.id = id;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.pose = makePose(d.x, d.y, d.rect_yaw);
      box.scale.x = std::max(d.length, 0.01);
      box.scale.y = std::max(d.width, 0.01);
      box.scale.z = 0.05;
      box.color.r = 0.1f;
      box.color.g = 0.9f;
      box.color.b = 0.2f;
      box.color.a = 0.6f;
      arr.markers.push_back(box);

      // mũi tên yaw xuất ra
      visualization_msgs::Marker arrow;
      arrow.header = header;
      arrow.ns = "reflector_yaw";
      arrow.id = id;
      arrow.type = visualization_msgs::Marker::ARROW;
      arrow.action = visualization_msgs::Marker::ADD;
      arrow.pose = makePose(d.x, d.y, d.yaw);
      arrow.scale.x = 0.30;
      arrow.scale.y = 0.03;
      arrow.scale.z = 0.03;
      arrow.color.r = 1.0f;
      arrow.color.g = 0.2f;
      arrow.color.b = 0.1f;
      arrow.color.a = 1.0f;
      arr.markers.push_back(arrow);

      ++id;
    }
    marker_pub_.publish(arr);
  }

  // ---------------------------------------------------------------- members
  rd::Params params_;
  rd::ReflectorDetector detector_;
  rd::PoseOffset offset_;

  std::string target_frame_;
  double tf_timeout_{0.1};
  bool publish_debug_{true};

  // service mode
  bool service_mode_{false};
  std::string service_name_{"detect_dock"};
  double detect_duration_{1.5};
  int min_samples_{5};
  double max_std_xy_{0.05};
  std::atomic<bool> collecting_{false};
  std::mutex service_mutex_;   // chỉ cho 1 request chạy tại 1 thời điểm
  std::mutex samples_mutex_;   // bảo vệ samples_ giữa callback scan và handler
  std::vector<rd::PoseSample> samples_;
  std::string sample_frame_;
  int scans_total_{0};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Subscriber scan_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher cloud_pub_;
  ros::Publisher marker_pub_;
  ros::ServiceServer detect_srv_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reflector_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ReflectorDetectorNode node(nh, pnh);

  // service handler chặn trong lúc thu mẫu => cần nhiều thread để callback scan vẫn chạy
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}