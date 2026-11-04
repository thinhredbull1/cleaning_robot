# reflector_detector

Node ROS1 (C++) detect reflector hình chữ nhật từ `sensor_msgs/LaserScan` 2D (có `intensities`).

## Pipeline

1. **Filter intensity** – giữ điểm có `intensity` trong `[intensity_min, intensity_max]`, đồng thời lọc range và vùng góc (ROI).
2. **Cluster** – gom cụm theo thứ tự tia (khoảng cách giữa 2 điểm liên tiếp ≤ `tolerance`, cho phép trống ≤ `max_index_gap` tia). O(N), tự ghép cụm bị cắt ở biên ±180° nếu lidar quay 360°.
3. **Filter hình dạng** – hộp bao chữ nhật của cụm phải thoả `length` (cạnh dài) và `width` (cạnh ngắn) trong khoảng cho phép, và số điểm trong `[min_points, max_points]`.
4. **Tâm + yaw** – tâm = tâm hộp bao (x, y); yaw từ hướng cạnh dài.
   - `min_area_rect` (mặc định): quét góc tìm hộp bao diện tích nhỏ nhất, tie-break bằng tiêu chí closeness → đúng cả khi chỉ thấy 1 cạnh (đoạn thẳng) lẫn khi thấy 2 cạnh (chữ L).
   - `pca`: trục chính PCA (nhanh hơn, nhưng lệch khi thấy hình chữ L).

## Topic

| | Topic | Kiểu |
|---|---|---|
| Sub | `scan` (remap bằng `scan_topic`) | `sensor_msgs/LaserScan` |
| Pub | `~reflector_poses` | `geometry_msgs/PoseArray` – x, y, yaw (gần → xa), publish cả khi rỗng |
| Pub | `~filtered_points` | `sensor_msgs/PointCloud2` – điểm sau lọc intensity (debug) |
| Pub | `~markers` | `visualization_msgs/MarkerArray` – hộp xanh + mũi tên yaw đỏ (frame của scan) |

`yaw_mode`:
- `major_axis`: yaw = hướng cạnh dài, trong (-90°, 90°] (trục không có chiều).
- `facing_sensor`: yaw = pháp tuyến chỉ về phía sensor (tiện cho docking).

`target_frame` khác rỗng → `PoseArray` được transform sang frame đó bằng tf2.

## Build & chạy

```bash
cd ~/catkin_ws/src && cp -r /path/to/reflector_detector .
cd ~/catkin_ws && catkin_make        # hoặc catkin build
source devel/setup.bash

roslaunch reflector_detector reflector_detector.launch scan_topic:=/scan
roslaunch reflector_detector reflector_detector.launch scan_topic:=/scan target_frame:=base_link
```

Chỉnh tham số trong `config/reflector_detector.yaml`. Việc đầu tiên cần làm: đặt `filter/intensity_min` phù hợp lidar của bạn (xem `rostopic echo /scan/intensities`), sau đó chỉnh `shape/*` theo kích thước reflector thật.

Debug: `rosrun rqt_logger_level rqt_logger_level` → bật DEBUG cho node để xem số điểm/cụm/detection mỗi frame.

## Test

```bash
catkin_make run_tests_reflector_detector
```

Test giả lập lidar bằng ray-casting: kiểm tra sai số tâm/yaw, loại vật thể nhỏ/quá dài/intensity thấp, sắp xếp gần → xa, ghép cụm qua biên ±180°.

## Lưu ý

- Chỉ thấy được phần reflector hướng về sensor. Nếu nhìn trực diện vào đầu mút (cạnh dài song song tia nhìn) thì `length` đo được chỉ bằng bề dày → sẽ bị `length_min` loại.
- Khi chỉ thấy 1 cạnh, tâm là tâm của cạnh nhìn thấy (không phải tâm khối nếu reflector có độ dày).
