# localize_manager

ROS1 Noetic package dùng để quản lý các chế độ **mapping**, **save map** và **localization** của robot.

Package đảm bảo tại một thời điểm chỉ có một localization source hoạt động:

* `gmapping` khi robot ở chế độ `MAPPING`
* `map_server + AMCL/GMCL` khi robot ở chế độ `LOCALIZING`
* `map_saver` được gọi tạm thời khi lưu map

## 1. Architecture

```text
                         localize_manager
                                |
              +-----------------+-----------------+
              |                 |                 |
              v                 v                 v
        start_mapping       save_map      start_localization
              |                                   |
              v                                   v
          gmapping                         map_server + AMCL
              |                                   |
              +------------------+----------------+
                                 |
                               /map
```

State machine:

```text
                 +---------+
                 |  IDLE   |
                 +----+----+
                      |
              start_mapping
                      |
                      v
                +-----------+
                |  MAPPING  |
                |  gmapping |
                +-----+-----+
                      |
                   save_map
                      |
                      v
                    stop
                      |
                      v
                 +---------+
                 |  IDLE   |
                 +----+----+
                      |
             start_localization
                      |
                      v
               +--------------+
               | LOCALIZING   |
               | map_server    |
               | AMCL / GMCL   |
               +--------------+
```

Các state hiện tại:

```text
IDLE
MAPPING
SAVING_MAP
LOCALIZING
ERROR
```

> Lưu ý: `LOCALIZING` chỉ có nghĩa process localization đã được khởi động. Nó chưa đảm bảo AMCL/GMCL đã hội tụ vị trí. Việc kiểm tra `LOCALIZED` sẽ được bổ sung sau.

---

## 2. Package structure

```text
localize_manager/
├── CMakeLists.txt
├── package.xml
│
├── config/
│   └── localize_manager.yaml
│
├── launch/
│   ├── localize_manager.launch
│   ├── mapping.launch
│   └── localization.launch
│
├── maps/
│   └── <map_name>/
│       ├── <map_name>.yaml
│       └── <map_name>.pgm
│
├── scripts/
│   └── localize_manager_node.py
│
└── srv/
    ├── StartMapping.srv
    ├── SaveMap.srv
    ├── StartLocalization.srv
    └── Stop.srv
```

---

# 3. Requirements

Package yêu cầu:

* Ubuntu
* ROS1 Noetic
* `rospy`
* `geometry_msgs`
* `std_msgs`
* `std_srvs`
* `map_server`
* `gmapping`
* AMCL hoặc GMCL

Kiểm tra ROS:

```bash
echo $ROS_DISTRO
```

Kết quả mong muốn:

```text
noetic
```

---

# 4. Build package

Giả sử workspace nằm tại:

```text
~/catkin_ws
```

Copy package vào:

```text
~/catkin_ws/src/localize_manager
```

Sau đó:

```bash
cd ~/catkin_ws
catkin_make
```

Source workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

Kiểm tra service đã được generate:

```bash
rossrv list | grep localize_manager
```

Kết quả:

```text
localize_manager/SaveMap
localize_manager/StartLocalization
localize_manager/StartMapping
localize_manager/Stop
```

---

# 5. Configuration

File:

```text
config/localize_manager.yaml
```

Ví dụ:

```yaml
map_directory: maps

state_publish_rate: 2.0
```

## `map_directory`

Thư mục chứa các map.

Nếu:

```yaml
map_directory: maps
```

thì package sẽ lưu map tại:

```text
localize_manager/maps/
```

Ví dụ:

```text
maps/
└── warehouse_01/
    ├── warehouse_01.yaml
    └── warehouse_01.pgm
```

---

# 6. Mapping launch

File:

```text
launch/mapping.launch
```

Mặc định:

```xml
<launch>

    <node
        pkg="gmapping"
        type="slam_gmapping"
        name="slam_gmapping"
        output="screen"
    />

</launch>
```

Nếu robot đã có một launch file gmapping riêng, nên include launch file đó thay vì cấu hình toàn bộ parameter trong `localize_manager`.

Ví dụ:

```xml
<launch>

    <include
        file="$(find my_robot_navigation)/launch/gmapping.launch"
    />

</launch>
```

Điều này giúp `localize_manager` không phụ thuộc vào các parameter cụ thể của robot.

---

# 7. Localization launch

File:

```text
launch/localization.launch
```

Ví dụ với AMCL:

```xml
<launch>

    <arg name="map_file"/>

    <node
        pkg="map_server"
        type="map_server"
        name="map_server"
        args="$(arg map_file)"
        output="screen"
    />

    <node
        pkg="amcl"
        type="amcl"
        name="amcl"
        output="screen"
    />

</launch>
```

Nếu robot sử dụng `gmcl` thay vì AMCL, thay node AMCL bằng node GMCL tương ứng.

Ví dụ:

```xml
<node
    pkg="gmcl"
    type="gmcl"
    name="gmcl"
    output="screen"
/>
```

Các parameter của AMCL/GMCL nên được cấu hình trong launch/config riêng.

---

# 8. Start manager

Đảm bảo `roscore` đang chạy:

```bash
roscore
```

Terminal khác:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch localize_manager localize_manager.launch
```

Nếu khởi động thành công sẽ thấy:

```text
[localize_manager] Started
[localize_manager] Map directory: ...
```

---

# 9. Kiểm tra state

Manager publish state trên:

```text
/localize_manager/state
```

Kiểm tra:

```bash
rostopic echo /localize_manager/state
```

Ban đầu:

```text
data: "IDLE"
```

---

# 10. Start Mapping

Service:

```text
/localize_manager/start_mapping
```

Gọi:

```bash
rosservice call /localize_manager/start_mapping
```

Response:

```text
success: True
message: "Mapping started"
```

State chuyển thành:

```text
MAPPING
```

Kiểm tra:

```bash
rosnode list
```

Phải có:

```text
/slam_gmapping
```

Kiểm tra map:

```bash
rostopic list | grep map
```

---

# 11. Save Map

Khi robot đang ở state:

```text
MAPPING
```

gọi:

```bash
rosservice call /localize_manager/save_map \
"{map_name: 'warehouse_01'}"
```

Manager sẽ thực hiện tương đương:

```bash
rosrun map_server map_saver \
    -f <map_directory>/warehouse_01/warehouse_01
```

Map sẽ được lưu:

```text
maps/
└── warehouse_01/
    ├── warehouse_01.yaml
    └── warehouse_01.pgm
```

Nếu thành công:

```text
success: True
message: "Map saved successfully"
```

Sau khi save thành công, `gmapping` sẽ được stop và state trở về:

```text
IDLE
```

---

# 12. Start Localization

Giả sử đã có:

```text
maps/
└── warehouse_01/
    ├── warehouse_01.yaml
    └── warehouse_01.pgm
```

Gọi:

```bash
rosservice call /localize_manager/start_localization \
"{map_name: 'warehouse_01'}"
```

Manager sẽ:

1. kiểm tra map tồn tại
2. stop `gmapping` nếu đang chạy
3. start `map_server`
4. start AMCL/GMCL
5. chuyển state sang `LOCALIZING`

Kiểm tra:

```bash
rosnode list
```

Mong muốn:

```text
/map_server
/amcl
```

hoặc:

```text
/map_server
/gmcl
```

Không được đồng thời có:

```text
/slam_gmapping
```

---

# 13. Stop

Dừng tất cả process do manager quản lý:

```bash
rosservice call /localize_manager/stop
```

Response:

```text
success: True
message: "Localization manager stopped"
```

State:

```text
IDLE
```

---

# 14. Service API

## `/localize_manager/start_mapping`

Type:

```text
localize_manager/StartMapping
```

Request:

```text
---
```

Response:

```text
bool success
string message
```

Ví dụ:

```bash
rosservice call /localize_manager/start_mapping
```

---

## `/localize_manager/save_map`

Type:

```text
localize_manager/SaveMap
```

Request:

```text
string map_name
```

Response:

```text
bool success
string message
```

Ví dụ:

```bash
rosservice call /localize_manager/save_map \
"{map_name: 'warehouse_01'}"
```

---

## `/localize_manager/start_localization`

Type:

```text
localize_manager/StartLocalization
```

Request:

```text
string map_name
```

Response:

```text
bool success
string message
```

Ví dụ:

```bash
rosservice call /localize_manager/start_localization \
"{map_name: 'warehouse_01'}"
```

---

## `/localize_manager/stop`

Type:

```text
localize_manager/Stop
```

Request:

```text
---
```

Response:

```text
bool success
string message
```

Ví dụ:

```bash
rosservice call /localize_manager/stop
```

---

# 15. State topic

Topic:

```text
/localize_manager/state
```

Type:

```text
std_msgs/String
```

Các state:

```text
IDLE
MAPPING
SAVING_MAP
LOCALIZING
ERROR
```

Theo dõi:

```bash
rostopic echo /localize_manager/state
```

---

# 16. Complete workflow

## Tạo map mới

Khởi động:

```bash
roscore
```

Sau đó:

```bash
roslaunch localize_manager localize_manager.launch
```

Start mapping:

```bash
rosservice call /localize_manager/start_mapping
```

Robot chạy và mapping.

Khi hoàn thành:

```bash
rosservice call /localize_manager/save_map \
"{map_name: 'warehouse_01'}"
```

Kết quả:

```text
maps/
└── warehouse_01/
    ├── warehouse_01.yaml
    └── warehouse_01.pgm
```

---

## Localization với map đã lưu

Khởi động manager:

```bash
roslaunch localize_manager localize_manager.launch
```

Load map:

```bash
rosservice call /localize_manager/start_localization \
"{map_name: 'warehouse_01'}"
```

Sau đó AMCL/GMCL sẽ sử dụng:

```text
warehouse_01.yaml
```

để localization.

---

# 17. Important safety rules

### Không chạy gmapping và AMCL/GMCL cùng lúc

Không được:

```text
gmapping
   +
AMCL
```

đồng thời publish:

```text
map -> odom
```

Điều này có thể gây TF conflict.

Manager đảm bảo:

```text
MAPPING:
    gmapping = ON
    map_server = OFF
    AMCL/GMCL = OFF
```

và:

```text
LOCALIZING:
    gmapping = OFF
    map_server = ON
    AMCL/GMCL = ON
```

### Không lưu map khi không ở MAPPING

Service `/save_map` sẽ reject nếu state hiện tại không phải:

```text
MAPPING
```

### Map name không được chứa path

Hợp lệ:

```text
warehouse_01
factory_a
office_floor_2
```

Không hợp lệ:

```text
../warehouse
/home/robot/map
foo/bar
```

Điều này tránh việc map name được sử dụng để truy cập các đường dẫn ngoài thư mục map.

---

# 18. Debug

## Kiểm tra node

```bash
rosnode list
```

## Kiểm tra service

```bash
rosservice list | grep localize_manager
```

## Kiểm tra state

```bash
rostopic echo /localize_manager/state
```

## Kiểm tra map

```bash
rostopic echo /map
```

## Kiểm tra TF

```bash
rosrun tf tf_echo map odom
```

Khi mapping:

```text
map -> odom
```

được publish bởi `gmapping`.

Khi localization:

```text
map -> odom
```

được publish bởi AMCL/GMCL.

## Kiểm tra process

```bash
ps aux | grep gmapping
ps aux | grep amcl
ps aux | grep map_server
```

---

# 19. Current limitations

Phiên bản hiện tại tập trung vào lifecycle cơ bản:

```text
Mapping
   ↓
Save Map
   ↓
Localization
```

Chưa bao gồm:

* kiểm tra AMCL/GMCL đã hội tụ
* state `LOCALIZED`
* list map cho App
* delete map
* rename map
* map metadata
* quản lý navigation state trước khi đổi localization mode
* automatic recovery khi localization process crash
* robot pose initialization từ App

Các chức năng này có thể được bổ sung ở các version tiếp theo.

---

# 20. Recommended next step

Sau khi `localize_manager` hoạt động ổn định, nên tích hợp nó với một `robot_manager` cấp cao hơn.

Kiến trúc:

```text
                    APP
                     |
                     v
              +--------------+
              | robot_manager|
              +------+-------+
                     |
          +----------+----------+
          |                     |
          v                     v
  localize_manager       navigation_manager
          |                     |
          |                     v
          |                /cmd_vel/navi
          |                     |
          +---------------------+
                                |
                                v
                         cmd_vel_mux
                                |
                                v
                             /cmd_vel
```

Khi đó App không cần trực tiếp điều khiển `gmapping`, `AMCL`, `TEB` hay các ROS node bên dưới. App chỉ gửi task cấp cao như:

```text
START_MAPPING
SAVE_MAP
LOCALIZE
NAVIGATE
TRACK
CHARGE
STOP
```

`robot_manager` sẽ chịu trách nhiệm kiểm soát state và đảm bảo không xảy ra các chuyển mode không hợp lệ.
