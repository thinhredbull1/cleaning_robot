#!/bin/bash

# Lấy IP của máy hiện tại (ROS_IP)
ROS_IP=$(hostname -I | awk '{print $1}')
if [ -z "$ROS_IP" ]; then
    echo "Cannot get ros_ip!"
    exit 1
fi

# Kiểm tra tham số đầu vào: IP của ROS master
if [ -z "$1" ]; then
    echo "Please update ROS_MASTER IP WHEN CALLING."
    echo "EXAMPLE:'source ros_master.sh 192.168.0.1'"
    ROS_MASTER_IP=$ROS_IP
    echo "UPDATE ROSMASTER WITH CURRENT IP"

else
    ROS_MASTER_IP=$1
fi

# Cổng mặc định của ROS master là 11311
ROS_MASTER_PORT=11311

# Thiết lập ROS_IP và ROS_MASTER_URI
export ROS_IP=$ROS_IP
export ROS_MASTER_URI="http://$ROS_MASTER_IP:$ROS_MASTER_PORT"

# Thông báo kết quả
echo "Success Update ROS_MASTER:"
echo "ROS_IP=$ROS_IP"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
