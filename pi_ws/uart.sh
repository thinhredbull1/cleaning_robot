#!/bin/bash

# Script để tự động cấp quyền cho cổng UART và Arduino trên Raspberry Pi 3
# Cấp quyền 666 cho /dev/ttyS0, /dev/ttyAMA0, /dev/ttyUSB*, /dev/ttyACM* và thêm người dùng vào nhóm dialout

# Kiểm tra quyền root
if [ "$EUID" -ne 0 ]; then
  echo "Vui lòng chạy script này với quyền sudo (sudo bash $0)"
  exit 1
fi

# Biến cấu hình
UART_PORTS=("/dev/ttyS0" "/dev/ttyAMA0") # Cổng UART trên Raspberry Pi
ARDUINO_PORTS=("/dev/ttyUSB*" "/dev/ttyACM*") # Cổng Arduino
USER="skid_robot" # Người dùng mặc định trên Raspberry Pi
GROUP="dialout"
UDEV_RULE="/etc/udev/rules.d/99-uart-arduino-permissions.rules"

# Hàm kiểm tra và thêm người dùng vào nhóm dialout
add_user_to_dialout() {
  if id -nG "$USER" | grep -qw "$GROUP"; then
    echo "Người dùng $USER đã thuộc nhóm $GROUP."
  else
    echo "Thêm người dùng $USER vào nhóm $GROUP..."
    usermod -a -G "$GROUP" "$USER"
    if [ $? -eq 0 ]; then
      echo "Đã thêm $USER vào nhóm $GROUP thành công."
    else
      echo "Lỗi khi thêm $USER vào nhóm $GROUP."
      exit 1
    fi
  fi
}

# Hàm kiểm tra và cấp quyền 666 cho cổng UART
set_uart_permissions() {
  for port in "${UART_PORTS[@]}"; do
    if [ -e "$port" ]; then
      echo "Kiểm tra quyền cho $port..."
      current_perm=$(stat -c %a "$port")
      if [ "$current_perm" != "666" ]; then
        echo "Cấp quyền 666 cho $port..."
        chmod 666 "$port"
        if [ $? -eq 0 ]; then
          echo "Đã cấp quyền 666 cho $port thành công."
        else
          echo "Lỗi khi cấp quyền cho $port."
          exit 1
        fi
      else
        echo "$port đã có quyền 666."
      fi
    else
      echo "Cổng $port không tồn tại."
    fi
  done
}

# Hàm kiểm tra và cấp quyền 666 cho cổng Arduino
set_arduino_permissions() {
  for pattern in "${ARDUINO_PORTS[@]}"; do
    for port in $pattern; do
      if [ -e "$port" ]; then
        echo "Kiểm tra quyền cho $port..."
        current_perm=$(stat -c %a "$port")
        if [ "$current_perm" != "666" ]; then
          echo "Cấp quyền 666 cho $port..."
          chmod 666 "$port"
          if [ $? -eq 0 ]; then
            echo "Đã cấp quyền 666 cho $port thành công."
          else
            echo "Lỗi khi cấp quyền cho $port."
            exit 1
          fi
        else
          echo "$port đã có quyền 666."
        fi
      else
        echo "Cổng $port không tồn tại."
      fi
    done
  done
}

# Hàm tạo rule udev để tự động cấp quyền sau khi khởi động lại
create_udev_rule() {
  if [ -f "$UDEV_RULE" ]; then
    echo "Rule udev đã tồn tại tại $UDEV_RULE."
  else
    echo "Tạo rule udev để tự động cấp quyền cho cổng UART và Arduino..."
    cat > "$UDEV_RULE" << EOL
# Quyền cho cổng UART trên Raspberry Pi
ACTION=="add", KERNEL=="ttyS0", MODE="0666", GROUP="dialout"
ACTION=="add", KERNEL=="ttyAMA0", MODE="0666", GROUP="dialout"
# Quyền cho cổng Arduino (ttyUSB* và ttyACM*)
ACTION=="add", KERNEL=="ttyUSB*", MODE="0666", GROUP="dialout"
ACTION=="add", KERNEL=="ttyACM*", MODE="0666", GROUP="dialout"
EOL
    if [ $? -eq 0 ]; then
      echo "Đã tạo rule udev tại $UDEV_RULE."
      # Tải lại rules udev
      udevadm control --reload-rules && udevadm trigger
      echo "Đã tải lại rules udev."
    else
      echo "Lỗi khi tạo rule udev."
      exit 1
    fi
  fi
}

# Hàm kiểm tra cấu hình UART
check_uart_config() {
  echo "Kiểm tra cấu hình UART..."
  if grep -q "enable_uart=1" /boot/config.txt; then
    echo "UART đã được bật trong /boot/config.txt."
  else
    echo "Bật UART trong /boot/config.txt..."
    echo "enable_uart=1" >> /boot/config.txt
    echo "Đã thêm enable_uart=1 vào /boot/config.txt."
  fi
}

# Thực thi các bước
echo "Bắt đầu cấu hình quyền UART và Arduino trên Raspberry Pi 3..."
# add_user_to_dialout
# check_uart_config
# set_uart_permissions
# set_arduino_permissions
create_udev_rule

echo "Hoàn tất! Vui lòng khởi động lại Raspberry Pi để áp dụng thay đổi."
echo "Sử dụng lệnh: sudo reboot"