import serial
import serial.tools.list_ports
import threading
import time

# =====================================================
# CONFIGURATION
# =====================================================

PORT = "/dev/pts/15"       # Cổng nối với COM của app
BAUDRATE = 57600

PUBLISH_INTERVAL = 0.05  # 50 ms

# Motor channel -> bánh vật lý thực tế
# Thay đổi thứ tự để giả lập đấu dây sai
MOTOR_CHANNEL_TO_WHEEL = [
    "RD",   # Motor 0 thực tế điều khiển RD
    "LU",   # Motor 1 thực tế điều khiển LU
    "LD",   # Motor 2 thực tế điều khiển LD
    "RU"    # Motor 3 thực tế điều khiển RU
]

# Encoder channel -> bánh vật lý thực tế
# Giả lập encoder bị đấu sai thứ tự
ENCODER_CHANNEL_TO_WHEEL = [
    "LD",   # Encoder 0 thuộc LD
    "RU",   # Encoder 1 thuộc RU
    "RD",   # Encoder 2 thuộc RD
    "LU"    # Encoder 3 thuộc LU
]

# Chiều đếm encoder giả lập
# 1: tăng khi motor chạy chiều dương
# -1: giảm khi motor chạy chiều dương
ENCODER_POLARITY = [1, -1, 1, -1]

# Số count mỗi giây ở tốc độ 255
MAX_COUNTS_PER_SECOND = 1500

# =====================================================
# GLOBAL
# =====================================================

motor_speed = [0, 0, 0, 0]

encoder_count = [0, 0, 0, 0]

lock = threading.Lock()

running = True


# =====================================================
# MOTOR CONTROL
# =====================================================

def control_motor(channel, speed):

    if channel < 0 or channel >= 4:
        return

    speed = max(-255, min(255, speed))

    with lock:
        motor_speed[channel] = speed

    print(
        f"[MOTOR] Channel {channel} "
        f"({MOTOR_CHANNEL_TO_WHEEL[channel]}) "
        f"Speed = {speed}"
    )


# =====================================================
# ENCODER SIMULATION
# =====================================================

def update_encoder(dt):

    with lock:

        for motor_channel in range(4):

            speed = motor_speed[motor_channel]

            if speed == 0:
                continue

            # Physical wheel driven by this motor
            physical_wheel = MOTOR_CHANNEL_TO_WHEEL[
                motor_channel
            ]

            # Find encoder channel belonging to this wheel
            encoder_channel = ENCODER_CHANNEL_TO_WHEEL.index(
                physical_wheel
            )

            polarity = ENCODER_POLARITY[
                encoder_channel
            ]

            delta = (
                speed / 255.0
                * MAX_COUNTS_PER_SECOND
                * dt
                * polarity
            )

            encoder_count[encoder_channel] += int(delta)


# =====================================================
# RECEIVE ARDUINO COMMANDS
# =====================================================

def parse_command(command):

    try:

        if "/" not in command:
            return

        motor, speed = command.split("/")

        motor = int(motor)
        speed = int(speed)

        control_motor(motor, speed)

    except Exception as e:

        print("[COMMAND ERROR]", e)


# =====================================================
# PUBLISH ENCODER
# =====================================================

def publish_encoder(ser):

    data = (
        f"{encoder_count[0]}/"
        f"{encoder_count[1]}&"
        f"{encoder_count[2]}*"
        f"{encoder_count[3]};"
    )

    ser.write((data + "\n").encode())


# =====================================================
# MAIN
# =====================================================

def main():

    global running

    print("====================================")
    print("       FAKE ARDUINO ESP32")
    print("====================================")

    print("Port:", PORT)
    print("Baudrate:", BAUDRATE)

    print("Motor mapping:", MOTOR_CHANNEL_TO_WHEEL)
    print("Encoder mapping:", ENCODER_CHANNEL_TO_WHEEL)

    print("Waiting for Python application...")

    try:

        ser = serial.Serial(
            PORT,
            BAUDRATE,
            timeout=0.01
        )

        print("[CONNECTED] Virtual Serial Ready!")

    except Exception as e:

        print("[SERIAL ERROR]", e)
        return

    last_publish = time.monotonic()

    last_update = time.monotonic()

    buffer = ""

    try:

        while running:

            now = time.monotonic()

            dt = now - last_update
            last_update = now

            # Update simulated encoder
            update_encoder(dt)

            # Read commands from Python UI
            if ser.in_waiting:

                received = ser.read(
                    ser.in_waiting
                ).decode(
                    errors="ignore"
                )

                buffer += received

                while ";" in buffer:

                    command, buffer = buffer.split(
                        ";", 1
                    )

                    parse_command(command.strip())

            # Publish encoder every 50ms
            if now - last_publish >= PUBLISH_INTERVAL:

                with lock:
                    publish_encoder(ser)

                last_publish = now

            time.sleep(0.005)

    except KeyboardInterrupt:

        print("\n[STOP] Fake Arduino stopped")

    finally:

        running = False

        with lock:
            for i in range(4):
                motor_speed[i] = 0

        ser.close()


if __name__ == "__main__":
    main()
