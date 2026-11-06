import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import re

# =====================================================
# CONFIG
# =====================================================

BAUDRATE = 57600

WHEELS = ["LU", "LD", "RU", "RD"]

# Pin configuration hiện tại trong config.h
DEFAULT_PWM = [19, 23, 17, 4]
DEFAULT_DIR = [21, 22, 18, 16]

DEFAULT_ENCA = [14, 15, 26, 25]
DEFAULT_ENCB = [13, 5, 27, 33]

DEFAULT_MOTOR_SIGN = [1, 1, 1, 1]
DEFAULT_ENCODER_SIGN = [-1, -1, -1, -1]

TEST_SPEED = 70
TEST_DURATION = 1800

# Encoder delta nhỏ hơn giá trị này được xem là không có feedback
MIN_ENCODER_DELTA = 2

# =====================================================
# GLOBAL DATA
# =====================================================

ser = None
running = True

encoder_values = [0, 0, 0, 0]
encoder_lock = threading.Lock()

current_channel = 0
selected_wheel = None

motor_mapping = [None] * 4
encoder_mapping = [None] * 4

motor_sign_result = [1] * 4
encoder_sign_result = [-1] * 4

encoder_baseline = [0] * 4

motor_running = False
test_finished = False

# =====================================================
# SERIAL
# =====================================================

def connect_serial():

    global ser

    try:

        if ser and ser.is_open:
            ser.close()

        ser = serial.Serial(
            port_var.get(),
            BAUDRATE,
            timeout=0.05
        )

        time.sleep(2)

        status_label.config(
            text="CONNECTED",
            fg="#00ff88"
        )

        log("Arduino connected.")

    except Exception as e:

        messagebox.showerror(
            "Serial Error",
            str(e)
        )


def send_motor(channel, speed):

    if ser is None or not ser.is_open:
        return

    command = f"{channel}/{speed};"

    try:
        ser.write(command.encode())

    except Exception as e:
        log(f"Serial error: {e}")


def stop_all():

    for i in range(4):
        send_motor(i, 0)


def read_encoder():

    global encoder_values

    while running:

        try:

            if ser is None or not ser.is_open:
                time.sleep(0.05)
                continue

            line = ser.readline().decode(
                errors="ignore"
            ).strip()

            match = re.search(
                r"(-?\d+)/(-?\d+)&(-?\d+)\*(-?\d+)",
                line
            )

            if match:

                values = [
                    int(match.group(1)),
                    int(match.group(2)),
                    int(match.group(3)),
                    int(match.group(4))
                ]

                with encoder_lock:
                    encoder_values = values

        except Exception:
            time.sleep(0.05)


threading.Thread(
    target=read_encoder,
    daemon=True
).start()


def get_encoder():

    with encoder_lock:
        return encoder_values.copy()


# =====================================================
# LOG
# =====================================================

def log(message):

    log_box.insert(
        tk.END,
        message + "\n"
    )

    log_box.see(tk.END)


# =====================================================
# ROBOT UI
# =====================================================

root = tk.Tk()

root.title("4WD AUTO CALIBRATION")

root.geometry("1050x850")

root.configure(
    bg="#202630"
)

tk.Label(
    root,
    text="4WD ROBOT AUTO CALIBRATION",
    font=("Arial", 22, "bold"),
    bg="#202630",
    fg="white"
).pack(pady=10)


# =====================================================
# SERIAL PANEL
# =====================================================

serial_frame = tk.Frame(
    root,
    bg="#303b49",
    padx=10,
    pady=10
)

serial_frame.pack(
    fill="x",
    padx=20
)

tk.Label(
    serial_frame,
    text="COM Port:",
    bg="#303b49",
    fg="white"
).pack(side="left")

ports = [
    p.device
    for p in serial.tools.list_ports.comports()
]

port_var = tk.StringVar(
    value=ports[0] if ports else ""
)

ttk.Combobox(
    serial_frame,
    textvariable=port_var,
    values=ports,
    width=15
).pack(side="left", padx=10)

tk.Button(
    serial_frame,
    text="CONNECT",
    command=connect_serial,
    bg="#168a65",
    fg="white"
).pack(side="left")

status_label = tk.Label(
    serial_frame,
    text="DISCONNECTED",
    bg="#303b49",
    fg="red"
)

status_label.pack(side="left", padx=15)


# =====================================================
# MAIN FRAME
# =====================================================

main = tk.Frame(
    root,
    bg="#202630"
)

main.pack(
    fill="both",
    expand=True
)


# =====================================================
# ROBOT CANVAS
# =====================================================

canvas = tk.Canvas(
    main,
    width=500,
    height=430,
    bg="#202630",
    highlightthickness=0
)

canvas.pack(
    side="left",
    padx=15,
    pady=10
)

canvas.create_rectangle(
    145, 50, 355, 380,
    fill="#536477",
    outline="white",
    width=3
)

canvas.create_text(
    250, 75,
    text="FRONT",
    fill="white",
    font=("Arial", 14, "bold")
)

canvas.create_text(
    250, 350,
    text="REAR",
    fill="white",
    font=("Arial", 14, "bold")
)

wheel_positions = {
    "LU": (110, 130),
    "LD": (110, 290),
    "RU": (390, 130),
    "RD": (390, 290)
}

wheel_objects = {}

for wheel, (x, y) in wheel_positions.items():

    rect = canvas.create_rectangle(
        x - 35,
        y - 50,
        x + 35,
        y + 50,
        fill="#222222",
        outline="#00bcd4",
        width=3,
        tags=wheel
    )

    text = canvas.create_text(
        x, y,
        text=wheel,
        fill="white",
        font=("Arial", 16, "bold"),
        tags=wheel
    )

    wheel_objects[wheel] = (rect, text)

    canvas.tag_bind(
        wheel,
        "<Button-1>",
        lambda event, w=wheel: choose_wheel(w)
    )


def choose_wheel(wheel):

    global selected_wheel

    selected_wheel = wheel

    for w in WHEELS:

        rect, text = wheel_objects[w]

        if w == wheel:
            canvas.itemconfig(
                rect,
                fill="#168a65"
            )
        else:
            canvas.itemconfig(
                rect,
                fill="#222222"
            )

    selected_label.config(
        text=f"Physical wheel selected: {wheel}"
    )

    log(f"Selected physical wheel: {wheel}")


# =====================================================
# CONTROL PANEL
# =====================================================

panel = tk.Frame(
    main,
    bg="#303b49",
    padx=15,
    pady=15
)

panel.pack(
    side="left",
    fill="both",
    expand=True,
    padx=10,
    pady=10
)

tk.Label(
    panel,
    text="AUTO CALIBRATION",
    bg="#303b49",
    fg="white",
    font=("Arial", 17, "bold")
).pack(pady=10)


channel_label = tk.Label(
    panel,
    text="Motor Channel: 0 (M_L_UP)",
    bg="#303b49",
    fg="#00ffcc",
    font=("Arial", 15, "bold")
)

channel_label.pack(pady=10)


MOTOR_NAMES = [
    "M_L_UP",
    "M_L_DOWN",
    "M_R_UP",
    "M_R_DOWN"
]


def update_channel():

    channel_label.config(
        text=f"Motor Channel: {current_channel} "
             f"({MOTOR_NAMES[current_channel]})"
    )


speed_var = tk.IntVar(value=TEST_SPEED)

tk.Label(
    panel,
    text="Test speed",
    bg="#303b49",
    fg="white"
).pack()

tk.Scale(
    panel,
    from_=30,
    to=150,
    orient="horizontal",
    variable=speed_var,
    bg="#303b49",
    fg="white",
    highlightthickness=0
).pack(fill="x")


selected_label = tk.Label(
    panel,
    text="Physical wheel selected: None",
    bg="#303b49",
    fg="#00ffcc"
)

selected_label.pack(pady=10)


invert_motor_var = tk.BooleanVar(value=False)

tk.Checkbutton(
    panel,
    text="Invert motor direction",
    variable=invert_motor_var,
    bg="#303b49",
    fg="white",
    selectcolor="#202630"
).pack(pady=5)


# =====================================================
# MOTOR TEST
# =====================================================

def start_test():

    global motor_running
    global test_finished
    global encoder_baseline
    global selected_wheel

    if ser is None or not ser.is_open:

        messagebox.showwarning(
            "Warning",
            "Connect Arduino first!"
        )

        return

    if motor_running:
        return

    selected_wheel = None

    selected_label.config(
        text="Click the physical wheel that rotates!"
    )

    encoder_baseline = get_encoder()

    test_finished = False
    motor_running = True

    speed = speed_var.get()

    log(
        f"Testing motor channel {current_channel}"
    )

    send_motor(
        current_channel,
        speed
    )

    run_button.config(
        state="disabled"
    )

    root.after(
        TEST_DURATION,
        finish_test
    )


def finish_test():

    global motor_running
    global test_finished

    send_motor(
        current_channel,
        0
    )

    motor_running = False
    test_finished = True

    run_button.config(
        state="normal"
    )

    log("Motor stopped. Select physical wheel if not selected.")

    analyze_button.config(
        state="normal"
    )


def emergency_stop():

    global motor_running

    stop_all()

    motor_running = False

    run_button.config(
        state="normal"
    )

    log("EMERGENCY STOP")


run_button = tk.Button(
    panel,
    text="1. RUN CURRENT MOTOR",
    command=start_test,
    bg="#168a65",
    fg="white",
    height=2
)

run_button.pack(
    fill="x",
    pady=8
)

tk.Button(
    panel,
    text="EMERGENCY STOP",
    command=emergency_stop,
    bg="#bd3030",
    fg="white",
    height=2
).pack(
    fill="x",
    pady=5
)


# =====================================================
# AUTO ENCODER DETECTION
# =====================================================

def analyze_test():

    global current_channel
    global motor_mapping
    global encoder_mapping
    global motor_sign_result
    global encoder_sign_result

    if not test_finished:

        messagebox.showwarning(
            "Warning",
            "Wait until motor stops!"
        )

        return

    if selected_wheel is None:

        messagebox.showwarning(
            "Warning",
            "Click the physical wheel that rotated!"
        )

        return

    physical = selected_wheel

    # Check duplicate motor mapping
    if physical in motor_mapping:

        messagebox.showerror(
            "Mapping Error",
            "This physical wheel was already assigned!"
        )

        return

    current_encoder = get_encoder()

    delta = [
        current_encoder[i] - encoder_baseline[i]
        for i in range(4)
    ]

    log(f"Encoder delta: {delta}")

    # Find encoder with largest absolute change
    encoder_channel = max(
        range(4),
        key=lambda i: abs(delta[i])
    )

    max_delta = abs(delta[encoder_channel])

    if max_delta < MIN_ENCODER_DELTA:

        messagebox.showerror(
            "Encoder Error",
            "No encoder feedback detected!\n"
            "Check encoder wiring or Arduino publishing."
        )

        return

    if encoder_mapping[encoder_channel] is not None:

        messagebox.showerror(
            "Encoder Error",
            "This encoder channel was already assigned!"
        )

        return

    # Record motor mapping
    motor_mapping[current_channel] = physical

    # Record encoder mapping
    encoder_mapping[encoder_channel] = physical

    # Motor direction correction
    motor_sign_result[current_channel] = (
        -1 if invert_motor_var.get() else 1
    )

    # Encoder direction correction
    #
    # Positive motor command should produce
    # positive encoder count after calibration.
    #
    # Include motor inversion because reversing
    # the motor also reverses encoder movement.

    sign = 1 if delta[encoder_channel] > 0 else -1

    encoder_sign_result[encoder_channel] = (
        DEFAULT_ENCODER_SIGN[encoder_channel]
        * sign
        * motor_sign_result[current_channel]
    )

    log("--------------------------------")
    log(f"Motor channel: {current_channel}")
    log(f"Physical wheel: {physical}")
    log(f"Encoder channel: {encoder_channel}")
    log(f"Encoder delta: {delta[encoder_channel]}")
    log(f"Motor sign: {motor_sign_result[current_channel]}")
    log(f"Encoder sign: {encoder_sign_result[encoder_channel]}")
    log("--------------------------------")

    current_channel += 1

    analyze_button.config(
        state="disabled"
    )

    if current_channel >= 4:

        complete_calibration()

    else:

        update_channel()

        invert_motor_var.set(False)

        log(
            f"Ready for motor channel {current_channel}"
        )


analyze_button = tk.Button(
    panel,
    text="2. CONFIRM WHEEL + DETECT ENCODER",
    command=analyze_test,
    bg="#007bca",
    fg="white",
    height=2,
    state="disabled"
)

analyze_button.pack(
    fill="x",
    pady=8
)


# =====================================================
# GENERATE CONFIGURATION
# =====================================================

def complete_calibration():

    log("ALL CALIBRATION COMPLETED!")

    messagebox.showinfo(
        "Calibration",
        "All four motors calibrated successfully!"
    )

    generate_config()


def generate_config():

    if None in motor_mapping or None in encoder_mapping:

        messagebox.showerror(
            "Error",
            "Calibration is incomplete!"
        )

        return

    # Output arrays indexed by physical wheel:
    # LU, LD, RU, RD

    new_pwm = [0] * 4
    new_dir = [0] * 4

    new_enca = [0] * 4
    new_encb = [0] * 4

    new_motor_sign = [0] * 4
    new_encoder_sign = [0] * 4

    # Motor mapping
    for channel in range(4):

        physical_index = WHEELS.index(
            motor_mapping[channel]
        )

        new_pwm[physical_index] = DEFAULT_PWM[channel]
        new_dir[physical_index] = DEFAULT_DIR[channel]

        new_motor_sign[physical_index] = (
            motor_sign_result[channel]
        )

    # Encoder mapping
    for channel in range(4):

        physical_index = WHEELS.index(
            encoder_mapping[channel]
        )

        new_enca[physical_index] = DEFAULT_ENCA[channel]
        new_encb[physical_index] = DEFAULT_ENCB[channel]

        new_encoder_sign[physical_index] = (
            encoder_sign_result[channel]
        )

    output = f"""
// =======================================
// AUTO GENERATED CONFIGURATION
// Index: LU, LD, RU, RD
// =======================================

#define NMOTORS 4

#define M_L_UP   0
#define M_L_DOWN 1
#define M_R_UP   2
#define M_R_DOWN 3

// MOTOR PINS

const int pwm[NMOTORS] = {{
    {", ".join(map(str, new_pwm))}
}};

const int dir[NMOTORS] = {{
    {", ".join(map(str, new_dir))}
}};

// ENCODER PINS

const int enca[NMOTORS] = {{
    {", ".join(map(str, new_enca))}
}};

const int encb[NMOTORS] = {{
    {", ".join(map(str, new_encb))}
}};

// MOTOR DIRECTION

const int dir_M_L_UP   = {new_motor_sign[0]};
const int dir_M_L_DOWN = {new_motor_sign[1]};
const int dir_M_R_UP   = {new_motor_sign[2]};
const int dir_M_R_DOWN = {new_motor_sign[3]};

const int motor_sign[NMOTORS] = {{
    dir_M_L_UP,
    dir_M_L_DOWN,
    dir_M_R_UP,
    dir_M_R_DOWN
}};

// ENCODER DIRECTION

int dir_encod[NMOTORS] = {{
    {", ".join(map(str, new_encoder_sign))}
}};

// =======================================
"""

    log("========== GENERATED CONFIGURATION ==========")

    log(output)

    log("=============================================")

    log("Arduino configuration generated.")


tk.Button(
    panel,
    text="3. GENERATE ARDUINO CONFIG",
    command=generate_config,
    bg="#8e44ad",
    fg="white",
    height=2
).pack(
    fill="x",
    pady=15
)


# =====================================================
# ENCODER LIVE MONITOR
# =====================================================

monitor = tk.LabelFrame(
    panel,
    text="LIVE ENCODER FEEDBACK",
    bg="#303b49",
    fg="white"
)

monitor.pack(
    fill="x",
    pady=10
)

encoder_labels = []

for i in range(4):

    label = tk.Label(
        monitor,
        text=f"ENC {i}: 0",
        bg="#303b49",
        fg="#00ffcc",
        font=("Consolas", 12)
    )

    label.pack(anchor="w", padx=10)

    encoder_labels.append(label)


def update_encoder_ui():

    values = get_encoder()

    for i in range(4):

        encoder_labels[i].config(
            text=f"ENC {i}: {values[i]}"
        )

    root.after(
        100,
        update_encoder_ui
    )


# =====================================================
# LOG PANEL
# =====================================================

log_frame = tk.LabelFrame(
    root,
    text="CALIBRATION LOG",
    bg="#202630",
    fg="white"
)

log_frame.pack(
    fill="both",
    padx=20,
    pady=5
)

log_box = tk.Text(
    log_frame,
    height=8,
    bg="#151b22",
    fg="#00ff99",
    font=("Consolas", 10)
)

log_box.pack(
    fill="both",
    expand=True
)


# =====================================================
# CLOSE
# =====================================================

def on_close():

    global running

    running = False

    stop_all()

    if ser and ser.is_open:
        ser.close()

    root.destroy()


root.protocol(
    "WM_DELETE_WINDOW",
    on_close
)

update_encoder_ui()
update_channel()

root.mainloop()
