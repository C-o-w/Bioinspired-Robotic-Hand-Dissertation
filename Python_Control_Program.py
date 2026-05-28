import tkinter as tk
from tkinter import ttk
import math
import serial
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import serial.tools.list_ports
import time
import threading

#Settings set up for my laptop, adjust as needed, baud rate also needs to be changed on the ESP32
DEFAULT_ESP32_PORT = "COM3"
DEFAULT_ESP32_BAUD = 115200

DEFAULT_CUBEMARS_PORT = "COM5"
DEFAULT_CUBEMARS_BAUD = 921600
5
esp = None
cubemars = None
running = True

#Storage arrays
servo_targets = [90] * 15

finger_groups = {
    "Finger 1": [0, 1, 2],
    "Finger 2": [3, 4, 5],
    "Finger 3": [6, 7, 8],
    "Finger 4": [9, 10, 11],
    "Finger 5": [12, 13, 14],
}

finger_visual_ratios = {
    "Finger 1": 1.0,
    "Finger 2": 1.0,
    "Finger 3": 1.0,
    "Finger 4": 1.0,
    "Finger 5": 1.0,
}

#AK servo settings
ak_position_deg = 0.0
ak_speed_erpm = 10000
ak_accel_erps = 30000

AK_POS_MIN = -270.0
AK_POS_MAX = 360.0

COMM_GET_VALUES = 0x04
COMM_SET_POS = 0x09
COMM_SET_POS_SPD = 0x5B
COMM_SET_POS_ORIGIN = 0x5F

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def get_available_ports():
    return [port.device for port in serial.tools.list_ports.comports()]


def refresh_ports():
    ports = get_available_ports()

    esp_port_box["values"] = ports
    ak_port_box["values"] = ports

    add_terminal_message("[SYSTEM] Refreshed COM ports")


def add_terminal_message(message):
    terminal.configure(state="normal")
    terminal.insert(tk.END, message + "\n")
    terminal.see(tk.END)
    terminal.configure(state="disabled")


def add_ak_terminal_message(message):
    if not ak_terminal_output_enabled.get():
        return

    if ak_output_to_main_terminal.get():
        add_terminal_message(message)

    if ak_output_to_separate_terminal.get():
        ak_terminal.configure(state="normal")
        ak_terminal.insert(tk.END, message + "\n")
        ak_terminal.see(tk.END)
        ak_terminal.configure(state="disabled")


def int32_to_bytes(value):
    value = int(value)

    if value < 0:
        value = (1 << 32) + value

    return bytes([
        (value >> 24) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ])


def crc16_ccitt(data):
    crc = 0x0000
    poly = 0x1021

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def build_cubemars_packet(data_frame):
    length = len(data_frame)
    crc = crc16_ccitt(data_frame)

    packet = bytearray()
    packet.append(0x02)
    packet.append(length)
    packet.extend(data_frame)
    packet.append((crc >> 8) & 0xFF)
    packet.append(crc & 0xFF)
    packet.append(0x03)

    return bytes(packet)

#Connection control
def connect_esp32():
    global esp

    if esp and esp.is_open:
        add_terminal_message("[ESP32] Already connected")
        return

    port = esp_port_var.get().strip()
    baud = int(esp_baud_var.get())

    try:
        esp = serial.Serial(port, baud, timeout=0.1)
        time.sleep(1)

        esp_status_label.config(text=f"ESP32 connected: {port}", foreground="green")
        add_terminal_message(f"[ESP32] Connected to {port} at {baud}")

    except Exception as e:
        esp = None
        esp_status_label.config(text="ESP32 not connected", foreground="red")
        add_terminal_message(f"[ESP32] Connection failed: {e}")


def disconnect_esp32():
    global esp

    try:
        if esp:
            esp.close()

        esp = None
        esp_status_label.config(text="ESP32 disconnected", foreground="red")
        add_terminal_message("[ESP32] Disconnected")

    except Exception as e:
        add_terminal_message(f"[ESP32] Disconnect error: {e}")


def connect_cubemars():
    global cubemars

    if cubemars and cubemars.is_open:
        add_ak_terminal_message("[AK80] Already connected")
        return

    port = ak_port_var.get().strip()
    baud = int(ak_baud_var.get())

    try:
        cubemars = serial.Serial(port, baud, timeout=0.1)
        time.sleep(1)

        ak_connection_status_label.config(
            text=f"AK80 R-Link connected: {port}",
            foreground="green"
        )
        add_ak_terminal_message(f"[AK80] Connected to {port} at {baud}")

    except Exception as e:
        cubemars = None
        ak_connection_status_label.config(
            text="AK80 R-Link not connected",
            foreground="red"
        )
        add_ak_terminal_message(f"[AK80] Connection failed: {e}")


def disconnect_cubemars():
    global cubemars

    try:
        if cubemars:
            cubemars.close()

        cubemars = None
        ak_connection_status_label.config(
            text="AK80 R-Link disconnected",
            foreground="red"
        )
        add_ak_terminal_message("[AK80] Disconnected")

    except Exception as e:
        add_ak_terminal_message(f"[AK80] Disconnect error: {e}")

#Visualisation graphs
def get_finger_joint_angles(finger_name):
    servo_ids = finger_groups[finger_name]
    ratio = finger_visual_ratios[finger_name]

    angles = []

    for servo_num in servo_ids:
        servo_angle = servo_targets[servo_num]

        normalised = (servo_angle - 20) / (160 - 20)
        normalised = clamp(normalised, 0.0, 1.0)

        # Convert servo angle into a visual bend angle.
        # Tune the finger ratio sliders after testing real actuation.
        bend_deg = normalised * 75.0 * ratio
        bend_rad = math.radians(clamp(bend_deg, 0.0, 110.0))

        angles.append(bend_rad)

    return angles


def update_finger_visualisation():
    if "hand_ax" not in globals():
        return

    hand_ax.clear()

    hand_ax.set_title("3D Finger Visualisation")
    hand_ax.set_xlabel("X")
    hand_ax.set_ylabel("Y")
    hand_ax.set_zlabel("Z")

    hand_ax.set_xlim(-3.5, 3.5)
    hand_ax.set_ylim(-0.5, 5.5)
    hand_ax.set_zlim(-0.5, 4.0)

    try:
        hand_ax.set_box_aspect((7, 6, 4.5))
    except Exception:
        pass

    # Palm
    palm_x = [-2.6, 2.6, 2.6, -2.6, -2.6]
    palm_y = [0.0, 0.0, 1.2, 1.2, 0.0]
    palm_z = [0.0, 0.0, 0.0, 0.0, 0.0]
    hand_ax.plot(palm_x, palm_y, palm_z, linewidth=3)

    finger_base_positions = {
        "Finger 1": -2.0,
        "Finger 2": -1.0,
        "Finger 3": 0.0,
        "Finger 4": 1.0,
        "Finger 5": 2.0,
    }

    segment_lengths = [1.4, 1.1, 0.85]

    for finger_name, base_x in finger_base_positions.items():
        joint_angles = get_finger_joint_angles(finger_name)

        x_points = [base_x]
        y_points = [1.1]
        z_points = [0.0]

        total_angle = 0.0

        for index, length in enumerate(segment_lengths):
            total_angle += joint_angles[index]

            # Finger extends in +Y and bends upward in +Z.
            next_x = x_points[-1]
            next_y = y_points[-1] + length * math.cos(total_angle)
            next_z = z_points[-1] + length * math.sin(total_angle)

            x_points.append(next_x)
            y_points.append(next_y)
            z_points.append(next_z)

        hand_ax.plot(
            x_points,
            y_points,
            z_points,
            marker="o",
            linewidth=4
        )

        hand_ax.text(
            base_x,
            0.85,
            0.0,
            finger_name.replace("Finger ", "F"),
            fontsize=8
        )

    hand_canvas.draw_idle()


def set_finger_ratio(finger_name, value):
    finger_visual_ratios[finger_name] = float(value)
    update_finger_visualisation()

#Command sending to ESP32
def send_command(command):
    if esp and esp.is_open:
        esp.write((command + "\n").encode())
        add_terminal_message(f"[ESP32 TX] {command}")
    else:
        add_terminal_message("[ESP32] Not connected")

#Command sending to r-link
def send_cubemars_packet(packet, label="AK80 packet"):
    if not ak_command_output_enabled.get():
        add_ak_terminal_message("[AK80] Command output disabled")
        return

    if cubemars and cubemars.is_open:
        cubemars.write(packet)
        add_ak_terminal_message(f"[AK80 TX] {label}: {packet.hex(' ').upper()}")
    else:
        add_ak_terminal_message("[AK80] R-Link not connected")


def move_ak80(position):
    global ak_position_deg

    ak_position_deg = clamp(float(position), AK_POS_MIN, AK_POS_MAX)

    ak_status_label.config(
        text=f"AK80 Pending Position: {ak_position_deg:.2f} deg"
    )


def send_ak80_servo_position():
    pos_int = int(ak_position_deg * 1000.0)
    speed_int = int(ak_speed_erpm)
    accel_int = int(ak_accel_erps)

    data_frame = bytes([COMM_SET_POS_SPD])
    data_frame += int32_to_bytes(pos_int)
    data_frame += int32_to_bytes(speed_int)
    data_frame += int32_to_bytes(accel_int)

    packet = build_cubemars_packet(data_frame)

    send_cubemars_packet(
        packet,
        f"POS_SPD pos={ak_position_deg:.2f}deg speed={ak_speed_erpm} accel={ak_accel_erps}",
    )

    ak_status_label.config(
        text=f"AK80 Position Sent: {ak_position_deg:.2f} deg"
    )


def send_ak80_position_only():
    pos_int = int(ak_position_deg * 1000000.0)

    data_frame = bytes([COMM_SET_POS])
    data_frame += int32_to_bytes(pos_int)

    packet = build_cubemars_packet(data_frame)

    send_cubemars_packet(packet, f"POSITION_ONLY pos={ak_position_deg:.2f}deg")

    ak_status_label.config(
        text=f"AK80 Position-only Sent: {ak_position_deg:.2f} deg"
    )


def ak_get_values():
    data_frame = bytes([COMM_GET_VALUES])
    packet = build_cubemars_packet(data_frame)

    send_cubemars_packet(packet, "GET_VALUES / connection test")
    ak_status_label.config(text="AK80 connection test sent")


def ak_stop():
    data_frame = bytes([0x08])
    data_frame += int32_to_bytes(0)

    packet = build_cubemars_packet(data_frame)

    send_cubemars_packet(packet, "STOP speed=0")
    ak_status_label.config(text="AK80 stop command sent")


def ak_zero():
    global ak_position_deg

    ak_position_deg = 0.0
    ak_slider.set(0)

    data_frame = bytes([COMM_SET_POS_ORIGIN, 0x00])
    packet = build_cubemars_packet(data_frame)

    send_cubemars_packet(packet, "ZERO temporary origin")
    ak_status_label.config(text="AK80 zero command sent")

#Serial reader
def serial_reader():
    while running:
        if esp and esp.is_open:
            try:
                if esp.in_waiting:
                    msg = esp.readline().decode(errors="ignore").strip()
                    if msg:
                        root.after(0, add_terminal_message, f"[ESP32 RX] {msg}")
            except Exception:
                pass

        if cubemars and cubemars.is_open:
            try:
                if cubemars.in_waiting:
                    data = cubemars.read(cubemars.in_waiting)
                    if data:
                        root.after(
                            0,
                            add_ak_terminal_message,
                            f"[AK80 RX] {data.hex(' ').upper()}",
                        )
            except Exception:
                pass

        time.sleep(0.01)

#Servo control
def move_servo(servo_num, angle):
    angle = int(float(angle))
    servo_targets[servo_num] = angle

    status_label.config(text=f"Pending -> Servo {servo_num}: {angle}")
    update_finger_visualisation()


def move_finger(finger_name, angle):
    angle = int(float(angle))

    for servo_num in finger_groups[finger_name]:
        sliders[servo_num].set(angle)
        servo_targets[servo_num] = angle

    status_label.config(text=f"Pending -> {finger_name}: {angle}")
    update_finger_visualisation()


def send_all_servos():
    if not esp or not esp.is_open:
        add_terminal_message("[ESP32] Not connected")
        return

    status_label.config(text="Sending servo positions...")
    root.update()

    for servo_num in range(15):
        angle = servo_targets[servo_num]
        command = f"SERVO:{servo_num}:{angle}"
        send_command(command)
        time.sleep(0.02)

    status_label.config(text="All servo positions sent")

#Buttons for servo control
def clench():
    for i in range(15):
        sliders[i].set(160)
        servo_targets[i] = 160

    for finger_name in group_sliders:
        group_sliders[finger_name].set(160)

    send_command("Clench:")
    status_label.config(text="Clench sent")
    update_finger_visualisation()


def unclench():
    for i in range(15):
        sliders[i].set(20)
        servo_targets[i] = 20

    for finger_name in group_sliders:
        group_sliders[finger_name].set(20)

    send_command("Unclench:")
    status_label.config(text="Unclench sent")
    update_finger_visualisation()


def list_angles():
    send_command("ListAngles:")
    status_label.config(text="Requested servo angles")

#GUI setup
root = tk.Tk()
root.title("ESP32 Servo + CubeMars AK80 Controller")
root.geometry("1200x1200")

ak_command_output_enabled = tk.BooleanVar(value=True)
ak_terminal_output_enabled = tk.BooleanVar(value=True)
ak_output_to_main_terminal = tk.BooleanVar(value=False)
ak_output_to_separate_terminal = tk.BooleanVar(value=True)

esp_port_var = tk.StringVar(value=DEFAULT_ESP32_PORT)
esp_baud_var = tk.StringVar(value=str(DEFAULT_ESP32_BAUD))

ak_port_var = tk.StringVar(value=DEFAULT_CUBEMARS_PORT)
ak_baud_var = tk.StringVar(value=str(DEFAULT_CUBEMARS_BAUD))

main_frame = tk.Frame(root)
main_frame.pack(fill="both", expand=True)

canvas = tk.Canvas(main_frame)
canvas.pack(side="left", fill="both", expand=True)

scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
scrollbar.pack(side="right", fill="y")

canvas.configure(yscrollcommand=scrollbar.set)

content_frame = tk.Frame(canvas)
canvas_window = canvas.create_window((0, 0), window=content_frame, anchor="nw")


def update_scroll_region(event=None):
    canvas.configure(scrollregion=canvas.bbox("all"))


def resize_content_width(event):
    canvas.itemconfig(canvas_window, width=event.width)


content_frame.bind("<Configure>", update_scroll_region)
canvas.bind("<Configure>", resize_content_width)


def _on_mousewheel(event):
    canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")


canvas.bind_all("<MouseWheel>", _on_mousewheel)

#Title, replace down the line with better title
title = tk.Label(
    content_frame,
    text="Bioinspired Hand Controller",
    font=("Arial", 20, "bold"),
)
title.pack(pady=10)

#Com port connection stuff
connection_frame = tk.LabelFrame(
    content_frame,
    text="Connection Setup",
    padx=10,
    pady=10,
)
connection_frame.pack(fill="x", padx=10, pady=10)

available_ports = get_available_ports()

tk.Label(connection_frame, text="ESP32 Port").grid(row=0, column=0, padx=5, pady=4)
esp_port_box = ttk.Combobox(
    connection_frame,
    textvariable=esp_port_var,
    values=available_ports,
    width=12,
)
esp_port_box.grid(row=0, column=1, padx=5, pady=4)

tk.Label(connection_frame, text="Baud").grid(row=0, column=2, padx=5, pady=4)
esp_baud_box = ttk.Combobox(
    connection_frame,
    textvariable=esp_baud_var,
    values=["9600", "57600", "115200", "921600"],
    width=10,
)
esp_baud_box.grid(row=0, column=3, padx=5, pady=4)

ttk.Button(connection_frame, text="Connect ESP32", command=connect_esp32).grid(
    row=0, column=4, padx=5, pady=4
)
ttk.Button(connection_frame, text="Disconnect ESP32", command=disconnect_esp32).grid(
    row=0, column=5, padx=5, pady=4
)

esp_status_label = ttk.Label(
    connection_frame,
    text="ESP32 not connected",
    foreground="red",
)
esp_status_label.grid(row=0, column=6, padx=8, pady=4, sticky="w")

tk.Label(connection_frame, text="AK80 Port").grid(row=1, column=0, padx=5, pady=4)
ak_port_box = ttk.Combobox(
    connection_frame,
    textvariable=ak_port_var,
    values=available_ports,
    width=12,
)
ak_port_box.grid(row=1, column=1, padx=5, pady=4)

tk.Label(connection_frame, text="Baud").grid(row=1, column=2, padx=5, pady=4)
ak_baud_box = ttk.Combobox(
    connection_frame,
    textvariable=ak_baud_var,
    values=["9600", "57600", "115200", "921600"],
    width=10,
)
ak_baud_box.grid(row=1, column=3, padx=5, pady=4)

ttk.Button(connection_frame, text="Connect AK80", command=connect_cubemars).grid(
    row=1, column=4, padx=5, pady=4
)
ttk.Button(connection_frame, text="Disconnect AK80", command=disconnect_cubemars).grid(
    row=1, column=5, padx=5, pady=4
)

ak_connection_status_label = ttk.Label(
    connection_frame,
    text="AK80 R-Link not connected",
    foreground="red",
)
ak_connection_status_label.grid(row=1, column=6, padx=8, pady=4, sticky="w")

ttk.Button(connection_frame, text="Refresh Ports", command=refresh_ports).grid(
    row=2, column=0, padx=5, pady=4
)

#Finger graphs 
visual_frame = tk.LabelFrame(
    content_frame,
    text="3D Finger Visualisation",
    padx=10,
    pady=10,
)
visual_frame.pack(fill="both", expand=True, padx=10, pady=10)

hand_figure = Figure(figsize=(7, 5), dpi=100)
hand_ax = hand_figure.add_subplot(111, projection="3d")

hand_canvas = FigureCanvasTkAgg(hand_figure, master=visual_frame)
hand_canvas.get_tk_widget().pack(fill="both", expand=True)

ratio_frame = tk.LabelFrame(
    visual_frame,
    text="Visual Ratio Calibration",
    padx=10,
    pady=10,
)
ratio_frame.pack(fill="x", pady=10)

for finger_name in finger_groups:
    row = tk.Frame(ratio_frame)
    row.pack(fill="x", pady=2)

    label = tk.Label(row, text=f"{finger_name} Ratio", width=16)
    label.pack(side="left")

    ratio_slider = tk.Scale(
        row,
        from_=0.1,
        to=2.5,
        resolution=0.05,
        orient="horizontal",
        length=350,
        command=lambda value, f=finger_name: set_finger_ratio(f, value),
    )
    ratio_slider.set(finger_visual_ratios[finger_name])
    ratio_slider.pack(side="left", fill="x", expand=True)

#Slider grouping
group_frame = tk.LabelFrame(
    content_frame,
    text="Finger Group Controls",
    padx=10,
    pady=10,
)
group_frame.pack(fill="x", padx=10, pady=10)

group_sliders = {}

for finger_name in finger_groups:
    frame = tk.Frame(group_frame)
    frame.pack(fill="x", pady=5)

    label = tk.Label(frame, text=finger_name, width=12)
    label.pack(side="left")

    slider = tk.Scale(
        frame,
        from_=20,
        to=160,
        orient="horizontal",
        length=500,
        command=lambda value, f=finger_name: move_finger(f, value),
    )
    slider.set(90)
    slider.pack(side="left", fill="x", expand=True)

    group_sliders[finger_name] = slider

#Individual sliders
servo_frame = tk.LabelFrame(
    content_frame,
    text="Individual Servo Controls",
    padx=10,
    pady=10,
)
servo_frame.pack(fill="both", expand=True, padx=10, pady=10)

sliders = []

for i in range(15):
    frame = tk.Frame(servo_frame)
    frame.pack(fill="x", pady=3)

    label = tk.Label(frame, text=f"Servo {i}", width=10)
    label.pack(side="left")

    slider = tk.Scale(
        frame,
        from_=20,
        to=160,
        orient="horizontal",
        length=500,
        command=lambda value, s=i: move_servo(s, value),
    )
    slider.set(90)
    slider.pack(side="left", fill="x", expand=True)

    sliders.append(slider)

#Servo status
status_label = tk.Label(content_frame, text="Ready", font=("Arial", 11), anchor="w")
status_label.pack(fill="x", padx=10, pady=5)

#Servo buttons
button_frame = tk.Frame(content_frame)
button_frame.pack(pady=20)

ttk.Button(button_frame, text="Clench", command=clench).grid(
    row=0, column=0, padx=10
)
ttk.Button(button_frame, text="Unclench", command=unclench).grid(
    row=0, column=1, padx=10
)
ttk.Button(button_frame, text="List Angles", command=list_angles).grid(
    row=0, column=2, padx=10
)
ttk.Button(button_frame, text="SEND ALL SERVOS", command=send_all_servos).grid(
    row=0, column=3, padx=10
)

#AK80-8
ak_frame = tk.LabelFrame(
    content_frame,
    text="CubeMars AK80-8 Servo Mode Controls",
    padx=10,
    pady=10,
)
ak_frame.pack(fill="x", padx=10, pady=10)

position_frame = tk.Frame(ak_frame)
position_frame.pack(fill="x", pady=5)

position_label = tk.Label(position_frame, text="Position", width=12)
position_label.pack(side="left")

ak_slider = tk.Scale(
    position_frame,
    from_=AK_POS_MIN,
    to=AK_POS_MAX,
    resolution=0.5,
    orient="horizontal",
    length=500,
    command=move_ak80,
)
ak_slider.set(0)
ak_slider.pack(side="left", fill="x", expand=True)

ak_status_label = tk.Label(
    ak_frame,
    text="AK80 Ready",
    font=("Arial", 11),
    anchor="w",
)
ak_status_label.pack(fill="x", padx=5, pady=5)

ak_button_frame = tk.Frame(ak_frame)
ak_button_frame.pack(pady=10)

ttk.Button(ak_button_frame, text="SEND POS+SPEED", command=send_ak80_servo_position).grid(
    row=0, column=0, padx=10
)
ttk.Button(ak_button_frame, text="SEND POSITION ONLY", command=send_ak80_position_only).grid(
    row=0, column=1, padx=10
)
ttk.Button(ak_button_frame, text="GET VALUES", command=ak_get_values).grid(
    row=0, column=2, padx=10
)
ttk.Button(ak_button_frame, text="STOP", command=ak_stop).grid(
    row=0, column=3, padx=10
)
ttk.Button(ak_button_frame, text="ZERO", command=ak_zero).grid(
    row=0, column=4, padx=10
)

#Output routing for AK80-8
ak_output_frame = tk.LabelFrame(
    content_frame,
    text="CubeMars Output Routing",
    padx=10,
    pady=10,
)
ak_output_frame.pack(fill="x", padx=10, pady=10)

ttk.Checkbutton(
    ak_output_frame,
    text="Enable AK80 command output",
    variable=ak_command_output_enabled,
).grid(row=0, column=0, padx=10, sticky="w")

ttk.Checkbutton(
    ak_output_frame,
    text="Show AK80 messages",
    variable=ak_terminal_output_enabled,
).grid(row=0, column=1, padx=10, sticky="w")

ttk.Checkbutton(
    ak_output_frame,
    text="AK80 to main terminal",
    variable=ak_output_to_main_terminal,
).grid(row=1, column=0, padx=10, sticky="w")

ttk.Checkbutton(
    ak_output_frame,
    text="AK80 to separate terminal",
    variable=ak_output_to_separate_terminal,
).grid(row=1, column=1, padx=10, sticky="w")

#Main terminal
terminal_frame = tk.LabelFrame(
    content_frame,
    text="Main System Terminal",
    padx=10,
    pady=10,
)
terminal_frame.pack(fill="both", expand=True, padx=10, pady=10)

terminal_scroll = ttk.Scrollbar(terminal_frame)
terminal_scroll.pack(side="right", fill="y")

terminal = tk.Text(
    terminal_frame,
    height=12,
    bg="black",
    fg="lime",
    insertbackground="white",
    state="disabled",
    yscrollcommand=terminal_scroll.set,
)
terminal.pack(fill="both", expand=True)

terminal_scroll.config(command=terminal.yview)

#AK80-8 terminal split off for readability
ak_terminal_frame = tk.LabelFrame(
    content_frame,
    text="CubeMars AK80 Terminal",
    padx=10,
    pady=10,
)
ak_terminal_frame.pack(fill="both", expand=True, padx=10, pady=10)

ak_terminal_scroll = ttk.Scrollbar(ak_terminal_frame)
ak_terminal_scroll.pack(side="right", fill="y")

ak_terminal = tk.Text(
    ak_terminal_frame,
    height=10,
    bg="black",
    fg="cyan",
    insertbackground="white",
    state="disabled",
    yscrollcommand=ak_terminal_scroll.set,
)
ak_terminal.pack(fill="both", expand=True)

ak_terminal_scroll.config(command=ak_terminal.yview)

#Threading
thread = threading.Thread(target=serial_reader, daemon=True)
thread.start()

root.after(200, update_finger_visualisation)

#Exit code
def on_close():
    global running

    running = False

    if esp:
        esp.close()

    if cubemars:
        cubemars.close()

    root.destroy()


root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
