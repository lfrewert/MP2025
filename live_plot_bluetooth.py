#!/usr/bin/env python3
"""
Live visualization for ESP32 Self-Balancing Robot via Bluetooth

Shows:
- Pitch angle (degrees)
- PID output (PWM)
- Reads ESP32 debug output via Bluetooth Serial

Usage (Direct Connection - Recommended):
1. Pair ESP32 Bluetooth ("ESP32_Balancer") using bluetoothctl:
   bluetoothctl
   scan on
   pair B0:A7:32:2A:86:06
   trust B0:A7:32:2A:86:06
   connect B0:A7:32:2A:86:06
   exit

2. Run this script (connects directly via MAC address):
   python3 live_plot_bluetooth.py

Alternative (Legacy rfcomm method):
- Create /dev/rfcomm0: sudo rfcomm bind 0 B0:A7:32:2A:86:06 1
- Set BLUETOOTH_PORT = '/dev/rfcomm0' in this script
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import sys
import time

# ========================================
# CONFIGURATION
# ========================================
# ESP32 Bluetooth MAC address (found during pairing)
ESP32_MAC = 'B0:A7:32:2A:86:06'

# Alternative: Use serial port if available
# Linux: /dev/rfcomm0 or /dev/ttyBluetooth0
# macOS: /dev/cu.ESP32_Balancer-SPP
# Windows: COM5 (check Device Manager)
BLUETOOTH_PORT = None  # Set to port path if using rfcomm, or None for direct connection
BAUD_RATE = 115200
MAX_POINTS = 200

# ========================================
# DATA BUFFERS
# ========================================
time_data = deque(maxlen=MAX_POINTS)
pitch_data = deque(maxlen=MAX_POINTS)
output_data = deque(maxlen=MAX_POINTS)
time_counter = 0

# ========================================
# REGEX
# ========================================
pattern = re.compile(
    r'^Pitch:\s*(-?\d+(?:\.\d+)?)°\s*\|\s*'
    r'Output:\s*(-?\d+(?:\.\d+)?)\s*\|\s*'
    r'FallCnt:\s*(\d+)\s*\|\s*'
    r'Freq:\s*(\d+(?:\.\d+)?)\s*Hz$'
)

# ========================================
# BLUETOOTH CONNECTION
# ========================================
def open_bluetooth():
    if BLUETOOTH_PORT:
        # Use traditional serial port (rfcomm)
        print(f"Attempting to connect to Bluetooth at {BLUETOOTH_PORT}...")
        port_url = BLUETOOTH_PORT
    else:
        # Use direct Bluetooth connection via MAC address
        print(f"Attempting direct Bluetooth connection to ESP32_Balancer...")
        print(f"MAC Address: {ESP32_MAC}")
        # Format: rfcomm://MAC:CHANNEL (channel 1 for SPP)
        port_url = f"rfcomm://{ESP32_MAC}:1"

    print("Make sure ESP32 is paired and Bluetooth is enabled!")
    print("-" * 60)

    try:
        ser = serial.Serial(port_url, BAUD_RATE, timeout=1)
        print(f"✓ Connected via Bluetooth @ {BAUD_RATE} baud")
        time.sleep(2)

        # Enable ESP debug mode
        print("→ Enabling debug mode (R)")
        ser.write(b'R')
        ser.flush()
        time.sleep(0.5)

        print("✓ Debug mode enabled")
        print("-" * 60)
        return ser

    except serial.SerialException as e:
        print(f"✗ Could not connect: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure ESP32 'ESP32_Balancer' is paired and trusted:")
        print("   bluetoothctl info B0:A7:32:2A:86:06")
        print("   (Should show: Paired: yes, Trusted: yes)")
        print("\n2. Try connecting via bluetoothctl:")
        print("   bluetoothctl connect B0:A7:32:2A:86:06")
        print("\n3. Check if rfcomm support is available:")
        print("   python3 -c 'import serial.tools.list_ports; print(serial.tools.list_ports.__version__)'")
        print("\nAvailable serial ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  - {p.device}: {p.description}")
        sys.exit(1)

ser = open_bluetooth()

# ========================================
# PLOT SETUP
# ========================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.suptitle("Self-Balancing Robot – Bluetooth Monitor", fontsize=16, fontweight="bold")

# Pitch plot
line_pitch, = ax1.plot([], [], linewidth=2, label="Pitch [°]", color='blue')
ax1.axhline(0, linestyle="--", alpha=0.5, label="Setpoint", color='green')
ax1.axhline(70, linestyle="--", alpha=0.3, color='red')
ax1.axhline(-70, linestyle="--", alpha=0.3, color='red')
ax1.set_ylabel("Pitch [°]")
ax1.set_ylim(-100, 100)
ax1.grid(alpha=0.3)
ax1.legend()
ax1.set_title("Pitch Angle (Bluetooth)")

# Output plot
line_output, = ax2.plot([], [], linewidth=2, label="PID Output", color='orange')
ax2.axhline(0, linestyle="-", alpha=0.4)
ax2.axhline(70, linestyle="--", alpha=0.3, color='green')
ax2.axhline(-70, linestyle="--", alpha=0.3, color='green')
ax2.set_ylabel("PWM")
ax2.set_ylim(-300, 300)
ax2.grid(alpha=0.3)
ax2.legend()
ax2.set_title("Motor Output")

plt.tight_layout()

# ========================================
# SERIAL READ FUNCTION
# ========================================
def read_serial():
    global time_counter

    if ser.in_waiting == 0:
        return False

    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()

        # Ignore ESP messages
        if not line or line.startswith((">>>", "!!!")):
            return False

        match = pattern.match(line)
        if not match:
            return False

        pitch = float(match.group(1))
        output = float(match.group(2))
        fallcnt = int(match.group(3))
        freq = float(match.group(4))

        time_data.append(time_counter)
        pitch_data.append(pitch)
        output_data.append(output)
        time_counter += 1

        if time_counter % 10 == 0:
            print(
                f"[{time_counter:4d}] "
                f"Pitch={pitch:6.2f}° | "
                f"Output={output:6.1f} | "
                f"Freq={freq:4.0f} Hz"
            )

        return True

    except Exception as e:
        print(f"[ERROR] {e}")
        return False

# ========================================
# ANIMATION UPDATE
# ========================================
def animate(_):
    for _ in range(5):
        read_serial()

    if len(time_data) > 0:
        line_pitch.set_data(time_data, pitch_data)
        line_output.set_data(time_data, output_data)

        x_min = time_data[0]
        x_max = max(time_data[-1], MAX_POINTS)

        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)

    return line_pitch, line_output

# ========================================
# CLEANUP
# ========================================
def on_close(_):
    print("\n" + "=" * 60)
    print("Program terminated.")
    if pitch_data:
        print(
            f"Pitch: min={min(pitch_data):.2f}°, "
            f"max={max(pitch_data):.2f}°, "
            f"avg={sum(pitch_data)/len(pitch_data):.2f}°"
        )
    print("=" * 60)
    ser.close()
    sys.exit(0)

fig.canvas.mpl_connect("close_event", on_close)

# ========================================
# START
# ========================================
print("Opening plot window...")
print("Receiving data via Bluetooth...")
print("Close the window to exit.")
print("-" * 60)

try:
    ani = animation.FuncAnimation(
        fig,
        animate,
        interval=50,
        blit=True,
        cache_frame_data=False
    )
    plt.show()

except KeyboardInterrupt:
    print("\nInterrupted by user")
    ser.close()
    sys.exit(0)
