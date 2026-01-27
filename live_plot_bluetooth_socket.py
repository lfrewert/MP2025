#!/usr/bin/env python3
"""
Live visualization for ESP32 Self-Balancing Robot via Bluetooth
Uses direct Bluetooth socket connection (no rfcomm needed)

Shows:
- Pitch angle (degrees)
- PID output (PWM)
- Reads ESP32 debug output via Bluetooth Serial

Usage:
1. Pair ESP32 Bluetooth ("ESP32_Balancer"):
   bluetoothctl
   pair B0:A7:32:2A:86:06
   trust B0:A7:32:2A:86:06
   exit

2. Run this script:
   python3 live_plot_bluetooth_socket.py
"""

import socket
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import sys
import time
import select

# ========================================
# CONFIGURATION
# ========================================
ESP32_MAC = 'B0:A7:32:2A:86:06'  # ESP32 Bluetooth MAC address
RFCOMM_CHANNEL = 1  # SPP channel (always 1 for ESP32 Bluetooth Serial)
MAX_POINTS = 200

# ========================================
# DATA BUFFERS
# ========================================
time_data = deque(maxlen=MAX_POINTS)
pitch_data = deque(maxlen=MAX_POINTS)
output_data = deque(maxlen=MAX_POINTS)
time_counter = 0

# ========================================
# PID PARAMETERS (updated from ESP32 messages)
# ========================================
pid_params = {
    'Kp': 40.0,
    'Ki': 0.0,
    'Kd': 1.0,
    'complementary': 0.98,
    'sample_rate': 1000,
    'pitch_filter': 'AUS',
    'filter_alpha': 0.5
}

# ========================================
# REGEX
# ========================================
pattern = re.compile(
    r'^Pitch:\s*(-?\d+(?:\.\d+)?)°\s*\|\s*'
    r'Output:\s*(-?\d+(?:\.\d+)?)\s*\|\s*'
    r'FallCnt:\s*(\d+)\s*\|\s*'
    r'Freq:\s*(\d+(?:\.\d+)?)\s*Hz$'
)

# Regex for parameter updates
param_pattern = re.compile(r'>>>\s*(Kp|Ki|Kd)\s*=\s*(-?\d+(?:\.\d+)?)')
filter_status_pattern = re.compile(r'>>>\s*Pitch-Filter\s+(AN|AUS)')
filter_ema_pattern = re.compile(r'>>>\s*Pitch-Filter AN \(EMA, Alpha=(-?\d+(?:\.\d+)?)\)')
filter_butterworth_pattern = re.compile(r'>>>\s*Pitch-Filter AN \(Butterworth')
filter_alpha_pattern = re.compile(r'>>>\s*Filter Alpha\s*=\s*(-?\d+(?:\.\d+)?)')

# ========================================
# BLUETOOTH SOCKET CONNECTION
# ========================================
def open_bluetooth_socket():
    print(f"Attempting direct Bluetooth connection to ESP32_Balancer...")
    print(f"MAC Address: {ESP32_MAC}")
    print(f"RFCOMM Channel: {RFCOMM_CHANNEL}")
    print("Make sure ESP32 is paired and powered on!")
    print("-" * 60)

    try:
        # Create Bluetooth RFCOMM socket
        sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)

        # Set socket options for keep-alive
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)

        # Set connection timeout
        sock.settimeout(10)

        print("→ Connecting to ESP32...")
        sock.connect((ESP32_MAC, RFCOMM_CHANNEL))
        print("✓ Connected to ESP32 via Bluetooth")

        # Keep socket in blocking mode with longer timeout
        sock.settimeout(5.0)  # 5 second timeout for reads
        time.sleep(2)

        # Enable ESP debug mode
        print("→ Enabling debug mode (R)")
        sock.send(b'R')
        time.sleep(0.5)

        print("✓ Debug mode enabled")
        print("-" * 60)
        return sock

    except OSError as e:
        print(f"✗ Could not connect: {e}")
        print("\nTroubleshooting:")
        print("1. Make sure ESP32 'ESP32_Balancer' is paired:")
        print("   bluetoothctl paired-devices")
        print("\n2. Pair the device if not already paired:")
        print("   bluetoothctl")
        print("   pair B0:A7:32:2A:86:06")
        print("   trust B0:A7:32:2A:86:06")
        print("   exit")
        print("\n3. Check if ESP32 is powered on and Bluetooth is active")
        print("   (The ESP32 should show 'Bluetooth gestartet!' on USB serial)")
        print("\n4. Make sure bluetooth service is running:")
        print("   systemctl status bluetooth")
        sys.exit(1)

sock = open_bluetooth_socket()
buffer = ""

# ========================================
# PLOT SETUP
# ========================================
from matplotlib.widgets import Button

fig = plt.figure(figsize=(14, 9))
gs = fig.add_gridspec(4, 2, height_ratios=[2, 2, 0.4, 0.4], width_ratios=[3, 1], hspace=0.4, wspace=0.3)

ax1 = fig.add_subplot(gs[0, 0])
ax2 = fig.add_subplot(gs[1, 0])
ax_params = fig.add_subplot(gs[2, 0])
ax_params.axis('off')

# Button axis
ax_btn_stop = fig.add_subplot(gs[3, 0])

# Commands info panel (right side)
ax_commands = fig.add_subplot(gs[:, 1])
ax_commands.axis('off')

fig.suptitle("Self-Balancing Robot – Bluetooth Monitor (Direct Socket)", fontsize=16, fontweight="bold")

# Pitch plot
line_pitch, = ax1.plot([], [], linewidth=2, label="Pitch [°]", color='blue')
ax1.axhline(0, linestyle="--", alpha=0.5, label="Setpoint", color='green')
ax1.axhline(70, linestyle="--", alpha=0.3, color='red')
ax1.axhline(-70, linestyle="--", alpha=0.3, color='red')
ax1.set_ylabel("Pitch [°]")
ax1.set_ylim(-100, 100)
ax1.grid(alpha=0.3)
ax1.legend()
ax1.set_title("Pitch Angle (Bluetooth Socket)")

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

# Parameters display
param_text = ax_params.text(
    0.5, 0.5, '',
    transform=ax_params.transAxes,
    fontsize=11,
    verticalalignment='center',
    horizontalalignment='center',
    family='monospace',
    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3)
)

def update_param_text():
    filter_info = ""
    if pid_params['pitch_filter'] == 'AN (Butterworth)':
        filter_info = "  |  Filter: Butterworth 2nd"
    elif pid_params['pitch_filter'] == 'AN':
        filter_info = f"  |  Filter: EMA (α={pid_params['filter_alpha']:.2f})"
    else:
        filter_info = "  |  Filter: AUS"

    text = (
        f"PID: Kp={pid_params['Kp']:.1f}  Ki={pid_params['Ki']:.2f}  Kd={pid_params['Kd']:.2f}  |  "
        f"Complementary={pid_params['complementary']:.2f}{filter_info}  |  "
        f"Sample Rate={pid_params['sample_rate']} Hz"
    )
    param_text.set_text(text)

update_param_text()

# Emergency Stop Button
def on_stop_click(_):
    print("→ Sending STOP command (S)")
    sock.send(b'S')

btn_stop = Button(ax_btn_stop, 'EMERGENCY STOP (S)', color='lightcoral', hovercolor='red')
btn_stop.on_clicked(on_stop_click)

# Commands info text
commands_info = """
VERFÜGBARE KOMMANDOS:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━

PID-Tuning:
  P/p  Kp ±1.0
  I/i  Ki ±0.1
  D/d  Kd ±0.1

EMI-Filter:
  F/f  EMA-Filter (1. Ord.)
  B/b  Butterworth (2. Ord.)
  L/l  Filter-Stärke ±0.05
       (nur für EMA)

System:
  R/r  Debug-Modus
  S/s  Nothalt
  ?    Hilfe

━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Tipp: Kommandos über
Serial-Terminal oder
Bluetooth senden!

EMA: Schnell, <1ms Delay
Butterworth: Steilerer
Roll-Off, ~2ms Delay
"""

ax_commands.text(0.05, 0.95, commands_info,
                 transform=ax_commands.transAxes,
                 fontsize=9,
                 verticalalignment='top',
                 horizontalalignment='left',
                 family='monospace',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))

# ========================================
# SOCKET READ FUNCTION
# ========================================
def read_socket():
    global time_counter, buffer

    try:
        # Use select with small timeout to check for data
        ready = select.select([sock], [], [], 0.1)
        if not ready[0]:
            return False

        # Read available data
        data = sock.recv(4096)  # Larger buffer
        if not data:
            print("[WARNING] No data received - connection may be closed")
            return False

        # Decode and add to buffer
        buffer += data.decode("utf-8", errors="ignore")

        # Process complete lines from buffer
        while '\n' in buffer:
            line, buffer = buffer.split('\n', 1)
            line = line.strip()

            # Check for parameter updates (>>> Kp = 41.0)
            param_match = param_pattern.match(line)
            if param_match:
                param_name = param_match.group(1)
                param_value = float(param_match.group(2))
                pid_params[param_name] = param_value
                update_param_text()
                print(f"[PARAM] {param_name} = {param_value}")
                continue

            # Check for EMA filter (>>> Pitch-Filter AN (EMA, Alpha=0.50))
            filter_ema_match = filter_ema_pattern.match(line)
            if filter_ema_match:
                alpha_value = float(filter_ema_match.group(1))
                pid_params['pitch_filter'] = 'AN'
                pid_params['filter_alpha'] = alpha_value
                update_param_text()
                print(f"[FILTER] EMA Filter AN (Alpha={alpha_value})")
                continue

            # Check for Butterworth filter (>>> Pitch-Filter AN (Butterworth 2nd, Fc=100Hz))
            filter_butterworth_match = filter_butterworth_pattern.match(line)
            if filter_butterworth_match:
                pid_params['pitch_filter'] = 'AN (Butterworth)'
                update_param_text()
                print(f"[FILTER] Butterworth Filter AN")
                continue

            # Check for pitch filter status OFF (>>> Pitch-Filter AUS)
            filter_status_match = filter_status_pattern.match(line)
            if filter_status_match:
                filter_status = filter_status_match.group(1)
                if filter_status == 'AUS':
                    pid_params['pitch_filter'] = 'AUS'
                    update_param_text()
                    print(f"[FILTER] Pitch-Filter AUS")
                continue

            # Check for filter alpha updates (>>> Filter Alpha = 0.50)
            filter_alpha_match = filter_alpha_pattern.match(line)
            if filter_alpha_match:
                alpha_value = float(filter_alpha_match.group(1))
                pid_params['filter_alpha'] = alpha_value
                update_param_text()
                print(f"[FILTER] Alpha = {alpha_value}")
                continue

            # Ignore other ESP messages
            if not line or line.startswith((">>>", "!!!")):
                continue

            match = pattern.match(line)
            if not match:
                continue

            pitch = float(match.group(1))
            output = float(match.group(2))
            freq = float(match.group(4))

            # Update sample rate from actual frequency
            if freq > 0:
                pid_params['sample_rate'] = int(freq)

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

    except socket.timeout:
        # Timeout is normal when no data is available
        return False
    except socket.error as e:
        print(f"[SOCKET ERROR] Connection lost: {e}")
        print("Try reconnecting or restart the script")
        return False
    except Exception as e:
        print(f"[ERROR] {e}")
        return False

# ========================================
# ANIMATION UPDATE
# ========================================
def animate(_):
    for _ in range(5):
        read_socket()

    if len(time_data) > 0:
        line_pitch.set_data(time_data, pitch_data)
        line_output.set_data(time_data, output_data)

        x_min = time_data[0]
        x_max = max(time_data[-1], MAX_POINTS)

        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)

    # Update parameter text periodically
    update_param_text()

    return line_pitch, line_output, param_text

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
    sock.close()
    sys.exit(0)

fig.canvas.mpl_connect("close_event", on_close)

# ========================================
# START
# ========================================
print("Opening plot window...")
print("Receiving data via Bluetooth socket...")
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
    sock.close()
    sys.exit(0)
