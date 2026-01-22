#!/usr/bin/env python3
"""
Live-Visualisierung für Self-Balancing Robot
Zeigt Pitch-Winkel und PWM-Output in Echtzeit an

Verwendung:
1. Arduino mit Debug-Modus starten (Befehl 'R' senden)
2. Python-Skript ausführen: python3 live_plot.py
3. Richtigen Serial-Port angeben wenn nötig
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import sys

# ========================================
# KONFIGURATION
# ========================================
SERIAL_PORT = '/dev/ttyACM0'  # Anpassen! (Linux: /dev/ttyUSB0, /dev/ttyACM0; Windows: COM3, COM4)
BAUD_RATE = 115200
MAX_POINTS = 200  # Anzahl der anzuzeigenden Datenpunkte (ca. 10 Sekunden bei 20 Hz)

# ========================================
# DATEN-BUFFER
# ========================================
time_data = deque(maxlen=MAX_POINTS)
pitch_data = deque(maxlen=MAX_POINTS)
output_data = deque(maxlen=MAX_POINTS)
freq_data = deque(maxlen=MAX_POINTS)

time_counter = 0

# ========================================
# SERIAL PORT ÖFFNEN
# ========================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"✓ Verbunden mit {SERIAL_PORT} @ {BAUD_RATE} Baud")
    print("Warte auf Daten vom Arduino...")
    print("WICHTIG: Debug-Modus muss aktiv sein (Befehl 'R' senden)")
    print("-" * 60)
except serial.SerialException as e:
    print(f"✗ Fehler beim Öffnen von {SERIAL_PORT}: {e}")
    print("\nVerfügbare Ports:")
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"  - {port.device}: {port.description}")
    sys.exit(1)

# ========================================
# PLOT SETUP
# ========================================
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 9))
fig.suptitle('Self-Balancing Robot - Live Monitoring', fontsize=16, fontweight='bold')

# Pitch-Plot
line_pitch, = ax1.plot([], [], 'b-', linewidth=2, label='Pitch')
ax1.axhline(y=0, color='g', linestyle='--', alpha=0.5, label='Setpoint (0°)')
ax1.axhline(y=90, color='r', linestyle='--', alpha=0.3, label='Fall Threshold')
ax1.axhline(y=-90, color='r', linestyle='--', alpha=0.3)
ax1.set_ylabel('Pitch [°]', fontsize=12, fontweight='bold')
ax1.set_ylim(-100, 100)
ax1.grid(True, alpha=0.3)
ax1.legend(loc='upper right')
ax1.set_title('Neigungswinkel', fontsize=11)

# PWM Output-Plot
line_output, = ax2.plot([], [], 'r-', linewidth=2, label='PWM Output')
ax2.axhline(y=0, color='gray', linestyle='-', alpha=0.5)
ax2.axhline(y=140, color='orange', linestyle='--', alpha=0.3, label='Min Speed')
ax2.axhline(y=-140, color='orange', linestyle='--', alpha=0.3)
ax2.axhline(y=70, color='yellow', linestyle='--', alpha=0.3, label='Deadzone')
ax2.axhline(y=-70, color='yellow', linestyle='--', alpha=0.3)
ax2.set_ylabel('PWM Output', fontsize=12, fontweight='bold')
ax2.set_ylim(-300, 300)
ax2.grid(True, alpha=0.3)
ax2.legend(loc='upper right')
ax2.set_title('Motor-Output', fontsize=11)

# Frequenz-Plot
line_freq, = ax3.plot([], [], 'g-', linewidth=2, label='Loop Frequency')
ax3.axhline(y=500, color='b', linestyle='--', alpha=0.5, label='Target (500 Hz)')
ax3.set_xlabel('Sample #', fontsize=12, fontweight='bold')
ax3.set_ylabel('Frequenz [Hz]', fontsize=12, fontweight='bold')
ax3.set_ylim(0, 600)
ax3.grid(True, alpha=0.3)
ax3.legend(loc='upper right')
ax3.set_title('Control Loop Frequenz', fontsize=11)

plt.tight_layout()

# ========================================
# REGEX PATTERN FÜR DATEN-PARSING
# ========================================
# Erwartet Format: "Pitch: X.XX° | Output: XXX.X | FallCnt: X | Freq: XXX Hz"
pattern = re.compile(r'Pitch:\s*([-\d.]+).*?Output:\s*([-\d.]+).*?Freq:\s*([\d.]+)')

# ========================================
# DATEN LESEN UND PARSEN
# ========================================
def read_serial():
    """Liest eine Zeile vom Serial Port und extrahiert die Werte"""
    global time_counter

    try:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            # Pattern matchen
            match = pattern.search(line)
            if match:
                pitch = float(match.group(1))
                output = float(match.group(2))
                freq = float(match.group(3))

                # Daten zu Buffer hinzufügen
                time_data.append(time_counter)
                pitch_data.append(pitch)
                output_data.append(output)
                freq_data.append(freq)

                time_counter += 1

                # Konsolenausgabe (alle 10 Samples)
                if time_counter % 10 == 0:
                    print(f"[{time_counter:4d}] Pitch: {pitch:6.2f}° | Output: {output:6.1f} | Freq: {freq:3.0f} Hz")

                return True
            else:
                # Zeile ohne Daten (z.B. Info-Meldungen)
                if line and not line.startswith(">>>"):
                    print(f"[INFO] {line}")
    except Exception as e:
        print(f"Fehler beim Lesen: {e}")

    return False

# ========================================
# ANIMATION UPDATE FUNKTION
# ========================================
def animate(frame):
    """Wird periodisch aufgerufen um Plot zu aktualisieren"""

    # Mehrere Zeilen lesen pro Frame für bessere Performance
    for _ in range(5):
        read_serial()

    # Plot-Daten aktualisieren
    if len(time_data) > 0:
        line_pitch.set_data(time_data, pitch_data)
        line_output.set_data(time_data, output_data)
        line_freq.set_data(time_data, freq_data)

        # X-Achse anpassen (sliding window)
        if len(time_data) >= MAX_POINTS:
            x_min = time_data[0]
            x_max = time_data[-1]
        else:
            x_min = 0
            x_max = MAX_POINTS

        ax1.set_xlim(x_min, x_max)
        ax2.set_xlim(x_min, x_max)
        ax3.set_xlim(x_min, x_max)

    return line_pitch, line_output, line_freq

# ========================================
# CLEANUP BEI PROGRAMMENDE
# ========================================
def on_close(event):
    """Wird aufgerufen wenn Plot-Fenster geschlossen wird"""
    print("\n" + "=" * 60)
    print("Programm beendet.")
    if len(pitch_data) > 0:
        print(f"Insgesamt {len(pitch_data)} Datenpunkte aufgezeichnet")
        print(f"Pitch: min={min(pitch_data):.2f}°, max={max(pitch_data):.2f}°, avg={sum(pitch_data)/len(pitch_data):.2f}°")
        if len(freq_data) > 0:
            print(f"Frequenz: avg={sum(freq_data)/len(freq_data):.0f} Hz")
    print("=" * 60)
    ser.close()
    sys.exit(0)

fig.canvas.mpl_connect('close_event', on_close)

# ========================================
# ANIMATION STARTEN
# ========================================
print("Plot-Fenster öffnet...")
print("Schließe das Fenster um Programm zu beenden.")
print("-" * 60)

try:
    ani = animation.FuncAnimation(fig, animate, interval=50, blit=True, cache_frame_data=False)
    plt.show()
except KeyboardInterrupt:
    print("\n\nProgramm durch Benutzer abgebrochen (Ctrl+C)")
    ser.close()
    sys.exit(0)
