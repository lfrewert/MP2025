# Self-Balancing Robot (Inverted Pendulum)

Ein zweiradriger selbstbalancierender Roboter mit PID-Regelung, implementiert auf einem Arduino Uno mit ICM42688 IMU-Sensor.

## Features

### Regelung
- **PID-Controller** mit einstellbaren Parametern (Kp, Ki, Kd)
- **Hysterese-System** zur Vermeidung von Motor-Oszillationen bei kleinen Auslenkungen
- **Echtzeit-Tuning** über Serial-Interface während des Betriebs
- Regelfrequenz: **500 Hz** für präzise Stabilisierung

### Sensorfusion
- **Complementary Filter** kombiniert Gyroskop und Beschleunigungssensor
  - 92% Gyroskop (schnell, aber driftet)
  - 8% Beschleunigungssensor (stabil, aber langsam)
- **Spike Detection** erkennt und filtert mechanische Stöße
- **EMA-Filter** (Exponential Moving Average) für geglättete Beschleunigungsdaten
- Automatische Gyro-Kalibrierung beim Start

### Sicherheit
- **Sturzerkennung** mit Debouncing (30 Samples > 90°)
- **Nothalt-Funktion** per Serial-Kommando
- **Anti-Windup** für Integral-Anteil
- Automatischer Motor-Stopp bei Sturz

### Monitoring
- **Python Live-Plotting** für Pitch-Winkel, PWM-Output und Loop-Frequenz
- Debug-Modus mit 20 Hz Datenausgabe
- Echtzeit-Visualisierung mit Matplotlib

## Hardware-Anforderungen

### Elektronik
- **Mikrocontroller**: Arduino Uno (oder kompatibel)
- **IMU-Sensor**: ICM42688-P (6-Achsen, I²C)
- **Motor-Treiber**: L298N H-Bridge (oder ähnlich)
- **Motoren**: 2x DC-Motoren mit Getriebe
- **Stromversorgung**: 7-12V für Motoren, 5V für Arduino

### Pin-Belegung

**Motor A:**
- ENA (PWM): Pin 9
- IN1: Pin 7
- IN2: Pin 8

**Motor B:**
- ENB (PWM): Pin 6
- IN3: Pin 3
- IN4: Pin 4

**IMU (I²C):**
- SDA: A4
- SCL: A5
- I²C-Adresse: 0x68

## Software-Anforderungen

### Arduino
- PlatformIO oder Arduino IDE
- ICM42688 Bibliothek

### Python (für Live-Plotting)
```bash
pip install pyserial matplotlib
```

## Installation

### 1. Repository klonen
```bash
git clone https://github.com/lfrewert/MP2025.git
cd MP2025
```

### 2. Arduino-Code hochladen

**Mit PlatformIO:**
```bash
pio run --target upload
```

**Mit Arduino IDE:**
- Öffne `src/MP_Controller_derivateSmoothing.ino`
- Board auswählen: Arduino Uno
- Port auswählen
- Upload

### 3. Berechtigungen setzen (Linux)
```bash
sudo usermod -a -G dialout,uucp $USER
# Neu anmelden erforderlich
```

## Verwendung

### Erste Inbetriebnahme

1. **Roboter aufrecht halten** während der Kalibrierung (5 Sekunden nach Power-On)
2. Nach Kalibrierung beginnt die Balance-Regelung automatisch
3. Roboter vorsichtig loslassen

### Serial-Befehle für PID-Tuning

Öffne Serial Monitor (115200 Baud):

| Befehl | Funktion |
|--------|----------|
| `P` | Kp erhöhen (+1.0) |
| `p` | Kp verringern (-1.0) |
| `I` | Ki erhöhen (+0.1) |
| `i` | Ki verringern (-0.1) |
| `D` | Kd erhöhen (+0.1) |
| `d` | Kd verringern (-0.1) |
| `R` / `r` | Debug-Modus an/aus |
| `S` / `s` | Nothalt |
| `?` | Hilfe anzeigen |

### Live-Visualisierung starten

```bash
python3 live_plot.py
```

Das Script:
- Verbindet automatisch mit dem Arduino
- Aktiviert Debug-Modus automatisch
- Zeigt 3 Plots in Echtzeit:
  - Pitch-Winkel (±100°)
  - PWM-Output (±255)
  - Loop-Frequenz (Hz)

**Tipp**: Schließe den PlatformIO Serial Monitor vor dem Start des Python-Scripts.

## PID-Tuning Guide

### Tuning-Reihenfolge

1. **Kp (Proportional)** - Start: 10.0
   - Zu niedrig: Roboter reagiert zu langsam und fällt um
   - Zu hoch: Oszillationen und Überschwingen
   - **Ziel**: Roboter balanciert, aber schwingt noch

2. **Kd (Differential)** - Start: 2.5
   - Dämpft die Oszillationen von Kp
   - Macht Regelung vorausschauend
   - **Ziel**: Ruhige, stabile Balance

3. **Ki (Integral)** - Start: 0.0
   - Korrigiert bleibende Abweichungen
   - Nur bei Bedarf erhöhen (meist nicht nötig)
   - **Vorsicht**: Kann zu Instabilität führen

### Aktuelle Werte (Beispiel)
```cpp
float Kp = 50.0;  // Proportional
float Ki = 0.0;   // Integral
float Kd = 5.0;   // Differential
```

### Motor-Parameter
```cpp
int minSpeed = 100;      // Minimale PWM
int maxSpeed = 255;      // Maximale PWM
int deadzone = 50;       // Startthreshold
int hysteresis = 80;     // Stopp-Threshold
```

**Hysterese-Prinzip:**
- Motor startet erst bei `deadzone` (50)
- Motor stoppt erst bei `hysteresis` (80)
- Verhindert Zappeln bei kleinen Auslenkungen

## Konfiguration

### Sensor-Filter anpassen

**Complementary Filter:**
```cpp
float complementaryFilter = 0.92;  // 92% Gyro, 8% Accel
// Höher = mehr Gyro = schneller aber mehr Drift
// Niedriger = mehr Accel = stabiler aber langsamer
```

**Beschleunigungssensor-Glättung:**
```cpp
float smoothingFactor = 0.3;    // 0.0-1.0 (höher = weniger Glättung)
float spikeThreshold = 0.2;     // Spike-Erkennung in g
```

### Loop-Frequenz ändern
```cpp
#define TARGET_LOOP_TIME_US 2000  // 2000µs = 500 Hz
// 1000µs = 1000 Hz (schneller, mehr CPU-Last)
```

## Troubleshooting

### Roboter balanciert nicht
- Prüfe PID-Werte, starte mit niedrigem Kp
- Erhöhe Kp schrittweise bis Roboter reagiert
- Füge Kd hinzu um Oszillationen zu dämpfen

### Motor zappelt bei kleinen Auslenkungen
- Erhöhe `deadzone` (z.B. auf 70)
- Vergrößere Hysterese-Bereich
- Erhöhe Kd für mehr Dämpfung

### Roboter fällt trotz Regelung
- Zu niedriges Kp → erhöhen
- Complementary Filter zu träge → erhöhen auf 0.95
- Mechanische Probleme: Schwerpunkt zu hoch

### Python-Plotting zeigt keine Daten
- Debug-Modus aktivieren: Befehl `R` senden
- Prüfe Serial-Port in `live_plot.py` (Zeile 23)
- Schließe andere Serial-Programme (PlatformIO Monitor)

### IMU-Initialisierung fehlgeschlagen
- Prüfe I²C-Verkabelung (SDA/SCL)
- Prüfe I²C-Adresse (0x68 oder 0x69)
- Prüfe Stromversorgung des Sensors

## Projektstruktur

```
MP2025/
├── src/
│   └── MP_Controller_derivateSmoothing.ino  # Haupt-Arduino-Code
├── live_plot.py                              # Python Live-Visualisierung
├── platformio.ini                            # PlatformIO-Konfiguration
└── README.md                                 # Diese Datei
```

## Technische Details

### Control Loop
- **Frequenz**: 500 Hz (2000µs pro Zyklus)
- **Timing**: Microsecond-präzise mit `micros()`
- **IMU Sample Rate**: 1000 Hz (ODR)
- **Debug Output**: 20 Hz (bei aktiviertem Debug-Modus)

### Sensor-Spezifikationen
- **Beschleunigung**: ±4g (optimal für Inverted Pendulum)
- **Gyroskop**: ±500°/s (ausreichend für schnelle Bewegungen)
- **I²C-Frequenz**: 400 kHz (Fast Mode)

### PID-Implementierung
```
Output = Kp × error + Ki × ∫error dt + Kd × d(error)/dt

wobei:
- error = setpoint - input
- setpoint = 0° (vertikal)
- input = pitch (aktueller Winkel)
```

## Lizenz

Dieses Projekt ist Open Source und frei verwendbar für Bildungszwecke.

## Danksagungen

Entwickelt mit Claude Sonnet 4.5 für das Modul Mechatronische Prozesse 2025.

## Kontakt

Repository: https://github.com/lfrewert/MP2025

---

**Hinweis**: Dieser Roboter kann sich schnell bewegen. Teste ihn in einem sicheren Bereich mit ausreichend Platz und vermeide Hindernisse.
