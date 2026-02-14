# Bluetooth Setup - ESP32 Self-Balancing Robot

## Übersicht

Der ESP32 sendet **gleichzeitig** über USB und Bluetooth, so dass du:
- **USB:** Verwenden während Entwicklung/Debugging (Kabel)
- **Bluetooth:** Verwenden während Roboter fährt (kabellos)

**Bluetooth Name:** `ESP32_Balancer`

---

## 1. ESP32 Bluetooth aktivieren

Der Code ist bereits angepasst! Der ESP32 startet automatisch Bluetooth beim Hochfahren.

Beim Start siehst du im Serial Monitor:
```
Bluetooth gestartet! Verbinde mit 'ESP32_Balancer'
```

---

## 2. ESP32 mit deinem Computer pairen

### Linux (Ubuntu/Debian/Manjaro)

```bash
# Bluetooth Manager öffnen
bluetoothctl

# ESP32 suchen
scan on

# Warte bis "ESP32_Balancer" erscheint, dann:
pair XX:XX:XX:XX:XX:XX   # MAC-Adresse vom ESP32
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX

# Bluetooth Serial Port binden
sudo rfcomm bind 0 XX:XX:XX:XX:XX:XX 1

# Exit
exit
```

**Bluetooth Serial Port:** `/dev/rfcomm0`

### macOS

1. **System Settings** → **Bluetooth**
2. Suche nach **ESP32_Balancer**
3. Klicke **Verbinden**

**Bluetooth Serial Port:** `/dev/cu.ESP32_Balancer-SPP`

### Windows

1. **Settings** → **Devices** → **Bluetooth & other devices**
2. **Add Bluetooth or other device**
3. Wähle **ESP32_Balancer**
4. Klicke **Pair**
5. Öffne **Device Manager** → **Ports (COM & LPT)**
6. Finde **Standard Serial over Bluetooth link (COMX)**

**Bluetooth Serial Port:** `COM5` (oder andere COM-Nummer)

---

## 3. Bluetooth Serial Port finden

### Linux
```bash
# Methode 1: rfcomm
ls /dev/rfcomm*

# Methode 2: ttyBluetooth
ls /dev/tty* | grep Bluetooth

# Methode 3: Alle Bluetooth devices
ls -l /dev/serial/by-id/ | grep Bluetooth
```

### macOS
```bash
ls /dev/cu.* | grep ESP32
```

### Windows
```
Öffne Device Manager → Ports (COM & LPT)
Suche nach "Standard Serial over Bluetooth link"
```

---

## 4. Python Script für Bluetooth starten

### Schritt 1: Bluetooth Port anpassen

Öffne `live_plot_bluetooth.py` und ändere:

```python
# Linux
BLUETOOTH_PORT = '/dev/rfcomm0'

# macOS
BLUETOOTH_PORT = '/dev/cu.ESP32_Balancer-SPP'

# Windows
BLUETOOTH_PORT = 'COM5'
```

### Schritt 2: Script ausführen

```bash
chmod +x live_plot_bluetooth.py
python3 live_plot_bluetooth.py
```

---

## 5. Automatische USB/Bluetooth Erkennung

Für noch mehr Komfort kannst du ein Script erstellen, das automatisch USB oder Bluetooth wählt:

```python
import serial.tools.list_ports

def find_port():
    """Findet ESP32 automatisch (USB oder Bluetooth)"""
    ports = serial.tools.list_ports.comports()

    for port in ports:
        desc = port.description.lower()
        device = port.device.lower()

        # ESP32 über USB
        if 'cp210' in desc or 'usb' in desc:
            if 'ttyUSB' in device or 'ttyACM' in device:
                return port.device, "USB"

        # ESP32 über Bluetooth
        if 'bluetooth' in desc or 'esp32' in device:
            return port.device, "Bluetooth"

    return None, None

port, connection_type = find_port()
if port:
    print(f"ESP32 found via {connection_type}: {port}")
else:
    print("ESP32 not found!")
```

---

## 6. Bluetooth Reichweite

| Typ | Reichweite | Bemerkung |
|-----|------------|-----------|
| **Bluetooth Classic** | ~10-30m | ESP32 Standard |
| **Im Raum** | 10-20m | Beste Performance |
| **Durch Wände** | 5-10m | Reduzierte Reichweite |
| **Freie Sicht** | bis 30m | Maximale Reichweite |

**Tipp:** Für beste Performance:
- ESP32 so positionieren dass Antenne nicht abgeschirmt ist
- Keine Metallgehäuse um ESP32
- Laptop/PC nicht zu weit entfernen

---

## 7. Fehlerbehebung

### Problem: "Could not open /dev/rfcomm0"

**Lösung 1:** Überprüfe ob ESP32 verbunden ist
```bash
bluetoothctl
info XX:XX:XX:XX:XX:XX
```

**Lösung 2:** Bind rfcomm neu
```bash
sudo rfcomm release 0
sudo rfcomm bind 0 XX:XX:XX:XX:XX:XX 1
```

**Lösung 3:** Berechtigungen prüfen
```bash
sudo chmod 666 /dev/rfcomm0
```

### Problem: "Bluetooth gestartet!" wird nicht angezeigt

**Lösung:** Compile-Error - BluetoothSerial Bibliothek fehlt

Die Bibliothek ist im ESP32 Arduino Core enthalten. Falls Fehler:
```ini
# In platformio.ini hinzufügen:
lib_deps =
    finani/ICM42688
    BluetoothSerial
```

### Problem: Verbindung bricht ab

**Lösung:**
- ESP32 näher zum Computer bringen
- Andere Bluetooth-Geräte ausschalten (weniger Interferenz)
- ESP32 neu starten

### Problem: Keine Daten empfangen

**Lösung:** Debug-Modus aktivieren
- Sende 'R' über Bluetooth Serial
- Oder verbinde kurz per USB und sende 'R'

---

## 8. Gleichzeitige USB + Bluetooth Nutzung

Der ESP32 sendet **gleichzeitig** über beide Kanäle:

**Anwendungsfall 1:** Entwicklung
- USB für Serial Monitor (schneller Upload, Debugging)
- Bluetooth für Live-Plot (keine Kabel)

**Anwendungsfall 2:** Betrieb
- Nur Bluetooth (Roboter fährt frei)

**Anwendungsfall 3:** Tuning
- USB für PID Parameter ändern (P/p, I/i, D/d)
- Bluetooth für Live-Plot beobachten

---

## 9. Performance

| Verbindung | Latenz | Datenrate | Stabilität |
|------------|--------|-----------|------------|
| **USB** | ~1ms | Sehr hoch | ✅ Sehr stabil |
| **Bluetooth** | ~10-50ms | Mittel | ✅ Stabil (10m) |

**Für Self-Balancing:** Bluetooth ist ausreichend, da Plot nur 20 Hz braucht.

---

## Zusammenfassung

1. ✅ ESP32 Code ist bereits angepasst (Bluetooth läuft automatisch)
2. ✅ Pair ESP32 "ESP32_Balancer" in deinem OS
3. ✅ Finde Bluetooth Serial Port (`/dev/rfcomm0`, `/dev/cu.*`, `COM5`)
4. ✅ Passe `BLUETOOTH_PORT` in `live_plot_bluetooth.py` an
5. ✅ Starte `python3 live_plot_bluetooth.py`
6. ✅ Genieße kabelloses Monitoring!

**Viel Erfolg mit deinem Bluetooth Self-Balancing Robot!** 🎉🤖
