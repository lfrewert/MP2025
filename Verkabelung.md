# Vollständige Verkabelung - ESP32 Self-Balancing Robot

## Komponenten
- **ESP32 Dev Board** (240 MHz, Dual-Core)
- **ICM42688-P IMU** (6-Achsen Gyroskop + Beschleunigungssensor)
- **L298N Motortreiber** (Dual H-Bridge)
- **2x DC-Motoren** mit Getriebe
- **Stromversorgung** (empfohlen: 7.4V LiPo 2S oder 9V-12V)

---

## 1. ESP32 ↔ ICM42688 IMU (SPI-Verbindung)

### Für dein ICM42688 Board (mit SAO, CS, Int1, Int2 Pins):

| ESP32 Pin | ICM42688 Pin | Funktion | Beschreibung |
|-----------|--------------|----------|--------------|
| **GPIO 23** | **SDA** | SPI MOSI | Master Out Slave In (Daten ESP32→IMU) |
| **GPIO 19** | **SAO** | SPI MISO | Master In Slave Out (Daten IMU→ESP32) |
| **GPIO 18** | **SCL** | SPI Clock | Serial Clock Signal |
| **GPIO 5** | **CS** | Chip Select | Slave Select (LOW = aktiv) |
| **3.3V** | **VCC** | Stromversorgung | **NUR 3.3V!** |
| **GND** | **GND** | Ground | Gemeinsame Masse |
| - | **Int1** | Interrupt 1 | Nicht benötigt (nicht verbinden) |
| - | **Int2** | Interrupt 2 | Nicht benötigt (nicht verbinden) |

**Wichtige Pin-Erklärungen:**
- **SDA** = MOSI im SPI-Modus (Daten vom ESP32 zum IMU)
- **SCL** = SCLK im SPI-Modus (Clock-Signal)
- **SAO** = MISO im SPI-Modus (Daten vom IMU zum ESP32)
  - Im I²C-Modus wäre SAO für die Adress-Auswahl (0x68 oder 0x69)
  - Im SPI-Modus ist SAO der Daten-Ausgang (MISO)
- **CS** = Chip Select (LOW = IMU aktiv, HIGH = IMU inaktiv)
- **Int1/Int2** = Interrupt-Pins (für fortgeschrittene Features, wir brauchen sie nicht)

**Einfache Merkregel für dein Board:**
```
ICM42688 (dein Board)    →    ESP32 (suche auf Board)
────────────────────────────────────────────────────
VCC                      →    3.3V  (NICHT 5V!)
GND                      →    GND
SDA                      →    GPIO23 (oder IO23)
SCL                      →    GPIO18 (oder IO18)
SAO                      →    GPIO19 (oder IO19)
CS                       →    GPIO5  (oder IO5)
Int1                     →    nicht verbinden
Int2                     →    nicht verbinden
```

**Wichtig:**
- ICM42688 ist **NICHT 5V-tolerant** - nur 3.3V verwenden!
- Kurze Leitungen verwenden (< 10 cm) um EMI zu minimieren
- Optional: 100nF Kondensator zwischen VCC und GND am IMU
- Bei manchen Boards muss ein Jumper gesetzt werden um SPI statt I²C zu aktivieren

---

## 2. ESP32 ↔ L298N Motortreiber

### Motor A (Linker Motor)
| ESP32 Pin | L298N Pin | Funktion | Beschreibung |
|-----------|-----------|----------|--------------|
| GPIO 25   | ENA       | PWM Motor A | Geschwindigkeitssteuerung (20 kHz PWM) |
| GPIO 26   | IN1       | Richtung 1 | Motor A Vorwärts/Rückwärts |
| GPIO 27   | IN2       | Richtung 2 | Motor A Vorwärts/Rückwärts |

### Motor B (Rechter Motor)
| ESP32 Pin | L298N Pin | Funktion | Beschreibung |
|-----------|-----------|----------|--------------|
| GPIO 33   | ENB       | PWM Motor B | Geschwindigkeitssteuerung (20 kHz PWM) |
| GPIO 32   | IN3       | Richtung 1 | Motor B Vorwärts/Rückwärts |
| GPIO 14   | IN4       | Richtung 2 | Motor B Vorwärts/Rückwärts |

### Stromversorgung
| ESP32/Komponente | L298N Pin | Beschreibung |
|------------------|-----------|--------------|
| GND (ESP32)      | GND       | **Gemeinsame Masse** (sehr wichtig!) |
| -                | +12V      | Motoren-Stromversorgung (7-12V) |
| -                | +5V (OUT) | Optional: ESP32 über L298N 5V-Ausgang versorgen |

**Wichtig:**
- **Gemeinsame Masse (GND)** zwischen ESP32 und L298N ist zwingend erforderlich!
- L298N hat einen 5V-Regler (wenn Jumper gesetzt) → kann ESP32 versorgen
- Alternativ: Separate Stromversorgung für ESP32 (USB oder 5V-Regler)

---

## 3. L298N ↔ DC-Motoren

| L298N Pin | Motor | Beschreibung |
|-----------|-------|--------------|
| OUT1      | Motor A + | Motor A Anschluss 1 |
| OUT2      | Motor A - | Motor A Anschluss 2 |
| OUT3      | Motor B + | Motor B Anschluss 1 |
| OUT4      | Motor B - | Motor B Anschluss 2 |

**Motor-Drehrichtung testen:**
- Wenn Motor in falsche Richtung dreht: OUT1 und OUT2 vertauschen
- Beide Motoren sollten bei `driveMotors(pitch)` synchron laufen

---

## 4. Stromversorgung - Empfohlene Konfiguration

### Option A: Gemeinsame Stromversorgung (einfach, aber EMI-anfällig)
```
LiPo 2S (7.4V) oder 9V-12V Netzteil
    │
    ├── L298N +12V (Motor-Versorgung)
    │   └── L298N +5V OUT → ESP32 VIN (5V)
    │
    └── GND (gemeinsam!)
```

### Option B: Getrennte Stromversorgung (besser für EMI)
```
Batterie 1 (7.4V - 12V)          Batterie 2 (3.7V LiPo oder USB)
    │                                │
    ├── L298N +12V                   ├── ESP32 VIN
    └── L298N GND ───────────────────┴── ESP32 GND (gemeinsame Masse!)
```

**Vorteile Option B:**
- Motor-Störungen beeinflussen ESP32 weniger
- Stabilere Spannung für ESP32
- Bessere EMI-Isolation

---

## 5. Kompletter Schaltplan (Übersicht)

```
┌─────────────────────────────────────────────────────────────────┐
│                         ESP32 DEV BOARD                          │
├─────────────────────────────────────────────────────────────────┤
│  3.3V ────────────────────────────► VCC (ICM42688)              │
│  GND  ────────┬───────────────────► GND (ICM42688)              │
│               │                                                  │
│  GPIO 23 (MOSI) ──────────────────► SDI (ICM42688)              │
│  GPIO 19 (MISO) ──────────────────► SDO (ICM42688)              │
│  GPIO 18 (SCK)  ──────────────────► SCL (ICM42688)              │
│  GPIO 5  (CS)   ──────────────────► CS  (ICM42688)              │
│               │                                                  │
│  GPIO 25 (ENA) ────────────────────► ENA (L298N) ──► Motor A PWM│
│  GPIO 26 (IN1) ────────────────────► IN1 (L298N)                │
│  GPIO 27 (IN2) ────────────────────► IN2 (L298N)                │
│               │                                                  │
│  GPIO 33 (ENB) ────────────────────► ENB (L298N) ──► Motor B PWM│
│  GPIO 32 (IN3) ────────────────────► IN3 (L298N)                │
│  GPIO 14 (IN4) ────────────────────► IN4 (L298N)                │
│               │                                                  │
│  GND  ────────┴───────────────────► GND (L298N) ─────┐          │
│  VIN (5V) ◄────────────────────────► +5V (L298N)     │          │
└──────────────────────────────────────────────────────┼──────────┘
                                                        │
                 ┌──────────────────────────────────────┘
                 │  (Gemeinsame Masse)
                 │
          ┌──────┴──────┐
          │   L298N     │
          ├─────────────┤
          │ +12V ◄──────┼─── 7.4V - 12V Batterie (+)
          │ GND  ◄──────┼─── Batterie (-)
          │             │
          │ OUT1/OUT2 ──┼──► Motor A
          │ OUT3/OUT4 ──┼──► Motor B
          └─────────────┘
```

---

## 6. Wichtige Hinweise

### Verkabelung Best Practices:
✅ **Kurze SPI-Leitungen** (< 10 cm) für ICM42688
✅ **Gemeinsame Masse** zwischen allen Komponenten
✅ **Twisted-Pair Kabel** für Motor-Leitungen (reduziert EMI)
✅ **100nF Kondensatoren** an IMU VCC/GND
✅ **Große Kondensatoren** (1000µF) an Motor-Versorgung

### EMI-Reduktion:
- IMU so weit wie möglich von Motoren und L298N entfernen
- Abgeschirmtes Kabel für SPI-Verbindung (optional)
- Ferrit-Ringe an Motor-Kabeln (optional)
- Separate Stromversorgung für Logik und Motoren

### Spannungspegel:
⚠️ **ICM42688 ist 3.3V only** - NICHT 5V verwenden!
⚠️ **L298N Logic-Pins sind 5V-tolerant** - ESP32 (3.3V) funktioniert trotzdem
⚠️ **Motor-Spannung:** 7-12V optimal für L298N

### Sicherheit:
🔴 **Immer Motoren stoppen** bevor Verkabelung geändert wird
🔴 **Verpolung vermeiden** - kann ESP32 und IMU zerstören
🔴 **Kurzschluss-Schutz** durch Überprüfung vor Einschalten

---

## 7. Test-Checkliste

Nach dem Verkabeln:

1. ☑️ Alle GND-Verbindungen prüfen (Multimeter: Durchgang)
2. ☑️ ICM42688 VCC = 3.3V messen (NICHT 5V!)
3. ☑️ L298N +12V = Batteriespannung messen
4. ☑️ ESP32 einstecken (ohne Motoren) und seriell prüfen
5. ☑️ IMU-Initialisierung erfolgreich ("IMU OK")
6. ☑️ Motoren anschließen und Richtung testen
7. ☑️ Kalibrierung durchführen (Roboter aufrecht halten)
8. ☑️ Pitch-Werte im Seriell-Monitor prüfen (stabil?)

---

## 8. Fehlerbehebung

| Problem | Mögliche Ursache | Lösung |
|---------|------------------|--------|
| IMU nicht gefunden | Falsche SPI-Verkabelung | Pins prüfen, CS auf HIGH |
| Motor dreht nicht | PWM zu niedrig | minSpeed anpassen (70-100) |
| Motor dreht falsch herum | Verkabelung vertauscht | OUT1/OUT2 tauschen |
| ESP32 resettet ständig | Zu wenig Strom | Separate 5V-Versorgung |
| Pitch-Werte schwanken stark | EMI von Motoren | SPI-Kabel kürzen, abschirmen |
| Keine Serial-Ausgabe | Falsche Baudrate | 115200 in Monitor einstellen |

---

Viel Erfolg beim Aufbau! 🤖
