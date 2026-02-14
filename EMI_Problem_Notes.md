# EMI Problem - Zusammenfassung

## Problem
Massive Winkelschwankungen (5-18°) bei der Pitch-Messung, obwohl der Roboter in konstantem Winkel gehalten wurde.

**Symptome:**
- Pitch-Werte oszillierten stark, auch bei abgeschalteten Motoren
- complementaryFilter=0.96 (hoher Gyro-Anteil) verstärkte das Problem
- complementaryFilter=0.40 (hoher Accel-Anteil) verbesserte die Situation

## Ursache
**Elektromagnetische Interferenz (EMI) vom L298N Motortreiber**

Der L298N erzeugt starke elektromagnetische Störungen, die die I²C-Kommunikation zwischen ESP32 und ICM42688 IMU beeinflussen:
- I²C verwendet zwei offene Leitungen (SDA, SCL) mit Pull-up Widerständen
- Diese Leitungen wirken wie Antennen für EMI
- Motor-PWM bei 20 kHz erzeugt elektromagnetische Felder
- Störungen verfälschen die Sensor-Daten während der Übertragung

## Lösung
**Migration von I²C auf SPI-Kommunikation**

### Vorteile SPI:
- **Schneller:** 10 MHz statt 1 MHz (10x schneller)
- **EMI-resistent:** Differentielle Signale, kürzere Übertragungszeiten
- **Robuster:** Dedizierte Chip-Select Leitung, weniger anfällig für Störungen
- **Performanter:** 4-8% schnellere Loop-Zeiten

### Implementierung:
- SPI Pins: MOSI=23, MISO=19, SCK=18, CS=5
- IMU Objekt: `ICM42688 IMU(SPI, SPI_CS);`
- SPI Initialisierung statt Wire/I²C

## Weitere Verbesserungen (optional)
1. **TB6612FNG statt L298N:** Modernerer Motortreiber mit weniger EMI
2. **LC-Filter für IMU:** Separate, gefilterte 3.3V Versorgung für den Sensor
3. **Twisted-Pair Kabel:** Für Motor-Leitungen (reduziert Abstrahlung)
4. **Abschirmung:** Metallgehäuse um IMU herum
5. **Getrennte Stromkreise:** Separate Batterie für Logik und Motoren mit gemeinsamem Ground

## Ergebnis
Mit SPI-Kommunikation sollten die Pitch-Schwankungen deutlich reduziert werden, da die Datenübertragung schneller und weniger anfällig für EMI ist.
