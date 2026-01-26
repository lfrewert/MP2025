#include <SPI.h>
#include <ICM42688.h>

// ========================================
// TIMING KONFIGURATION
// ========================================
// Ziel-Loop-Frequenz: 1000 Hz = 1000 Mikrosekunden pro Loop
#define TARGET_LOOP_TIME_US 1000 // in Mikrosekunden

// ========================================
// MOTOR PINS DEFINITION (ESP32)
// ========================================
int ENA = 25;  // PWM für Motor A Geschwindigkeit (ESP32 GPIO25)
int IN1 = 26;  // Motor A Richtung 1 (ESP32 GPIO26)
int IN2 = 27;  // Motor A Richtung 2 (ESP32 GPIO27)
int ENB = 33;  // PWM für Motor B Geschwindigkeit (ESP32 GPIO33)
int IN3 = 32;  // Motor B Richtung 1 (ESP32 GPIO32)
int IN4 = 14;  // Motor B Richtung 2 (ESP32 GPIO14)

// ESP32 PWM Channels
#define PWM_CHANNEL_A 0    // LEDC Channel für Motor A
#define PWM_CHANNEL_B 1    // LEDC Channel für Motor B
#define PWM_FREQUENCY 20000 // 20 kHz PWM Frequenz
#define PWM_RESOLUTION 8    // 8-bit Resolution (0-255)

// ========================================
// SPI PINS DEFINITION (ESP32 VSPI)
// ========================================
#define SPI_MOSI 23  // Master Out Slave In
#define SPI_MISO 19  // Master In Slave Out
#define SPI_SCK  18  // Serial Clock
#define SPI_CS   5   // Chip Select (ICM42688)

// ========================================
// IMU SENSOR OBJEKT (SPI)
// ========================================
// ICM42688-P: 6-Achsen IMU (3-Achsen Beschleunigung + 3-Achsen Gyroskop)
// SPI-Kommunikation für höhere Geschwindigkeit und bessere EMI-Resistenz
ICM42688 IMU(SPI, SPI_CS);

// ========================================
// PID REGLER PARAMETE
// ========================================
// Diese Werte müssen angepasst werden!
// Tuning-Reihenfolge: Erst Kp, dann Kd, zuletzt Ki
float Kp = 20.0;  // Proportional-Anteil: Reagiert auf aktuelle Abweichung 20
float Ki = 0.00;   // Integral-Anteil: Korrigiert bleibende Abweichung über Zeit 0.05
float Kd = 0.0;  // Differential-Anteil: Dämpft Überschwingen 0.4

// ========================================
// PID REGLER VARIABLEN
// ========================================
float setpoint = 0.0;      // Ziel-Winkel in Grad (0° = perfekt vertikal)
float input = 0.0;         // Aktueller Winkel vom IMU (pitch)
float output = 0.0;        // PID-Ausgabe → Motor-PWM-Wert
float lastError = 0.0;     // Fehler vom letzten Durchlauf (für D-Anteil)
float integral = 0.0;      // Aufsummierter Fehler (für I-Anteil)
float derivativeSmoothed = 0.0;  // Geglätteter D-Anteil (gegen Rauschen/Oszillation)

// ========================================
// TIMING VARIABLEN
// ========================================
unsigned long lastLoopTime = 0;  // Letzte Loop-Ausführung in Mikrosekunden
float deltaTime = 0.0;           // Zeit zwischen zwei Loop-Durchläufen in Sekunden

// ========================================
// MOTOR GRENZEN
// ========================================
int minSpeed = 70;         // Minimale PWM (unter diesem Wert dreht Motor nicht)
int maxSpeed = 245;        // Maximale PWM (255 = volle Geschwindigkeit)
int deadzone = 70;         // Totzone: PID-Output unter diesem Wert wird ignoriert
int hysteresis = 50;       // Hysterese: Läuft Motor schon, darf er bis hier weiterlaufen

// ========================================
// WINKEL-BERECHNUNG VARIABLEN
// ========================================
float pitch = 0.0;         // Aktueller Neigungswinkel (Pitch) in Grad
float gyroX = 0.0;         // Gyro-Messwert für X-Achse in °/s
float gyroXOffset = 0.0;   // Gyro-Offset (wird bei Kalibrierung bestimmt)
float pitchOffset = 0.0;   // Pitch-Offset für Referenzposition

// ========================================
// HYBRID FILTER VARIABLEN (EMA + Spike Detection)
// ========================================
float accelXSmoothed = 0.0;      // Geglätteter AccelX Wert
float accelZSmoothed = 0.0;      // Geglätteter AccelZ Wert
float smoothingFactor = 0.3;     // 0.0 = sehr glatt, 1.0 = keine Glättung
float spikeThreshold = 0.3;      // Sprünge über 0.5g werden als Spike ignoriert

// Complementary Filter Gewichtung:
// 0.98 = 98% Gyro (schnell, aber driftet), 2% Accel (langsam, aber stabil)
float complementaryFilter = 0.98;

// ========================================
// STURZ-ERKENNUNG MIT DEBOUNCING
// ========================================
#define FALL_THRESHOLD 70.0       // Winkel-Grenze in Grad
#define FALL_SAMPLES_REQUIRED 150  // Anzahl aufeinanderfolgende Samples über Grenze
int fallCounter = 0;              // Zähler für Samples über Grenze

// ========================================
// DEBUG-MODUS (per Serial-Kommando steuerbar)r
// ========================================
bool debugMode = false;  // Debug-Ausgaben an/aus (Standard: aus für maximale Performance)

// ========================================
// MOTOR-STATUS (für Hysterese)
// ========================================
bool motorRunning = false;  // Merkt sich ob Motor aktuell läuft

// ========================================
// FUNKTIONS-DEKLARATIONEN
// ========================================
void stopMotor();
void calibrateIMU();
void updateIMU();
float computePID();
void driveMotors(float speed);

// ========================================
// SETUP FUNKTION - Wird einmal beim Start ausgeführt
// ========================================
void setup() {
  // Serial-Kommunikation mit 115200 Baud starten
  Serial.begin(115200);
  
  // ========================================
  // ESP32 PWM KONFIGURATION
  // ========================================
  // LEDC (LED Controller) für Motor-PWM nutzen
  ledcSetup(PWM_CHANNEL_A, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ENA, PWM_CHANNEL_A);
  ledcAttachPin(ENB, PWM_CHANNEL_B);

  // Motor-Richtungs-Pins als Ausgänge konfigurieren
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Motoren initial stoppen (Sicherheit)
  stopMotor();

  // ========================================
  // SPI INITIALISIERUNG
  // ========================================
  // SPI mit 10 MHz für schnelle, EMI-resistente Kommunikation
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);  // CS initial HIGH (inaktiv)

  // ========================================
  // IMU SENSOR INITIALISIERUNG (SPI)
  // ========================================
  int status = IMU.begin();
  if (status < 0) {
    // IMU konnte nicht initialisiert werden - Programm anhalten
    Serial.println("IMU Initialisierung fehlgeschlagen!");
    Serial.print("Fehlercode: ");
    Serial.println(status);
    while(1);  // Endlosschleife - Programm stoppt hier
  }
  
  // ========================================
  // BESCHLEUNIGUNGSSENSOR KONFIGURATION
  // ========================================
  // Messbereich: ±4g
  // Für Inverted Pendulum optimal: ausreichender Bereich + gute Auflösung
  IMU.setAccelFS(ICM42688::gpm4);
  
  // ========================================
  // GYROSKOP KONFIGURATION
  // ========================================
  // Messbereich: ±500°/s
  // Für Self-Balancing: optimal zwischen Auflösung und Bereich
  IMU.setGyroFS(ICM42688::dps500);
  
  // ========================================
  // SAMPLE RATE KONFIGURATION
  // ========================================
  // Output Data Rate (ODR) auf 2000 Hz setzen
  // Sensor misst intern 2000x pro Sekunde (ausreichend für 1000 Hz Loop)
  IMU.setAccelODR(ICM42688::odr2k);   // 2000 Hz Beschleunigung
  IMU.setGyroODR(ICM42688::odr2k);    // 2000 Hz Gyroskop
  
  Serial.println("Self-Balancing Robot initialisiert!");
  Serial.println("Kalibrierung läuft... Roboter aufrecht halten!");

  // 1 Sekunde Wartezeit zum Aufstellen des Roboters
  delay(1000);
  
  // Gyro-Offset und Pitch-Offset bestimmen
  calibrateIMU();
  
  Serial.println("Kalibrierung abgeschlossen. Balance-Regelung aktiv!");
  Serial.println("---------------------------------------------------");
  Serial.println("Tuning-Befehle:");
  Serial.println("P/p = Kp erhöhen/verringern");
  Serial.println("I/i = Ki erhöhen/verringern");
  Serial.println("D/d = Kd erhöhen/verringern");
  Serial.println("R/r = Debug-Modus an/aus (Pitch-Ausgabe @ 20Hz)");
  Serial.println("S   = Nothalt");
  Serial.println("---------------------------------------------------");
  Serial.print("Ziel-Frequenz: ");
  Serial.print(1000000 / TARGET_LOOP_TIME_US);
  Serial.println(" Hz");
  Serial.print("Sturz-Erkennung: ");
  Serial.print(FALL_SAMPLES_REQUIRED);
  Serial.print(" Samples > ");
  Serial.print(FALL_THRESHOLD);
  Serial.println("° erforderlich");
  Serial.println("---------------------------------------------------");
  Serial.println("Serial-Ausgaben deaktiviert für maximale Performance.");
  Serial.println("Befehl 'R' um Debug-Modus (mit Pitch-Ausgabe) zu aktivieren.");
  Serial.println("---------------------------------------------------");
  
  // Initialisierung der Timing-Variable mit Mikrosekunden
  lastLoopTime = micros();
}

// ========================================
// KALIBRIERUNG: GYRO-OFFSET BESTIMMEN
// ========================================
// Das Gyroskop hat einen kleinen Drift (Offset), auch wenn es stillsteht
// Wir messen diesen Offset und ziehen ihn später von allen Messungen ab
void calibrateIMU() {
  float gyroXSum = 0;
  int samples = 200;  // 200 Messungen für guten Durchschnitt

  Serial.print("Kalibriere Gyroskop");

  // 100 Samples vom Gyroskop sammeln
  for(int i = 0; i < samples; i++) {
    IMU.getAGT();  // Alle Achsen lesen (Accel, Gyro, Temperatur)
    gyroXSum += IMU.gyrX();  // X-Achse aufsummieren
    
    // Fortschritt anzeigen (alle 20 Samples ein Punkt)
    if (i % 20 == 0) Serial.print(".");
    delay(10);  // Kurze Pause zwischen Messungen
  }
  
  // Durchschnitt berechnen = Offset
  gyroXOffset = gyroXSum / samples;
  
  Serial.println(" fertig!");
  Serial.print("Gyro X Offset: ");
  Serial.print(gyroXOffset);
  Serial.println(" °/s");
  
  // ========================================
  // INITIAL-WINKEL AUS BESCHLEUNIGUNG BESTIMMEN
  // ========================================
  // Wichtig damit wir nicht bei 0° starten wenn Roboter schief steht
  IMU.getAGT();
  
  // Achsen-Zuordnung für deine IMU-Montage:
  // AccelZ wird zu X-Achse für Pitch-Berechnung
  // AccelY wird zu Z-Achse (zeigt nach unten bei aufrechtem Roboter)
  float accelX = IMU.accZ();
  float accelZ = IMU.accY();
  
  // Pitch-Offset berechnen (Winkel bei "aufrecht")
  pitchOffset = atan2(accelX, accelZ) * 180.0 / PI;
  
  // Filter-Variablen mit Startwerten initialisieren
  // Wichtig für EMA-Filter beim ersten Durchlauf
  accelXSmoothed = accelX;
  accelZSmoothed = accelZ;
  
  // Start-Position als 0° definieren (Referenz)
  pitch = 0.0;
  
  Serial.print("Pitch-Offset gespeichert: ");
  Serial.print(pitchOffset);
  Serial.println("°");
  Serial.println("Start-Neigung: 0.00° (Referenzposition)");
}

// ========================================
// IMU DATEN LESEN UND WINKEL BERECHNEN
// ========================================
// Liest Sensor-Daten, filtert Spikes, glättet mit EMA und 
// fusioniert Accel + Gyro mit Complementary Filter
void updateIMU() {
  // ========================================
  // SENSOR-DATEN LESEN
  // ========================================
  // Liest alle Achsen auf einmal (effizient!)
  IMU.getAGT();
  
  // Rohe Beschleunigungsdaten mit korrekter Achsen-Zuordnung
  float accelXRaw = IMU.accZ();   // Z-Achse für Pitch
  float accelZRaw = IMU.accY();   // Y-Achse (zeigt nach unten)
  
  // ========================================
  // SPIKE DETECTION (Getriebeschlag-Erkennung)
  // ========================================
  // Berechnet wie stark sich die Beschleunigung seit letzter Messung geändert hat
  float deltaX = abs(accelXRaw - accelXSmoothed);
  float deltaZ = abs(accelZRaw - accelZSmoothed);
  
  // Wenn Änderung zu groß → mechanischer Schlag/Spike!
  // Solche Spikes entstehen durch Getriebespiel oder plötzliche Stöße
  if(deltaX > spikeThreshold || deltaZ > spikeThreshold) {
    // SPIKE ERKANNT! Verwerfe den neuen Wert und behalte den alten
    accelXRaw = accelXSmoothed;
    accelZRaw = accelZSmoothed;
  }
  
  // ========================================
  // EXPONENTIAL MOVING AVERAGE - EMA (Low-Pass Filter)
  // ========================================
  // Glättet die Beschleunigungswerte um hochfrequentes Rauschen zu filtern
  // Formel: smoothed = smoothed × (1-α) + raw × α
  // 
  // Mit α=0.3:
  // - 70% vom alten Wert (Trägheit, dämpft Rauschen)
  // - 30% vom neuen Wert (Reaktionsfähigkeit)
  accelXSmoothed = accelXSmoothed * (1.0 - smoothingFactor) + accelXRaw * smoothingFactor;
  accelZSmoothed = accelZSmoothed * (1.0 - smoothingFactor) + accelZRaw * smoothingFactor;
  
  // ========================================
  // GYROSKOP-DATEN AUSLESEN
  // ========================================
  // Gyroskop misst Drehgeschwindigkeit in °/s
  // Offset wird abgezogen für genauere Messung
  gyroX = IMU.gyrX() - gyroXOffset;
  
  // ========================================
  // WINKEL AUS BESCHLEUNIGUNG BERECHNEN
  // ========================================
  // atan2 berechnet Winkel aus X- und Z-Beschleunigung
  // Ergebnis in Radianten → Umrechnung in Grad
  float accelPitch = atan2(accelXSmoothed, accelZSmoothed) * 180.0 / PI;
  
  // ========================================
  // COMPLEMENTARY FILTER (SENSOR FUSION)
  // ========================================
  // Kombiniert Gyroskop (schnell, aber driftet) mit 
  // Beschleunigung (stabil, aber verrauscht)
  // 
  // Gewichtung 96/4:
  // - 96% vom Gyro (integriert über Zeit): pitch + gyroX × Δt
  // - 4% vom Accel (korrigiert Drift): accelPitch - pitchOffset
  pitch = complementaryFilter * (pitch + gyroX * deltaTime) + 
          (1.0 - complementaryFilter) * (accelPitch - pitchOffset);

  
  // Aktuellen Winkel für PID-Regler speichern
  input = pitch;
}

// ========================================
// PID REGLER BERECHNUNG
// ========================================
// Berechnet Motor-Output basierend auf Abweichung vom Sollwert
float computePID() {
  // Fehler = Aktueller Winkel minus Soll-Winkel (für Segway-Logik)
  // Kippt nach vorne (+pitch) → positiver error → Motor fährt vorwärts
  float error = input - setpoint;
  
  // ========================================
  // P - PROPORTIONAL ANTEIL
  // ========================================
  // Reagiert direkt proportional zum aktuellen Fehler
  // Großer Fehler → große Korrektur
  float P = Kp * error;
  
  // ========================================
  // I - INTEGRAL ANTEIL
  // ========================================
  // Summiert Fehler über Zeit auf
  // Korrigiert bleibende Abweichungen (z.B. Schwerpunkt-Offset)
  integral += error * deltaTime;
  
  // Anti-Windup: Begrenzung verhindert dass Integral zu groß wird
  integral = constrain(integral, -100, 100);
  float I = Ki * integral;
  
  // ========================================
  // D - DIFFERENTIAL ANTEIL (MIT GLÄTTUNG)
  // ========================================
  // Reagiert auf Änderungsgeschwindigkeit des Fehlers
  // Dämpft Oszillationen und Überschwingen
  float derivative = (error - lastError) / deltaTime;

  // D-Anteil glätten um Rauschen zu unterdrücken (Balance: Glättung vs. Reaktion)
  // 70% alter Wert, 30% neuer Wert = moderate Glättung
  derivativeSmoothed = derivativeSmoothed * 0.70 + derivative * 0.30;

  float D = Kd * derivativeSmoothed;  // Nutze geglätteten Wert!
  
  // Fehler für nächsten Durchlauf speichern
  lastError = error;
  
  // ========================================
  // GESAMT-OUTPUT
  // ========================================
  // Alle drei Anteile addieren
  float pidOutput = P + I + D;

  pidOutput = constrain(pidOutput, -255, 255);  // maxSpeed ist 255

  return pidOutput;
}

// ========================================
// MOTOR ANSTEUERUNG
// ========================================
// Steuert beide Motoren basierend auf PID-Output
void driveMotors(float speed) {
  // Richtung aus Vorzeichen bestimmen
  bool forward = (speed > 0);
  
  // Betrag der Geschwindigkeit (absoluter Wert)
  int motorSpeed = abs(speed);
  
  // ========================================
  // DEADZONE MIT HYSTERESE
  // ========================================
  // Hysterese verhindert Oszillation bei kleinen Auslenkungen:
  // - Motor startet erst bei 'deadzone' (z.B. 120)
  // - Motor stoppt erst bei 'hysteresis' (z.B. 80)
  // Dadurch: kein Zappeln mehr bei geringen Fehlern!

  if (motorRunning) {
    // Motor läuft bereits → niedrigere Schwelle zum Weiterlaufen
    if (motorSpeed < hysteresis) {
      stopMotor();
      motorRunning = false;
      return;
    }
  } else {
    // Motor steht → höhere Schwelle zum Starten
    if (motorSpeed < deadzone) {
      stopMotor();
      return;
    }
    motorRunning = true;
  }

  // PWM auf gültige Werte begrenzen (minSpeed bis maxSpeed)
  motorSpeed = constrain(motorSpeed, minSpeed, maxSpeed);
  
  // ========================================
  // MOTOR-RICHTUNG SETZEN (SEGWAY-LOGIK)
  // ========================================
  // H-Bridge Ansteuerung über IN1-IN4 Pins
  if (forward) {
    // Vorwärts: Fängt nach-vorne-Kippen ab (Räder fahren unter Schwerpunkt)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  } else {
    // Rückwärts: Fängt nach-hinten-Kippen ab
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  
  // ========================================
  // MOTOR-GESCHWINDIGKEIT SETZEN (PWM)
  // ========================================
  // PWM-Wert 0-255:
  // 0 = Motor aus, 255 = volle Geschwindigkeit
  ledcWrite(PWM_CHANNEL_A, motorSpeed);  // Motor A (ESP32 LEDC)
  ledcWrite(PWM_CHANNEL_B, motorSpeed);  // Motor B (ESP32 LEDC)
}

// ========================================
// MOTOR STOPPEN
// ========================================
// Stoppt beide Motoren sofort und sicher
void stopMotor() {
  // PWM auf 0 setzen (Motoren aus)
  ledcWrite(PWM_CHANNEL_A, 0);  // Motor A (ESP32 LEDC)
  ledcWrite(PWM_CHANNEL_B, 0);  // Motor B (ESP32 LEDC)

  // Alle Richtungs-Pins auf LOW (zusätzliche Sicherheit)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Integral zurücksetzen wenn gestoppt
  // Verhindert dass sich Fehler während Stillstand aufsummiert
  integral = 0;

  // Motor-Status zurücksetzen
  motorRunning = false;
}

// ========================================
// HAUPT-LOOP
// ========================================
// Wird kontinuierlich wiederholt mit stabiler 500 Hz Frequenz
void loop() {
  // ========================================
  // TIMING-CONTROL (STABLE 500 HZ)
  // ========================================
  // Aktuelle Zeit in Mikrosekunden holen
  unsigned long currentTime = micros();
  
  // Prüfen ob genug Zeit seit letztem Loop vergangen ist
  // Wenn zu früh → Loop überspringen und sofort zurückkehren
  if (currentTime - lastLoopTime < TARGET_LOOP_TIME_US) {
    return;  // Noch nicht bereit für nächsten Loop
  }
  
  // deltaTime in Sekunden berechnen (für PID und Filter)
  deltaTime = (currentTime - lastLoopTime) / 1000000.0;
  lastLoopTime = currentTime;
  
  // ========================================
  // SENSOR DATEN AKTUALISIEREN
  // ========================================
  // Liest IMU, filtert Spikes, berechnet Pitch-Winkel
  updateIMU();
  
  // ========================================
  // DEBUG-AUSGABE (NUR WENN AKTIVIERT)
  // ========================================
  // Ausgaben nur wenn Debug-Modus aktiv (spart Ressourcen)
  // Mit Befehl 'R' über Serial aktivierbar
  if (debugMode) {
    static unsigned long lastDiag = 0;
    if (millis() - lastDiag > 50) {  // 20 Hz Update-Rate
      Serial.print("Pitch: "); Serial.print(pitch, 2);
      Serial.print("° | Output: "); Serial.print(output, 1);
      Serial.print(" | FallCnt: "); Serial.print(fallCounter);
      Serial.print(" | Freq: "); Serial.print(1.0 / deltaTime, 0);
      Serial.println(" Hz");
      lastDiag = millis();
    }
  }
  
  // ========================================
  // STURZ-ERKENNUNG MIT DEBOUNCING
  // ========================================
  // Prüft ob Roboter zu weit gekippt ist (> 70°)
  if (fabs(pitch) > FALL_THRESHOLD) {
    fallCounter++;  // Zähler erhöhen
    
    // Nur wenn N Samples IN FOLGE über Grenze → tatsächlich gefallen
    // Verhindert Fehlalarme durch kurze Spikes
    if (fallCounter >= FALL_SAMPLES_REQUIRED) {
      // Roboter ist gefallen!
      stopMotor();
      Serial.println("!!! GEFALLEN !!!");
      Serial.print("Erkannt nach ");
      Serial.print(fallCounter);
      Serial.println(" Samples");
      Serial.println("Roboter aufrichten und neu starten...");
      delay(1000);  // Pause damit Nachricht lesbar ist
      return;  // Loop beenden
    }
  } else {
    // Winkel wieder OK → Zähler zurücksetzen
    fallCounter = 0;
  }
  
  // ========================================
  // PID REGELUNG
  // ========================================
  // Berechnet benötigten Motor-Output basierend auf aktuellem Winkel
  output = computePID();
  
  // ========================================
  // MOTOR ANSTEUERUNG
  // ========================================
  // Setzt Motor-Richtung und -Geschwindigkeit
  driveMotors(output);
  
  // ========================================
  // LOOP-FREQUENZ MONITOR (NUR IM DEBUG-MODUS)
  // ========================================
  // Misst tatsächliche Loop-Frequenz
  // if (debugMode) {
  //   static unsigned long loopCount = 0;
  //   static unsigned long lastFreqCheck = 0;
  //   loopCount++;
  //   if (millis() - lastFreqCheck > 1000) {
  //     Serial.print(">>> Actual Loop Frequency: ");
  //     Serial.print(loopCount);
  //     Serial.println(" Hz");
  //     loopCount = 0;
  //     lastFreqCheck = millis();
  //   }
  // }
  
  // ========================================
  // SERIAL KOMMANDOS FÜR PID-TUNING
  // ========================================
  // Verarbeitet Kommandos über Serial Monitor
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Kp Tuning
    if (cmd == 'P') {
      Kp += 1.0;
      Serial.print(">>> Kp = ");
      Serial.println(Kp);
    }
    else if (cmd == 'p') {
      Kp -= 1.0;
      Kp = max(0.0, Kp);  // Nicht negativ werden lassen
      Serial.print(">>> Kp = ");
      Serial.println(Kp);
    }
    
    // Ki Tuning
    else if (cmd == 'I') {
      Ki += 0.1;
      Serial.print(">>> Ki = ");
      Serial.println(Ki, 2);
    }
    else if (cmd == 'i') {
      Ki -= 0.1;
      Ki = max(0.0, Ki);
      Serial.print(">>> Ki = ");
      Serial.println(Ki, 2);
    }
    
    // Kd Tuning
    else if (cmd == 'D') {
      Kd += 0.1;
      Serial.print(">>> Kd = ");
      Serial.println(Kd, 2);
    }
    else if (cmd == 'd') {
      Kd -= 0.1;
      Kd = max(0.0, Kd);
      Serial.print(">>> Kd = ");
      Serial.println(Kd, 2);
    }
    
    // Debug-Modus umschalten
    else if (cmd == 'R' || cmd == 'r') {
      debugMode = !debugMode;
      if (debugMode) {
        Serial.println(">>> Debug-Modus AN - Pitch-Ausgabe aktiv @ 20Hz");
      } else {
        Serial.println(">>> Debug-Modus AUS - Ressourcen gespart");
      }
    }
    
    // Nothalt
    else if (cmd == 'S' || cmd == 's') {
      stopMotor();
      Serial.println("!!! NOTHALT aktiviert !!!");
      Serial.println("Aktuelle PID-Werte:");
      Serial.print("Kp = ");
      Serial.println(Kp);
      Serial.print("Ki = ");
      Serial.println(Ki, 2);
      Serial.print("Kd = ");
      Serial.println(Kd, 2);
    }
    
    // Hilfe anzeigen
    else if (cmd == '?') {
      Serial.println("---------------------------------------------------");
      Serial.println("Tuning-Befehle:");
      Serial.println("P/p = Kp erhöhen/verringern (±1.0)");
      Serial.println("I/i = Ki erhöhen/verringern (±0.1)");
      Serial.println("D/d = Kd erhöhen/verringern (±0.1)");
      Serial.println("R/r = Debug-Modus an/aus (Pitch-Ausgabe @ 20Hz)");
      Serial.println("S/s = Nothalt");
      Serial.println("?   = Diese Hilfe");
      Serial.println("---------------------------------------------------");
      Serial.print("Aktuell: Kp=");
      Serial.print(Kp);
      Serial.print(" Ki=");
      Serial.print(Ki, 2);
      Serial.print(" Kd=");
      Serial.println(Kd, 2);
      Serial.print("Debug-Modus: ");
      Serial.println(debugMode ? "AN" : "AUS");
      Serial.print("Ziel-Frequenz: ");
      Serial.print(1000000 / TARGET_LOOP_TIME_US);
      Serial.println(" Hz");
      Serial.println("---------------------------------------------------");
    }
  }
  
  // ========================================
  // KEIN delay() MEHR!
  // ========================================
  // Timing wird über micros() und return gesteuert
  // Loop läuft so schnell wie möglich, aber wird automatisch
  // auf TARGET_LOOP_TIME_US (2000µs = 500Hz) begrenzt
}