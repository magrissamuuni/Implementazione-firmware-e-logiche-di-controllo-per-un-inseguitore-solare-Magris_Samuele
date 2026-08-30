#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <Preferences.h>
#include "webpage.h"

// Modalità di Funzionamento
enum OperatingMode {
  MODE_AUTO,
  MODE_MANUAL,
  MODE_NIGHT,
  MODE_ECO,
  MODE_SEARCH_SUN,
  MODE_AUTOTUNE
};

OperatingMode currentMode = MODE_AUTO;
String modeString = "auto";

// Configurazione Access Point Wi-Fi
const char* ap_ssid = "SolarTracker-ESP32";
const char* ap_password = "12345678";

WebServer server(80);

// Pin Servo 360 Continuous
const int pinServoH = 19; 
const int pinServoV = 18; 

Servo servoH;
Servo servoV;

// Valori di stop per servi 360 in microsecondi
int STOP_H_US = 1500; 
int STOP_V_US = 1500; 

// Pin Sensori LDR 
const int ldrTL = 35; // Top Left (Alto SX)
const int ldrTR = 33; // Top Right (Alto DX)
const int ldrBL = 34; // Bottom Left (Basso SX)
const int ldrBR = 32; // Bottom Right (Basso DX)

// Pin Lettura Tensione Pannello Solare
const int pinSolarVolt = 36; 

// Parametri PID
double setpointH = 0, inputH, outputH;
double setpointV = 0, inputV, outputV;
double Kp = 0.5, Ki = 0.0, Kd = 0.05;

PID pidH(&inputH, &outputH, &setpointH, Kp, Ki, Kd, REVERSE);
PID pidV(&inputV, &outputV, &setpointV, Kp, Ki, Kd, REVERSE);

// True quando il rispettivo PID sta effettivamente guidando il motore 
bool pidActiveH = false;
bool pidActiveV = false;

// Soglie operative
int sogliaNotte = 100;
int zonaMorta = 350; // Zona morta per evitare oscillazioni e pendolamento
int sogliaPuntoMorto = 700; // Soglia per rilevare il punto morto diagonale (saddle point)

// Consente di disattivare da interfaccia la logica di sblocco del punto morto
// diagonale (utile in fase di test per isolarne l'effetto dal resto del controllo).
bool puntoMortoAbilitato = true;

// Isteresi sulla zona morta: per uscire dalla sosta serve superare zonaMorta per intero,
// ma per rientrarci basta scendere sotto zonaMorta*RATIO. Senza questa differenza tra le
// due soglie, un errore che oscilla per rumore proprio attorno a zonaMorta farebbe
// accendere/spegnere il PID continuamente ad ogni ciclo di loop().
const float DEADZONE_HYSTERESIS_RATIO = 0.6f;

// Velocità ridotta in AUTO per test 
int maxAutoSpeed = 3; // Limite di velocità per rotazione fluida e precisa in AUTO

// Opzione Finecorsa Hardware  Asse Y
const bool USE_HARDWARE_LIMITS = false;
const int pinLimitYMin = 22;
const int pinLimitYMax = 23;

// Variabili per Controllo Manuale
int manualVelH = 0; // -15 a +15
int manualVelV = 0; // -15 a +15

// Failsafe controllo manuale: se non arriva nessun comando entro
// questo intervallo, il tracker torna da solo in AUTO invece di restare fermo in
// manuale a tempo indeterminato 
const unsigned long MANUAL_TIMEOUT_MS = 2000;
unsigned long lastManualCmdMillis = 0;

// Telemetria Pannello Solare
float solarVoltage = 0.0;
float solarCurrent = 0.0;
float solarPower = 0.0;
float totalEnergyWh = 0.0;
unsigned long lastEnergyCalc = 0;

// Valori LDR (filtrati) e Diagonali
int valTL = 0, valTR = 0, valBL = 0, valBR = 0;
int valDiag1 = 0;      // Diagonale 1: TL + BR
int valDiag2 = 0;      // Diagonale 2: TR + BL
int diffDiagonali = 0; // Differenza (Diag1 - Diag2)
int mediaTotale = 0;
bool puntoMortoAttivo = false; // True se rilevato punto morto diagonale
bool isSearchingSun = false;   // True durante la scansione ricerca sole
bool isAutotuning = false;     // True durante il test di autotuning PID (relay feedback)

// Filtro passa-basso (media mobile esponenziale) sulle letture LDR grezze
const float LDR_FILTER_TAU_S = 0.15f;
float filtTL = 0, filtTR = 0, filtBL = 0, filtBR = 0;
bool ldrFilterInit = false;

// Fattori di calibrazione per compensare le differenze di sensibilità tra i 4 LDR
float calTL = 1.0f, calTR = 1.0f, calBL = 1.0f, calBR = 1.0f;

// Oversampling ADC: più letture consecutive mediate riducono il rumore residuo
// dell'ADC dell'ESP32 rispetto a una singola lettura.
const int ADC_OVERSAMPLE_COUNT = 4;

// Risparmio energetico: quando i servo restano fermi a lungo (Notte/Eco), si
// staccano dal PWM invece di tenerli agganciati all'impulso neutro.
bool servosPowerSaved = false;

// Ultimo impulso in microsecondi realmente scritto sui servo (dopo deadband e,
// per V, dopo l'inversione di segno per il montaggio fisico invertito). Usato in
// /api/data al posto di ricalcolare la formula, cosi' la telemetria riporta
// sempre esattamente ciò che è stato scritto.
int lastPulseHus = 1500;
int lastPulseVus = 1500;

// Preferences NVS
Preferences prefs;
unsigned long lastEnergySaveMs = 0;
const unsigned long ENERGY_SAVE_INTERVAL_MS = 5UL * 60UL * 1000UL; // ogni 5 min, per limitare l'usura della flash

// Stima cinematica della posizione e controllo di velocità proporzionale
float posH = 0.0f; // Posizione stimata H (0°..360°)
float posV = 0.0f; // Posizione stimata V (-60°..+60°)
const float SPEED_TO_DEG_PER_SEC = 5.0f;
unsigned long lastPosUpdate = 0;
double activeSpeedH = 0;
double activeSpeedV = 0;

// Logging in RAM con esportazione CSV
struct LogSample {
  uint32_t t;
  int16_t errH, errV, pulseH, pulseV, tl, tr, bl, br;
};
const int MAX_LOG_SAMPLES = 1500;
LogSample logBuffer[MAX_LOG_SAMPLES];
int logCount = 0;
bool loggingActive = false;
unsigned int logIntervalMs = 100;
unsigned long logStartMillis = 0;
unsigned long lastLogSampleMillis = 0;

// AUTOTUNING PID — Relay Feedback (Åström–Hägglund)
const double RELAY_AMPLITUDE = 2.0;         // "d": velocità fissa comandata dal relè
const double RELAY_HYSTERESIS = 40.0;       // banda morta di commutazione
const int RELAY_DISCARD_HALFCYCLES = 6;     // scarta il transitorio iniziale 
const int RELAY_COLLECT_HALFCYCLES = 14;    //  ~7 cicli per la stima
const unsigned long RELAY_SAMPLE_INTERVAL_MS = 20;
const unsigned long RELAY_TIMEOUT_MS = 45000; // failsafe

// Stato del relè per un singolo asse (H o V), aggiornato un campione alla volta.
struct RelayAxisState {
  int relayDir = 1;
  double halfCyclePeak = 0;
  unsigned long lastSwitchMs = 0;
  int halfCycleIndex = 0;
  double posPeakSum = 0; int posPeakCount = 0;
  double negPeakSum = 0; int negPeakCount = 0;
  unsigned long periodSumMs = 0; int periodCount = 0;
  bool done = false;
};

// Dichiarazioni Funzioni
int speedToPulseUS(double speed, int stopUS);
void setServoH(double speed);
void setServoV(double speed);
void stopServos();
void powerSaveServos();
void resetPID(PID &pid);
int readLDROversampled(int pin);
void calibrateLDRs();
void loadSettingsFromNVS();
void saveSettingsToNVS();
void relayStep(RelayAxisState &st, double error, unsigned long nowMs, void (*setServo)(double));
bool runRelayAutotune(double &outKp, double &outKi, double &outKd);
void eseguiRicercaSole();
void setupWebServer();
void handleRoot();
void handleApiData();
void handleApiMode();
void handleApiControl();
void handleApiPID();
void handleApiFindSun();
void handleApiCalibrate();
void handleApiLogStart();
void handleApiLogStop();
void handleApiLogStatus();
void handleApiLogCsv();
void handleApiAutotune();

int speedToPulseUS(double speed, int stopUS) {
  if (speed > 0.02) {
    return stopUS - (int)round(45 + speed * 25);
  } else if (speed < -0.02) {
    return stopUS + (int)round(45 + fabs(speed) * 25);
  }
  return stopUS;
}
void setServoH(double speed) {
  if (!servoH.attached()) {
    servoH.attach(pinServoH, 1000, 2000);
    servosPowerSaved = false;
  }
  lastPulseHus = speedToPulseUS(speed, STOP_H_US);
  servoH.writeMicroseconds(lastPulseHus);
  activeSpeedH = speed;
}

// Il servo verticale è montato fisicamente con orientamento invertito rispetto
// a quello orizzontale
void setServoV(double speed) {
  if (!servoV.attached()) {
    servoV.attach(pinServoV, 1000, 2000);
    servosPowerSaved = false;
  }
  lastPulseVus = speedToPulseUS(-speed, STOP_V_US);
  servoV.writeMicroseconds(lastPulseVus);
  activeSpeedV = speed;
}

void stopServos() {
  setServoH(0);
  setServoV(0);
}

// Stacca il segnale PWM dai servo dopo averli portati a zero
void powerSaveServos() {
  if (servosPowerSaved) return;
  stopServos();
  delay(20); // margine perché il servo elabori l'ultimo impulso neutro prima di staccare il PWM
  servoH.detach();
  servoV.detach();
  servosPowerSaved = true;
}

// Azzera il termine integrale e lo storico del PID (transizione MANUAL->AUTOMATIC
// forza una reinizializzazione "bumpless" in PID_v1). Va chiamato ogni volta che il
// tracker si ferma o esce dall'inseguimento continuo, altrimenti l'integrale accumulato
// durante la sosta genererebbe uno scatto  al rientro in modalità AUTO.
void resetPID(PID &pid) {
  pid.SetMode(MANUAL);
  pid.SetMode(AUTOMATIC);
}

// Media di ADC_OVERSAMPLE_COUNT letture consecutive sullo stesso pin, per ridurre
// il rumore residuo dell'ADC dell'ESP32 rispetto a una singola lettura.
int readLDROversampled(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) sum += analogRead(pin);
  return (int)(sum / ADC_OVERSAMPLE_COUNT);
}

// Carica da NVS la taratura PID, le soglie operative, l'abilitazione del punto
// morto e i fattori di calibrazione LDR salvati in una sessione precedente.
void loadSettingsFromNVS() {
  prefs.begin("solartrk", false);
  Kp = (double)prefs.getFloat("kp", 0.5f);
  Ki = (double)prefs.getFloat("ki", 0.0f);
  Kd = (double)prefs.getFloat("kd", 0.05f);
  zonaMorta = prefs.getInt("zonaMorta", 350);
  sogliaPuntoMorto = prefs.getInt("sogliaPM", 700);
  sogliaNotte = prefs.getInt("sogliaNotte", 100);
  puntoMortoAbilitato = prefs.getBool("pmAbil", true);
  calTL = prefs.getFloat("calTL", 1.0f);
  calTR = prefs.getFloat("calTR", 1.0f);
  calBL = prefs.getFloat("calBL", 1.0f);
  calBR = prefs.getFloat("calBR", 1.0f);
  totalEnergyWh = prefs.getFloat("energyWh", 0.0f);
  prefs.end();
}

// Salva su NVS la taratura PID, le soglie operative, l'abilitazione del punto
// morto e i fattori di calibrazione LDR correnti (persistenza tra un riavvio e
// l'altro dell'ESP32).
void saveSettingsToNVS() {
  prefs.begin("solartrk", false);
  prefs.putFloat("kp", (float)Kp);
  prefs.putFloat("ki", (float)Ki);
  prefs.putFloat("kd", (float)Kd);
  prefs.putInt("zonaMorta", zonaMorta);
  prefs.putInt("sogliaPM", sogliaPuntoMorto);
  prefs.putInt("sogliaNotte", sogliaNotte);
  prefs.putBool("pmAbil", puntoMortoAbilitato);
  prefs.putFloat("calTL", calTL);
  prefs.putFloat("calTR", calTR);
  prefs.putFloat("calBL", calBL);
  prefs.putFloat("calBR", calBR);
  prefs.end();
}

/**
 * Calibrazione dei 4 LDR
 */
void calibrateLDRs() {
  const int N = 10;
  long sumTL = 0, sumTR = 0, sumBL = 0, sumBR = 0;
  for (int i = 0; i < N; i++) {
    sumTL += readLDROversampled(ldrTL);
    sumTR += readLDROversampled(ldrTR);
    sumBL += readLDROversampled(ldrBL);
    sumBR += readLDROversampled(ldrBR);
    delay(20);
  }
  float avgTL = sumTL / (float)N, avgTR = sumTR / (float)N;
  float avgBL = sumBL / (float)N, avgBR = sumBR / (float)N;
  float avgAll = (avgTL + avgTR + avgBL + avgBR) / 4.0f;

  calTL = (avgTL > 1.0f) ? (avgAll / avgTL) : 1.0f;
  calTR = (avgTR > 1.0f) ? (avgAll / avgTR) : 1.0f;
  calBL = (avgBL > 1.0f) ? (avgAll / avgBL) : 1.0f;
  calBR = (avgBR > 1.0f) ? (avgAll / avgBR) : 1.0f;

  Serial.printf("[CALIBRAZIONE LDR] TL:%.3f TR:%.3f BL:%.3f BR:%.3f\n", calTL, calTR, calBL, calBR);

  saveSettingsToNVS();
}

// Avanza lo stato del relè di un asse di un campione: aggiorna il picco del
// semiciclo corrente, rileva le commutazioni (con isteresi contro il rumore) e,
// una volta scartato il transitorio iniziale, accumula i picchi e i periodi dei
// semicicli "buoni" per il calcolo finale di Ku/Pu.
void relayStep(RelayAxisState &st, double error, unsigned long nowMs, void (*setServo)(double)) {
  if (st.done) return;

  // Aggiorna l'estremo raggiunto nel semiciclo corrente
  if (st.relayDir > 0) {
    if (error > st.halfCyclePeak) st.halfCyclePeak = error;
  } else {
    if (error < st.halfCyclePeak) st.halfCyclePeak = error;
  }

  // Commutazione del relè: isteresi per non scattare su un attraversamento
  // dello zero dovuto solo a rumore residuo del segnale d'errore.
  bool shouldSwitch = (st.relayDir > 0) ? (error < -RELAY_HYSTERESIS) : (error > RELAY_HYSTERESIS);
  if (shouldSwitch) {
    st.halfCycleIndex++;
    unsigned long duration = nowMs - st.lastSwitchMs;

    if (st.halfCycleIndex > RELAY_DISCARD_HALFCYCLES) {
      // Il semiciclo appena concluso (direzione st.relayDir, prima del flip sotto)
      if (st.relayDir > 0) { st.posPeakSum += st.halfCyclePeak; st.posPeakCount++; }
      else { st.negPeakSum += st.halfCyclePeak; st.negPeakCount++; }
      st.periodSumMs += duration;
      st.periodCount++;
    }

    if (st.halfCycleIndex >= RELAY_DISCARD_HALFCYCLES + RELAY_COLLECT_HALFCYCLES) {
      st.done = true;
    }

    st.relayDir = -st.relayDir;
    st.halfCyclePeak = error;
    st.lastSwitchMs = nowMs;
  }

  setServo(st.relayDir * RELAY_AMPLITUDE);
}

/**
 * Autotuning PID con Relay Feedback (Astrom-Hagglund) su entrambi gli assi in
 * parallelo. Pilota H e V con un relè bang-bang indipendente ciascuno, basato
 * sull'errore grezzo (oversampled+calibrato, non passato dal filtro lento a
 * costante di tempo: servirebbe solo a introdurre ritardo/attenuazione
 * nell'oscillazione che invece va misurata il più fedelmente possibile).
 * Ritorna true e valorizza Kp/Ki/Kd (media tra i due assi, che nel firmware
 * condividono la stessa taratura) se il test converge; false se va in timeout
 * (sistema che non oscilla a sufficienza: relè troppo piccolo, attrito
 * eccessivo, o luce insufficiente).
 */
bool runRelayAutotune(double &outKp, double &outKi, double &outKd) {
  Serial.println("\n==================================================");
  Serial.println("[AUTOTUNE] Avvio relay feedback (Astrom-Hagglund)...");
  Serial.println("==================================================");

  stopServos();
  delay(200);

  RelayAxisState stH, stV;
  stH.lastSwitchMs = stV.lastSwitchMs = millis();

  // Riusa il buffer di log esistente per
  // registrare il transitorio del test, cosi' è ispezionabile via /api/log/csv
  // subito dopo. Il campionamento qui è manuale 
  logCount = 0;
  logStartMillis = millis();
  loggingActive = false;

  unsigned long testStart = millis();
  unsigned long lastSample = 0;

  while (!(stH.done && stV.done)) {
    if (millis() - testStart > RELAY_TIMEOUT_MS) {
      Serial.println("[AUTOTUNE] Timeout: oscillazione insufficiente. Autotuning annullato.");
      stopServos();
      return false;
    }

    unsigned long nowMs = millis();
    if (nowMs - lastSample >= RELAY_SAMPLE_INTERVAL_MS) {
      lastSample = nowMs;

      float tl = readLDROversampled(ldrTL) * calTL;
      float tr = readLDROversampled(ldrTR) * calTR;
      float bl = readLDROversampled(ldrBL) * calBL;
      float br = readLDROversampled(ldrBR) * calBR;
      double errH = (double)((tl + bl) - (tr + br));
      double errV = (double)((tl + tr) - (bl + br));

      relayStep(stH, errH, nowMs, setServoH);
      relayStep(stV, errV, nowMs, setServoV);

      if (logCount < MAX_LOG_SAMPLES) {
        LogSample &s = logBuffer[logCount++];
        s.t = nowMs - logStartMillis;
        s.errH = (int16_t)errH;
        s.errV = (int16_t)errV;
        s.pulseH = (int16_t)lastPulseHus;
        s.pulseV = (int16_t)lastPulseVus;
        s.tl = (int16_t)tl; s.tr = (int16_t)tr; s.bl = (int16_t)bl; s.br = (int16_t)br;
      }
    }

    server.handleClient();
  }

  stopServos();
  delay(200);

  // Ku = 4d/(pi*a), a = semi-ampiezza = (picco max - picco min)/2,
  // Pu = periodo medio dell'oscillazione completa
  auto computeKuPu = [](RelayAxisState &st, double &ku, double &pu) {
    double avgPos = st.posPeakCount > 0 ? (st.posPeakSum / st.posPeakCount) : 0;
    double avgNeg = st.negPeakCount > 0 ? (st.negPeakSum / st.negPeakCount) : 0;
    double a = (avgPos - avgNeg) / 2.0;
    pu = st.periodCount > 0 ? (2.0 * (st.periodSumMs / (double)st.periodCount) / 1000.0) : 0;
    ku = (a > 1.0) ? (4.0 * RELAY_AMPLITUDE / (PI * a)) : 0;
  };

  double kuH, puH, kuV, puV;
  computeKuPu(stH, kuH, puH);
  computeKuPu(stV, kuV, puV);

  Serial.printf("[AUTOTUNE] Asse H -> Ku:%.4f Pu:%.2fs | Asse V -> Ku:%.4f Pu:%.2fs\n", kuH, puH, kuV, puV);

  if (kuH <= 0 || puH <= 0 || kuV <= 0 || puV <= 0) {
    Serial.println("[AUTOTUNE] Dati insufficienti per calcolare Ku/Pu. Autotuning annullato.");
    return false;
  }

  double Ku = (kuH + kuV) / 2.0;
  double Pu = (puH + puV) / 2.0;

  // Ziegler-Nichols ad anello chiuso
  outKp = 0.6 * Ku;
  outKi = 2.0 * outKp / Pu;
  outKd = outKp * Pu / 8.0;

  Serial.printf("[AUTOTUNE] Ku:%.4f Pu:%.2fs -> Kp:%.3f Ki:%.3f Kd:%.3f\n", Ku, Pu, outKp, outKi, outKd);
  Serial.println("==================================================\n");
  return true;
}


void eseguiRicercaSole() {
  Serial.println("\n==================================================");
  Serial.println("☀️ [SUN FINDER] AVVIO PROCEDURA RICERCA SOLE");
  Serial.println("==================================================");

  isSearchingSun = true;
  stopServos();
  delay(200);

  // 1. Lettura iniziale sensori (oversampled e calibrati)
  int ldr1 = (int)(readLDROversampled(ldrTL) * calTL);
  int ldr2 = (int)(readLDROversampled(ldrTR) * calTR);
  int ldr3 = (int)(readLDROversampled(ldrBL) * calBL);
  int ldr4 = (int)(readLDROversampled(ldrBR) * calBR);
  int maxIniziale = max(max(ldr1, ldr2), max(ldr3, ldr4));
  int mediaIniziale = (ldr1 + ldr2 + ldr3 + ldr4) / 4;

  Serial.printf("[SUN FINDER] Lettura iniziale -> Media: %d | Max: %d | Soglia Notte: %d\n", 
                mediaIniziale, maxIniziale, sogliaNotte);

  // Se la luminosità è sotto la soglia notte, non ruotare a vuoto
  if (maxIniziale < sogliaNotte) {
    Serial.println("[SUN FINDER] Luminosita' ambientale insufficiente (< sogliaNotte). Tracker in standby.");
    stopServos();
    isSearchingSun = false;
    return;
  }

  // 2. Scansione Orizzontale a 360° 
  const int searchSpeed = 4;
  const unsigned long SCAN_DURATION_MS = 6000;
  const unsigned long SAMPLE_INTERVAL_MS = 50;

  int maxLightFound = 0;
  unsigned long timeOfMaxLight = 0;
  unsigned long startScan = millis();

  Serial.println("[SUN FINDER] Inizio scansione orizzontale a 360 gradi...");
  setServoH(searchSpeed);

  while (millis() - startScan < SCAN_DURATION_MS) {
    unsigned long elapsed = millis() - startScan;
    
    int tl = (int)(readLDROversampled(ldrTL) * calTL);
    int tr = (int)(readLDROversampled(ldrTR) * calTR);
    int bl = (int)(readLDROversampled(ldrBL) * calBL);
    int br = (int)(readLDROversampled(ldrBR) * calBR);
    int currentAvg = (tl + tr + bl + br) / 4;

    if (currentAvg > maxLightFound) {
      maxLightFound = currentAvg;
      timeOfMaxLight = elapsed;
    }

    server.handleClient();
    delay(SAMPLE_INTERVAL_MS);
  }

  stopServos();
  delay(250);

  Serial.printf("[SUN FINDER] Scansione terminata! Picco max: %d a t=%lu ms\n", 
                maxLightFound, timeOfMaxLight);

  // 3. Ritorno orientato verso la posizione con massima luce
  if (maxLightFound > sogliaNotte) {
    unsigned long returnTime = SCAN_DURATION_MS - timeOfMaxLight;
    Serial.printf("[SUN FINDER] Ritorno indietro verso la posizione del sole per %lu ms...\n", returnTime);

    setServoH(-searchSpeed);
    unsigned long startReturn = millis();

    while (millis() - startReturn < returnTime) {
      int tl = (int)(readLDROversampled(ldrTL) * calTL);
      int tr = (int)(readLDROversampled(ldrTR) * calTR);
      int bl = (int)(readLDROversampled(ldrBL) * calBL);
      int br = (int)(readLDROversampled(ldrBR) * calBR);
      int cur = (tl + tr + bl + br) / 4;

      // Se riagganciamo il picco con anticipo (>= 95%), blocca per massima precisione
      if (cur >= (int)(maxLightFound * 0.95)) {
        Serial.println("[SUN FINDER] Picco luminoso riagganciato con precisione!");
        break;
      }

      server.handleClient();
      delay(30);
    }
  }

  stopServos();
  delay(200);

  // 4. Regolazione zenitale verticale iniziale (inclinazione verso l'alto)
  Serial.println("[SUN FINDER] Ottimizzazione inclinazione zenitale iniziale...");
  setServoV(2);
  delay(350);
  stopServos();

  Serial.println("☀️ [SUN FINDER] Sole agganciato! Modalita' automatica attiva.");
  Serial.println("==================================================\n");

  isSearchingSun = false;
  currentMode = MODE_AUTO;
  modeString = "auto";
  // Riparte con PID puliti: niente integrale ereditato dalla scansione manuale appena eseguita.
  resetPID(pidH);
  resetPID(pidV);
  pidActiveH = false;
  pidActiveV = false;
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n--- Avvio Solar Tracker ESP32 ---");

  // Inizializzazione Servi
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  servoH.setPeriodHertz(50);
  servoV.setPeriodHertz(50);
  servoH.attach(pinServoH, 1000, 2000);
  servoV.attach(pinServoV, 1000, 2000);
  
  // Imposta i servi fermi all'avvio (1500us neutro)
  stopServos();

  // Ripristino da NVS di taratura PID, soglie, calibrazione LDR ed energia
  // accumulata, PRIMA di applicare i guadagni al PID cosi' parte subito con i
  // valori corretti invece dei default.
  loadSettingsFromNVS();
  lastEnergySaveMs = millis();
  Serial.printf("Impostazioni ripristinate da NVS -> Kp:%.2f Ki:%.2f Kd:%.2f Deadzone:%d Energia:%.2fWh\n",
                Kp, Ki, Kd, zonaMorta, totalEnergyWh);

  // Inizializzazione Controller PID
  pidH.SetMode(AUTOMATIC);
  pidV.SetMode(AUTOMATIC);
  pidH.SetOutputLimits(-maxAutoSpeed, maxAutoSpeed);
  pidV.SetOutputLimits(-maxAutoSpeed, maxAutoSpeed);
  pidH.SetTunings(Kp, Ki, Kd);
  pidV.SetTunings(Kp, Ki, Kd);
  pidH.SetSampleTime(50); // Ricalcolo PID ogni 50ms per un inseguimento reattivo
  pidV.SetSampleTime(50);

  // Configurazione Pin Analogici
  pinMode(ldrTL, INPUT);
  pinMode(ldrTR, INPUT);
  pinMode(ldrBL, INPUT);
  pinMode(ldrBR, INPUT);
  pinMode(pinSolarVolt, INPUT);

  // Configurazione Wi-Fi SoftAP
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress apIP = WiFi.softAPIP();
  
  Serial.print("Access Point Wi-Fi Creato!");
  Serial.print(" SSID: "); Serial.println(ap_ssid);
  Serial.print(" IP per collegarsi da Smartphone: "); Serial.println(apIP);

  // Configurazione Web Server
  setupWebServer();
  server.begin();
  Serial.println("Web Server avviato sulla porta 80!");

  lastEnergyCalc = millis();

  // Esecuzione Ricerca Sole all'Avvio
  eseguiRicercaSole();
}

void loop() {
  // Gestione delle richieste dagli smartphone collegati al WebServer
  server.handleClient();

  // 0. Aggiornamento cinematica posizione virtuale e velocità H/V
  unsigned long nowPos = millis();
  float dt = (nowPos - lastPosUpdate) / 1000.0f;
  if (dt > 1.0f) dt = 0.01f;
  lastPosUpdate = nowPos;

  if (activeSpeedH != 0) {
    posH += activeSpeedH * SPEED_TO_DEG_PER_SEC * dt;
    while (posH >= 360.0f) posH -= 360.0f;
    while (posH < 0.0f) posH += 360.0f;
  }
  if (activeSpeedV != 0) {
    posV += activeSpeedV * SPEED_TO_DEG_PER_SEC * dt;
    if (posV > 60.0f) posV = 60.0f;
    if (posV < -60.0f) posV = -60.0f;
  }

  // 1. Lettura Sensori LDR  e Calcolo Diagonali
  float rawTL = readLDROversampled(ldrTL) * calTL;
  float rawTR = readLDROversampled(ldrTR) * calTR;
  float rawBL = readLDROversampled(ldrBL) * calBL;
  float rawBR = readLDROversampled(ldrBR) * calBR;

  if (!ldrFilterInit) {
    // Al primo giro il filtro parte dal valore letto, non da 0, per non introdurre
    // un transitorio artificiale all'avvio.
    filtTL = rawTL; filtTR = rawTR; filtBL = rawBL; filtBR = rawBR;
    ldrFilterInit = true;
  } else {
    // Alpha derivato dalla costante di tempo reale (dt gia' calcolato sopra per la
    // cinematica), non fisso: cosi' il filtro resta coerente anche ora che il loop
    // non ha più un delay(10) fisso a fine ciclo (vedi fondo di loop()).
    float alpha = dt / (LDR_FILTER_TAU_S + dt);
    filtTL += alpha * (rawTL - filtTL);
    filtTR += alpha * (rawTR - filtTR);
    filtBL += alpha * (rawBL - filtBL);
    filtBR += alpha * (rawBR - filtBR);
  }
  valTL = (int)round(filtTL);
  valTR = (int)round(filtTR);
  valBL = (int)round(filtBL);
  valBR = (int)round(filtBR);

  valDiag1 = valTL + valBR; // Diagonale 1: Top-Left + Bottom-Right
  valDiag2 = valTR + valBL; // Diagonale 2: Top-Right + Bottom-Left
  diffDiagonali = valDiag1 - valDiag2; // Differenza tra le due diagonali

  int mediaTop = (valTL + valTR) / 2;
  int mediaBottom = (valBL + valBR) / 2;
  mediaTotale = (mediaTop + mediaBottom) / 2;

  // 2. Lettura e Calcolo Rendimento Solare (Voltage Divider / Sensore)
  int rawADC = analogRead(pinSolarVolt);
  // Lettura reale della tensione solare dal pin (Partitore R1=10k, R2=10k)
  solarVoltage = (rawADC / 4095.0) * 3.3 * 2.0; 
  
  // Se la tensione letta è trascurabile (< 0.1V), considera il pannello scollegato (0V, 0mA)
  if (solarVoltage < 0.1) {
    solarVoltage = 0.0;
    solarCurrent = 0.0;
  } else {
    // Stima della corrente erogata in base alla luminosità (max 500mA per il pannello)
    solarCurrent = (mediaTotale / 4095.0) * 500.0;
  }
  
  solarPower = (solarVoltage * solarCurrent) / 1000.0; // In Watt

  // Calcolo accumulo energia (Wh)
  unsigned long now = millis();
  float deltaHours = (now - lastEnergyCalc) / 3600000.0;
  lastEnergyCalc = now;
  totalEnergyWh += solarPower * deltaHours;

  // Salvataggio periodico in NVS: non a ogni ciclo per non consumare inutilmente
  // i cicli di scrittura della flash, ma abbastanza spesso da non perdere troppo
  // in caso di spegnimento improvviso.
  if (now - lastEnergySaveMs >= ENERGY_SAVE_INTERVAL_MS) {
    lastEnergySaveMs = now;
    prefs.begin("solartrk", false);
    prefs.putFloat("energyWh", totalEnergyWh);
    prefs.end();
  }

  // 3. Esecuzione Logica in base alla Modalità Selezionata da Smartphone
  switch (currentMode) {

    case MODE_AUTO: {
      // Calcolo Errori Assiali Primari (Orizzontale H e Verticale V)
      // rawH: (Sinistra) - (Destra) = (valTL + valBL) - (valTR + valBR)
      // rawV: (Alto) - (Basso) = (valTL + valTR) - (valBL + valBR)
      double rawH = (double)((valTL + valBL) - (valTR + valBR));
      double rawV = (double)((valTL + valTR) - (valBL + valBR));

      // Trova il sensore più luminoso (maxLDR), usato per il controllo notte
      int maxLDR = max(max(valTL, valTR), max(valBL, valBR));

      // Controllo Notte (se anche il sensore più luminoso è sotto sogliaNotte, spegne i motori)
      if (maxLDR < sogliaNotte) {
        powerSaveServos();
        inputH = 0;
        inputV = 0;
        outputH = 0;
        outputV = 0;
        puntoMortoAttivo = false;
        pidActiveH = false;
        pidActiveV = false;
        break;
      }

      // 4. LOGICA RISOLUZIONE PUNTO MORTO DIAGONALE:
      // Se l'errore H e V sono a zero (o entro la zona morta),
      // MA la differenza tra le diagonali è altissima (|diffDiagonali| >= sogliaPuntoMorto),
      // siamo nel punto morto : spostiamo la posizione per sbloccare il sistema.
      bool errHZero = (abs(rawH) <= zonaMorta);
      bool errVZero = (abs(rawV) <= zonaMorta);
      bool diagAltissima = (abs(diffDiagonali) >= sogliaPuntoMorto);

      if (puntoMortoAbilitato && errHZero && errVZero && diagAltissima) {
          puntoMortoAttivo = true;

        int escapeSpeedH = 0;
        int escapeSpeedV = 0;

        if (diffDiagonali > 0) {
          // Diagonale 1 (TL + BR) dominante su Diagonale 2 (TR + BL)
          if (valTL >= valBR) {
            // Predilige Top-Left (+H, +V)
            escapeSpeedH = maxAutoSpeed;
            escapeSpeedV = maxAutoSpeed;
          } else {
            // Predilige Bottom-Right (-H, -V)
            escapeSpeedH = -maxAutoSpeed;
            escapeSpeedV = -maxAutoSpeed;
          }
        } else {
          // Diagonale 2 (TR + BL) dominante su Diagonale 1 (TL + BR)
          if (valTR >= valBL) {
            // Predilige Top-Right (-H, +V)
            escapeSpeedH = -maxAutoSpeed;
            escapeSpeedV = maxAutoSpeed;
          } else {
            // Predilige Bottom-Left (+H, -V)
            escapeSpeedH = maxAutoSpeed;
            escapeSpeedV = -maxAutoSpeed;
          }
        }

        // Failsafe nel caso di perfetta simmetria diagonale: impulso di sblocco
        if (escapeSpeedH == 0 && escapeSpeedV == 0) {
          escapeSpeedH = (diffDiagonali > 0) ? maxAutoSpeed : -maxAutoSpeed;
          escapeSpeedV = maxAutoSpeed;
        }

        setServoH(escapeSpeedH);
        setServoV(escapeSpeedV);
        outputH = escapeSpeedH;
        outputV = escapeSpeedV;
        inputH = rawH;
        inputV = rawV;
        pidActiveH = false;
        pidActiveV = false;
      } else {
        puntoMortoAttivo = false;

        inputH = rawH;
        inputV = rawV;

        double parkThresholdH = zonaMorta * DEADZONE_HYSTERESIS_RATIO;
        bool shouldParkH = pidActiveH ? (abs(inputH) <= parkThresholdH) : (abs(inputH) <= zonaMorta);
        if (shouldParkH) {
          outputH = 0;
          if (pidActiveH) { resetPID(pidH); pidActiveH = false; }
        } else {
          if (!pidActiveH) { resetPID(pidH); pidActiveH = true; }
          pidH.Compute();
        }
        setServoH(outputH);

        // Asse Verticale: stessa logica con isteresi dell'asse orizzontale.
        double parkThresholdV = zonaMorta * DEADZONE_HYSTERESIS_RATIO;
        bool shouldParkV = pidActiveV ? (abs(inputV) <= parkThresholdV) : (abs(inputV) <= zonaMorta);
        if (shouldParkV) {
          outputV = 0;
          if (pidActiveV) { resetPID(pidV); pidActiveV = false; }
        } else {
          if (!pidActiveV) { resetPID(pidV); pidActiveV = true; }
          pidV.Compute();
        }
        setServoV(outputV);
      }
      break;
    }

    case MODE_MANUAL: {
      puntoMortoAttivo = false;
      if (millis() - lastManualCmdMillis > MANUAL_TIMEOUT_MS) {
        stopServos();
        manualVelH = 0;
        manualVelV = 0;
        currentMode = MODE_AUTO;
        modeString = "auto";
        inputH = 0;
        inputV = 0;
        outputH = 0;
        outputV = 0;
        resetPID(pidH);
        resetPID(pidV);
        pidActiveH = false;
        pidActiveV = false;
      } else {
        setServoH(manualVelH);
        setServoV(manualVelV);
        inputH = 0;
        inputV = 0;
        outputH = manualVelH;
        outputV = manualVelV;
      }
      break;
    }

    case MODE_NIGHT: {
      puntoMortoAttivo = false;
      powerSaveServos();
      inputH = 0;
      inputV = 0;
      outputH = 0;
      outputV = 0;
      break;
    }

    case MODE_ECO: {
      puntoMortoAttivo = false;
      powerSaveServos();
      inputH = 0;
      inputV = 0;
      outputH = 0;
      outputV = 0;
      break;
    }

    case MODE_SEARCH_SUN: {
      eseguiRicercaSole();
      break;
    }

    case MODE_AUTOTUNE: {
      isAutotuning = true;
      double newKp, newKi, newKd;
      if (runRelayAutotune(newKp, newKi, newKd)) {
        Kp = constrain(newKp, 0.0, 50.0);
        Ki = constrain(newKi, 0.0, 50.0);
        Kd = constrain(newKd, 0.0, 50.0);
        pidH.SetTunings(Kp, Ki, Kd);
        pidV.SetTunings(Kp, Ki, Kd);
        saveSettingsToNVS();
      }
      isAutotuning = false;
      currentMode = MODE_AUTO;
      modeString = "auto";
      resetPID(pidH);
      resetPID(pidV);
      pidActiveH = false;
      pidActiveV = false;
      break;
    }
  }

  // Campionamento log CSV in RAM, a intervallo configurabile
  // indipendente dalla velocità del loop.
  if (loggingActive) {
    unsigned long tNow = millis();
    if (tNow - lastLogSampleMillis >= logIntervalMs) {
      lastLogSampleMillis = tNow;
      if (logCount < MAX_LOG_SAMPLES) {
        LogSample &s = logBuffer[logCount++];
        s.t = tNow - logStartMillis;
        s.errH = (int16_t)inputH;
        s.errV = (int16_t)inputV;
        s.pulseH = (int16_t)lastPulseHus;
        s.pulseV = (int16_t)lastPulseVus;
        s.tl = (int16_t)valTL;
        s.tr = (int16_t)valTR;
        s.bl = (int16_t)valBL;
        s.br = (int16_t)valBR;
      } else {
        loggingActive = false; // buffer pieno, stop automatico
      }
    }
  }

  // Monitor Seriale Debug (ogni 500ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("[Mode: %s%s] LDR(TL/TR/BL/BR): %d/%d/%d/%d | Diag1/Diag2/Diff: %d/%d/%d | Volt: %.1fV | Watt: %.2fW\n",
                  modeString.c_str(),
                  puntoMortoAttivo ? " (PUNTO MORTO!)" : "",
                  valTL, valTR, valBL, valBR,
                  valDiag1, valDiag2, diffDiagonali,
                  solarVoltage, solarPower);
  }
}

// Setup Rotte WebServer
void setupWebServer() {
  // Pagina Principale HTML5
  server.on("/", HTTP_GET, handleRoot);

  // API REST Telemetria in tempo reale JSON
  server.on("/api/data", HTTP_GET, handleApiData);

  // API Cambia Modalità (mutante: solo POST)
  server.on("/api/mode", HTTP_POST, handleApiMode);

  // API Comandi Manuali D-Pad (mutante: solo POST)
  server.on("/api/control", HTTP_POST, handleApiControl);

  // API Aggiornamento Taratura PID, Soglie e Deadlock (mutante: solo POST)
  server.on("/api/pid", HTTP_POST, handleApiPID);

  // API Trigger Ricerca Sole On-Demand (mutante: solo POST)
  server.on("/api/findsun", HTTP_POST, handleApiFindSun);

  // API Calibrazione dei 4 LDR sotto luce uniforme (mutante: solo POST)
  server.on("/api/calibrate", HTTP_POST, handleApiCalibrate);

  // API Logging CSV in RAM: avvio/stop (mutanti, POST) e lettura stato/export (GET)
  server.on("/api/log/start", HTTP_POST, handleApiLogStart);
  server.on("/api/log/stop", HTTP_POST, handleApiLogStop);
  server.on("/api/log/status", HTTP_GET, handleApiLogStatus);
  server.on("/api/log/csv", HTTP_GET, handleApiLogCsv);

  // API Autotuning PID (Relay Feedback) On-Demand (mutante: solo POST)
  server.on("/api/autotune", HTTP_POST, handleApiAutotune);

  // Gestione errore 404
  server.onNotFound([]() {
    server.send(404, "text/plain", "404: Not Found");
  });
}

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleApiData() {
  if (currentMode == MODE_MANUAL) {
    lastManualCmdMillis = millis();
  }

  char jsonBuffer[900];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{"
      "\"v\":%.2f,"
      "\"i\":%.0f,"
      "\"p\":%.2f,"
      "\"energy\":%.2f,"
      "\"tl\":%d,"
      "\"tr\":%d,"
      "\"bl\":%d,"
      "\"br\":%d,"
      "\"diag1\":%d,"
      "\"diag2\":%d,"
      "\"diffDiag\":%d,"
      "\"deadlock\":%s,"
      "\"findingSun\":%s,"
      "\"autotuning\":%s,"
      "\"powerSaved\":%s,"
      "\"errH\":%.0f,"
      "\"errV\":%.0f,"
      "\"velH\":%d,"
      "\"velV\":%d,"
      "\"kp\":%.2f,"
      "\"ki\":%.2f,"
      "\"kd\":%.2f,"
      "\"deadzone\":%d,"
      "\"deadlockThresh\":%d,"
      "\"deadlockEnabled\":%s,"
      "\"night\":%d,"
      "\"calTL\":%.3f,"
      "\"calTR\":%.3f,"
      "\"calBL\":%.3f,"
      "\"calBR\":%.3f,"
      "\"logging\":%s,"
      "\"logSamples\":%d,"
      "\"logMaxSamples\":%d,"
      "\"mode\":\"%s\","
      "\"uptime\":%lu,"
      "\"heap\":%u"
    "}",
    solarVoltage,
    solarCurrent,
    solarPower,
    totalEnergyWh,
    valTL, valTR, valBL, valBR,
    valDiag1, valDiag2, diffDiagonali,
    puntoMortoAttivo ? "true" : "false",
    isSearchingSun ? "true" : "false",
    isAutotuning ? "true" : "false",
    servosPowerSaved ? "true" : "false",
    inputH, inputV,
    lastPulseHus,
    lastPulseVus,
    Kp, Ki, Kd,
    zonaMorta, sogliaPuntoMorto,
    puntoMortoAbilitato ? "true" : "false",
    sogliaNotte,
    calTL, calTR, calBL, calBR,
    loggingActive ? "true" : "false",
    logCount, MAX_LOG_SAMPLES,
    modeString.c_str(),
    millis() / 1000,
    ESP.getFreeHeap()
  );
  server.send(200, "application/json", jsonBuffer);
}

void handleApiMode() {
  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    resetPID(pidH);
    resetPID(pidV);
    pidActiveH = false;
    pidActiveV = false;
    if (m == "auto") {
      currentMode = MODE_AUTO;
      modeString = "auto";
      manualVelH = 0;
      manualVelV = 0;
    } else if (m == "manual") {
      currentMode = MODE_MANUAL;
      modeString = "manual";
      lastManualCmdMillis = millis();
    } else if (m == "night") {
      currentMode = MODE_NIGHT;
      modeString = "night";
      manualVelH = 0;
      manualVelV = 0;
    } else if (m == "eco") {
      currentMode = MODE_ECO;
      modeString = "eco";
      manualVelH = 0;
      manualVelV = 0;
    } else if (m == "findsun") {
      currentMode = MODE_SEARCH_SUN;
      modeString = "findsun";
      manualVelH = 0;
      manualVelV = 0;
    }
    char jsonBuffer[96];
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"status\":\"ok\",\"mode\":\"%s\"}", modeString.c_str());
    server.send(200, "application/json", jsonBuffer);
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing mode\"}");
  }
}

void handleApiControl() {
  if (server.hasArg("cmd")) {
    String cmd = server.arg("cmd");
    currentMode = MODE_MANUAL;
    modeString = "manual";
    lastManualCmdMillis = millis(); 

    if (cmd == "up") {
      manualVelV = 12; // Ruota in alto
      manualVelH = 0;
    } else if (cmd == "down") {
      manualVelV = -12; // Ruota in basso
      manualVelH = 0;
    } else if (cmd == "left") {
      manualVelH = -12; // Ruota a sinistra
      manualVelV = 0;
    } else if (cmd == "right") {
      manualVelH = 12; // Ruota a destra
      manualVelV = 0;
    } else if (cmd == "stop") {
      manualVelH = 0;
      manualVelV = 0;
    }
    char jsonBuffer[96];
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"status\":\"ok\",\"cmd\":\"%s\"}", cmd.c_str());
    server.send(200, "application/json", jsonBuffer);
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing cmd\"}");
  }
}

void handleApiPID() {
  for (int i = 0; i < server.args(); i++) {
    String name = server.argName(i);
    String val = server.arg(i);
    if (name == "kp") Kp = val.toDouble();
    else if (name == "ki") Ki = val.toDouble();
    else if (name == "kd") Kd = val.toDouble();
    else if (name == "deadzone") zonaMorta = val.toInt();
    else if (name == "deadlock") sogliaPuntoMorto = val.toInt();
    else if (name == "night") sogliaNotte = val.toInt();
    else if (name == "pmEnabled") puntoMortoAbilitato = (val == "1" || val == "true");
  }
  Kp = constrain(Kp, 0.0, 50.0);
  Ki = constrain(Ki, 0.0, 50.0);
  Kd = constrain(Kd, 0.0, 50.0);
  zonaMorta = constrain(zonaMorta, 0, 4095);
  sogliaPuntoMorto = constrain(sogliaPuntoMorto, 0, 8190);
  sogliaNotte = constrain(sogliaNotte, 0, 4095);

  pidH.SetTunings(Kp, Ki, Kd);
  pidV.SetTunings(Kp, Ki, Kd);

  saveSettingsToNVS();

  Serial.printf("Nuovi Parametri -> Kp: %.2f | Ki: %.2f | Kd: %.2f | Deadzone: %d | SogliaDeadlock: %d | SogliaNotte: %d | PuntoMorto: %s\n",
                Kp, Ki, Kd, zonaMorta, sogliaPuntoMorto, sogliaNotte, puntoMortoAbilitato ? "on" : "off");

  char jsonBuffer[300];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"status\":\"ok\",\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"deadzone\":%d,\"deadlock\":%d,\"deadlockEnabled\":%s,\"night\":%d}",
    Kp, Ki, Kd, zonaMorta, sogliaPuntoMorto, puntoMortoAbilitato ? "true" : "false", sogliaNotte
  );
  server.send(200, "application/json", jsonBuffer);
}

void handleApiFindSun() {
  currentMode = MODE_SEARCH_SUN;
  modeString = "findsun";
  server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Ricerca sole avviata\"}");
}

void handleApiCalibrate() {
  calibrateLDRs();
  char jsonBuffer[160];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"status\":\"ok\",\"calTL\":%.3f,\"calTR\":%.3f,\"calBL\":%.3f,\"calBR\":%.3f}",
    calTL, calTR, calBL, calBR
  );
  server.send(200, "application/json", jsonBuffer);
}

void handleApiAutotune() {
  currentMode = MODE_AUTOTUNE;
  modeString = "autotune";
  server.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Autotuning PID avviato (relay feedback)\"}");
}

void handleApiLogStart() {
  if (server.hasArg("interval")) {
    int iv = server.arg("interval").toInt();
    if (iv > 0) logIntervalMs = (unsigned int)iv;
  }
  logCount = 0;
  logStartMillis = millis();
  lastLogSampleMillis = 0;
  loggingActive = true;
  char jsonBuffer[96];
  snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"status\":\"ok\",\"logging\":true,\"intervalMs\":%u}", logIntervalMs);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiLogStop() {
  loggingActive = false;
  char jsonBuffer[64];
  snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"status\":\"ok\",\"logging\":false}");
  server.send(200, "application/json", jsonBuffer);
}

void handleApiLogStatus() {
  char jsonBuffer[128];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"logging\":%s,\"samples\":%d,\"maxSamples\":%d,\"intervalMs\":%u}",
    loggingActive ? "true" : "false", logCount, MAX_LOG_SAMPLES, logIntervalMs
  );
  server.send(200, "application/json", jsonBuffer);
}

void handleApiLogCsv() {
  server.sendHeader("Content-Disposition", "attachment; filename=solar_log.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("t_ms,errH,errV,pulseH_us,pulseV_us,tl,tr,bl,br\r\n");
  char row[140];
  for (int i = 0; i < logCount; i++) {
    LogSample &s = logBuffer[i];
    snprintf(row, sizeof(row), "%lu,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
             (unsigned long)s.t, s.errH, s.errV, s.pulseH, s.pulseV, s.tl, s.tr, s.bl, s.br);
    server.sendContent(row);
  }
  server.sendContent("");
}
