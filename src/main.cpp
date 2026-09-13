#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <HTTPClient.h>
#include "webpage.h"

// Modalità di Funzionamento
enum OperatingMode {
  MODE_AUTO,
  MODE_MANUAL,
  MODE_AUTOTUNE,
  MODE_DOE
};

OperatingMode currentMode = MODE_AUTO;
String modeString = "auto";

// Configurazione Access Point Wi-Fi
const char* ap_ssid = "SolarTracker-ESP32";
const char* ap_password = "12345678";

const char* sta_ssid = "Vodafone-A88177131";
const char* sta_password = "fzp8cufl6yftdu63";
const char* thingspeakApiKey = "ARCV8FVL0QVK370W";
const unsigned long THINGSPEAK_INTERVAL_MS = 30000UL;
unsigned long lastThingspeakSendMillis = 0;
bool thingspeakLastSendOk = false;

WebServer server(80);

// Pin Servo 360 Continuous
const int pinServoH = 21; //21
const int pinServoV = 18; //18

Servo servoH;
Servo servoV;

// Valori di stop per servi 360 in microsecondi
int STOP_H_US = 1500; 
int STOP_V_US = 1500; 

// Pin Sensori LDR 
const int ldrTL = 35; // Top Left (Alto SX)
const int ldrTR = 33; // Top Right (Alto DX)
const int ldrBL = 32; // Bottom Left (Basso SX)
const int ldrBR = 34; // Bottom Right (Basso DX)

// Pin Lettura Tensione Pannello Solare
const int pinSolarVolt = 36;

const int pinSDA = 4;
const int pinSCL = 5;

uint8_t inaAddrPanel = 0x40;
uint8_t inaAddrLoad = 0x41;
Adafruit_INA219 *inaPanel = nullptr;
Adafruit_INA219 *inaLoad = nullptr;
bool inaPanelOk = false;
bool inaLoadOk = false;

float inaPanelV = 0.0f, inaPanelI_mA = 0.0f, inaPanelP_mW = 0.0f;
float inaLoadV = 0.0f, inaLoadI_mA = 0.0f, inaLoadP_mW = 0.0f;
float realEnergyWh = 0.0f;
float efficienzaPercent = 0.0f;

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


bool puntoMortoAbilitato = true;

const float DEADZONE_HYSTERESIS_RATIO = 0.6f;

// Velocità ridotta in AUTO per test 
int maxAutoSpeed = 3; // Limite di velocità per rotazione fluida e precisa in AUTO

// Variabili per Controllo Manuale
int manualVelH = 0; // -15 a +15
int manualVelV = 0; // -15 a +15

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
bool isAutotuning = false;     // True durante il test di autotuning PID (relay feedback)

const float LDR_FILTER_TAU_S = 0.15f;
float filtTL = 0, filtTR = 0, filtBL = 0, filtBR = 0;
bool ldrFilterInit = false;

// Fattori di calibrazione per compensare le differenze di sensibilità tra i 4 LDR
float calTL = 1.0f, calTR = 1.0f, calBL = 1.0f, calBR = 1.0f;

const int ADC_OVERSAMPLE_COUNT = 4;

// Risparmio energetico
bool servosPowerSaved = false;

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

const float V_ANGLE_MIN = -60.0f;
const float V_ANGLE_MAX = 60.0f;
bool vAxisAtLimit = false;

// Logging in RAM con esportazione CSV
struct LogSample {
  uint32_t t;
  int16_t errH, errV, pulseH, pulseV, tl, tr, bl, br;
  int16_t vPanel_mV, iPanel_mA, vLoad_mV, iLoad_mA;
};
const int MAX_LOG_SAMPLES = 1500;
LogSample logBuffer[MAX_LOG_SAMPLES];
int logCount = 0;
bool loggingActive = false;
unsigned int logIntervalMs = 100;
unsigned long logStartMillis = 0;
unsigned long lastLogSampleMillis = 0;

const double RELAY_AMPLITUDE = 2.0;         // "d": velocità fissa comandata dal relè
const double RELAY_HYSTERESIS = 40.0;       // banda morta di commutazione
const int RELAY_DISCARD_HALFCYCLES = 6;     // scarta il transitorio iniziale 
const int RELAY_COLLECT_HALFCYCLES = 14;    //  ~7 cicli per la stima
const unsigned long RELAY_SAMPLE_INTERVAL_MS = 20;
const unsigned long RELAY_TIMEOUT_MS = 45000; // failsafe

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

struct DoeRun {
  int run;
  int deadzone;
  int replica;
  String filename;
};
const int MAX_DOE_RUNS = 60;
DoeRun doeRuns[MAX_DOE_RUNS];
int doeRunCount = 0;
int doeCurrentIndex = -1;
bool doeActive = false;
bool doeRunReady = false;
unsigned long doeRunDurationMs = 60000;
unsigned long doeRunStartMillis = 0;
bool doeKicking = false;
unsigned long doeKickStartMillis = 0;
const int DOE_KICK_SPEED = 8;
const unsigned long DOE_KICK_DURATION_MS = 2000;

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
void runAutoTracking();
void initINA();
void readINASensors();
void doeStartKick();
void doeBeginLogging(int idx);
bool doeGuard();
void setupWebServer();
void handleRoot();
void handleApiData();
void handleApiMode();
void handleApiControl();
void handleApiPID();
void handleApiCalibrate();
void handleApiResetPos();
void handleApiLogStart();
void handleApiLogStop();
void handleApiLogStatus();
void handleApiLogCsv();
void handleApiAutotune();
void handleApiInaConfig();
void handleApiInaScan();
void handleApiDoeUpload();
void handleApiDoeStart();
void handleApiDoeNext();
void handleApiDoeStop();
void handleApiDoeStatus();
void sendToThingSpeak();

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
  vAxisAtLimit = false;
  if (speed > 0 && posV >= V_ANGLE_MAX) {
    speed = 0;
    vAxisAtLimit = true;
  } else if (speed < 0 && posV <= V_ANGLE_MIN) {
    speed = 0;
    vAxisAtLimit = true;
  }
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


void resetPID(PID &pid) {
  pid.SetMode(MANUAL);
  pid.SetMode(AUTOMATIC);
}

int readLDROversampled(int pin) {
  long sum = 0;
  for (int i = 0; i < ADC_OVERSAMPLE_COUNT; i++) sum += analogRead(pin);
  return (int)(sum / ADC_OVERSAMPLE_COUNT);
}

void initINA() {
  if (inaPanel) { delete inaPanel; inaPanel = nullptr; }
  if (inaLoad) { delete inaLoad; inaLoad = nullptr; }
  inaPanel = new Adafruit_INA219(inaAddrPanel);
  inaLoad = new Adafruit_INA219(inaAddrLoad);
  inaPanelOk = inaPanel->begin(&Wire);
  inaLoadOk = inaLoad->begin(&Wire);
}

void readINASensors() {
  if (inaPanelOk) {
    inaPanelV = inaPanel->getBusVoltage_V() + (inaPanel->getShuntVoltage_mV() / 1000.0f);
    inaPanelI_mA = inaPanel->getCurrent_mA();
    inaPanelP_mW = inaPanel->getPower_mW();
  }
  if (inaLoadOk) {
    inaLoadV = inaLoad->getBusVoltage_V() + (inaLoad->getShuntVoltage_mV() / 1000.0f);
    inaLoadI_mA = inaLoad->getCurrent_mA();
    inaLoadP_mW = inaLoad->getPower_mW();
  }
}

void sendToThingSpeak() {
  WiFiClient client;
  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" + String(thingspeakApiKey) +
    "&field1=" + String(inaPanelV, 3) +
    "&field2=" + String(inaPanelI_mA, 1) +
    "&field3=" + String(inaPanelP_mW, 1) +
    "&field4=" + String(inaLoadV, 3) +
    "&field5=" + String(inaLoadI_mA, 1) +
    "&field6=" + String(inaLoadP_mW, 1) +
    "&field7=" + String(efficienzaPercent, 1) +
    "&field8=" + String(realEnergyWh, 4);
  http.begin(client, url);
  int httpCode = http.GET();
  String payload = (httpCode == 200) ? http.getString() : "";
  thingspeakLastSendOk = (httpCode == 200) && (payload.toInt() > 0);
  http.end();
}

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
  realEnergyWh = prefs.getFloat("realEnergyWh", 0.0f);
  inaAddrPanel = prefs.getUChar("inaAddrP", 0x40);
  inaAddrLoad = prefs.getUChar("inaAddrL", 0x41);
  prefs.end();
}

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
      // Il semiciclo appena concluso 
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
      readINASensors();

      if (logCount < MAX_LOG_SAMPLES) {
        LogSample &s = logBuffer[logCount++];
        s.t = nowMs - logStartMillis;
        s.errH = (int16_t)errH;
        s.errV = (int16_t)errV;
        s.pulseH = (int16_t)lastPulseHus;
        s.pulseV = (int16_t)lastPulseVus;
        s.tl = (int16_t)tl; s.tr = (int16_t)tr; s.bl = (int16_t)bl; s.br = (int16_t)br;
        s.vPanel_mV = (int16_t)(inaPanelV * 1000.0f);
        s.iPanel_mA = (int16_t)inaPanelI_mA;
        s.vLoad_mV = (int16_t)(inaLoadV * 1000.0f);
        s.iLoad_mA = (int16_t)inaLoadI_mA;
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

void runAutoTracking() {
  double rawH = (double)((valTL + valBL) - (valTR + valBR));
  double rawV = (double)((valTL + valTR) - (valBL + valBR));

  int maxLDR = max(max(valTL, valTR), max(valBL, valBR));

  if (maxLDR < sogliaNotte) {
    powerSaveServos();
    inputH = 0;
    inputV = 0;
    outputH = 0;
    outputV = 0;
    puntoMortoAttivo = false;
    pidActiveH = false;
    pidActiveV = false;
    return;
  }

  bool errHZero = (abs(rawH) <= zonaMorta);
  bool errVZero = (abs(rawV) <= zonaMorta);
  bool diagAltissima = (abs(diffDiagonali) >= sogliaPuntoMorto);

  if (puntoMortoAbilitato && errHZero && errVZero && diagAltissima) {
    puntoMortoAttivo = true;

    int escapeSpeedH = 0;
    int escapeSpeedV = 0;

    if (diffDiagonali > 0) {
      if (valTL >= valBR) {
        escapeSpeedH = maxAutoSpeed;
        escapeSpeedV = maxAutoSpeed;
      } else {
        escapeSpeedH = -maxAutoSpeed;
        escapeSpeedV = -maxAutoSpeed;
      }
    } else {
      if (valTR >= valBL) {
        escapeSpeedH = -maxAutoSpeed;
        escapeSpeedV = maxAutoSpeed;
      } else {
        escapeSpeedH = maxAutoSpeed;
        escapeSpeedV = -maxAutoSpeed;
      }
    }

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
}

void doeStartKick() {
  doeKicking = true;
  doeKickStartMillis = millis();
  loggingActive = false;
  doeRunReady = false;
  pidActiveH = false;
  pidActiveV = false;
  setServoV(0);
  setServoH(-DOE_KICK_SPEED);
}

void doeBeginLogging(int idx) {
  doeKicking = false;
  stopServos();
  zonaMorta = doeRuns[idx].deadzone;
  logCount = 0;
  logStartMillis = millis();
  lastLogSampleMillis = 0;
  loggingActive = true;
  doeRunStartMillis = millis();
  doeRunReady = false;
  resetPID(pidH);
  resetPID(pidV);
  pidActiveH = false;
  pidActiveV = false;
}

bool doeGuard() {
  if (doeActive) {
    server.send(409, "application/json", "{\"status\":\"error\",\"message\":\"Test DOE in corso\"}");
    return true;
  }
  return false;
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

  Wire.begin(pinSDA, pinSCL);
  initINA();
  Serial.printf("INA219 -> Pannello(0x%02X): %s | Load(0x%02X): %s\n",
                inaAddrPanel, inaPanelOk ? "OK" : "NON TROVATO",
                inaAddrLoad, inaLoadOk ? "OK" : "NON TROVATO");

  // Configurazione Wi-Fi SoftAP
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress apIP = WiFi.softAPIP();
  
  Serial.print("Access Point Wi-Fi Creato!");
  Serial.print(" SSID: "); Serial.println(ap_ssid);
  Serial.print(" IP per collegarsi da Smartphone: "); Serial.println(apIP);

  WiFi.begin(sta_ssid, sta_password);
  unsigned long staStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - staStart < 15000) {
    delay(250);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connesso alla rete Wi-Fi! IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Connessione alla rete Wi-Fi non riuscita, continuo solo con Access Point.");
  }

  // Configurazione Web Server
  setupWebServer();
  server.begin();
  Serial.println("Web Server avviato sulla porta 80!");

  lastEnergyCalc = millis();
}

void loop() {
  // Gestione delle richieste dagli smartphone collegati al WebServer
  server.handleClient();

  readINASensors();

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
    if (posV > V_ANGLE_MAX) posV = V_ANGLE_MAX;
    if (posV < V_ANGLE_MIN) posV = V_ANGLE_MIN;
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
  realEnergyWh += (inaPanelP_mW / 1000.0f) * deltaHours;
  efficienzaPercent = (inaPanelP_mW > 0.1f) ? (inaLoadP_mW / inaPanelP_mW * 100.0f) : 0.0f;


  if (now - lastEnergySaveMs >= ENERGY_SAVE_INTERVAL_MS) {
    lastEnergySaveMs = now;
    prefs.begin("solartrk", false);
    prefs.putFloat("energyWh", totalEnergyWh);
    prefs.putFloat("realEnergyWh", realEnergyWh);
    prefs.end();
  }

  // 3. Esecuzione Logica in base alla Modalità Selezionata da Smartphone
  switch (currentMode) {

    case MODE_AUTO: {
      runAutoTracking();
      break;
    }

    case MODE_DOE: {
      if (doeActive && doeKicking) {
        if (millis() - doeKickStartMillis >= DOE_KICK_DURATION_MS) {
          doeBeginLogging(doeCurrentIndex);
        }
      } else {
        runAutoTracking();
        if (doeActive && !doeRunReady && (millis() - doeRunStartMillis >= doeRunDurationMs)) {
          stopServos();
          loggingActive = false;
          doeRunReady = true;
        }
      }
      break;
    }

    case MODE_MANUAL: {
      puntoMortoAttivo = false;
      if (millis() - lastManualCmdMillis > MANUAL_TIMEOUT_MS) {
        stopServos();
        manualVelH = 0;
        manualVelV = 0;
        inputH = 0;
        inputV = 0;
        outputH = 0;
        outputV = 0;
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
        s.vPanel_mV = (int16_t)(inaPanelV * 1000.0f);
        s.iPanel_mA = (int16_t)inaPanelI_mA;
        s.vLoad_mV = (int16_t)(inaLoadV * 1000.0f);
        s.iLoad_mA = (int16_t)inaLoadI_mA;
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

  if (WiFi.status() == WL_CONNECTED && (millis() - lastThingspeakSendMillis >= THINGSPEAK_INTERVAL_MS)) {
    lastThingspeakSendMillis = millis();
    sendToThingSpeak();
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

  // API Calibrazione dei 4 LDR sotto luce uniforme (mutante: solo POST)
  server.on("/api/calibrate", HTTP_POST, handleApiCalibrate);
  server.on("/api/resetpos", HTTP_POST, handleApiResetPos);

  // API Logging CSV in RAM: avvio/stop (mutanti, POST) e lettura stato/export (GET)
  server.on("/api/log/start", HTTP_POST, handleApiLogStart);
  server.on("/api/log/stop", HTTP_POST, handleApiLogStop);
  server.on("/api/log/status", HTTP_GET, handleApiLogStatus);
  server.on("/api/log/csv", HTTP_GET, handleApiLogCsv);

  // API Autotuning PID (Relay Feedback) On-Demand (mutante: solo POST)
  server.on("/api/autotune", HTTP_POST, handleApiAutotune);

  server.on("/api/ina/config", HTTP_POST, handleApiInaConfig);
  server.on("/api/ina/scan", HTTP_GET, handleApiInaScan);

  server.on("/api/doe/upload", HTTP_POST, handleApiDoeUpload);
  server.on("/api/doe/start", HTTP_POST, handleApiDoeStart);
  server.on("/api/doe/next", HTTP_POST, handleApiDoeNext);
  server.on("/api/doe/stop", HTTP_POST, handleApiDoeStop);
  server.on("/api/doe/status", HTTP_GET, handleApiDoeStatus);

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

  char jsonBuffer[1200];
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
      "\"heap\":%u,"
      "\"vPanelIna\":%.3f,"
      "\"iPanelIna\":%.1f,"
      "\"pPanelIna\":%.1f,"
      "\"vLoadIna\":%.3f,"
      "\"iLoadIna\":%.1f,"
      "\"pLoadIna\":%.1f,"
      "\"inaPanelOk\":%s,"
      "\"inaLoadOk\":%s,"
      "\"posV\":%.1f,"
      "\"vAtLimit\":%s,"
      "\"efficienza\":%.1f,"
      "\"realEnergy\":%.3f,"
      "\"wifiSta\":%s,"
      "\"cloudOk\":%s"
    "}",
    solarVoltage,
    solarCurrent,
    solarPower,
    totalEnergyWh,
    valTL, valTR, valBL, valBR,
    valDiag1, valDiag2, diffDiagonali,
    puntoMortoAttivo ? "true" : "false",
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
    ESP.getFreeHeap(),
    inaPanelV, inaPanelI_mA, inaPanelP_mW,
    inaLoadV, inaLoadI_mA, inaLoadP_mW,
    inaPanelOk ? "true" : "false",
    inaLoadOk ? "true" : "false",
    posV,
    vAxisAtLimit ? "true" : "false",
    efficienzaPercent,
    realEnergyWh,
    (WiFi.status() == WL_CONNECTED) ? "true" : "false",
    thingspeakLastSendOk ? "true" : "false"
  );
  server.send(200, "application/json", jsonBuffer);
}

void handleApiMode() {
  if (doeGuard()) return;
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
    }
    char jsonBuffer[96];
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"status\":\"ok\",\"mode\":\"%s\"}", modeString.c_str());
    server.send(200, "application/json", jsonBuffer);
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing mode\"}");
  }
}

void handleApiControl() {
  if (doeGuard()) return;
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
  if (doeGuard()) return;
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

void handleApiCalibrate() {
  calibrateLDRs();
  char jsonBuffer[160];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"status\":\"ok\",\"calTL\":%.3f,\"calTR\":%.3f,\"calBL\":%.3f,\"calBR\":%.3f}",
    calTL, calTR, calBL, calBR
  );
  server.send(200, "application/json", jsonBuffer);
}

void handleApiResetPos() {
  posH = 0.0f;
  posV = 0.0f;
  vAxisAtLimit = false;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleApiAutotune() {
  if (doeGuard()) return;
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
  String filename = "solar_log.csv";
  if (doeActive && doeCurrentIndex >= 0 && doeCurrentIndex < doeRunCount && doeRuns[doeCurrentIndex].filename.length() > 0) {
    filename = doeRuns[doeCurrentIndex].filename;
  }
  server.sendHeader("Content-Disposition", "attachment; filename=" + filename);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv", "");
  server.sendContent("t_ms,errH,errV,pulseH_us,pulseV_us,tl,tr,bl,br,v_panel_mV,i_panel_mA,v_load_mV,i_load_mA\r\n");
  char row[180];
  for (int i = 0; i < logCount; i++) {
    LogSample &s = logBuffer[i];
    snprintf(row, sizeof(row), "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
             (unsigned long)s.t, s.errH, s.errV, s.pulseH, s.pulseV, s.tl, s.tr, s.bl, s.br,
             s.vPanel_mV, s.iPanel_mA, s.vLoad_mV, s.iLoad_mA);
    server.sendContent(row);
  }
  server.sendContent("");
}

void handleApiInaConfig() {
  if (doeGuard()) return;
  if (server.hasArg("panelAddr")) {
    long a = strtol(server.arg("panelAddr").c_str(), nullptr, 0);
    if (a > 0 && a < 256) inaAddrPanel = (uint8_t)a;
  }
  if (server.hasArg("loadAddr")) {
    long a = strtol(server.arg("loadAddr").c_str(), nullptr, 0);
    if (a > 0 && a < 256) inaAddrLoad = (uint8_t)a;
  }
  prefs.begin("solartrk", false);
  prefs.putUChar("inaAddrP", inaAddrPanel);
  prefs.putUChar("inaAddrL", inaAddrLoad);
  prefs.end();
  initINA();
  char jsonBuffer[160];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"status\":\"ok\",\"panelAddr\":%d,\"loadAddr\":%d,\"panelOk\":%s,\"loadOk\":%s}",
    inaAddrPanel, inaAddrLoad, inaPanelOk ? "true" : "false", inaLoadOk ? "true" : "false"
  );
  server.send(200, "application/json", jsonBuffer);
}

void handleApiInaScan() {
  if (doeGuard()) return;
  String addrList = "";
  bool first = true;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (!first) addrList += ",";
      char buf[6];
      snprintf(buf, sizeof(buf), "%d", addr);
      addrList += buf;
      first = false;
    }
  }
  String json = "{\"status\":\"ok\",\"addresses\":[" + addrList + "]}";
  server.send(200, "application/json", json);
}

void handleApiDoeUpload() {
  if (doeGuard()) return;
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Corpo mancante\"}");
    return;
  }
  String body = server.arg("plain");
  doeRunCount = 0;
  int pos = 0;
  int len = body.length();
  while (pos < len && doeRunCount < MAX_DOE_RUNS) {
    int nl = body.indexOf('\n', pos);
    String line = (nl == -1) ? body.substring(pos) : body.substring(pos, nl);
    pos = (nl == -1) ? len : nl + 1;
    line.trim();
    if (line.length() == 0 || !isDigit(line.charAt(0))) continue;
    int c1 = line.indexOf(',');
    int c2 = (c1 == -1) ? -1 : line.indexOf(',', c1 + 1);
    int c3 = (c2 == -1) ? -1 : line.indexOf(',', c2 + 1);
    if (c1 < 0 || c2 < 0 || c3 < 0) continue;
    int c4 = line.indexOf(',', c3 + 1);
    String fname = (c4 == -1) ? line.substring(c3 + 1) : line.substring(c3 + 1, c4);
    fname.trim();
    doeRuns[doeRunCount].run = line.substring(0, c1).toInt();
    doeRuns[doeRunCount].deadzone = line.substring(c1 + 1, c2).toInt();
    doeRuns[doeRunCount].replica = line.substring(c2 + 1, c3).toInt();
    doeRuns[doeRunCount].filename = fname;
    doeRunCount++;
  }
  char jsonBuffer[96];
  snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"status\":\"ok\",\"runs\":%d}", doeRunCount);
  server.send(200, "application/json", jsonBuffer);
}

void handleApiDoeStart() {
  if (doeRunCount == 0) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Nessuna matrice DOE caricata\"}");
    return;
  }
  if (server.hasArg("duration")) {
    long d = server.arg("duration").toInt();
    if (d > 0) doeRunDurationMs = (unsigned long)d * 1000UL;
  }
  doeCurrentIndex = 0;
  doeActive = true;
  currentMode = MODE_DOE;
  modeString = "doe";
  doeStartKick();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleApiDoeNext() {
  if (!doeActive) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Test DOE non attivo\"}");
    return;
  }
  doeCurrentIndex++;
  if (doeCurrentIndex >= doeRunCount) {
    doeActive = false;
    doeKicking = false;
    doeRunReady = false;
    loggingActive = false;
    stopServos();
    currentMode = MODE_AUTO;
    modeString = "auto";
    resetPID(pidH);
    resetPID(pidV);
    pidActiveH = false;
    pidActiveV = false;
    server.send(200, "application/json", "{\"status\":\"ok\",\"done\":true}");
    return;
  }
  doeStartKick();
  server.send(200, "application/json", "{\"status\":\"ok\",\"done\":false}");
}

void handleApiDoeStop() {
  doeActive = false;
  doeKicking = false;
  doeRunReady = false;
  loggingActive = false;
  stopServos();
  currentMode = MODE_AUTO;
  modeString = "auto";
  resetPID(pidH);
  resetPID(pidV);
  pidActiveH = false;
  pidActiveV = false;
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleApiDoeStatus() {
  int run = 0, deadzone = 0, replica = 0;
  String filename = "";
  if (doeCurrentIndex >= 0 && doeCurrentIndex < doeRunCount) {
    run = doeRuns[doeCurrentIndex].run;
    deadzone = doeRuns[doeCurrentIndex].deadzone;
    replica = doeRuns[doeCurrentIndex].replica;
    filename = doeRuns[doeCurrentIndex].filename;
  }
  unsigned long elapsed = (doeActive && !doeKicking) ? (millis() - doeRunStartMillis) : 0;
  char jsonBuffer[340];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"active\":%s,\"positioning\":%s,\"ready\":%s,\"totalRuns\":%d,\"currentIndex\":%d,\"run\":%d,\"deadzone\":%d,\"replica\":%d,\"filename\":\"%s\",\"elapsedMs\":%lu,\"durationMs\":%lu}",
    doeActive ? "true" : "false",
    doeKicking ? "true" : "false",
    doeRunReady ? "true" : "false",
    doeRunCount, doeCurrentIndex, run, deadzone, replica, filename.c_str(),
    elapsed, doeRunDurationMs
  );
  server.send(200, "application/json", jsonBuffer);
}
