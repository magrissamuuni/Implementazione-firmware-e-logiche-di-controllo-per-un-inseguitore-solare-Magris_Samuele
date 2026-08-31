# Solar Tracker Biassiale con ESP32

Inseguitore solare a due assi (orizzontale/verticale) basato su microcontrollore
ESP32, con controllo PID reale, autotuning automatico via relay feedback,
interfaccia web integrata e un set di script Python per l'analisi statistica
delle prove sperimentali (Design of Experiments).

Progetto sviluppato nell'ambito della tesi di laurea triennale
 Dipartimento di Ingegneria dell'Informazione,
Università degli Studi di Padova. Relatore: Prof. Damiano Varagnolo.

## Indice

- [Introduzione](#introduzione)
- [Hardware](#hardware)
- [Firmware](#firmware)
- [Interfaccia web](#interfaccia-web)
- [Analisi dati](#analisi-dati)
- [Metodologia sperimentale](#metodologia-sperimentale)
- [Struttura del repository](#struttura-del-repository)
- [Documentazione](#documentazione)
- 
## Introduzione
L'obiettivo di questa Tesi è quello di costruire un dispositivo in grado di posizionare un pannello solare nella posizione ottimale rispetto al sole e di valutarne l'efficienza.

In particolare si vuole valutare l'efficienza di un dispositivo biassiale e confrontare i dati acquisiti con l'efficienza di un sistema fisso.



## Hardware

- Microcontrollore: ESP32 WROOM [ESP32](Datasheet/esp32-wroom-32_datasheet_en.pdf)
- Sensori: 4 fotoresistenze (LDR) [LDR](Datasheet/GL55-LDR%20DataSheet.pdf)
- Attuatori: 2 servomotori [SG90](Datasheet/SG90%20DataSheet.pdf)
- Batteria: 1 batteria ricaricabile al litio [103450](Datasheet/Battery%203.7%20V%202000mAh%20103450%20DataSheet.pdf)
- Ricarica e protezione: 1 modulo di ricarica con 1 chip di protezione [TP5046](Datasheet/TP4056%20(modulo%20ricarica)%20DataSheet.pdf) [DW01A](Datasheet/DW01A%20(chip%20protezione)%20DataSheet.pdf)
- Lettore Voltaggio/Corrente: [INA219](Datasheet/ina219%20(current%20power%20monitor)%20DataSheet.pdf)
- Modulo Step-UP: 1 modulo [MT3608](Datasheet/MT3608%20(step%20up)%20DataSheet.pdf)
- Pannello solare: 5V

### Mappa dei pin

Servo orizzontale (H) = 19 
Servo verticale (V) = 18  
LDR Alto-Sinistra (TL) = 35
LDR Alto-Destra (TR) = 33
LDR Basso-Sinistra (BL) = 34
LDR Basso-Destra (BR) = 32
Tensione pannello solare = 36 

## Firmware

File: [`firmware/main.cpp`](firmware/main.cpp) + [`firmware/webpage.h`](firmware/webpage.h)

### Librerie richieste (Arduino IDE / PlatformIO)

- `WiFi.h`, `WebServer.h`, `Preferences.h`
- `ESP32Servo`
- `PID_v1`

### Funzionalità principali

- Controllo PID reale su entrambi gli assi (filtro passa-basso sulle letture
  LDR, anti-windup, uscita continua sui servo)
- Autotuning automatico dei guadagni PID via relay feedback
  (metodo di Åström–Hägglund + formule di Ziegler-Nichols)
- Calibrazione automatica dei 4 sensori LDR
- Logging su buffer con esportazione CSV
- Persistenza dei parametri in NVS

### Come Avviare

1. Apri `firmware/main.cpp` in Arduino IDE o PlatformIO
2. Installa le librerie elencate sopra dal Library Manager
3. Seleziona la scheda ESP32 corretta e la porta seriale
4. Carica il firmware

All'avvio l'ESP32 crea una rete Wi-Fi propria:

- **SSID**: `SolarTracker-ESP32`
- **Password**: `12345678`

## Interfaccia web

Connettiti alla rete Wi-Fi del tracker e apri `http://192.168.4.1` dal
browser. L'interfaccia mostra in tempo reale: telemetria del pannello solare,
letture dei 4 LDR, stato del controllo PID, e permette di cambiare modalità,
controllare manualmente i motori, tarare il PID, calibrare i sensori e
scaricare i log come CSV.


## Metodologia sperimentale

//TODO

I guadagni PID (Kp/Ki/Kd) **non** sono variabili sperimentali del DOE: sono
tarati a parte tramite autotuning a relay feedback, avendo un metodo di
teoria dei controlli per calcolarli direttamente. 

Risultati completi in:
//TODO

## Struttura del repository

```
solar-tracker-esp32/
├── firmware/
│   ├── main.cpp          # firmware ESP32
│   └── webpage.h         # interfaccia web (HTML/CSS/JS)
├── analysis/
│   ├── analyze_log.py               # estrazione metriche da log grezzi
│   ├── anova_doe.py                 # analisi statistica del DOE
│   ├── analyze_energy_comparison.py # confronto energetico tracker/fisso
│   └── requirements.txt
├── data/
│   ├── doe_zona_morta/   
│   └── riepilogo_doe.csv # riepilogo aggregato 
└── docs/
    └── matrice_DOE_randomizzata.csv        # ordine di esecuzione delle prove
```

## Documentazione
 [ESP32](Datasheet/esp32-wroom-32_datasheet_en.pdf)
[LDR](Datasheet/GL55-LDR%20DataSheet.pdf)
[SG90](Datasheet/SG90%20DataSheet.pdf)
[103450](Datasheet/Battery%203.7%20V%202000mAh%20103450%20DataSheet.pdf)
[TP5046](Datasheet/TP4056%20(modulo%20ricarica)%20DataSheet.pdf) [DW01A](Datasheet/DW01A%20(chip%20protezione)%20DataSheet.pdf)
[INA219](Datasheet/ina219%20(current%20power%20monitor)%20DataSheet.pdf)
[MT3608](Datasheet/MT3608%20(step%20up)%20DataSheet.pdf)
