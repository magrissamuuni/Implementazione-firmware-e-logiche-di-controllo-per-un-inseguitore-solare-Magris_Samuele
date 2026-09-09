Progetto sviluppato nell'ambito della tesi di laurea triennale
Dipartimento di Ingegneria dell'Informazione, Università degli Studi di Padova. 
Relatore: Prof. Damiano Varagnolo.

## Indice

- [Introduzione](#introduzione)
- [Obiettivi](#obiettivi)
- [Progettazione 3D](#progettazione-3d)
- [Hardware](#hardware)
- [Firmware](#firmware)
- [Interfaccia web](#interfaccia-web)
- [Prototipo](#prototipo)
- [Stampa Definitiva](#stampa-definitiva)
- [Metodologia sperimentale](#metodologia-sperimentale)
- [Struttura del repository](#struttura-del-repository)
- [Documentazione](#documentazione)
## Introduzione
Le fonti rinnovabili di energia sono viste come un'alternativa affidabile ai combustibili fossili grazie alla loro capacità di essere inesauribili.
L'energia solare fotovoltaica è una delle fonti rinnovabili più a portata di mano rispetto all'energia geotermica, idrica, bioenergia oppure eolica.
Massimizzare il tempo passato perpendicolarmente al sole è uno dei metodi migliori per massimizzare l'efficienza di un pannello fotovoltaico. //Trovare Fonte
Per fare ciò, sistemi di tracciamento permettono ai pannelli solari di rimanere perpendicolari ai raggi solari per la maggior parte della giornata.
Lo scopo di questo progetto è quello di sviluppare un prototipo bi-assiale a scala ridotta di un sistema di tracciamento solare mediante l'uso di fotoresistenze per tracciare la posizione del sole.

## Obiettivi

L'obiettivo è quello di creare un sistema indipendente in grado di autosostenersi.
Il sistema deve essere in grado di:
- riconoscere la posizione del punto di massima luminosità e orientare il pannello verso di esso;
- evitare micro-movimenti una volta raggiunto tale punto (causati da valori diversi sulle fotoresistenze).

Il firmware deve:
- implementare una interfaccia per l'utilizzo del sistema;
- implementare un controllore PID e permetterne la modifica dei parametri;
- raccogliere le informazioni e i dati riguardo a: errori dei servomotori, luminosità delle fotoresistenze e parametri dei pannelli fotovoltaici;
- gestire il movimento dei servo-motori;

//TODO finire

## Progettazione 3D
Il sistema meccanico è stato progettato tramite l'utilizzo di Fusion 360.
E' progettato per permettere l'utilizzo di due servomotori SG90 che consentono il movimento bi-assiale. Per ricavare i dati di puntamento tramite le fotoresistenze è stata realizzata una struttura a croce che proietta un'ombra sulle LDR: quando il pannello è orientato perpendicolarmente al sole, l'ombra cade in modo simmetrico su tutti e quattro i sensori, azzerando la differenza di illuminamento tra loro:

Il sistema composto da:
- un ingranaggio con due strutture a forcella;
  <br>
  <img src="Images/SS%20-%20forcelle.png" width="500">
- una croce per permettere di proiettare ombre sulle LDR;
  <br>
  <img src="Images/SS%20-%20croce%20LDR.png" width="500">
- due pignoni, che collegano li SG90 al resto della struttura;
  <br>
  <img src="Images/SS%20-%20pignoneH.png" width="400">   <img src="Images/SS%20-%20pignoneV.png" width="400">
- una struttura rettangolare dove risiede il pannello;
  <br>
    <img src="Images/SS%20-%20porta%20pannello.png" width="500">
- un semi-ingranaggio;
  <br>
  <img src="Images/SS%20-%20semi%20ingranaggio.png" width="500">
- una struttura di base. su cui poggia l'intero sistema.
  <br>
    <img src="Images/SS-Base%20Rettangolare.png" width="500">

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

- Servo orizzontale (H) = 19
- Servo verticale (V) = 18 
- LDR Alto-Sinistra (TL) = 35
- LDR Alto-Destra (TR) = 33
- LDR Basso-Sinistra (BL) = 34
- LDR Basso-Destra (BR) = 32
- Tensione pannello solare = 36

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

## Prototipo

Il sistema è stato inizialmente stampato in  PLA, il quale inizia a subire deformazioni strutturali e a perdere la propria rigidità geometrica già all’interno dell’intervallo termico compreso tra i 50 °C e i 60 °C. 
Si è quindi optato per il PETG, in grado di garantire una stabilità dimensionale anche in condizioni di calore moderato (fino ai 70 °C  - 75°C) superando così i limiti del PLA. Inoltre, presenta una maggiore opacità alla luce rispetto al PLA.

Il circuito è stato assemblato e saldato manualmente con alcuni errori di collegamento verificatisi e successivamente corretti.
E' stato successivamente validato funzionalmente prima dei test firmware.

<img src="Images/foto%20prototipo%204.jpeg" width="400"> <img src="Images/foto%20prototipo%202%20.jpeg" width="400">
<img src="Images/foto%20prototipo%203.jpeg" width="400"> <img src="Images/foto%20prototipo%201%20-%20closeup.jpeg" width="400">

//TOGLIERE FOTO PIEDE

Il sistema risulta funzionale ma non esponibile al sola dato il materiale utilizzato. Occorre quindi una ristampa.

##Stampa Definitiva
La versione finale è stata stampata in PETG e non ha più presentato i problemi di deformazione termica riscontrati con il PLA, confermando la stabilità strutturale della soluzione adottata.

## Metodologia sperimentale

//TODO

I guadagni PID (Kp/Ki/Kd) **non** sono variabili sperimentali del DOE: sono
tarati a parte tramite autotuning a relay feedback, avendo un metodo di
teoria dei controlli per calcolarli direttamente. 

Risultati completi in:
//TODO

## Struttura del repository
//TODO rifinire a progetto terminato
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
