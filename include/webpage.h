#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <Arduino.h>

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Solar Tracker ESP32</title>
  <style>
    body {
      font-family: Arial, Helvetica, sans-serif;
      background-color: #f4f4f4;
      color: #333;
      margin: 0;
      padding: 15px;
    }
    h1 {
      font-size: 20px;
      color: #2c3e50;
      margin-bottom: 5px;
      text-align: center;
    }
    .status {
      text-align: center;
      font-size: 13px;
      font-weight: bold;
      color: green;
      margin-bottom: 10px;
    }
    .status.offline {
      color: red;
    }
    .alert-banner {
      display: none;
      text-align: center;
      padding: 8px 12px;
      font-size: 13px;
      font-weight: bold;
      margin-bottom: 15px;
      border-radius: 4px;
    }
    .alert-deadlock {
      background-color: #fff3cd;
      color: #856404;
      border: 1px solid #ffeeba;
    }
    .alert-sunsearch {
      background-color: #d1ecf1;
      color: #0c5460;
      border: 1px solid #bee5eb;
    }
    .alert-autotune {
      background-color: #e2d9f3;
      color: #432874;
      border: 1px solid #d6c6ec;
    }
    .box {
      background: #ffffff;
      border: 1px solid #ccc;
      padding: 12px;
      margin-bottom: 15px;
    }
    .box h2 {
      font-size: 15px;
      margin-top: 0;
      margin-bottom: 10px;
      border-bottom: 1px solid #eee;
      padding-bottom: 5px;
      color: #444;
    }
    table {
      width: 100%;
      border-collapse: collapse;
    }
    table td, table th {
      border: 1px solid #ddd;
      padding: 8px;
      text-align: center;
      font-size: 14px;
    }
    table th {
      background-color: #e9e9e9;
      color: #333;
    }
    .valore {
      font-size: 18px;
      font-weight: bold;
      color: #0056b3;
    }
    .valore-diag {
      font-size: 15px;
      font-weight: bold;
      color: #d9534f;
    }
    .btn-group {
      display: flex;
      gap: 5px;
      flex-wrap: wrap;
    }
    button {
      background-color: #e7e7e7;
      color: #333;
      border: 1px solid #adadad;
      padding: 8px 12px;
      font-size: 14px;
      cursor: pointer;
      flex: 1;
      min-width: 70px;
    }
    button:active {
      background-color: #ccc;
    }
    button.active {
      background-color: #007bff;
      color: white;
      border-color: #0056b3;
    }
    .btn-findsun {
      background-color: #ffc107 !important;
      color: #212529 !important;
      border-color: #d39e00 !important;
      font-weight: bold;
    }
    .dpad {
      display: grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: 5px;
      max-width: 200px;
      margin: 0 auto;
    }
    .dpad button {
      padding: 12px;
      font-size: 16px;
      font-weight: bold;
    }
    .btn-stop {
      background-color: #dc3545 !important;
      color: white !important;
      border-color: #bd2130 !important;
    }
    .pid-form {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 8px;
    }
    .pid-form div {
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .pid-form label {
      font-size: 13px;
    }
    .pid-form input {
      width: 65px;
      padding: 4px;
      font-size: 13px;
      border: 1px solid #ccc;
    }
    .btn-salva {
      grid-column: span 2;
      background-color: #28a745;
      color: white;
      border-color: #1e7e34;
      margin-top: 10px;
      padding: 8px;
    }
  </style>
</head>
<body>

  <h1>Solar Tracker ESP32</h1>
  <div id="status" class="status">Stato: Connesso</div>
  <div id="alert-banner" class="alert-banner"></div>

  <!-- Telemetria Solare -->
  <div class="box">
    <h2>Dati Pannello Solare (5V 500mA)</h2>
    <table>
      <tr>
        <th>Tensione</th>
        <th>Corrente</th>
        <th>Potenza</th>
        <th>Energia Prodotta</th>
      </tr>
      <tr>
        <td id="val-v" class="valore">0.0 V</td>
        <td id="val-i" class="valore">0 mA</td>
        <td id="val-p" class="valore">0.00 W</td>
        <td id="val-energy" class="valore">0.00 Wh</td>
      </tr>
    </table>
  </div>

  <!-- Modalità -->
  <div class="box">
    <h2>Modalità di Funzionamento</h2>
    <div class="btn-group">
      <button id="btn-auto" class="active" onclick="setMode('auto')">Auto</button>
      <button id="btn-manual" onclick="setMode('manual')">Manuale</button>
      <button id="btn-night" onclick="setMode('night')">Notte</button>
      <button id="btn-eco" onclick="setMode('eco')">Eco</button>
      <button id="btn-findsun" class="btn-findsun" onclick="triggerFindSun()">☀️ Trova Sole</button>
    </div>
  </div>

  <!-- Sensori LDR e Analisi Diagonali -->
  <div class="box">
    <h2>Sensori LDR e Analisi Diagonali</h2>
    <table>
      <tr>
        <th>Alto SX (TL)</th>
        <th>Alto DX (TR)</th>
      </tr>
      <tr>
        <td id="ldr-tl">0</td>
        <td id="ldr-tr">0</td>
      </tr>
      <tr>
        <th>Basso SX (BL)</th>
        <th>Basso DX (BR)</th>
      </tr>
      <tr>
        <td id="ldr-bl">0</td>
        <td id="ldr-br">0</td>
      </tr>
      <tr>
        <th>Diagonale 1 (TL+BR)</th>
        <th>Diagonale 2 (TR+BL)</th>
      </tr>
      <tr>
        <td id="val-diag1">0</td>
        <td id="val-diag2">0</td>
      </tr>
      <tr>
        <th colspan="2">Differenza Diagonali & Punto Morto</th>
      </tr>
      <tr>
        <td colspan="2">
          Diff: <span id="val-diffdiag" class="valore-diag">0</span> |
          Stato: <span id="val-deadlock-status" style="font-weight:bold;">Normale</span>
        </td>
      </tr>
    </table>
  </div>

  <!-- Stato Motori / PID -->
  <div class="box">
    <h2>Stato Motori (PID)</h2>
    <table>
      <tr>
        <th>Errore H</th>
        <th>Errore V</th>
      </tr>
      <tr>
        <td id="val-errh">0</td>
        <td id="val-errv">0</td>
      </tr>
      <tr>
        <th>Impulso Servo H</th>
        <th>Impulso Servo V</th>
      </tr>
      <tr>
        <td id="val-velh">1500 us</td>
        <td id="val-velv">1500 us</td>
      </tr>
    </table>
    <div id="powersave-indicator" style="display:none; margin-top:8px; padding:6px; font-size:12px; font-weight:bold; text-align:center; background-color:#fff3cd; color:#856404; border:1px solid #ffeeba; border-radius:4px;">
      🔋 Risparmio energetico attivo: servo scollegati dal segnale PWM
    </div>
    <canvas id="err-chart" width="300" height="80" style="width:100%; height:80px; margin-top:10px; border:1px solid #ddd; display:block;"></canvas>
  </div>

  <!-- Controllo Manuale -->
  <div class="box">
    <h2>Controllo Manuale Motori</h2>
    <div class="dpad">
      <div></div>
      <button onclick="sendCmd('up')">▲</button>
      <div></div>
      <button onclick="sendCmd('left')">◄</button>
      <button class="btn-stop" onclick="sendCmd('stop')">STOP</button>
      <button onclick="sendCmd('right')">►</button>
      <div></div>
      <button onclick="sendCmd('down')">▼</button>
      <div></div>
    </div>
  </div>

  <!-- Taratura PID, Deadzone e Soglia Punto Morto -->
  <div class="box">
    <h2>Impostazioni PID, Deadzone e Soglie</h2>
    <div class="pid-form">
      <div><label>Kp:</label><input type="number" id="input-kp" step="0.1" min="0" max="50" value="0.5"></div>
      <div><label>Ki:</label><input type="number" id="input-ki" step="0.05" min="0" max="50" value="0.0"></div>
      <div><label>Kd:</label><input type="number" id="input-kd" step="0.01" min="0" max="50" value="0.05"></div>
      <div><label>Deadzone:</label><input type="number" id="input-deadzone" step="10" min="0" max="4095" value="350"></div>
      <div><label>Soglia Deadlock:</label><input type="number" id="input-deadlock" step="50" min="0" max="8190" value="700"></div>
      <div><label>Soglia Notte:</label><input type="number" id="input-night" step="10" min="0" max="4095" value="100"></div>
      <div style="grid-column: span 2;"><label>Sblocco Punto Morto Abilitato:</label><input type="checkbox" id="input-pmenabled" checked></div>
      <button class="btn-salva" onclick="savePID()">Salva Impostazioni</button>
      <button id="btn-autotune" style="grid-column: span 2; background-color:#6f42c1; color:white; border-color:#59339d; margin-top:6px; padding:8px;" onclick="triggerAutotune()">🎯 Autotune PID (Relay Feedback)</button>
    </div>
    <p style="font-size:12px; color:#666; margin-top:8px; margin-bottom:0;">Fa oscillare volutamente entrambi gli assi per stimare Kp/Ki/Kd in automatico (metodo Ziegler-Nichols ad anello chiuso). Dura tipicamente 10-30s, fino a un massimo di 45s.</p>
  </div>

  <!-- Calibrazione LDR -->
  <div class="box">
    <h2>Calibrazione Sensori LDR</h2>
    <table>
      <tr>
        <th>TL</th>
        <th>TR</th>
        <th>BL</th>
        <th>BR</th>
      </tr>
      <tr>
        <td id="cal-tl">1.000</td>
        <td id="cal-tr">1.000</td>
        <td id="cal-bl">1.000</td>
        <td id="cal-br">1.000</td>
      </tr>
    </table>
    <button style="width:100%; margin-top:10px; background-color:#17a2b8; color:white; border-color:#117a8b;" onclick="calibrateLDRs()">🎯 Calibra LDR (sotto luce uniforme)</button>
    <p style="font-size:12px; color:#666; margin-top:8px; margin-bottom:0;">Posiziona una sorgente luminosa equidistante dai 4 sensori prima di avviare la calibrazione. Dura circa mezzo secondo.</p>
  </div>

  <!-- Registrazione Dati CSV -->
  <div class="box">
    <h2>Registrazione Dati (CSV)</h2>
    <div class="pid-form">
      <div><label>Intervallo (ms):</label><input type="number" id="input-log-interval" step="50" min="20" max="5000" value="100"></div>
      <div><label>Stato:</label><span id="log-status" style="font-size:13px; font-weight:bold; color:#666;">Ferma</span></div>
    </div>
    <div class="btn-group" style="margin-top:10px;">
      <button style="background-color:#28a745; color:white; border-color:#1e7e34;" onclick="startLogging()">▶ Avvia</button>
      <button style="background-color:#dc3545; color:white; border-color:#bd2130;" onclick="stopLogging()">■ Ferma</button>
      <button style="background-color:#6c757d; color:white; border-color:#545b62;" onclick="downloadLogCsv()">⬇ Scarica CSV</button>
    </div>
  </div>

  <script>
    let currentMode = 'auto';
    let pidInitialized = false;

    // Storico errH/errV per il mini-grafico di taratura: mostra a colpo d'occhio
    // se il PID sta convergendo o oscillando, cosa che il solo numero istantaneo non fa vedere.
    const ERR_HISTORY_LEN = 40;
    let errHistoryH = [];
    let errHistoryV = [];
    let lastDeadzone = 350;

    function pushErrHistory(errH, errV) {
      errHistoryH.push(errH);
      errHistoryV.push(errV);
      if (errHistoryH.length > ERR_HISTORY_LEN) errHistoryH.shift();
      if (errHistoryV.length > ERR_HISTORY_LEN) errHistoryV.shift();
    }

    function drawErrChart() {
      const canvas = document.getElementById('err-chart');
      if (!canvas || !canvas.getContext) return;
      const ctx = canvas.getContext('2d');
      const w = canvas.width, h = canvas.height;
      ctx.clearRect(0, 0, w, h);

      const allVals = errHistoryH.concat(errHistoryV, [lastDeadzone, -lastDeadzone]);
      const maxAbs = Math.max(200, ...allVals.map(v => Math.abs(v)));
      const toY = v => h / 2 - (v / maxAbs) * (h / 2 - 4);

      // Banda della zona morta: dentro questa fascia il PID è considerato "allineato"
      ctx.fillStyle = 'rgba(40, 167, 69, 0.15)';
      const yTop = toY(lastDeadzone);
      const yBot = toY(-lastDeadzone);
      ctx.fillRect(0, yTop, w, yBot - yTop);

      // Linea errore zero
      ctx.strokeStyle = '#ccc';
      ctx.beginPath();
      ctx.moveTo(0, h / 2);
      ctx.lineTo(w, h / 2);
      ctx.stroke();

      function drawLine(arr, color) {
        if (arr.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        arr.forEach((v, i) => {
          const x = (i / (ERR_HISTORY_LEN - 1)) * w;
          const y = toY(v);
          if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
        });
        ctx.stroke();
      }

      drawLine(errHistoryH, '#0056b3'); // Errore H
      drawLine(errHistoryV, '#d9534f'); // Errore V
    }

    // Traccia la transizione true->false di data.autotuning per sapere quando il
    // test di autotuning (bloccante lato firmware) è terminato e riflettere i
    // nuovi Kp/Ki/Kd nel form, senza bisogno di un secondo endpoint dedicato.
    let wasAutotuning = false;
    let autotunePrevKp = null, autotunePrevKi = null, autotunePrevKd = null;

    function updateActiveButton(mode) {
      currentMode = mode;
      ['auto', 'manual', 'night', 'eco'].forEach(m => {
        const b = document.getElementById('btn-' + m);
        if (b) {
          if (m === mode) b.classList.add('active');
          else b.classList.remove('active');
        }
      });
    }

    function updateUI(data) {
      document.getElementById('val-v').innerText = data.v.toFixed(1) + ' V';
      document.getElementById('val-i').innerText = Math.round(data.i) + ' mA';
      document.getElementById('val-p').innerText = data.p.toFixed(2) + ' W';
      if (data.energy !== undefined) document.getElementById('val-energy').innerText = data.energy.toFixed(2) + ' Wh';

      document.getElementById('ldr-tl').innerText = data.tl;
      document.getElementById('ldr-tr').innerText = data.tr;
      document.getElementById('ldr-bl').innerText = data.bl;
      document.getElementById('ldr-br').innerText = data.br;

      // Diagonali e Punto Morto
      if (data.diag1 !== undefined) document.getElementById('val-diag1').innerText = data.diag1;
      if (data.diag2 !== undefined) document.getElementById('val-diag2').innerText = data.diag2;
      if (data.diffDiag !== undefined) document.getElementById('val-diffdiag').innerText = data.diffDiag;

      // Stato Motori/PID: errore corrente, impulso servo e mini-grafico dell'errore
      if (data.errH !== undefined) document.getElementById('val-errh').innerText = Math.round(data.errH);
      if (data.errV !== undefined) document.getElementById('val-errv').innerText = Math.round(data.errV);
      if (data.velH !== undefined) document.getElementById('val-velh').innerText = data.velH + ' us';
      if (data.velV !== undefined) document.getElementById('val-velv').innerText = data.velV + ' us';
      if (data.deadzone !== undefined) lastDeadzone = data.deadzone;
      if (data.errH !== undefined && data.errV !== undefined) {
        pushErrHistory(data.errH, data.errV);
        drawErrChart();
      }

      const psIndicator = document.getElementById('powersave-indicator');
      if (psIndicator) psIndicator.style.display = (data.powerSaved === true) ? 'block' : 'none';

      // Fattori di calibrazione LDR correnti
      if (data.calTL !== undefined) document.getElementById('cal-tl').innerText = data.calTL.toFixed(3);
      if (data.calTR !== undefined) document.getElementById('cal-tr').innerText = data.calTR.toFixed(3);
      if (data.calBL !== undefined) document.getElementById('cal-bl').innerText = data.calBL.toFixed(3);
      if (data.calBR !== undefined) document.getElementById('cal-br').innerText = data.calBR.toFixed(3);

      // Stato registrazione CSV (i campi arrivano già dentro /api/data, nessuna chiamata separata)
      const logStatus = document.getElementById('log-status');
      if (logStatus && data.logging !== undefined) {
        if (data.logging) {
          logStatus.innerText = `Attiva (${data.logSamples}/${data.logMaxSamples})`;
          logStatus.style.color = '#28a745';
        } else {
          logStatus.innerText = `Ferma (${data.logSamples} campioni pronti)`;
          logStatus.style.color = '#666';
        }
      }

      const banner = document.getElementById('alert-banner');
      const statusSpan = document.getElementById('val-deadlock-status');
      const btnAutotune = document.getElementById('btn-autotune');

      // Autotuning: appena data.autotuning torna false dopo essere stato true,
      // il test è concluso (con o senza successo) e i valori Kp/Ki/Kd sono già
      // stati aggiornati lato firmware se il calcolo è andato a buon fine.
      if (data.autotuning === true) {
        if (btnAutotune) btnAutotune.disabled = true;
        wasAutotuning = true;
      } else if (wasAutotuning) {
        wasAutotuning = false;
        if (btnAutotune) btnAutotune.disabled = false;
        const changed = (data.kp !== autotunePrevKp) || (data.ki !== autotunePrevKi) || (data.kd !== autotunePrevKd);
        if (data.kp !== undefined) document.getElementById('input-kp').value = data.kp;
        if (data.ki !== undefined) document.getElementById('input-ki').value = data.ki;
        if (data.kd !== undefined) document.getElementById('input-kd').value = data.kd;
        if (changed) {
          alert(`Autotuning completato!\n\nNuovi parametri PID (Ziegler-Nichols da relay feedback):\nKp: ${data.kp}\nKi: ${data.ki}\nKd: ${data.kd}`);
        } else {
          alert('Autotuning terminato senza calcolare nuovi parametri: il sistema non ha oscillato a sufficienza entro il timeout (relè troppo piccolo, attrito eccessivo, o luce insufficiente). I parametri PID non sono stati modificati. Puoi controllare l\'andamento registrato scaricando il CSV.');
        }
      }

      if (data.autotuning === true) {
        banner.style.display = 'block';
        banner.className = 'alert-banner alert-autotune';
        banner.innerText = '🎯 Autotuning PID in corso (relay feedback)... i motori oscillano avanti e indietro.';
        statusSpan.innerText = 'Autotuning';
        statusSpan.style.color = '#6f42c1';
      } else if (data.findingSun === true) {
        banner.style.display = 'block';
        banner.className = 'alert-banner alert-sunsearch';
        banner.innerText = '☀️ Scansione 360° Ricerca Sole in corso...';
        statusSpan.innerText = 'Ricerca Sole';
        statusSpan.style.color = '#0056b3';
      } else if (data.deadlock === true) {
        banner.style.display = 'block';
        banner.className = 'alert-banner alert-deadlock';
        banner.innerText = '⚠️ Punto Morto Diagonale Rilevato! Sblocco posizione in corso...';
        statusSpan.innerText = 'Punto Morto (Sblocco)';
        statusSpan.style.color = '#dc3545';
      } else {
        banner.style.display = 'none';
        if (Math.abs(data.errH) <= data.deadzone && Math.abs(data.errV) <= data.deadzone) {
          statusSpan.innerText = 'Allineato (Fermo)';
          statusSpan.style.color = '#28a745';
        } else {
          statusSpan.innerText = 'Inseguimento Attivo';
          statusSpan.style.color = '#0056b3';
        }
      }

      // Popola i campi form al primo caricamento
      if (!pidInitialized) {
        if (data.kp !== undefined) document.getElementById('input-kp').value = data.kp;
        if (data.ki !== undefined) document.getElementById('input-ki').value = data.ki;
        if (data.kd !== undefined) document.getElementById('input-kd').value = data.kd;
        if (data.deadzone !== undefined) document.getElementById('input-deadzone').value = data.deadzone;
        if (data.deadlockThresh !== undefined) document.getElementById('input-deadlock').value = data.deadlockThresh;
        if (data.night !== undefined) document.getElementById('input-night').value = data.night;
        if (data.deadlockEnabled !== undefined) document.getElementById('input-pmenabled').checked = data.deadlockEnabled;
        pidInitialized = true;
      }

      if (data.mode && data.mode !== currentMode) {
        updateActiveButton(data.mode);
      }
    }

    function fetchData() {
      fetch('/api/data')
        .then(res => res.json())
        .then(data => {
          document.getElementById('status').innerText = 'Stato: Connesso';
          document.getElementById('status').classList.remove('offline');
          updateUI(data);
        })
        .catch(err => {
          document.getElementById('status').innerText = 'Stato: Disconnesso';
          document.getElementById('status').classList.add('offline');
        });
    }

    function setMode(mode) {
      updateActiveButton(mode);
      fetch('/api/mode?mode=' + encodeURIComponent(mode), {
        method: 'POST'
      })
      .then(res => res.json())
      .then(data => {
        if (data.mode) updateActiveButton(data.mode);
        fetchData();
      })
      .catch(err => console.error('Errore cambio modalita:', err));
    }

    function triggerFindSun() {
      if (confirm('Avviare la scansione a 360° per la ricerca del sole?')) {
        fetch('/api/findsun', { method: 'POST' })
          .then(res => res.json())
          .then(data => {
            fetchData();
          })
          .catch(err => console.error('Errore trigger ricerca sole:', err));
      }
    }

    function triggerAutotune() {
      const msg = 'Avviare l\'autotuning PID (relay feedback)?\n\n' +
        'I due assi oscillano avanti e indietro a velocità fissa per 10-30s ' +
        '(fino a un massimo di 45s) per stimare automaticamente Kp/Ki/Kd. ' +
        'Il tracker non insegue il sole durante il test.';
      if (!confirm(msg)) return;
      // Salva i valori correnti per poter dire, a fine test, se sono davvero
      // cambiati (il firmware lascia i parametri invariati se il test va in timeout).
      autotunePrevKp = parseFloat(document.getElementById('input-kp').value);
      autotunePrevKi = parseFloat(document.getElementById('input-ki').value);
      autotunePrevKd = parseFloat(document.getElementById('input-kd').value);
      fetch('/api/autotune', { method: 'POST' })
        .then(res => res.json())
        .then(() => fetchData())
        .catch(err => alert('Errore avvio autotuning: ' + err));
    }

    function sendCmd(cmd) {
      updateActiveButton('manual');
      fetch('/api/control?cmd=' + encodeURIComponent(cmd), {
        method: 'POST'
      })
      .catch(err => console.error('Errore invio comando:', err));
    }

    function savePID() {
      const kp = document.getElementById('input-kp').value;
      const ki = document.getElementById('input-ki').value;
      const kd = document.getElementById('input-kd').value;
      const deadzone = document.getElementById('input-deadzone').value;
      const deadlock = document.getElementById('input-deadlock').value;
      const night = document.getElementById('input-night').value;
      const pmEnabled = document.getElementById('input-pmenabled').checked ? '1' : '0';

      fetch(`/api/pid?kp=${encodeURIComponent(kp)}&ki=${encodeURIComponent(ki)}&kd=${encodeURIComponent(kd)}&deadzone=${encodeURIComponent(deadzone)}&deadlock=${encodeURIComponent(deadlock)}&night=${encodeURIComponent(night)}&pmEnabled=${pmEnabled}`, {
        method: 'POST'
      })
      .then(res => res.json())
      .then(data => {
        if (data.status === 'ok') {
          alert(`Parametri aggiornati sull'ESP32!\n\nKp: ${data.kp}\nKi: ${data.ki}\nKd: ${data.kd}\nDeadzone: ${data.deadzone}\nSoglia Deadlock: ${data.deadlock}\nSoglia Notte: ${data.night}\nSblocco Punto Morto: ${data.deadlockEnabled ? 'On' : 'Off'}`);
        } else {
          alert('Errore aggiornamento parametri!');
        }
        fetchData();
      })
      .catch(err => alert('Errore di connessione durante il salvataggio: ' + err));
    }

    function calibrateLDRs() {
      if (!confirm('Assicurati che i 4 sensori siano sotto luce uniforme (es. una lampada equidistante). Avviare la calibrazione?')) return;
      fetch('/api/calibrate', { method: 'POST' })
        .then(res => res.json())
        .then(data => {
          if (data.status === 'ok') {
            document.getElementById('cal-tl').innerText = data.calTL.toFixed(3);
            document.getElementById('cal-tr').innerText = data.calTR.toFixed(3);
            document.getElementById('cal-bl').innerText = data.calBL.toFixed(3);
            document.getElementById('cal-br').innerText = data.calBR.toFixed(3);
            alert('Calibrazione completata!');
          } else {
            alert('Errore durante la calibrazione');
          }
        })
        .catch(err => alert('Errore di connessione durante la calibrazione: ' + err));
    }

    function startLogging() {
      const interval = document.getElementById('input-log-interval').value;
      fetch('/api/log/start?interval=' + encodeURIComponent(interval), { method: 'POST' })
        .then(res => res.json())
        .then(() => fetchData())
        .catch(err => alert('Errore avvio registrazione: ' + err));
    }

    function stopLogging() {
      fetch('/api/log/stop', { method: 'POST' })
        .then(res => res.json())
        .then(() => fetchData())
        .catch(err => alert('Errore stop registrazione: ' + err));
    }

    function downloadLogCsv() {
      window.location.href = '/api/log/csv';
    }

    setInterval(fetchData, 500);
    fetchData();
  </script>
</body>
</html>
)rawliteral";

#endif
