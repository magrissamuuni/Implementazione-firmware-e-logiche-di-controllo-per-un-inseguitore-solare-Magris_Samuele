#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REQUIRED_COLUMNS = ["t_ms", "errH", "errV", "pulseH_us", "pulseV_us", "tl", "tr", "bl", "br"]

# Costanti allineate al firmware (main.cpp) — aggiornarle qui se cambiano lì.
FW_DEADZONE_HYSTERESIS_RATIO = 0.6      # DEADZONE_HYSTERESIS_RATIO
FW_RELAY_AMPLITUDE = 2.0                # RELAY_AMPLITUDE ("d")
FW_RELAY_HYSTERESIS = 40.0              # RELAY_HYSTERESIS
FW_RELAY_DISCARD_HALFCYCLES = 6         # RELAY_DISCARD_HALFCYCLES


def load_csv(path: str) -> pd.DataFrame:
    """Carica un CSV esportato dal tracker e verifica che abbia le colonne attese."""
    df = pd.read_csv(path)
    missing = [c for c in REQUIRED_COLUMNS if c not in df.columns]
    if missing:
        raise ValueError(
            f"Il file '{path}' non sembra un log del tracker: mancano le colonne {missing}. "
            f"Colonne trovate: {list(df.columns)}"
        )
    if len(df) < 5:
        raise ValueError(f"Il file '{path}' ha troppo pochi campioni ({len(df)}) per un'analisi affidabile.")
    return df


def analyze_doe(df: pd.DataFrame, deadzone: float = None, settle_window_ms: float = 1000.0,
                 hysteresis_ratio: float = FW_DEADZONE_HYSTERESIS_RATIO) -> dict:

    results = {}
    t = df["t_ms"].values.astype(float)
    effective_threshold = (deadzone * hysteresis_ratio) if deadzone is not None else None

    for axis in ("H", "V"):
        err = df[f"err{axis}"].values.astype(float)

        # Valore di assestamento: media dell'errore nell'ultima finestra di tempo del log
        t_end = t[-1]
        final_mask = t >= (t_end - settle_window_ms)
        if not final_mask.any():
            final_mask = np.array([len(t) - 1])
        steady_value = float(np.mean(err[final_mask]))
        residual_error = abs(steady_value)

        # Overshoot: massimo scostamento assoluto dal valore di assestamento
        overshoot = float(np.max(np.abs(err - steady_value)))

        # Tempo di assestamento: ultimo istante FUORI dalla soglia di rientro;
        # il campione subito dopo segna l'inizio dell'assestamento stabile.
        settling_time_ms = None
        if effective_threshold is not None:
            outside = np.where(np.abs(err) > effective_threshold)[0]
            if len(outside) == 0:
                settling_time_ms = 0.0
            else:
                idx = min(outside[-1] + 1, len(t) - 1)
                settling_time_ms = float(t[idx] - t[0])

        results[axis] = {
            "residual_error": round(residual_error, 2),
            "overshoot": round(overshoot, 2),
            "settling_time_ms": round(settling_time_ms, 1) if settling_time_ms is not None else None,
            "steady_value": round(steady_value, 2),
        }

    return results


def parse_params_from_filename(filename: str) -> dict:
    """
    Prova a leggere deadzone/deadlock/replica dal nome del file, seguendo la
    convenzione suggerita (es. run14_dz350_pm700_rep2.csv).
    """
    params = {}
    m = re.search(r"dz(\d+)", filename)
    if m:
        params["deadzone"] = float(m.group(1))
    m = re.search(r"pm(\d+)", filename)
    if m:
        params["deadlock"] = float(m.group(1))
    m = re.search(r"rep(\d+)", filename)
    if m:
        params["replica"] = int(m.group(1))
    m = re.search(r"run(\d+)", filename)
    if m:
        params["run"] = int(m.group(1))
    return params


def run_doe_single(path: str, deadzone: float, settle_window_ms: float, hysteresis_ratio: float):
    df = load_csv(path)
    res = analyze_doe(df, deadzone=deadzone, settle_window_ms=settle_window_ms,
                       hysteresis_ratio=hysteresis_ratio)

    print(f"\n=== Analisi DOE: {path} ===")
    print(f"Campioni nel log: {len(df)}  |  Durata: {df['t_ms'].iloc[-1] / 1000:.1f} s")
    if deadzone is None:
        print("(Nota: nessuna zona morta specificata -> settling time non calcolato. Usa --deadzone)")
    else:
        eff = deadzone * hysteresis_ratio
        print(f"(Soglia di rientro usata per il settling time: {deadzone} x {hysteresis_ratio} = {eff:.0f})")
    for axis in ("H", "V"):
        r = res[axis]
        print(f"\nAsse {axis}:")
        print(f"  Errore residuo:        {r['residual_error']}")
        print(f"  Overshoot:             {r['overshoot']}")
        st = r["settling_time_ms"]
        print(f"  Tempo di assestamento: {st} ms" if st is not None else "  Tempo di assestamento: n/d")


def run_doe_batch(folder: str, output: str, deadzone_override: float, settle_window_ms: float,
                   hysteresis_ratio: float):
    folder_path = Path(folder)
    csv_files = sorted(folder_path.glob("*.csv"))
    if not csv_files:
        print(f"Nessun file .csv trovato in '{folder}'.")
        return

    rows = []
    for f in csv_files:
        try:
            df = load_csv(str(f))
        except ValueError as e:
            print(f"[SALTATO] {f.name}: {e}")
            continue

        params = parse_params_from_filename(f.name)
        deadzone = deadzone_override if deadzone_override is not None else params.get("deadzone")

        res = analyze_doe(df, deadzone=deadzone, settle_window_ms=settle_window_ms,
                           hysteresis_ratio=hysteresis_ratio)

        row = {
            "file": f.name,
            "run": params.get("run"),
            "deadzone": params.get("deadzone"),
            "deadlock": params.get("deadlock"),
            "replica": params.get("replica"),
            "residual_error_H": res["H"]["residual_error"],
            "residual_error_V": res["V"]["residual_error"],
            "overshoot_H": res["H"]["overshoot"],
            "overshoot_V": res["V"]["overshoot"],
            "settling_time_H_ms": res["H"]["settling_time_ms"],
            "settling_time_V_ms": res["V"]["settling_time_ms"],
        }
        rows.append(row)
        print(f"[OK] {f.name}")

    if not rows:
        print("Nessun file valido elaborato.")
        return

    summary = pd.DataFrame(rows)
    summary.to_csv(output, index=False)
    print(f"\nRiepilogo di {len(rows)} prove salvato in: {output}")
    print("Colonne pronte per l'ANOVA: deadzone, deadlock, replica, residual_error_H/V, overshoot_H/V, settling_time_H/V_ms")


def _replicate_relay_axis(t: np.ndarray, err: np.ndarray, hysteresis: float,
                           discard_halfcycles: int):
    """Ricostruisce lo stato del relè campione per campione, come relayStep()."""
    relay_dir = 1
    half_cycle_peak = err[0]
    last_switch_t = t[0]
    half_cycle_index = 0
    pos_peaks, neg_peaks, periods_ms = [], [], []

    for i in range(1, len(err)):
        e = err[i]
        if relay_dir > 0:
            if e > half_cycle_peak:
                half_cycle_peak = e
        else:
            if e < half_cycle_peak:
                half_cycle_peak = e

        should_switch = (e < -hysteresis) if relay_dir > 0 else (e > hysteresis)
        if should_switch:
            half_cycle_index += 1
            duration = t[i] - last_switch_t

            if half_cycle_index > discard_halfcycles:
                if relay_dir > 0:
                    pos_peaks.append(half_cycle_peak)
                else:
                    neg_peaks.append(half_cycle_peak)
                periods_ms.append(duration)

            relay_dir = -relay_dir
            half_cycle_peak = e
            last_switch_t = t[i]

    return pos_peaks, neg_peaks, periods_ms


def _compute_ku_pu(pos_peaks, neg_peaks, periods_ms, relay_amplitude):
    """Stessa formula della lambda computeKuPu() in runRelayAutotune()."""
    avg_pos = float(np.mean(pos_peaks)) if pos_peaks else 0.0
    avg_neg = float(np.mean(neg_peaks)) if neg_peaks else 0.0
    a = (avg_pos - avg_neg) / 2.0
    pu_s = (2.0 * (float(np.mean(periods_ms)) / 1000.0)) if periods_ms else 0.0
    ku = (4.0 * relay_amplitude / (math.pi * a)) if a > 1.0 else 0.0
    return ku, pu_s, a


def analyze_relay(df: pd.DataFrame, axis: str = "both", relay_amplitude: float = FW_RELAY_AMPLITUDE,
                   hysteresis: float = FW_RELAY_HYSTERESIS,
                   discard_halfcycles: int = FW_RELAY_DISCARD_HALFCYCLES) -> dict:
    t = df["t_ms"].values.astype(float)
    axes_to_process = ["H", "V"] if axis == "both" else [axis]

    per_axis = {}
    for ax in axes_to_process:
        err = df[f"err{ax}"].values.astype(float)
        pos_peaks, neg_peaks, periods_ms = _replicate_relay_axis(t, err, hysteresis, discard_halfcycles)

        if len(periods_ms) < 3:
            raise ValueError(
                f"Asse {ax}: solo {len(periods_ms)} semicicli utili dopo lo scarto del transitorio "
                f"(discard_halfcycles={discard_halfcycles}). Servono almeno 3-4: il log e' abbastanza "
                "lungo? Il relay feedback e' effettivamente partito su questo asse?"
            )

        ku, pu_s, a = _compute_ku_pu(pos_peaks, neg_peaks, periods_ms, relay_amplitude)
        if ku <= 0 or pu_s <= 0:
            raise ValueError(f"Asse {ax}: Ku/Pu non validi (ampiezza troppo piccola o dati insufficienti).")

        per_axis[ax] = {
            "n_halfcycles_used": len(periods_ms),
            "amplitude_a": round(a, 2),
            "Pu_s": round(pu_s, 3),
            "Ku": round(ku, 4),
        }

    # Media tra gli assi analizzati, esattamente come fa runRelayAutotune()
    # quando axis == "both" (unico Kp/Ki/Kd condiviso dal firmware).
    Ku_final = float(np.mean([per_axis[a]["Ku"] for a in per_axis]))
    Pu_final = float(np.mean([per_axis[a]["Pu_s"] for a in per_axis]))

    Kp = 0.6 * Ku_final
    Ki = 2.0 * Kp / Pu_final
    Kd = Kp * Pu_final / 8.0

    return {
        "per_axis": per_axis,
        "Ku": round(Ku_final, 4),
        "Pu_s": round(Pu_final, 3),
        "Kp": round(Kp, 4),
        "Ki": round(Ki, 4),
        "Kd": round(Kd, 4),
    }


def run_relay(path: str, axis: str, amplitude: float, hysteresis: float, discard: int):
    df = load_csv(path)
    res = analyze_relay(df, axis=axis, relay_amplitude=amplitude,
                         hysteresis=hysteresis, discard_halfcycles=discard)

    print(f"\n=== Analisi Relay Feedback / Autotuning: {path} ===")
    print(f"Campioni nel log: {len(df)}  |  Durata: {df['t_ms'].iloc[-1] / 1000:.1f} s")
    print(f"(Replica l'algoritmo di relayStep()/runRelayAutotune() del firmware: stessa isteresi, "
          f"stesso scarto del transitorio — il risultato dovrebbe combaciare con quanto stampato "
          f"dal firmware sul monitor seriale durante il test)")

    for ax, r in res["per_axis"].items():
        print(f"\nAsse {ax}:")
        print(f"  Semicicli usati:       {r['n_halfcycles_used']}")
        print(f"  Ampiezza (a):          {r['amplitude_a']}")
        print(f"  Pu:                    {r['Pu_s']} s")
        print(f"  Ku:                    {r['Ku']}")

    print(f"\n--- Media tra gli assi (come fa il firmware) ---")
    print(f"Ku: {res['Ku']}   Pu: {res['Pu_s']} s")
    print(f"\nGuadagni PID (Ziegler-Nichols closed-loop):")
    print(f"  Kp = {res['Kp']}")
    print(f"  Ki = {res['Ki']}")
    print(f"  Kd = {res['Kd']}")
    print(f"\nConfronta questi valori con quelli stampati dal firmware su Serial durante l'autotuning.")
    print(f"Per riapplicarli manualmente: GET /api/pid?kp={res['Kp']}&ki={res['Ki']}&kd={res['Kd']}")


def main():
    parser = argparse.ArgumentParser(
        description="Analizza i CSV esportati dal solar tracker (prove DOE o relay feedback/autotuning)."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_doe = sub.add_parser("doe", help="Analizza una singola prova DOE")
    p_doe.add_argument("file", help="Percorso del CSV")
    p_doe.add_argument("--deadzone", type=float, default=None,
                        help="Zona morta usata in quella prova (serve per il settling time)")
    p_doe.add_argument("--settle-window", type=float, default=1000.0,
                        help="Finestra finale (ms) usata per calcolare il valore di assestamento (default 1000)")
    p_doe.add_argument("--hysteresis-ratio", type=float, default=FW_DEADZONE_HYSTERESIS_RATIO,
                        help=f"Rapporto soglia di rientro / zona morta, come DEADZONE_HYSTERESIS_RATIO nel firmware (default {FW_DEADZONE_HYSTERESIS_RATIO})")
    p_doe.add_argument("--no-hysteresis", action="store_true",
                        help="Usa la zona morta nominale (rapporto 1.0) invece della soglia di rientro con isteresi")

    p_batch = sub.add_parser("doe-batch", help="Analizza tutte le prove DOE di una cartella")
    p_batch.add_argument("folder", help="Cartella contenente i CSV delle prove")
    p_batch.add_argument("--output", default="riepilogo_doe.csv", help="Nome del CSV di riepilogo da generare")
    p_batch.add_argument("--deadzone", type=float, default=None,
                          help="Forza una zona morta fissa per tutti i file invece di leggerla dal nome file")
    p_batch.add_argument("--settle-window", type=float, default=1000.0,
                          help="Finestra finale (ms) usata per calcolare il valore di assestamento (default 1000)")
    p_batch.add_argument("--hysteresis-ratio", type=float, default=FW_DEADZONE_HYSTERESIS_RATIO,
                          help=f"Rapporto soglia di rientro / zona morta (default {FW_DEADZONE_HYSTERESIS_RATIO})")
    p_batch.add_argument("--no-hysteresis", action="store_true",
                          help="Usa la zona morta nominale (rapporto 1.0) invece della soglia di rientro con isteresi")

    p_relay = sub.add_parser("relay", help="Analizza un log di relay feedback/autotuning (Ku, Pu, Kp/Ki/Kd)")
    p_relay.add_argument("file", help="Percorso del CSV")
    p_relay.add_argument("--axis", choices=["H", "V", "both"], default="both",
                          help="Asse da analizzare, o 'both' per mediare come fa il firmware (default both)")
    p_relay.add_argument("--amplitude", type=float, default=FW_RELAY_AMPLITUDE,
                          help=f"Velocita' fissa (d) del rele' bang-bang, come RELAY_AMPLITUDE nel firmware (default {FW_RELAY_AMPLITUDE})")
    p_relay.add_argument("--hysteresis", type=float, default=FW_RELAY_HYSTERESIS,
                          help=f"Isteresi di commutazione del rele', come RELAY_HYSTERESIS nel firmware (default {FW_RELAY_HYSTERESIS})")
    p_relay.add_argument("--discard", type=int, default=FW_RELAY_DISCARD_HALFCYCLES,
                          help=f"Semicicli iniziali da scartare come transitorio, come RELAY_DISCARD_HALFCYCLES (default {FW_RELAY_DISCARD_HALFCYCLES})")

    args = parser.parse_args()

    try:
        if args.command == "doe":
            ratio = 1.0 if args.no_hysteresis else args.hysteresis_ratio
            run_doe_single(args.file, args.deadzone, args.settle_window, ratio)
        elif args.command == "doe-batch":
            ratio = 1.0 if args.no_hysteresis else args.hysteresis_ratio
            run_doe_batch(args.folder, args.output, args.deadzone, args.settle_window, ratio)
        elif args.command == "relay":
            run_relay(args.file, args.axis, args.amplitude, args.hysteresis, args.discard)
    except (ValueError, FileNotFoundError) as e:
        print(f"\nErrore: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
