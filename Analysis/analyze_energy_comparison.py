#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_energy_comparison.py — Analisi del confronto energetico tracker vs pannello fisso

VERSIONE SENZA COLONNA TIMESTAMP: usa solo t_ms (millisecondi relativi
dall'inizio del log, quello che il firmware genera davvero — vedi nota
nel main.cpp: nessun RTC/NTP, solo millis()). Le fasce orarie sono quindi
calcolate come terzi della durata totale del test (primo terzo/secondo
terzo/ultimo terzo), non come "mattina/mezzogiorno/pomeriggio" in senso
calendariale, dato che senza un orologio reale non possiamo saperlo con
certezza.

Prende in input un CSV con colonne:
  t_ms, P_prod_fisso_mW, P_cons_fisso_mW, P_netto_fisso_mW,
  P_prod_tracker_mW, P_cons_tracker_mW, P_netto_tracker_mW

Calcola:
  - Energia totale (Wh) per produzione/consumo/netto, entrambi i sistemi
  - Test-t appaiato sul netto (implementato senza scipy)
  - Vantaggio percentuale per terzo della durata del test
Genera 4 grafici (PNG) nella cartella corrente, con asse x in ore
trascorse dall'inizio del test (non orario di calendario).

USO:
    python3 analyze_energy_comparison.py confronto.csv --output-prefix giorno1

Richiede: pandas, numpy, matplotlib. NON richiede scipy.
"""

import argparse
import math
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ============================================================================
# Statistica senza scipy (stessa funzione beta incompleta di anova_doe.py)
# ============================================================================
def _betacf(a, b, x, max_iter=200, eps=3e-12):
    qab, qap, qam = a + b, a + 1.0, a - 1.0
    c, d = 1.0, 1.0 - qab * x / qap
    if abs(d) < 1e-30:
        d = 1e-30
    d = 1.0 / d
    h = d
    for m in range(1, max_iter + 1):
        m2 = 2 * m
        aa = m * (b - m) * x / ((qam + m2) * (a + m2))
        d = 1.0 + aa * d
        if abs(d) < 1e-30:
            d = 1e-30
        c = 1.0 + aa / c
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        h *= d * c
        aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2))
        d = 1.0 + aa * d
        if abs(d) < 1e-30:
            d = 1e-30
        c = 1.0 + aa / c
        if abs(c) < 1e-30:
            c = 1e-30
        d = 1.0 / d
        delta = d * c
        h *= delta
        if abs(delta - 1.0) < eps:
            break
    return h


def _betai(a, b, x):
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    ln_beta = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
    bt = math.exp(ln_beta + a * math.log(x) + b * math.log(1.0 - x))
    if x < (a + 1.0) / (a + b + 2.0):
        return bt * _betacf(a, b, x) / a
    return 1.0 - bt * _betacf(b, a, 1.0 - x) / b


def t_dist_pvalue_two_tailed(t_value, df):
    x = df / (df + t_value ** 2)
    return _betai(df / 2.0, 0.5, x)


def paired_ttest(a, b):
    """Equivalente di scipy.stats.ttest_rel(a, b)."""
    diff = np.asarray(a) - np.asarray(b)
    n = len(diff)
    mean_d = np.mean(diff)
    sd_d = np.std(diff, ddof=1)
    se_d = sd_d / math.sqrt(n)
    t = mean_d / se_d if se_d > 0 else float("nan")
    p = t_dist_pvalue_two_tailed(abs(t), n - 1)
    return t, p, mean_d, sd_d


# ============================================================================
# Analisi
# ============================================================================
def energy_Wh(power_mW, t_hours):
    return np.trapezoid(power_mW, t_hours) / 1000.0


def load(path):
    df = pd.read_csv(path)
    required = ["t_ms", "P_prod_fisso_mW", "P_cons_fisso_mW", "P_netto_fisso_mW",
                "P_prod_tracker_mW", "P_cons_tracker_mW", "P_netto_tracker_mW"]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"Colonne mancanti nel CSV: {missing}")
    df["t_hours"] = (df["t_ms"] - df["t_ms"].iloc[0]) / 3600000.0
    # Fasce come terzi della durata totale del test (nessun orologio reale disponibile)
    durata_totale = df["t_hours"].iloc[-1]
    df["terzo"] = pd.cut(
        df["t_hours"], bins=[-0.001, durata_totale / 3, 2 * durata_totale / 3, durata_totale + 0.001],
        labels=["1o terzo", "2o terzo", "3o terzo"]
    )
    return df


def print_report(df):
    print("=" * 70)
    print("BILANCIO ENERGETICO")
    print("=" * 70)
    print(f"Durata totale del test: {df['t_hours'].iloc[-1]:.2f} ore, {len(df)} campioni")
    print()
    for base in ["prod", "cons", "netto"]:
        e_f = energy_Wh(df[f"P_{base}_fisso_mW"], df["t_hours"])
        e_t = energy_Wh(df[f"P_{base}_tracker_mW"], df["t_hours"])
        diff = e_t - e_f
        pct = (diff / e_f * 100) if e_f != 0 else float("nan")
        print(f"{base:6s}: fisso={e_f:8.2f} Wh | tracker={e_t:8.2f} Wh | "
              f"diff={diff:+8.2f} Wh ({pct:+.1f}%)")

    print()
    t, p, mean_d, sd_d = paired_ttest(df["P_netto_tracker_mW"], df["P_netto_fisso_mW"])
    print(f"Test-t appaiato sul netto: differenza media = {mean_d:+.1f} mW "
          f"(sd={sd_d:.1f}), t={t:.3f}, p={p:.2e}")
    n_wins = (df["P_prod_tracker_mW"] > df["P_prod_fisso_mW"]).sum()
    print(f"Il tracker produce di piu' in {n_wins}/{len(df)} istanti")

    print()
    print("Vantaggio per terzo della durata del test:")
    for label in ["1o terzo", "2o terzo", "3o terzo"]:
        sub = df[df["terzo"] == label]
        if len(sub) == 0 or sub["P_prod_fisso_mW"].mean() == 0:
            continue
        gain = (sub["P_prod_tracker_mW"].mean() / sub["P_prod_fisso_mW"].mean() - 1) * 100
        print(f"  {label:10s}: fisso={sub['P_prod_fisso_mW'].mean():6.0f}mW  "
              f"tracker={sub['P_prod_tracker_mW'].mean():6.0f}mW  vantaggio={gain:+.0f}%")


def make_plots(df, prefix):
    t = df["t_hours"]

    # 1) Serie temporale della produzione
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, df["P_prod_fisso_mW"], label="Pannello fisso", color="#999999", linewidth=1.8)
    ax.plot(t, df["P_prod_tracker_mW"], label="Tracker", color="#0b5394", linewidth=1.8)
    ax.set_ylabel("Potenza prodotta (mW)")
    ax.set_xlabel("Ore trascorse dall'inizio del test")
    ax.set_title("Produzione nel tempo: tracker vs pannello fisso")
    ax.legend()
    plt.tight_layout()
    plt.savefig(f"{prefix}_produzione_nel_tempo.png", dpi=150)
    plt.close(fig)

    # 2) Serie temporale del netto
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(t, df["P_netto_fisso_mW"], label="Pannello fisso", color="#999999", linewidth=1.8)
    ax.plot(t, df["P_netto_tracker_mW"], label="Tracker", color="#0b5394", linewidth=1.8)
    ax.axhline(0, color="black", linewidth=0.7, linestyle="--")
    ax.set_ylabel("Potenza netta (mW)")
    ax.set_xlabel("Ore trascorse dall'inizio del test")
    ax.set_title("Bilancio netto nel tempo (produzione - consumo)")
    ax.legend()
    plt.tight_layout()
    plt.savefig(f"{prefix}_netto_nel_tempo.png", dpi=150)
    plt.close(fig)

    # 3) Energia totale a barre
    fig, ax = plt.subplots(figsize=(6, 4.5))
    labels = ["Produzione", "Consumo", "Netto"]
    e_fisso = [energy_Wh(df[f"P_{b}_fisso_mW"], df["t_hours"]) for b in ["prod", "cons", "netto"]]
    e_tracker = [energy_Wh(df[f"P_{b}_tracker_mW"], df["t_hours"]) for b in ["prod", "cons", "netto"]]
    x = np.arange(len(labels))
    width = 0.35
    ax.bar(x - width/2, e_fisso, width, label="Fisso", color="#999999")
    ax.bar(x + width/2, e_tracker, width, label="Tracker", color="#0b5394")
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Energia (Wh)")
    ax.set_title("Bilancio energetico totale del test")
    ax.legend()
    plt.tight_layout()
    plt.savefig(f"{prefix}_energia_totale.png", dpi=150)
    plt.close(fig)

    # 4) Vantaggio per terzo della durata
    fascia_labels, gains = [], []
    for label in ["1o terzo", "2o terzo", "3o terzo"]:
        sub = df[df["terzo"] == label]
        if len(sub) == 0 or sub["P_prod_fisso_mW"].mean() == 0:
            continue
        gains.append(sub["P_prod_tracker_mW"].mean() / sub["P_prod_fisso_mW"].mean() * 100 - 100)
        fascia_labels.append(label)
    fig, ax = plt.subplots(figsize=(6, 4.5))
    ax.bar(fascia_labels, gains, color="#0b5394", alpha=0.85)
    ax.set_ylabel("Vantaggio produzione tracker (%)")
    ax.set_title("Vantaggio del tracker per terzo della durata del test")
    for i, g in enumerate(gains):
        ax.text(i, g + max(gains)*0.02, f"{g:+.0f}%", ha="center", fontweight="bold")
    plt.tight_layout()
    plt.savefig(f"{prefix}_vantaggio_fasce.png", dpi=150)
    plt.close(fig)

    print(f"\nGrafici salvati con prefisso '{prefix}_*.png'")


def main():
    parser = argparse.ArgumentParser(description="Analizza un confronto energetico tracker vs fisso.")
    parser.add_argument("csv_file", help="Percorso del CSV di confronto")
    parser.add_argument("--output-prefix", default="confronto", help="Prefisso dei file PNG generati")
    args = parser.parse_args()

    try:
        df = load(args.csv_file)
        print_report(df)
        make_plots(df, args.output_prefix)
    except (ValueError, FileNotFoundError) as e:
        print(f"Errore: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
