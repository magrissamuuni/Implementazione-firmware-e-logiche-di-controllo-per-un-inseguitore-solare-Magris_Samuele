#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import itertools
import math
import sys

import numpy as np
import pandas as pd

RESPONSE_COLUMNS = [
    "residual_error_H", "residual_error_V",
    "overshoot_H", "overshoot_V",
    "settling_time_H_ms", "settling_time_V_ms",
]

FW_DEADZONE_HYSTERESIS_RATIO = 0.6  # DEADZONE_HYSTERESIS_RATIO nel firmware


def _betacf(a: float, b: float, x: float, max_iter: int = 200, eps: float = 3e-12) -> float:
    qab = a + b
    qap = a + 1.0
    qam = a - 1.0
    c = 1.0
    d = 1.0 - qab * x / qap
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


def _betai(a: float, b: float, x: float) -> float:
    if x <= 0.0:
        return 0.0
    if x >= 1.0:
        return 1.0
    ln_beta = math.lgamma(a + b) - math.lgamma(a) - math.lgamma(b)
    bt = math.exp(ln_beta + a * math.log(x) + b * math.log(1.0 - x))
    if x < (a + 1.0) / (a + b + 2.0):
        return bt * _betacf(a, b, x) / a
    else:
        return 1.0 - bt * _betacf(b, a, 1.0 - x) / b


def f_dist_pvalue(f_value: float, d1: int, d2: int) -> float:
    if f_value <= 0:
        return 1.0
    x = d2 / (d2 + d1 * f_value)
    return _betai(d2 / 2.0, d1 / 2.0, x)


def t_dist_pvalue_two_tailed(t_value: float, df: float) -> float:
    x = df / (df + t_value ** 2)
    return _betai(df / 2.0, 0.5, x)


def f_oneway_manual(*groups):
    all_data = np.concatenate(groups)
    grand_mean = np.mean(all_data)
    k = len(groups)
    n_total = len(all_data)

    ss_between = sum(len(g) * (np.mean(g) - grand_mean) ** 2 for g in groups)
    ss_within = sum(np.sum((g - np.mean(g)) ** 2) for g in groups)
    ss_total = ss_between + ss_within

    df_between = k - 1
    df_within = n_total - k

    ms_between = ss_between / df_between
    ms_within = ss_within / df_within if df_within > 0 else np.nan

    if ms_within == 0 or np.isnan(ms_within):
        return {"f_stat": np.nan, "p_value": np.nan, "ss_between": ss_between,
                "ss_within": ss_within, "ss_total": ss_total,
                "df_between": df_between, "df_within": df_within, "ms_within": ms_within}

    f_stat = ms_between / ms_within
    p_value = f_dist_pvalue(f_stat, df_between, df_within)
    return {"f_stat": f_stat, "p_value": p_value, "ss_between": ss_between,
            "ss_within": ss_within, "ss_total": ss_total,
            "df_between": df_between, "df_within": df_within, "ms_within": ms_within}


def welch_ttest_manual(a, b):
    mean_a, mean_b = np.mean(a), np.mean(b)
    var_a, var_b = np.var(a, ddof=1), np.var(b, ddof=1)
    n_a, n_b = len(a), len(b)

    se_sq = var_a / n_a + var_b / n_b
    if se_sq <= 0:
        return np.nan, np.nan
    se = math.sqrt(se_sq)
    t_stat = (mean_a - mean_b) / se

    denom = (var_a / n_a) ** 2 / (n_a - 1) + (var_b / n_b) ** 2 / (n_b - 1)
    df = se_sq ** 2 / denom if denom > 0 else n_a + n_b - 2

    p_value = t_dist_pvalue_two_tailed(abs(t_stat), df)
    return t_stat, p_value


def linear_regression_manual(x, y):
    """Regressione lineare semplice y = a + b*x, senza scipy (solo numpy)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)

    x_mean, y_mean = np.mean(x), np.mean(y)
    ss_xx = np.sum((x - x_mean) ** 2)
    ss_xy = np.sum((x - x_mean) * (y - y_mean))
    ss_yy = np.sum((y - y_mean) ** 2)

    if ss_xx == 0:
        return None

    slope = ss_xy / ss_xx
    intercept = y_mean - slope * x_mean

    y_pred = intercept + slope * x
    ss_res = np.sum((y - y_pred) ** 2)
    r_squared = 1.0 - ss_res / ss_yy if ss_yy > 0 else np.nan

    df = n - 2
    if df <= 0 or ss_res <= 0:
        return {"slope": slope, "intercept": intercept, "r_squared": r_squared,
                "p_value": np.nan, "n": n}

    s_err = math.sqrt(ss_res / df)
    se_slope = s_err / math.sqrt(ss_xx)
    t_stat = slope / se_slope if se_slope > 0 else np.nan
    p_value = t_dist_pvalue_two_tailed(abs(t_stat), df) if not np.isnan(t_stat) else np.nan

    return {"slope": slope, "intercept": intercept, "r_squared": r_squared,
            "p_value": p_value, "n": n}


def pearson_correlation_manual(x, y):
    """Correlazione di Pearson con p-value, senza scipy."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    n = len(x)
    if n < 3:
        return None

    r = float(np.corrcoef(x, y)[0, 1])
    df = n - 2
    denom = 1.0 - r ** 2
    if denom <= 0:
        return {"r": r, "p_value": 0.0, "n": n}
    t_stat = r * math.sqrt(df / denom)
    p_value = t_dist_pvalue_two_tailed(abs(t_stat), df)
    return {"r": r, "p_value": p_value, "n": n}


def simulate_power(group_means, pooled_std, n_per_group, alpha=0.05, n_trials=3000, seed=42):

    rng = np.random.default_rng(seed)
    significant = 0
    for _ in range(n_trials):
        groups = [rng.normal(m, pooled_std, n_per_group) for m in group_means]
        res = f_oneway_manual(*groups)
        if not np.isnan(res["p_value"]) and res["p_value"] < alpha:
            significant += 1
    return significant / n_trials


def required_n_for_power(group_means, pooled_std, alpha=0.05, target_power=0.8,
                          n_trials=1500, max_n=200, seed=42):
    """Cerca il piu' piccolo n per replica che raggiunge target_power (ricerca incrementale)."""
    for n in range(2, max_n + 1):
        power = simulate_power(group_means, pooled_std, n, alpha, n_trials, seed)
        if power >= target_power:
            return n, power
    return None, None


def load_summary(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    if len(df) == 0:
        raise ValueError(f"Il file '{path}' e' vuoto.")
    return df


def one_way_anova(df: pd.DataFrame, factor_col: str, response_col: str):
    sub = df.dropna(subset=[response_col, factor_col])
    groups = [g[response_col].values for _, g in sub.groupby(factor_col)]
    groups = [g for g in groups if len(g) > 0]

    if len(groups) < 2:
        return None

    fres = f_oneway_manual(*groups)
    stats_table = sub.groupby(factor_col)[response_col].agg(["mean", "std", "count"]).round(2)

    return {**fres, "stats_table": stats_table, "sub": sub, "groups": groups}


def pairwise_bonferroni(sub: pd.DataFrame, factor_col: str, response_col: str, alpha: float):
    levels = sorted(sub[factor_col].dropna().unique())
    pairs = list(itertools.combinations(levels, 2))
    n_comparisons = len(pairs)
    alpha_corrected = alpha / n_comparisons if n_comparisons > 0 else alpha

    rows = []
    for a, b in pairs:
        vals_a = sub[sub[factor_col] == a][response_col].values
        vals_b = sub[sub[factor_col] == b][response_col].values
        t_stat, p_raw = welch_ttest_manual(vals_a, vals_b)
        p_bonf = min(p_raw * n_comparisons, 1.0) if not np.isnan(p_raw) else np.nan
        rows.append({
            "livello_A": a, "livello_B": b,
            "media_A": round(float(np.mean(vals_a)), 2),
            "media_B": round(float(np.mean(vals_b)), 2),
            "p_raw": round(p_raw, 4) if not np.isnan(p_raw) else None,
            "p_bonferroni": round(p_bonf, 4) if not np.isnan(p_bonf) else None,
            "significativo": "SI" if (not np.isnan(p_bonf) and p_bonf < alpha) else "no",
        })

    return pd.DataFrame(rows), alpha_corrected


def print_regression(sub, factor_col, response_col):
    reg = linear_regression_manual(sub[factor_col].values, sub[response_col].values)
    if reg is None:
        print("\nRegressione lineare: non calcolabile (fattore costante).")
        return
    sig = "significativa" if (not np.isnan(reg["p_value"]) and reg["p_value"] < 0.05) else "non significativa"
    print(f"\nRegressione lineare ({response_col} = a + b*{factor_col}):")
    print(f"  Pendenza (b):   {reg['slope']:.5f}  ->  +1 unita' di {factor_col} = {reg['slope']:+.3f} su {response_col}")
    print(f"  Intercetta (a): {reg['intercept']:.3f}")
    print(f"  R^2:            {reg['r_squared']:.4f}  ({reg['r_squared']*100:.1f}% della variabilita' spiegata dalla relazione lineare)")
    print(f"  p-value pendenza: {reg['p_value']:.4f}  ->  pendenza {sig}")


def print_effect_size(fres):
    ss_between, ss_total, ms_within = fres["ss_between"], fres["ss_total"], fres["ms_within"]
    df_between, df_within = fres["df_between"], fres["df_within"]

    eta_sq = ss_between / ss_total if ss_total > 0 else np.nan
    # omega^2: stima meno distorta di eta^2, puo' risultare leggermente negativa
    # (in tal caso si interpreta come "effetto praticamente nullo")
    omega_sq = (ss_between - df_between * ms_within) / (ss_total + ms_within) if (ss_total + ms_within) > 0 else np.nan

    def interpret(e):
        if np.isnan(e):
            return "n/d"
        if e < 0.01:
            return "trascurabile"
        elif e < 0.06:
            return "piccolo"
        elif e < 0.14:
            return "medio"
        else:
            return "grande"

    print(f"\nEffect size:")
    print(f"  eta^2   = {eta_sq:.4f}  ({interpret(eta_sq)}, convenzioni di Cohen)")
    print(f"  omega^2 = {omega_sq:.4f}  (versione meno distorta di eta^2)")


def print_power_analysis(fres, stats_table, alpha, do_search):
    means = stats_table["mean"].values
    ns = stats_table["count"].values
    ms_within = fres["ms_within"]

    if np.isnan(ms_within) or ms_within <= 0:
        print("\nAnalisi di potenza: non calcolabile.")
        return

    pooled_std = math.sqrt(ms_within)
    current_n = int(np.min(ns))  # repliche minime tra i livelli (caso peggiore)

    power_now = simulate_power(means, pooled_std, current_n, alpha=alpha, n_trials=3000)
    print(f"\nAnalisi di potenza (simulazione Monte Carlo, 3000 run):")
    print(f"  Con le medie/deviazione standard osservate e n={current_n} repliche per livello:")
    print(f"  Potenza stimata attuale: {power_now*100:.1f}%  "
          f"({'buona' if power_now >= 0.8 else 'insufficiente'} per rilevare in modo affidabile questo effetto)")

    if do_search:
        req_n, req_power = required_n_for_power(means, pooled_std, alpha=alpha, target_power=0.8, n_trials=1500)
        if req_n is not None:
            print(f"  Repliche necessarie per l'80% di potenza: ~{req_n} per livello "
                  f"(vs le {current_n} attuali)")
        else:
            print(f"  Repliche necessarie per l'80% di potenza: >200 (effetto troppo piccolo rispetto al rumore "
                  f"per essere ragionevolmente rilevabile con questo disegno)")


def check_hysteresis_coherence(df, factor_col, response_col, axis, hysteresis_ratio, alpha=0.05):
    """Verifica che l'errore residuo osservato sia coerente con la soglia teorica
    di rientro nella zona morta (deadzone * hysteresis_ratio) prevista dal firmware."""
    sub = df.dropna(subset=[response_col, factor_col])
    if sub.empty:
        return

    violations = []
    for _, row in sub.iterrows():
        threshold = row[factor_col] * hysteresis_ratio
        if row[response_col] > threshold:
            violations.append({
                "file": row.get("file", "?"),
                factor_col: row[factor_col],
                response_col: row[response_col],
                "soglia_teorica": round(threshold, 1),
            })

    print(f"\nVerifica di coerenza teorica (asse {axis}): errore residuo osservato vs soglia "
          f"teorica ({factor_col} x {hysteresis_ratio}):")
    if not violations:
        print(f"  Tutte le {len(sub)} prove rispettano la soglia teorica. Comportamento firmware coerente con il progetto.")
    else:
        print(f"  {len(violations)} / {len(sub)} prove superano la soglia teorica attesa:")
        print(pd.DataFrame(violations).to_string(index=False))
        print("  (Puo' indicare rumore/outlier in quelle prove specifiche, o un log fermato troppo presto"
              " prima del vero assestamento — vale la pena controllarle singolarmente.)")


def cross_axis_correlation(df, metric_base):
    col_h, col_v = f"{metric_base}_H", f"{metric_base}_V"
    if col_h not in df.columns or col_v not in df.columns:
        return
    sub = df.dropna(subset=[col_h, col_v])
    if len(sub) < 3:
        return

    res = pearson_correlation_manual(sub[col_h].values, sub[col_v].values)
    if res is None:
        return

    sig = "significativa" if res["p_value"] < 0.05 else "non significativa"
    print(f"\nCorrelazione tra {col_h} e {col_v} (n={res['n']} prove):")
    print(f"  r = {res['r']:.3f}, p = {res['p_value']:.4f}  ->  correlazione {sig}")
    if res["p_value"] < 0.05:
        direction = "positiva" if res["r"] > 0 else "negativa"
        print(f"  (Correlazione {direction}: quando l'errore su H e' alto, tende ad esserlo anche su V "
              f"— suggerisce una fonte di rumore condivisa tra i due assi, es. luce variabile o vento, "
              f"piu' che rumore indipendente per asse.)")


def print_std_comparison(stats_table):
    stds = stats_table["std"].dropna()
    if len(stds) < 2 or stds.min() == 0:
        return
    ratio = stds.max() / stds.min()
    print(f"\nConfronto delle deviazioni standard tra livelli:")
    print(stats_table[["std"]].to_string())
    print(f"  Rapporto max/min deviazione standard: {ratio:.2f}"
          + ("  (elevato: la varianza NON e' costante tra i livelli — l'assunzione di omoschedasticita' "
             "dell'ANOVA e' discutibile, i risultati vanno interpretati con qualche cautela in piu')"
             if ratio > 2 else "  (ragionevolmente stabile tra i livelli)"))


def run(summary_csv: str, factor: str, alpha: float, make_plot: bool, hysteresis_ratio: float,
        power_search: bool):
    df = load_summary(summary_csv)

    if factor not in df.columns:
        raise ValueError(f"Colonna '{factor}' non trovata nel CSV. Colonne disponibili: {list(df.columns)}")

    levels = sorted(df[factor].dropna().unique())
    print(f"Fattore analizzato: {factor}")
    print(f"Livelli trovati: {levels}")
    print(f"Numero totale di prove: {len(df)}")
    print(f"Soglia di significativita' (alpha): {alpha}\n")

    if make_plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

    any_response_found = False

    for resp in RESPONSE_COLUMNS:
        if resp not in df.columns or df[resp].dropna().empty:
            continue
        any_response_found = True

        res = one_way_anova(df, factor, resp)
        print(f"{'=' * 70}")
        print(f"Variabile di risposta: {resp}")
        print(f"{'=' * 70}")

        if res is None or np.isnan(res["f_stat"]):
            print("Dati insufficienti per questa variabile, salto.\n")
            continue

        print("\nMedia / deviazione standard / numero prove per livello:")
        print(res["stats_table"].to_string())

        is_significant = res["p_value"] < alpha
        verdict = "SIGNIFICATIVO" if is_significant else "non significativo"
        print(f"\nANOVA: F = {res['f_stat']:.3f}, p = {res['p_value']:.4f}  ->  {verdict}")

        if is_significant:
            print(f"\n(Confronto a coppie tra i livelli, Bonferroni-corretto:)")
            pairwise_df, alpha_corr = pairwise_bonferroni(res["sub"], factor, resp, alpha)
            print(f"(alpha corretto per {len(pairwise_df)} confronti: {alpha_corr:.4f})\n")
            print(pairwise_df.to_string(index=False))
        else:
            print(f"\n(Le differenze osservate sono compatibili con il rumore sperimentale a questo "
                  f"livello di significativita'.)")

        # --- Nuove analisi ---
        print_regression(res["sub"], factor, resp)
        print_effect_size(res)
        print_power_analysis(res, res["stats_table"], alpha, power_search)
        print_std_comparison(res["stats_table"])

        if resp.startswith("residual_error"):
            axis = resp.split("_")[-1]
            check_hysteresis_coherence(df, factor, resp, axis, hysteresis_ratio, alpha)

        if make_plot:
            means = res["stats_table"]["mean"]
            stds = res["stats_table"]["std"].fillna(0)
            fig, ax = plt.subplots(figsize=(5, 4))
            means.plot(kind="bar", yerr=stds, ax=ax, capsize=4, color="#0b5394", alpha=0.85)
            ax.set_ylabel(resp)
            ax.set_xlabel(factor)
            ax.set_title(f"Effetto principale: {resp}" + (" *" if is_significant else ""))
            plt.xticks(rotation=0)
            plt.tight_layout()
            outname = f"effetto_{resp}.png"
            plt.savefig(outname, dpi=150)
            plt.close(fig)
            print(f"\nGrafico salvato: {outname}")

            # Grafico di regressione (scatter + retta), solo se abbiamo la pendenza
            reg = linear_regression_manual(res["sub"][factor].values, res["sub"][resp].values)
            if reg is not None:
                fig2, ax2 = plt.subplots(figsize=(5, 4))
                x_vals = res["sub"][factor].values
                y_vals = res["sub"][resp].values
                ax2.scatter(x_vals, y_vals, color="#0b5394", alpha=0.6, label="prove")
                x_line = np.linspace(x_vals.min(), x_vals.max(), 50)
                y_line = reg["intercept"] + reg["slope"] * x_line
                ax2.plot(x_line, y_line, color="#cc0000",
                         label=f"fit (R2={reg['r_squared']:.2f})")
                ax2.set_xlabel(factor)
                ax2.set_ylabel(resp)
                ax2.set_title(f"Regressione: {resp} vs {factor}")
                ax2.legend()
                plt.tight_layout()
                outname2 = f"regressione_{resp}.png"
                plt.savefig(outname2, dpi=150)
                plt.close(fig2)
                print(f"Grafico di regressione salvato: {outname2}")

        print()

    if not any_response_found:
        print("Nessuna delle colonne di risposta attese e' presente/valorizzata nel CSV.")
        print(f"Colonne cercate: {RESPONSE_COLUMNS}")
        return

    # --- Correlazione incrociata H/V, una volta sola per metrica ---
    print(f"\n{'=' * 70}")
    print("Correlazioni tra assi H e V")
    print(f"{'=' * 70}")
    for base in ["residual_error", "overshoot", "settling_time"]:
        cross_axis_correlation(df, base)


def main():
    parser = argparse.ArgumentParser(
        description="ANOVA + regressione + effect size + potenza + coerenza teorica, sul riepilogo DOE (senza scipy)."
    )
    parser.add_argument("summary_csv", help="Percorso del CSV di riepilogo (es. riepilogo_doe.csv)")
    parser.add_argument("--factor", default="deadzone", help="Colonna del fattore da analizzare (default: deadzone)")
    parser.add_argument("--alpha", type=float, default=0.05, help="Soglia di significativita' (default 0.05)")
    parser.add_argument("--plot", action="store_true", help="Salva grafici (effetto principale + regressione) per ogni risposta")
    parser.add_argument("--hysteresis-ratio", type=float, default=FW_DEADZONE_HYSTERESIS_RATIO,
                         help=f"Rapporto soglia di rientro / zona morta per la verifica di coerenza (default {FW_DEADZONE_HYSTERESIS_RATIO})")
    parser.add_argument("--power-search", action="store_true",
                         help="Cerca anche il numero di repliche necessario per l'80%% di potenza (piu' lento, richiede piu' simulazioni)")

    args = parser.parse_args()

    try:
        run(args.summary_csv, args.factor, args.alpha, args.plot, args.hysteresis_ratio, args.power_search)
    except (ValueError, FileNotFoundError) as e:
        print(f"\nErrore: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
