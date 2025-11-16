# -*- coding: utf-8 -*-

import json, math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def identify_first_order_single_period_clean(
    csv_path: str,
    low_us: float = 1330.0,
    high_us: float = 1334.0,
    period_s: float = 10.0,
    avg_window_s: float = 1.0,
    sampling_rate_hz: float | None = 100.0,
    out_plot: str = "first_order_fit.png",
    out_json: str = "first_order_params.json",
):
    # --- Carga de datos ---
    df = pd.read_csv(csv_path, comment="#")
    t_col = "Recording_Time_s" if "Recording_Time_s" in df.columns else None
    u_col = "PWM_Input_us"
    y_col = "Arm_Angle_deg"
    if u_col not in df.columns or y_col not in df.columns:
        raise RuntimeError("El CSV debe contener columnas 'PWM_Input_us' y 'Arm_Angle_deg'.")

    t_raw = pd.to_numeric(df[t_col], errors="coerce").to_numpy() if t_col else None
    u = pd.to_numeric(df[u_col], errors="coerce").to_numpy()
    y = pd.to_numeric(df[y_col], errors="coerce").to_numpy()

    mask = ~np.isnan(u) & ~np.isnan(y)
    if t_raw is not None:
        mask &= ~np.isnan(t_raw)
    u, y = u[mask], y[mask]
    if t_raw is not None:
        t_raw = t_raw[mask]

    # --- Eje de tiempo ---
    if t_raw is not None and (np.nanmax(t_raw) - np.nanmin(t_raw)) > 0:
        t = t_raw.astype(float)
        dt_med = np.nanmedian(np.diff(t[: min(1000, len(t))]))
        if dt_med and dt_med > 1.0:  # probablemente milisegundos
            t = t / 1000.0
        t = t - t[0]
        time_unit = "s"
        if sampling_rate_hz is None and dt_med and dt_med > 0:
            sampling_rate_hz = 1.0 / dt_med
    else:
        t = np.arange(len(u), dtype=float)    # eje en muestras
        time_unit = "samples"

    # --- Niveles LOW/HIGH más cercanos a 1330/1334 (no se cambian) ---
    ur = np.rint(u[~np.isnan(u)])
    if ur.size >= 2:
        vals = np.unique(ur)
        low_val = float(vals[np.argmin(np.abs(vals - low_us))])
        high_val = float(vals[np.argmin(np.abs(vals - high_us))])
        if low_val == high_val and len(vals) >= 2:
            low_val, high_val = float(vals[0]), float(vals[-1])
        if low_val > high_val:
            low_val, high_val = high_val, low_val
    else:
        low_val, high_val = float(low_us), float(high_us)

    # --- Parámetros de ventana y conversión a muestras si aplica ---
    if time_unit == "samples":
        to_samples = (lambda s: int(round(s * sampling_rate_hz))) if sampling_rate_hz else (lambda s: int(round(s * 100)))
        min_hold = max(50, to_samples(period_s / 3.0))   # ~≥3.3 s por lado
        prefer_len = to_samples(period_s)                 # ~10 s deseados
        left_avg_len = max(30, to_samples(avg_window_s))
        right_avg_len = max(30, to_samples(avg_window_s))
    else:
        min_hold = max(1.0, period_s / 3.0)
        prefer_len = period_s
        left_avg_len = avg_window_s
        right_avg_len = avg_window_s

    # --- Buscar periodo único LOW→HIGH (por PWM) ---
    state = np.where(np.abs(u - low_val) <= np.abs(u - high_val), 0, 1)   # 0: low, 1: high
    transitions = np.where((state[:-1] == 0) & (state[1:] == 1))[0]
    best_idx, best_quality, window = None, -np.inf, None

    for idx in transitions:
        i_start = idx
        while i_start > 0 and state[i_start - 1] == 0:
            i_start -= 1
        i_end = idx + 1
        while i_end + 1 < len(state) and state[i_end + 1] == 1:
            i_end += 1
        hold_low = t[idx] - t[i_start]
        hold_high = t[i_end] - t[idx + 1]
        if hold_low >= min_hold and hold_high >= min_hold:
            y_low = y[i_start:idx + 1]
            y_high = y[idx + 1:i_end + 1]
            if len(y_low) >= 5 and len(y_high) >= 5:
                noise = float(np.nanstd(y_low) + np.nanstd(y_high))
                duration = t[i_end] - t[i_start]
                quality = -noise - abs(duration - prefer_len)
                if quality > best_quality:
                    best_quality = quality
                    best_idx = idx
                    window = (i_start, i_end)

    if window is None:
        # Respaldo: primer cruce de umbral
        i_start, i_end = 0, len(t) - 1
        mid = (low_val + high_val) / 2.0
        cand = np.where((u[:-1] < mid) & (u[1:] >= mid))[0]
        step_idx = int(cand[0] + 1) if len(cand) else int(np.argmax(u))
    else:
        i_start, i_end = window
        step_idx = best_idx + 1

    # --- Recortar al periodo seleccionado ---
    t = t[i_start:i_end + 1]
    u = u[i_start:i_end + 1]
    y = y[i_start:i_end + 1]
    step_idx = step_idx - i_start
    t_step = t[step_idx]

    # --- Promedios para y0 e y_ss (dentro del periodo) ---
    if time_unit == "samples":
        left_idx = max(0, step_idx - int(left_avg_len))
        right_idx = min(len(t), step_idx + 1 + int(right_avg_len))
        low_mask = np.arange(left_idx, step_idx)
        high_mask = np.arange(step_idx + 1, right_idx)
    else:
        low_mask = np.where((t >= (t_step - left_avg_len)) & (t < t_step))[0]
        high_mask = np.where((t > t_step) & (t <= min(t_step + right_avg_len, t[-1])))[0]
        if low_mask.size == 0:
            low_mask = np.where((t >= (t_step - 2 * left_avg_len)) & (t < t_step))[0]
        if high_mask.size == 0:
            high_mask = np.where((t > t_step) & (t <= min(t_step + 2 * right_avg_len, t[-1])))[0]

    y0 = float(np.nanmean(y[low_mask]))
    yss = float(np.nanmean(y[high_mask]))
    dy = yss - y0
    if abs(dy) < 1e-12:
        raise RuntimeError("Δy demasiado pequeño para estimar τ.")

    # --- Normalización y τ (63.2%) ---
    y_norm = (y - y0) / dy
    target = 1.0 - math.exp(-1.0)   # 0.632...
    y_tau_level = y0 + target * dy

    post_idx = np.arange(step_idx, len(t))
    yy = y_norm[post_idx]
    tt = t[post_idx]
    tau_axis_value = None
    for k in range(1, len(yy)):
        if (yy[k - 1] <= target and yy[k] >= target) or (yy[k - 1] >= target and yy[k] <= target):
            t1, t2 = tt[k - 1], tt[k]
            y1, y2 = yy[k - 1], yy[k]
            frac = (target - y1) / (y2 - y1) if (y2 != y1) else 0.0
            tau_axis_value = (t1 + frac * (t2 - t1)) - 0.0  # relativo a t_step
            break
    # Fallback (log-lin 10–90%) si no hay cruce claro
    if tau_axis_value is None:
        mask_fit = (yy >= 0.1) & (yy <= 0.9)
        if np.sum(mask_fit) >= 5:
            xf = tt[mask_fit] - 0.0
            yf = 1 - yy[mask_fit]
            yf = np.clip(yf, 1e-6, None)
            A = np.vstack([xf, np.ones_like(xf)]).T
            coeff, *_ = np.linalg.lstsq(A, np.log(yf), rcond=None)
            slope = coeff[0]
            tau_axis_value = -1.0 / slope if slope < 0 else np.nan

    if time_unit == "samples":
        tau_samples = float(tau_axis_value) if (tau_axis_value is not None and np.isfinite(tau_axis_value)) else None
        tau_seconds = float(tau_samples / sampling_rate_hz) if (sampling_rate_hz and tau_samples is not None) else None
        tau_label = f"τ ≈ {tau_samples:.2f} muestras" + (f" ({tau_seconds:.3f} s @ {sampling_rate_hz:.0f} Hz)" if tau_seconds is not None else "")
    else:
        tau_samples = None
        tau_seconds = float(tau_axis_value) if (tau_axis_value is not None and np.isfinite(tau_axis_value)) else None
        tau_label = f"τ ≈ {tau_seconds:.3f} s"

    # --- Modelo 1er orden “limpio” ---
    y_model = np.full_like(y, np.nan, dtype=float)
    for i in range(len(t)):
        if i < step_idx:
            y_model[i] = y0
        else:
            dt = (i - step_idx) if time_unit == "samples" else (t[i] - t_step)
            if time_unit == "samples" and tau_samples is not None:
                y_model[i] = y0 + dy * (1.0 - math.exp(-dt / tau_samples))
            elif time_unit == "s" and tau_seconds is not None:
                y_model[i] = y0 + dy * (1.0 - math.exp(-dt / tau_seconds))

    # --- Pulso ideal en ángulo (y0 → yss) ---
    u_angle = np.full_like(y, y0)
    u_angle[step_idx:] = yss

    # --- Gráfica (una sola figura, sin seaborn, sin colores forzados) ---
    fig = plt.figure(figsize=(10, 5))
    plt.plot(t, y, label="Ángulo (real)")
    plt.plot(t, y_model, linestyle="--", label="Modelo 1er orden")
    plt.plot(t, u_angle, linestyle="-.", label="Pulso ideal (y0→yss)")
    plt.axhline(y0, linestyle=":", label=f"y0 = {y0:.3f}°")
    plt.axhline(yss, linestyle=":", label=f"y_ss = {yss:.3f}°")
    plt.axhline(y_tau_level, linestyle=":", label="Nivel 63.2%")
    plt.axvline(t[step_idx], linestyle=":", label="Inicio del step")
    if tau_axis_value is not None and np.isfinite(tau_axis_value):
        plt.axvline(t[step_idx] + tau_axis_value, linestyle=":", label=tau_label)

    plt.title(f"Periodo único LOW→HIGH @ {low_val:.0f}→{high_val:.0f} µs | {tau_label}")
    plt.xlabel(f"Tiempo [{time_unit}]")
    plt.ylabel("Ángulo [deg]")
    plt.legend()
    plt.tight_layout()
    fig.savefig(out_plot, dpi=150)
    plt.close(fig)

    # --- Guardar JSON ---
    result = {
        "period_window_indices": {"i_start": int(i_start), "i_step": int(step_idx + i_start), "i_end": int(i_end)},
        "operating_point": {"y0_deg": float(y0), "y_ss_deg": float(yss), "delta_y_deg": float(dy)},
        "pwm_levels_us": {"low": float(low_val), "high": float(high_val)},
        "tau": {
            "samples": float(tau_samples) if tau_samples is not None else None,
            "seconds": float(tau_seconds) if tau_seconds is not None else None,
            "time_axis_unit": time_unit,
            "sampling_rate_hz_used": float(sampling_rate_hz) if sampling_rate_hz else None,
        },
        "artifacts": {"plot": out_plot, "json": out_json},
        "notes": "Periodo único LOW→HIGH; normalización en torno a y0/y_ss; τ por 63.2%; sin curva de duty.",
    }
    with open(out_json, "w") as f:
        json.dump(result, f, indent=2)

    return result
