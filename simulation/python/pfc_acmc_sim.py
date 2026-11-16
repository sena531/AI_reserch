#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ブリッジレス昇圧PFC 平均電流制御(ACMC)閉ループ オフライン検証シミュレータ
================================================================================
対象回路: bridgeless PFC.scicir
  AC100Vrms(50Hz) -> DC400V、定電流負荷 IDC=1.25A (500W)
  L=350uH(Esr 10m) / C=1.2mF(Esr 80m) / Fsw=100kHz
  Q1,Q2: ローサイドスイッチ Ron=100m、ボディダイオード Vonb=1V/Ronb=100m
  D1,D2: Vf=700m / Ron=100m
二重ループ（内側=電流ループ、外側=直流バス電圧ループ）の平均電流制御を
素子の導通降下まで含めて離散時間で再現する。

Scideam スクリプト closed_loop_bridgeless_pfc.sciprc と制御則を 1:1 対応させて
あるため、ここで得たゲインをそのままスクリプトへ転記できる。

【サンプリング周波数について】
  既定値は指定どおり  電流ループ 50 Hz / 電圧ループ 100 kHz。
  ・電圧ループ: サンプリング100kHzでも帯域は約8Hzに設計するので問題ない
    （サンプリング周波数 != ループ帯域）。100Hzリプルは30Hz LPF 2段で除去。
  ・電流ループ: 50Hzでは商用1周期に1回しかデューティを更新できないため、
    瞬時波形整形は「完全フィードフォワード」が担当し、
    PI は1周期分の平均電流誤差（＝振幅誤差）を補正する役割になる。
    F_ILOOP を上げれば同じコードがそのまま通常の瞬時ACMCとして動作する。

使い方:
  py -3 pfc_acmc_sim.py                      # 既定条件（電流50Hz/電圧100kHz）
  py -3 pfc_acmc_sim.py --f-iloop 100e3      # 教科書的ACMC
  py -3 pfc_acmc_sim.py --sweep              # 電流ループ実行周波数の比較
  py -3 pfc_acmc_sim.py --load-step          # 負荷ステップ応答
  py -3 pfc_acmc_sim.py --switching          # 実PWM（スイッチングリプル込み）
  py -3 pfc_acmc_sim.py --startup            # ソフトスタート（141V->400V）
  py -3 pfc_acmc_sim.py --l-err 1.2          # 実回路のLが公称値からずれている場合
  py -3 pfc_acmc_sim.py --pwm-steps 1000     # 実機PWMのデューティ分解能を模擬

設計メモと結果のまとめ: README_pfc_control.md
Scideam版の移植確認:     verify_sciprc_port.py
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass, field

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


# =============================================================================
# 1. 回路・制御パラメータ
# =============================================================================
@dataclass
class Params:
    # ---- 電力回路（"bridgeless PFC.scicir" の実値）---------------------------
    vac_rms: float = 100.0      # VAC.Voltage [Vrms]（Scideamの交流電圧源は実効値指定）
    f_line: float = 50.0        # VAC.Frequency [Hz]
    vbus_ref: float = 400.0     # 直流バス電圧指令 [V]（回路側は電流源負荷なので制御で決まる）
    L: float = 350.0e-6         # L.Inductance [H]
    RL: float = 0.010           # L.Esr [ohm]
    C: float = 1.2e-3           # C.Capacitance [F]
    c_esr: float = 0.080        # C.Esr [ohm]（測定電圧のリプルに効く）
    fsw: float = 100.0e3        # MainFrequency [Hz]

    # ---- 素子の導通損（bridgeless PFC.scicir より）--------------------------
    # 電流経路：
    #   充電時 : VAC → L → Q(ON) → GND → 反対側スイッチのボディダイオード → VAC
    #   放出時 : VAC → L → D(昇圧ダイオード) → Vbus → GND → 同上 → VAC
    ron_sw: float = 0.100       # Q1/Q2 OnResistance [ohm]
    ron_d: float = 0.100        # D1/D2 OnResistance [ohm]
    vf_d: float = 0.700         # D1/D2 TurnOnVoltage [V]
    ron_ret: float = 0.100      # 帰路スイッチのボディダイオード Ronb [ohm]
    vf_ret: float = 1.000       # 帰路スイッチのボディダイオード Vonb [V]
    sync_return: bool = False   # True: 帰路スイッチを常時ONで駆動（同期整流、Vonb=0）

    # ---- 負荷 ---------------------------------------------------------------
    load: str = "cc"            # "cc"=定電流 / "r"=抵抗 / "cp"=定電力
    io: float = 1.25            # 定電流負荷 [A]  (400V x 1.25A = 500W)

    # ---- 制御ループ実行周波数 ----------------------------------------------
    f_scan: float = 100.0e3     # 制御スキャン周波数 [Hz] (= Scideamのサブ周波数)
    f_iloop: float = 50.0       # 電流ループ実行周波数 [Hz]
    f_vloop: float = 100.0e3    # 電圧ループ実行周波数 [Hz]

    # ---- ループ帯域（ゲインはここから自動算出） ----------------------------
    fc_i: float = 0.0           # 電流ループ交差周波数 [Hz] (0=自動)
    fc_v: float = 8.0           # 電圧ループ交差周波数 [Hz]（位相余裕 約47deg）
    fz_ratio_i: float = 0.1     # 電流PIゼロ = fc_i * この比（ゲイン余裕で決まる場合は自動）
    gm_target: float = 0.30     # 電流ループのゲイン余裕目標（0.30 ≒ 10.5 dB）
    fz_ratio_v: float = 0.25    # 電圧PIゼロ = fc_v * この比
    f_vfilt: float = 30.0       # バス電圧LPF遮断周波数 [Hz] (1次x2段, 100Hz -21dB)
    # F_ILOOP がこの値[Hz]未満なら PI 出力（デューティ補正）を正弦テンプレートで
    # 整形する。低レートでは整形ONが有利、2kHz以上ではOFF（素のACMC）が有利。
    shape_thr: float = 1000.0
    f_dfilt: float = 2.0e3      # dv/dt 推定用LPF [Hz]（増減の符号判定に使う）
    didt_analytic: bool = True  # di_ref/dt を解析式 Ipk*w*|cos| で求める

    # ---- 制限値 -------------------------------------------------------------
    # d_max はゼロクロス歪みに直結する。d=1 は「昇圧スイッチが常時ON」で
    # インダクタ電圧≒0（電流を保持）。d_max を下げると vac≒0 の区間で
    # (1-d_max)*Vbus の逆電圧がインダクタに加わり、電流が負に振れて歪む。
    #   d_max = 0.98 -> (1-d)*Vbus = 8 V の逆電圧 -> THD 約 8 %
    #   d_max = 0.995-> 2 V                       -> THD 約 3.7 %
    # 実機で最小オフ時間が必要な場合はここを実際の制限値に合わせること。
    d_max: float = 0.995
    d_min: float = 0.0
    ipk_max: float = 10.6       # 電流指令振幅の上限 [A] (定格7.07Aの1.5倍)
    dpi_max: float = 0.30       # 電流PI出力（デューティ補正）の上限
    # 実機PWMのデューティ分解能（1周期あたりの刻み数、0=理想）。
    # 分解能 1/N のデューティ誤差は (1/N)*Vbus の電圧誤差としてインダクタに
    # 積分されるため、電流ループが遅いほど効いてくる。
    #   例) 100MHz カウンタ / 100kHz -> 1000 ステップ
    pwm_steps: int = 0

    # ---- シミュレーション設定 ----------------------------------------------
    t_sim: float = 1.00         # シミュレーション時間 [s]（電圧ループが5Hzなので1s必要）
    meas_cycles: int = 4        # 評価に使う末尾の商用周期数
    switching: bool = False     # True: 実PWMモデル / False: 状態平均化モデル
    startup: bool = False       # True: 141V から立ち上げる
    l_err: float = 1.0          # プラント側 L の誤差係数（制御器は公称値を使用）

    # ---- 負荷ステップ -------------------------------------------------------
    load_step_time: float | None = None   # ステップ時刻 [s]（None=なし）
    io_after: float = 0.0                 # ステップ後の負荷電流 [A]

    # ---- 派生量 -------------------------------------------------------------
    vac_pk: float = field(init=False)
    w_line: float = field(init=False)
    po: float = field(init=False)

    def __post_init__(self):
        self.vac_pk = self.vac_rms * math.sqrt(2.0)
        self.w_line = 2.0 * math.pi * self.f_line
        self.po = self.vbus_ref * self.io          # 定格出力電力 [W]

    # ---- 導通経路の等価定数 -------------------------------------------------
    @property
    def r_eq(self) -> float:
        """インダクタ電流が常に見る等価直列抵抗 [ohm]。

        充電時は ron_sw、放出時は ron_d を通るが、この回路では両者が等しいので
        デューティに依らず一定になる（d*ron_sw + (1-d)*ron_d = ron_sw）。
        両者を別々の値にした場合は d=0.5 での近似値になる。
        """
        return self.RL + self.ron_ret + 0.5 * (self.ron_sw + self.ron_d)

    @property
    def v_ret(self) -> float:
        """帰路の定電圧降下 [V]（ボディダイオード順方向。同期整流なら 0）"""
        return 0.0 if self.sync_return else self.vf_ret

    def load_current(self, vbus: float, io_now: float) -> float:
        """負荷モデル：定電流 / 抵抗 / 定電力"""
        if self.load == "cc":
            return io_now
        if self.load == "cp":
            return (self.vbus_ref * io_now) / max(vbus, 1.0)
        return vbus / (self.vbus_ref / io_now)     # 抵抗 R = Vbus_ref / Io


PO_HYST = 0.005     # 極性判定のヒステリシス幅（入力ピーク比）


# =============================================================================
# 2. 制御器の部品
# =============================================================================
def design_pi(fc: float, fz: float, plant_gain_at_fc: float):
    """交差周波数 fc で開ループゲイン1[0dB]になる PI ゲインを返す。

    PI(s) = Kp * (1 + 2*pi*fz / s)  として
      Kp = 1 / |Plant(j2*pi*fc)|
      Ki = Kp * 2*pi*fz        （速度形では Ki*Ts を毎回積算）
    fz << fc としておけば fc における PI の位相遅れは小さい。
    """
    kp = 1.0 / plant_gain_at_fc
    ki = kp * 2.0 * math.pi * fz
    return kp, ki


def current_plant_gain(p: Params, f: float) -> float:
    """デューティ -> 整流座標のインダクタ電流 のゲイン |(Vbus+Vf) / (R_eq + j*w*L)|"""
    w = 2.0 * math.pi * f
    return (p.vbus_ref + p.vf_d) / math.hypot(p.r_eq, w * p.L)


def voltage_plant_gain(p: Params, f: float) -> float:
    """電流指令振幅 Ipk -> バス電圧 のゲイン。

    電力平衡 (1/2)*Vac_pk*Ipk = Vbus*i_bus より d(i_bus)/d(Ipk) = Vac_pk/(2*Vbus)。
    これがコンデンサ C を充電するので、低周波では純積分器:
        |Gvi(f)| = (Vac_pk / (2*Vbus)) / (2*pi*f*C)
    定電流負荷では負荷極が存在しないため、この式がそのまま成立する。
    """
    return (p.vac_pk / (2.0 * p.vbus_ref)) / (2.0 * math.pi * f * p.C)


def loop_margin(tf, f_lo: float = 1e-2, f_hi: float = 1e5, n: int = 4000):
    """開ループ伝達関数 tf(jw) からゲイン交差周波数・位相余裕・ゲイン余裕を求める。

    戻り値: (fc [Hz], 位相余裕 [deg], f_180 [Hz], ゲイン余裕 [dB])
    交差が見つからない項目は nan。
    """
    f = np.logspace(math.log10(f_lo), math.log10(f_hi), n)
    t = tf(2j * math.pi * f)
    mag = np.abs(t)
    ph = np.unwrap(np.angle(t))

    fc = pm = float("nan")
    idx = np.where(mag >= 1.0)[0]
    if len(idx) > 0 and idx[-1] < n - 1:
        k = idx[-1]
        w = math.log10(mag[k]) / (math.log10(mag[k]) - math.log10(mag[k + 1]))
        fc = 10 ** (math.log10(f[k]) + w * (math.log10(f[k + 1]) - math.log10(f[k])))
        pm = 180.0 + np.angle(tf(2j * math.pi * fc), deg=True)

    # 位相が -180deg を最初に下回る点でのゲイン
    f180 = gm = float("nan")
    below = np.where(ph <= -math.pi)[0]
    if len(below) > 0 and below[0] > 0:
        k = below[0]
        w = (ph[k - 1] + math.pi) / (ph[k - 1] - ph[k])
        f180 = 10 ** (math.log10(f[k - 1]) + w * (math.log10(f[k]) - math.log10(f[k - 1])))
        gm = -20.0 * math.log10(abs(tf(2j * math.pi * f180)))
    return fc, pm, f180, gm


def current_loop_tf(p: Params, kp: float, ki: float, ts: float):
    """電流ループ開ループ： PI x プラント x (平均化+ZOHの無駄時間 1.5*Ts)"""
    def tf(s):
        pi = kp + ki / s
        plant = (p.vbus_ref + p.vf_d) / (p.r_eq + s * p.L)
        delay = np.exp(-s * 1.5 * ts)
        return pi * plant * delay
    return tf


def voltage_loop_tf(p: Params, kp: float, ki: float, fvf: float):
    """電圧ループ開ループ： PI x 積分器プラント x 1次LPF2段"""
    wf = 2.0 * math.pi * fvf

    def tf(s):
        pi = kp + ki / s
        plant = (p.vac_pk / (2.0 * p.vbus_ref)) / (s * p.C)
        lpf = 1.0 / (1.0 + s / wf) ** 2
        return pi * plant * lpf
    return tf


class PI:
    """クランプ型アンチワインドアップ付き離散PI（位置形）。

    u[n] = Kp*e[n] + I[n],  I[n] = I[n-1] + Ki*Ts*e[n]
    出力が制限に当たったら I を u が制限値になるところで止める。
    """

    def __init__(self, kp: float, ki: float, ts: float, lo: float, hi: float, i0: float = 0.0):
        self.kp, self.ki, self.ts = kp, ki, ts
        self.lo, self.hi = lo, hi
        self.integ = i0
        self.u = i0

    def update(self, e: float) -> float:
        self.integ += self.ki * self.ts * e
        # アンチワインドアップ：u が制限内に収まるように積分項をクランプ
        lo_i = self.lo - self.kp * e
        hi_i = self.hi - self.kp * e
        if self.integ > hi_i:
            self.integ = hi_i
        elif self.integ < lo_i:
            self.integ = lo_i
        self.u = self.kp * e + self.integ
        if self.u > self.hi:
            self.u = self.hi
        elif self.u < self.lo:
            self.u = self.lo
        return self.u


class LPF2:
    """1次LPF 2段（Scideam側でも配列なしで実装できる形）。"""

    def __init__(self, fc: float, fs: float, y0: float = 0.0):
        self.a = 2.0 * math.pi * fc / fs        # 前進オイラー係数
        if self.a > 0.5:
            self.a = 0.5
        self.y1 = y0
        self.y2 = y0

    def update(self, x: float) -> float:
        self.y1 += (x - self.y1) * self.a
        self.y2 += (self.y1 - self.y2) * self.a
        return self.y2


# =============================================================================
# 3. シミュレーション本体
# =============================================================================
def simulate(p: Params, verbose: bool = True) -> dict:
    # ---- 時間刻みと分周比 ---------------------------------------------------
    if p.switching:
        dt = 50.0e-9
        log_rate = 500.0e3
    else:
        dt = 1.0e-6
        log_rate = 200.0e3
    n_sub = int(round(1.0 / (p.f_scan * dt)))       # 1スキャンあたりの積分ステップ数
    log_div = int(round(1.0 / (log_rate * dt)))     # ログ間引き
    ts_scan = 1.0 / p.f_scan
    n_i = max(1, int(round(p.f_scan / p.f_iloop)))  # 電流ループ分周
    n_v = max(1, int(round(p.f_scan / p.f_vloop)))  # 電圧ループ分周
    f_iloop_eff = p.f_scan / n_i
    f_vloop_eff = p.f_scan / n_v

    ts_i = 1.0 / f_iloop_eff
    ts_v = 1.0 / f_vloop_eff

    # ---- ゲイン設計 ---------------------------------------------------------
    # 交差周波数は「実行周波数の 1/20」を超えないよう自動クランプ
    fc_i = p.fc_i if p.fc_i > 0 else min(p.fsw / 20.0, f_iloop_eff / 20.0)
    fc_i = min(fc_i, f_iloop_eff / 20.0)
    fc_v = min(p.fc_v, f_vloop_eff / 20.0)

    # 電流ループの比例ゲイン：次の2つの小さい方を採る
    #   (a) 交差周波数 fc_i で 0dB になるゲイン（教科書どおりの設計）
    #   (b) 無駄時間で位相が -180deg になる周波数 f180 でゲイン余裕を確保するゲイン
    # F_ILOOP が低いとき（=Ts が長いとき）プラントは fc 付近で抵抗性になり
    # |Gid| がほぼ平坦なので、(a) だけだと開ループゲインが広い帯域で 1 に張り付き、
    # 無駄時間が -180deg を作る周波数でゲイン余裕がゼロになる。(b) がそれを防ぐ。
    f180_i = 1.0 / (3.0 * ts_i)                 # 1.5*Ts の無駄時間で位相 -180deg
    kp_a = 1.0 / current_plant_gain(p, fc_i)
    kp_b = p.gm_target / current_plant_gain(p, f180_i)
    kp_i = min(kp_a, kp_b)
    # PI ゼロ：0dB 交差が fc_i に来るように置く
    g_fc = kp_i * current_plant_gain(p, fc_i)
    if g_fc < 0.999:
        fz_i = fc_i * math.sqrt(1.0 / (g_fc * g_fc) - 1.0)
    else:
        fz_i = fc_i * p.fz_ratio_i
    ki_i = kp_i * 2.0 * math.pi * fz_i

    kp_v, ki_v = design_pi(fc_v, fc_v * p.fz_ratio_v, voltage_plant_gain(p, fc_v))

    # 電流PI出力の整形：実行周波数が商用の10倍未満なら、デューティ補正を
    # 電流指令と同じ正弦テンプレートで整形する（ゼロクロス歪みを防ぐ）
    shape_pi = f_iloop_eff < p.shape_thr

    # 定常動作点（電力平衡より）
    ipk_ss = 2.0 * p.vbus_ref * p.io / p.vac_pk if p.load == "cc" else \
        2.0 * p.po / p.vac_pk

    pi_i = PI(kp_i, ki_i, ts_i, -p.dpi_max, p.dpi_max, 0.0)
    pi_v = PI(kp_v, ki_v, ts_v, 0.0, p.ipk_max, 0.0 if p.startup else ipk_ss)

    # 開ループ特性（設計確認用）
    fc_i_act, pm_i, f180_ia, gm_i = loop_margin(
        current_loop_tf(p, kp_i, ki_i, ts_i), 1e-3, f_iloop_eff / 2)
    fc_v_act, pm_v, f180_va, gm_v = loop_margin(
        voltage_loop_tf(p, kp_v, ki_v, p.f_vfilt), 1e-3, 1e4)

    if verbose:
        print("-" * 74)
        print("GAIN DESIGN  (transcribe these into closed_loop_bridgeless_pfc.sciprc)")
        print("-" * 74)
        print(f"  scan rate         F_SCAN  = {p.f_scan/1e3:9.3f} kHz  (Ts = {ts_scan*1e6:.2f} us)")
        print(f"  current loop      F_ILOOP = {f_iloop_eff:9.3f} Hz   (N_I = {n_i}, Ts = {ts_i*1e3:.3f} ms)")
        print(f"    crossover       FC_I    = {fc_i:9.3f} Hz   zero = {fz_i:.3f} Hz")
        print(f"    plant |Gid|     at FC_I = {current_plant_gain(p, fc_i):9.1f} A/duty")
        print(f"    gain limited by         : {'delay/gain-margin' if kp_b < kp_a else 'crossover'}")
        print(f"    KP_I = {kp_i:.6e}   KI_I = {ki_i:.6e}   (KI_I*Ts = {ki_i*ts_i:.6e})")
        print(f"    open loop       fc = {fc_i_act:9.3f} Hz   PM = {pm_i:6.1f} deg"
              f"   GM = {gm_i:5.1f} dB @ {f180_ia:.2f} Hz")
        print(f"    PI output shaping (template scaling) : {'ON' if shape_pi else 'OFF'}")
        print(f"  voltage loop      F_VLOOP = {f_vloop_eff/1e3:9.3f} kHz  (N_V = {n_v}, Ts = {ts_v*1e6:.3f} us)")
        print(f"    crossover       FC_V    = {fc_v:9.3f} Hz   zero = {fc_v*p.fz_ratio_v:.3f} Hz")
        print(f"    plant |Gvi|     at FC_V = {voltage_plant_gain(p, fc_v):9.3f} V/A")
        print(f"    KP_V = {kp_v:.6e}   KI_V = {ki_v:.6e}   (KI_V*Ts = {ki_v*ts_v:.6e})")
        print(f"    open loop       fc = {fc_v_act:9.3f} Hz   PM = {pm_v:6.1f} deg"
              f"   GM = {gm_v:5.1f} dB @ {f180_va:.2f} Hz")
        print(f"  Vbus LPF          {p.f_vfilt:.1f} Hz x2 (alpha = {2*math.pi*p.f_vfilt/p.f_scan:.6e})")
        print(f"  operating point   Ipk_ss  = {ipk_ss:9.3f} A    Po = {p.vac_pk*ipk_ss/2:.1f} W")
        print("-" * 74)

    # ---- 状態変数 -----------------------------------------------------------
    L_plant = p.L * p.l_err                 # プラント側の実インダクタンス
    r_eq = p.r_eq                           # 導通経路の等価直列抵抗
    v_ret = p.v_ret                         # 帰路の定電圧降下
    i_r = 0.0                               # 整流座標のインダクタ電流 [A]
    vbus = p.vac_pk if p.startup else p.vbus_ref   # コンデンサ内部電圧
    vbus_meas = vbus                        # 端子電圧（ESRリプル込み、制御が読む値）
    vbus_lpf = LPF2(p.f_vfilt, p.f_scan, vbus)
    d_lpf_y = 0.0                           # dvr/dt 用LPF状態
    a_d = 2.0 * math.pi * p.f_dfilt / p.f_scan

    vpk_est = p.vac_pk                      # 入力ピーク推定
    vpk_decay = 1.0 - math.exp(-ts_scan / 0.100)   # 時定数100msで減衰

    carrier_step = dt * p.fsw               # 1積分ステップあたりの鋸波の進み
    d_cmd = 0.0
    d_pi = 0.0
    ipk_ref = ipk_ss if not p.startup else 0.0
    i_ref = 0.0
    err_sum = 0.0
    err_cnt = 0
    vr_prev = 0.0
    pol = 1                                 # 極性（+1 / -1）
    carrier = 0.0

    vref = p.vac_pk if p.startup else p.vbus_ref
    ramp_rate = (p.vbus_ref - p.vac_pk) / 0.100 if p.startup else 0.0

    # ---- ログ用配列 ---------------------------------------------------------
    n_steps = int(round(p.t_sim / dt))
    n_log = n_steps // log_div + 1
    log = {k: np.zeros(n_log) for k in
           ("t", "vac", "iin", "iref", "vbus", "vbus_f", "d", "ipk_ref", "io")}
    k_log = 0

    # ---- メインループ -------------------------------------------------------
    scan_cnt = 0
    for n in range(n_steps):
        t = n * dt

        # ===== 制御スキャン（F_SCAN = 100 kHz ごと） ========================
        if n % n_sub == 0:
            # --- センシング -------------------------------------------------
            vac = p.vac_pk * math.sin(p.w_line * t)
            # 極性判定（ヒステリシス付き：ゼロクロス近傍のチャタリング防止）
            # ここで決めた pol は「どちらのスイッチを高周波駆動するか」の選択にのみ使う。
            # 整流座標のモデル自体は |vac| で記述されるため pol には依存しない。
            if pol > 0 and vac < -PO_HYST * p.vac_pk:
                pol = -1
            elif pol < 0 and vac > PO_HYST * p.vac_pk:
                pol = 1
            vr = abs(vac)                       # 整流入力電圧
            # i_r はプラント状態（実機では pol*IL_sense に相当）

            # --- 入力ピーク推定 ---------------------------------------------
            if vr > vpk_est:
                vpk_est = vr
            else:
                vpk_est -= vpk_est * vpk_decay
            if vpk_est < 0.2 * p.vac_pk:
                vpk_est = 0.2 * p.vac_pk

            # --- バス電圧の平均化（100Hzリプル除去） -------------------------
            vbus_f = vbus_lpf.update(vbus_meas)

            # --- ソフトスタート用の指令ランプ -------------------------------
            if p.startup and vref < p.vbus_ref:
                vref = min(p.vbus_ref, vref + ramp_rate * ts_scan)

            # --- 電圧ループ（N_V スキャンごと） -----------------------------
            if scan_cnt % n_v == 0:
                ipk_ref = pi_v.update(vref - vbus_f)

            # --- 電流指令とその微分（フィードフォワード用） -----------------
            template = vr / vpk_est
            if template > 1.0:
                template = 1.0
            i_ref = ipk_ref * template
            dvr = (vr - vr_prev) * p.f_scan
            vr_prev = vr
            d_lpf_y += (dvr - d_lpf_y) * a_d
            if p.didt_analytic:
                # i_ref = Ipk*|sin| なので di_ref/dt = Ipk*w*|cos| （符号は |vac| の増減）
                # 差分＋LPF から大きさを取ると位相遅れがそのまま電流誤差になり、
                # ゼロクロスで電流が先に 0 に張り付く（クロスオーバ歪み）。
                # 差分は「増減の符号」だけに使い、大きさは解析式で求める。
                cos_t = math.sqrt(max(0.0, 1.0 - template * template))
                sgn_d = 1.0 if d_lpf_y >= 0.0 else -1.0
                didt_ref = ipk_ref * p.w_line * cos_t * sgn_d
            else:
                didt_ref = ipk_ref * d_lpf_y / vpk_est

            # --- 電流ループ（誤差を平均して N_I スキャンごとに更新） --------
            err_sum += (i_ref - i_r)
            err_cnt += 1
            if scan_cnt % n_i == 0 and err_cnt > 0:
                d_pi = pi_i.update(err_sum / err_cnt)
                err_sum = 0.0
                err_cnt = 0

            # --- 完全フィードフォワード -------------------------------------
            #   (1-d)*(Vbus + Vf_D) = vr - V_ret - R_eq*i_ref - L*di_ref/dt
            # を満たす d。素子の導通降下（帰路ボディダイオード V_ret、昇圧
            # ダイオード Vf_D、各オン抵抗）を含めないと、指令どおりの電流が
            # 流れない。V_ret = 1V は |vac| がそれ以下の区間（ゼロクロス近傍）で
            # 電流を流せないことも意味する。
            vb = vbus_meas if vbus_meas > 1.0 else 1.0
            d_ff = 1.0 - (vr - v_ret - r_eq * i_ref - p.L * didt_ref) / (vb + p.vf_d)

            # --- デューティ合成 ---------------------------------------------
            d_cmd = d_ff + (d_pi * template if shape_pi else d_pi)
            if d_cmd > p.d_max:
                d_cmd = p.d_max
            elif d_cmd < p.d_min:
                d_cmd = p.d_min
            if p.pwm_steps > 0:              # 実機PWMの分解能を模擬
                d_cmd = round(d_cmd * p.pwm_steps) / p.pwm_steps

            scan_cnt += 1

        # ===== 電力回路の積分 ==============================================
        vr_t = p.vac_pk * abs(math.sin(p.w_line * t))
        if p.switching:
            # 鋸波コンパレータ。ただしスイッチング瞬時が積分ステップの途中に
            # 来る場合はその1ステップだけ「重なり割合」を使う（ボルト秒を厳密に
            # 保つため）。単純な 0/1 判定だと 1/n_sub のデューティ量子化誤差が
            # そのままインダクタ電圧の誤差として積分され、電流が発散する。
            frac = (d_cmd - carrier) / carrier_step
            if frac >= 1.0:
                s = 1.0
            elif frac <= 0.0:
                s = 0.0
            else:
                s = frac
            carrier += carrier_step
            if carrier >= 1.0:
                carrier -= 1.0
        else:
            s = d_cmd                        # 状態平均化モデル

        # 負荷電流（負荷ステップ対応）
        if p.load_step_time is not None and t >= p.load_step_time:
            io_now = p.io_after
        else:
            io_now = p.io
        i_load = p.load_current(vbus, io_now)

        # 導通降下を含む状態方程式（vbus はコンデンサ内部電圧）
        di = (vr_t - v_ret - r_eq * i_r - (1.0 - s) * (vbus + p.vf_d)) / L_plant
        i_c = (1.0 - s) * i_r - i_load
        dv = i_c / p.C
        i_r += di * dt
        vbus += dv * dt
        # 昇圧ダイオードが逆流を阻止するため整流座標の電流は負にならない
        if i_r < 0.0:
            i_r = 0.0
        if vbus < 1.0:
            vbus = 1.0
        # 端子で観測されるバス電圧（ESR によるリプルを含む）
        vbus_meas = vbus + p.c_esr * i_c

        # ===== ログ =========================================================
        if n % log_div == 0 and k_log < n_log:
            vac_t = p.vac_pk * math.sin(p.w_line * t)
            # 入力電流はインダクタ電流そのもの：iin = sign(vac) * i_r
            sgn = 1.0 if vac_t >= 0.0 else -1.0
            log["t"][k_log] = t
            log["vac"][k_log] = vac_t
            log["iin"][k_log] = sgn * i_r
            log["iref"][k_log] = sgn * i_ref
            log["vbus"][k_log] = vbus_meas
            log["vbus_f"][k_log] = vbus_lpf.y2
            log["d"][k_log] = d_cmd
            log["ipk_ref"][k_log] = ipk_ref
            log["io"][k_log] = i_load
            k_log += 1

    for k in log:
        log[k] = log[k][:k_log]

    # ---- 評価 ---------------------------------------------------------------
    metrics = evaluate(p, log, log_rate)
    metrics.update(dict(kp_i=kp_i, ki_i=ki_i, kp_v=kp_v, ki_v=ki_v,
                        fc_i=fc_i, fc_v=fc_v, f_iloop=f_iloop_eff,
                        f_vloop=f_vloop_eff, n_i=n_i, n_v=n_v,
                        shape_pi=shape_pi, ipk_ss=ipk_ss, fz_i=fz_i,
                        pm_i=pm_i, pm_v=pm_v, fc_i_act=fc_i_act, fc_v_act=fc_v_act,
                        gm_i=gm_i, gm_v=gm_v))
    return {"log": log, "metrics": metrics, "params": p}


# =============================================================================
# 4. 評価指標
# =============================================================================
def evaluate(p: Params, log: dict, log_rate: float) -> dict:
    """末尾 meas_cycles 周期分で 力率・THD・バス電圧リプルを計算する。"""
    n_per_cycle = int(round(log_rate / p.f_line))
    n_win = n_per_cycle * p.meas_cycles
    if len(log["t"]) < n_win:
        n_win = (len(log["t"]) // n_per_cycle) * n_per_cycle
    v = log["vac"][-n_win:]
    i = log["iin"][-n_win:]
    vb = log["vbus"][-n_win:]

    vrms = float(np.sqrt(np.mean(v**2)))
    irms = float(np.sqrt(np.mean(i**2)))
    pin = float(np.mean(v * i))
    pf = pin / (vrms * irms) if vrms * irms > 1e-9 else 0.0

    # THD（基本波=商用周波数、40次まで）。窓は整数周期なので窓関数なしで良い
    spec = np.fft.rfft(i)
    h1 = p.meas_cycles                       # 基本波が立つビン
    mag = np.abs(spec)
    fund = mag[h1]
    harm = np.sqrt(np.sum(mag[h1 * 2: h1 * 41: h1] ** 2))
    thd = harm / fund if fund > 1e-9 else float("nan")

    pout = float(np.mean(vb * log["io"][-n_win:]))
    return dict(
        pf=pf, thd=thd, vrms=vrms, irms=irms, pin=pin, pout=pout,
        vbus_mean=float(np.mean(vb)),
        vbus_ripple_pp=float(np.max(vb) - np.min(vb)),
        vbus_err=float(np.mean(vb) - p.vbus_ref),
        eff=pout / pin if pin > 1e-9 else 0.0,
    )


def print_metrics(m: dict, title: str = "RESULTS") -> None:
    print("-" * 74)
    print(title)
    print("-" * 74)
    print(f"  Vbus mean      = {m['vbus_mean']:8.2f} V   (error {m['vbus_err']:+.2f} V, "
          f"{100*m['vbus_err']/400:+.2f} %)")
    print(f"  Vbus ripple    = {m['vbus_ripple_pp']:8.2f} Vpp")
    print(f"  Input Vrms     = {m['vrms']:8.2f} V")
    print(f"  Input Irms     = {m['irms']:8.3f} A")
    print(f"  Input power    = {m['pin']:8.1f} W")
    print(f"  Output power   = {m['pout']:8.1f} W")
    print(f"  Power factor   = {m['pf']:8.4f}")
    print(f"  Input THD      = {100*m['thd']:8.2f} %")
    print("-" * 74)


# =============================================================================
# 5. グラフ出力
# =============================================================================
def result_dir() -> str:
    d = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(d, exist_ok=True)
    return d


def plot_waveforms(res: dict, tag: str) -> str:
    log, m, p = res["log"], res["metrics"], res["params"]
    t = log["t"] * 1e3
    fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True)

    ax[0].plot(t, log["vac"], lw=0.8, label="v_ac [V]")
    ax0b = ax[0].twinx()
    ax0b.plot(t, log["iin"], lw=0.8, color="tab:red", label="i_in [A]")
    ax0b.set_ylabel("i_in [A]", color="tab:red")
    ax[0].set_ylabel("v_ac [V]")
    ax[0].set_title(f"Bridgeless boost PFC / ACMC  "
                    f"(F_ILOOP = {m['f_iloop']:.4g} Hz, F_VLOOP = {m['f_vloop']:.4g} Hz)  "
                    f"PF = {m['pf']:.4f}, THD = {100*m['thd']:.2f} %")
    ax[0].grid(alpha=0.3)

    ax[1].plot(t, log["iref"], lw=0.8, label="i_ref")
    ax[1].plot(t, log["iin"], lw=0.8, label="i_in (measured)")
    ax[1].set_ylabel("current [A]")
    ax[1].legend(loc="upper right", fontsize=8)
    ax[1].grid(alpha=0.3)

    ax[2].plot(t, log["vbus"], lw=0.8, label="Vbus")
    ax[2].plot(t, log["vbus_f"], lw=1.0, label="Vbus filtered (loop input)")
    ax[2].axhline(p.vbus_ref, color="k", ls="--", lw=0.8, label="Vbus ref")
    ax[2].set_ylabel("Vbus [V]")
    ax[2].legend(loc="lower right", fontsize=8)
    ax[2].grid(alpha=0.3)

    ax[3].plot(t, log["d"], lw=0.8, label="duty")
    ax3b = ax[3].twinx()
    ax3b.plot(t, log["ipk_ref"], lw=1.0, color="tab:green", label="Ipk_ref")
    ax3b.set_ylabel("Ipk_ref [A]", color="tab:green")
    ax[3].set_ylabel("duty")
    ax[3].set_xlabel("time [ms]")
    ax[3].grid(alpha=0.3)

    fig.tight_layout()
    path = os.path.join(result_dir(), f"pfc_acmc_{tag}.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def plot_sweep(results: list, tag: str = "sweep") -> str:
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for res in results:
        log, m = res["log"], res["metrics"]
        n = len(log["t"])
        sl = slice(int(n * 0.8), n)
        lbl = f"F_ILOOP = {m['f_iloop']:.4g} Hz (PF {m['pf']:.4f}, THD {100*m['thd']:.1f} %)"
        ax[0].plot(log["t"][sl] * 1e3, log["iin"][sl], lw=0.9, label=lbl)
        ax[1].plot(log["t"][sl] * 1e3, log["vbus"][sl], lw=0.9, label=lbl)
    ax[0].set_ylabel("i_in [A]")
    ax[0].set_title("Current-loop sampling rate sweep")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[1].set_ylabel("Vbus [V]")
    ax[1].set_xlabel("time [ms]")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(result_dir(), f"pfc_acmc_{tag}.png")
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


# =============================================================================
# 6. エントリポイント
# =============================================================================
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Bridgeless boost PFC average-current-mode closed loop simulator")
    ap.add_argument("--f-iloop", type=float, default=50.0, help="current loop rate [Hz]")
    ap.add_argument("--f-vloop", type=float, default=100e3, help="voltage loop rate [Hz]")
    ap.add_argument("--fc-i", type=float, default=0.0, help="current loop crossover [Hz] (0=auto)")
    ap.add_argument("--fc-v", type=float, default=8.0, help="voltage loop crossover [Hz]")
    ap.add_argument("--io", type=float, default=1.25, help="load current [A]")
    ap.add_argument("--load", choices=["cc", "r", "cp"], default="cc", help="load model")
    ap.add_argument("--vbus", type=float, default=400.0, help="bus voltage reference [V]")
    ap.add_argument("--vac", type=float, default=100.0, help="input voltage rms [V]")
    ap.add_argument("--l", type=float, default=350e-6, help="PFC inductor [H]")
    ap.add_argument("--c", type=float, default=1.2e-3, help="bus capacitor [F]")
    ap.add_argument("--sync-return", action="store_true",
                    help="drive the return-path switch fully ON (synchronous, removes 1V body diode drop)")
    ap.add_argument("--l-err", type=float, default=1.0,
                    help="plant inductance error factor (controller uses nominal)")
    ap.add_argument("--d-max", type=float, default=0.995,
                    help="max duty (lower value increases zero-crossing distortion)")
    ap.add_argument("--pwm-steps", type=int, default=0,
                    help="PWM duty resolution steps per period (0 = ideal)")
    ap.add_argument("--t-sim", type=float, default=0.0, help="simulation time [s] (0=auto)")
    ap.add_argument("--switching", action="store_true", help="use real PWM model")
    ap.add_argument("--startup", action="store_true", help="soft start from 141 V")
    ap.add_argument("--load-step", action="store_true", help="50 %% -> 100 %% load step")
    ap.add_argument("--sweep", action="store_true", help="sweep current loop rate")
    return ap


def make_params(a, f_iloop: float | None = None) -> Params:
    p = Params(
        vac_rms=a.vac, vbus_ref=a.vbus, L=a.l, C=a.c,
        load=a.load, io=a.io,
        f_iloop=a.f_iloop if f_iloop is None else f_iloop,
        f_vloop=a.f_vloop,
        fc_i=a.fc_i, fc_v=a.fc_v,
        switching=a.switching, startup=a.startup, l_err=a.l_err,
        d_max=a.d_max, pwm_steps=a.pwm_steps, sync_return=a.sync_return,
    )
    if a.t_sim > 0:
        p.t_sim = a.t_sim
    elif a.switching:
        p.t_sim = 0.30          # dt=50ns なので時間がかかる（約35秒）
    elif a.startup or a.load_step:
        p.t_sim = 1.20
    return p


def main() -> None:
    a = build_parser().parse_args()

    if a.sweep:
        rates = [50.0, 500.0, 5.0e3, 100.0e3]
        results = []
        rows = []
        for r in rates:
            p = make_params(a, f_iloop=r)
            print(f"\n>>> F_ILOOP = {r:.4g} Hz")
            res = simulate(p, verbose=True)
            print_metrics(res["metrics"], f"RESULTS  (F_ILOOP = {r:.4g} Hz)")
            results.append(res)
            m = res["metrics"]
            rows.append((r, m["fc_i"], m["pf"], 100 * m["thd"],
                         m["vbus_mean"], m["vbus_ripple_pp"], m["irms"]))
        print("\n" + "=" * 78)
        print("CURRENT LOOP SAMPLING RATE SWEEP")
        print("=" * 78)
        print(f"{'F_ILOOP[Hz]':>12} {'FC_I[Hz]':>10} {'PF':>8} {'THD[%]':>8} "
              f"{'Vbus[V]':>9} {'ripple[Vpp]':>12} {'Irms[A]':>9}")
        for r in rows:
            print(f"{r[0]:12.4g} {r[1]:10.3f} {r[2]:8.4f} {r[3]:8.2f} "
                  f"{r[4]:9.2f} {r[5]:12.2f} {r[6]:9.3f}")
        print("=" * 78)
        print("saved:", plot_sweep(results))
        for res in results:
            print("saved:", plot_waveforms(res, f"iloop{res['metrics']['f_iloop']:.0f}Hz"))
        return

    p = make_params(a)
    if a.load_step:
        p.io = a.io * 0.5          # 50 % 負荷から開始
        p.io_after = a.io          # 100 % 負荷へステップ
        p.load_step_time = 0.70
        p.po = p.vbus_ref * p.io

    res = simulate(p, verbose=True)
    print_metrics(res["metrics"])
    mode = "switching" if a.switching else ("startup" if a.startup else
                                            ("loadstep" if a.load_step else "steady"))
    tag = f"{mode}_iloop{res['metrics']['f_iloop']:.0f}Hz"
    print("saved:", plot_waveforms(res, tag))


if __name__ == "__main__":
    main()
