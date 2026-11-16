# -*- coding: utf-8 -*-
"""closed_loop_bridgeless_pfc.sciprc の移植確認スクリプト

Scideam スクリプトの制御則を 1 行ずつ Python へ逐語移植し、pfc_acmc_sim.py と
同じプラント（bridgeless PFC.scicir の定数）に接続して結果が一致するかを確認する。
Scideam スクリプト側の定数を書き換えたら、このファイルの定数ブロックにも同じ値を
写して実行すること。ゲインの転記ミスをここで潰せる。

  py -3 verify_sciprc_port.py

期待値（既定条件）: Vbus 400.0 V, 力率 0.998, THD 3.1 % 前後
"""
import os
import sys
import math

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pfc_acmc_sim as S

# ---- .sciprc の定数ブロックをそのまま写す --------------------------------
FS = 100e3; TS = 1/FS
F_LINE = 50; W_LINE = 314.15927; VAC_PK = 141.42136; VBUS_REF = 400
L_PFC = 350e-6
R_EQ = 0.21; V_RET = 1.0; VF_D = 0.7
N_I = 2000; N_V = 1
K_VAC = 1.0; K_ISENSE = 1.0; K_VBUS = 1.0
D_MAX = 0.995; D_MIN = 0.0; IPK_MAX = 10.6; DPI_MAX = 0.30
KP_I = 1.596016e-04; KI_I_TS = 1.568842e-04
KP_V = 3.412134e-01; KI_V_TS = 4.287814e-05
A_VBUS = 1.884956e-03; A_DVDT = 0.1256637; A_VPK = 1.0e-4
VREF_SLEW = 2600
SHAPE_PI = 1
SYNC_RETURN = 0
PO_HYST = 0.70711

# ---- プラント（pfc_acmc_sim と同一：bridgeless PFC.scicir の実値）--------
p = S.Params()
dt = 1e-6; n_sub = 10; log_div = 5; log_rate = 200e3
t_sim = 1.0
w = p.w_line
r_eq = p.r_eq
v_ret = p.v_ret

i_r = 0.0
vbus = p.vbus_ref          # コンデンサ内部電圧
vbus_meas = vbus           # 端子電圧（ESRリプル込み）

# .sciprc の永続変数（Scideam は全変数 0 初期化）
Init = 0; Half = 0; ScanCnt = 0
Vbus_f1 = Vbus_f2 = Vpk_est = Vref = Vr_prev = 0.0
Iv = Ii = 0.0
Dvr_f = 0.0; ErrSum = 0.0; ErrCnt = 0
Ipk_ref = 0.0; D_pi = 0.0; D_pfc = 0.0; Iref = 0.0

n_steps = int(t_sim/dt); n_log = n_steps//log_div + 1
log = {k: np.zeros(n_log) for k in
       ("t", "vac", "iin", "iref", "vbus", "vbus_f", "d", "ipk_ref", "io")}
kl = 0

for n in range(n_steps):
    t = n*dt
    if n % n_sub == 0:
        # ② センシング（.sciprc は output(...,"AVE") ＝ スイッチング1周期の平均値。
        #    ここは状態平均化モデルなのでリプルが無く、そのまま平均値に相当する）
        Vbus_raw = K_VBUS * vbus_meas
        Vac = K_VAC * (p.vac_pk*math.sin(w*t))
        IL = K_ISENSE * (i_r if math.sin(w*t) >= 0 else -i_r)
        Vr = abs(Vac)
        # ③ 初期化
        if Init == 0:
            Vbus_f1 = Vbus_raw; Vbus_f2 = Vbus_raw
            Vpk_est = VAC_PK; Vref = Vbus_raw; Vr_prev = Vr
            Iv = 0.0; Ii = 0.0; Init = 1
        # 極性
        if Half == 1:
            if Vac < -PO_HYST: Half = 0
        else:
            if Vac > PO_HYST: Half = 1
        Ir = IL if Half == 1 else -IL
        # ④ ピーク推定
        if Vr > Vpk_est: Vpk_est = Vr
        else: Vpk_est = Vpk_est - Vpk_est*A_VPK
        if Vpk_est < 0.2*VAC_PK: Vpk_est = 0.2*VAC_PK
        # ⑤ バス電圧LPF
        Vbus_f1 = Vbus_f1 + (Vbus_raw - Vbus_f1)*A_VBUS
        Vbus_f2 = Vbus_f2 + (Vbus_f1 - Vbus_f2)*A_VBUS
        # ⑥ ソフトスタート
        if Vref < VBUS_REF:
            Vref = Vref + VREF_SLEW*TS
            if Vref > VBUS_REF: Vref = VBUS_REF
        # ⑦ 電圧ループ
        if ScanCnt % N_V == 0:
            Ev = Vref - Vbus_f2
            Iv = Iv + KI_V_TS*Ev
            Iv_lo = 0 - KP_V*Ev; Iv_hi = IPK_MAX - KP_V*Ev
            if Iv > Iv_hi: Iv = Iv_hi
            if Iv < Iv_lo: Iv = Iv_lo
            Ipk_ref = KP_V*Ev + Iv
            if Ipk_ref > IPK_MAX: Ipk_ref = IPK_MAX
            if Ipk_ref < 0: Ipk_ref = 0.0
        # ⑧ 指令と微分（解析式）
        Templ = Vr/Vpk_est
        if Templ > 1: Templ = 1.0
        Iref = Ipk_ref*Templ
        Dvr = (Vr - Vr_prev)*FS
        Vr_prev = Vr
        Dvr_f = Dvr_f + (Dvr - Dvr_f)*A_DVDT
        Sgn = 1.0 if Dvr_f >= 0 else -1.0
        Cos_t = math.sqrt(max(0.0, 1 - Templ*Templ))
        Didt = Ipk_ref*W_LINE*Cos_t*Sgn
        # ⑨ 電流ループ
        ErrSum = ErrSum + (Iref - Ir)
        ErrCnt = ErrCnt + 1
        if ScanCnt % N_I == 0 and ErrCnt > 0:
            Ei = ErrSum/ErrCnt
            Ii = Ii + KI_I_TS*Ei
            Ii_lo = -DPI_MAX - KP_I*Ei; Ii_hi = DPI_MAX - KP_I*Ei
            if Ii > Ii_hi: Ii = Ii_hi
            if Ii < Ii_lo: Ii = Ii_lo
            D_pi = KP_I*Ei + Ii
            if D_pi > DPI_MAX: D_pi = DPI_MAX
            if D_pi < -DPI_MAX: D_pi = -DPI_MAX
            ErrSum = 0.0; ErrCnt = 0
        # ⑩ 完全フィードフォワード
        Vb = Vbus_raw
        if Vb < 1: Vb = 1.0
        D_ff = 1 - (Vr - V_RET - R_EQ*Iref - L_PFC*Didt)/(Vb + VF_D)
        # ⑪ 合成
        D_pfc = D_ff + D_pi*Templ if SHAPE_PI == 1 else D_ff + D_pi
        if D_pfc > D_MAX: D_pfc = D_MAX
        if D_pfc < D_MIN: D_pfc = D_MIN
        # ⑬ カウンタ
        ScanCnt = ScanCnt + 1
        if ScanCnt >= N_I: ScanCnt = 0

    # ---- プラント（状態平均化、導通降下込み）----
    vr_t = p.vac_pk*abs(math.sin(w*t))
    s = D_pfc
    i_load = p.io
    i_r += (vr_t - v_ret - r_eq*i_r - (1-s)*(vbus + p.vf_d))/p.L * dt
    i_c = (1-s)*i_r - i_load
    vbus += i_c/p.C * dt
    if i_r < 0: i_r = 0.0
    if vbus < 1: vbus = 1.0
    vbus_meas = vbus + p.c_esr*i_c

    if n % log_div == 0 and kl < n_log:
        vac_t = p.vac_pk*math.sin(w*t); sgn = 1.0 if vac_t >= 0 else -1.0
        log["t"][kl] = t; log["vac"][kl] = vac_t; log["iin"][kl] = sgn*i_r
        log["iref"][kl] = sgn*Iref; log["vbus"][kl] = vbus_meas
        log["vbus_f"][kl] = Vbus_f2
        log["d"][kl] = D_pfc; log["ipk_ref"][kl] = Ipk_ref; log["io"][kl] = i_load
        kl += 1

for k in log: log[k] = log[k][:kl]
m = S.evaluate(p, log, log_rate)
print("=== closed_loop_bridgeless_pfc.sciprc 逐語移植版 ===")
print("  Vbus mean = %.2f V   ripple = %.2f Vpp" % (m["vbus_mean"], m["vbus_ripple_pp"]))
print("  PF = %.4f   THD = %.2f %%   Irms = %.3f A   Pin = %.1f W"
      % (m["pf"], 100*m["thd"], m["irms"], m["pin"]))
