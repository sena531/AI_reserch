#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
電力変換器のデジタル制御ゲインを、回路定数から固定小数点の #define まで一気に計算する。

物理量で設計 → 単位換算 → シフト量の検証、という順序をそのまま辿る。
手計算で写し間違えるより確実で、オーバーフローと桁落ちも自動で確認する。

使い方:
    python gains.py --help
    python gains.py --preset boost-pfc
    python gains.py --l 1e-3 --vdc 220 --fs 100e3 --period 9600 \
                    --isens 0.200 --vac-rms 100 --vo 48 --cout 7.8e-3

出力をそのまま C のヘッダに貼れる形（--emit c）でも出せる。
"""

import argparse
import math
import sys

# Windows の既定コンソールは cp932 なので、一部の記号で落ちる。UTF-8 に固定する。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

PI2 = 2.0 * math.pi


# ----------------------------------------------------------------------
# 設計（物理量）
# ----------------------------------------------------------------------

def design_current_loop(L, Vdc, fs, fc=None, zero_ratio=5.0):
    """昇圧インダクタ電流ループの PI ゲインを物理量で求める。

    プラント P(s) = Vdc/(sL) [A/duty]。PI を C(s)=Kp+Ki/s として、
    交差周波数 fc で |C*P| = 1 になるように Kp を決める。
    零点は fc/zero_ratio に置く（位相余裕を稼ぐため）。
    """
    if fc is None:
        fc = fs / 30.0          # fs/40〜fs/20 の中ほど
    Kp = PI2 * fc * L / Vdc     # [duty/A]
    fz = fc / zero_ratio
    Ki = Kp * PI2 * fz          # [duty/(A*s)]
    return {"Kp": Kp, "Ki": Ki, "fc": fc, "fz": fz}


def design_voltage_loop(Vac_pk, Vo, Cout, f_line, fc=None, zero_ratio=5.0):
    """PFC 出力電圧ループの PI ゲインを物理量で求める。

    プラント P(s) = Vac_pk/(2*Vo*C*s) [V/(Ipk の単位)]。
    分母の 2 は、正弦波入力の平均電力が Vac_pk*Ipk/2 であることから来る。
    交差周波数は商用周波数より十分低くする（乗算器を通って電流を歪ませるため）。
    """
    if fc is None:
        fc = f_line / 25.0      # 50Hz なら 2Hz
    # ★零点は fc/5。以前は fz = fc（zero_ratio=1）にしていたが、
    #   移動平均と 1 周期更新の遅れ（Td = 1/f_line）を入れると
    #   位相余裕が 14 度しか残らず設計として成立しない。
    #   電流ループと同じく fc/5 に下げること。
    G0 = Vac_pk / (2.0 * Vo * Cout)
    Kp = PI2 * fc / G0          # [A/V]
    fz = fc / zero_ratio
    Ki = Kp * PI2 * fz
    return {"Kp": Kp, "Ki": Ki, "fc": fc, "fz": fz, "G0": G0}


def design_phase_shift(Vo, Llk, Io, Ts, Vdc, n):
    """位相シフトフルブリッジの位相シフト量 D1 を静特性から求める。

        D1 = sqrt( Vo*Llk*Io / (Ts*Vdc*(Vdc - n*Vo)) )

    (Vdc - n*Vo) が分母にあるので、中間電圧が出力の n 倍に近づくと発散する。
    軽負荷で Vdc が n*Vo に張り付くのはこのため。
    """
    margin = Vdc - n * Vo
    if margin <= 0:
        return {"error": "Vdc <= n*Vo なので電力を伝送できない。"
                         "Vdc を上げるか n を下げること。"}
    D1 = math.sqrt(Vo * Llk * Io / (Ts * Vdc * margin))
    return {"D1": D1, "deg": D1 * 360.0, "margin": margin}


# ----------------------------------------------------------------------
# 固定小数点への換算
# ----------------------------------------------------------------------

def to_counts(Kp_phys, Ki_phys, T, period, cnt_per_unit):
    """物理ゲインを「PWMカウント / ADCカウント」に換算し、Tustin の係数を返す。

    u[n] = u[n-1] + A*e[n] - B*e[n-1]
    A = Kp + Ki*T/2,  B = Kp - Ki*T/2
    """
    Kp_cnt = Kp_phys * period / cnt_per_unit
    KiT_cnt = Ki_phys * T * period / cnt_per_unit
    return {"Kp_cnt": Kp_cnt, "KiT_cnt": KiT_cnt,
            "A": Kp_cnt + KiT_cnt / 2.0,
            "B": Kp_cnt - KiT_cnt / 2.0}


def pick_shift(A, B, e_max, adc_bits=12, headroom=8.0, int16_mac=True):
    """四つの制約からシフト量 S を選ぶ。

    (a) 下限・積分の死にゾーン : KiT = A-B が 1 PWMカウント未満なら、毎回シフトして
        丸めると積分が永久に進まない。アキュムレータを 2^S 倍で保持する前提で、
        必要な小数ビット数 ceil(-log2(KiT)) + 余裕 6 bit を下限とする。
    (b) 上限・オーバーフロー   : |A|*2^S*e_max*headroom < 2^31。
        e_max は定格ではなく ADC フルスケールで評価する（異常時にレールまで振れる）。
    (c) 上限・int16 MAC        : |A|*2^S < 32767 なら係数が int16 に収まり、
        dsPIC33 / C2000 の 16x16->32 MAC が 1 命令で回る。
    (d) 桁落ち                 : Ki の相対誤差 ~ 1/((A-B)*2^S)。数 % を超えるなら
        Q30 / 並列形への移行を勧める。

    戻り値: (S, notes, detail)
    """
    notes = []
    detail = {}
    KiT = A - B
    adc_fs = float((1 << adc_bits) - 1)

    # (a) 下限
    if KiT > 0:
        need_frac = max(0, -__import__("math").log2(KiT))
        s_lo = int(__import__("math").ceil(need_frac)) + 6
    else:
        need_frac = 0.0
        s_lo = 4
        notes.append("A - B = Ki*T が 0 以下。零点がサンプリング周波数に近すぎて"
                     "設計が成立していない。fz を下げること。")
    detail["KiT"] = KiT
    detail["need_frac_bits"] = need_frac
    detail["s_lo_deadzone"] = s_lo

    # (b)(c) 上限
    s_hi_of = 0
    s_hi_mac = 0
    for S in range(1, 31):
        if abs(A) * (1 << S) * e_max * headroom < 2 ** 31:
            s_hi_of = S
        if abs(A) * (1 << S) < 32767:
            s_hi_mac = S
    detail["s_hi_overflow"] = s_hi_of
    detail["s_hi_int16mac"] = s_hi_mac

    s_hi = min(s_hi_of, s_hi_mac) if int16_mac else s_hi_of
    if s_hi < s_lo:
        if int16_mac and s_hi_of >= s_lo:
            notes.append("int16 MAC の制約(c)と死にゾーンの下限(a)が両立しない。"
                         "係数が大きすぎるので fc を下げるか、32bit 係数を許容すること。"
                         "ここでは (c) を外して選んだ。")
            s_hi = s_hi_of
        else:
            notes.append("オーバーフロー(b)と死にゾーン(a)が両立しない。"
                         "交差周波数か e_max の見積もりを見直すこと。")
            return None, notes, detail

    S = s_hi  # 上限いっぱい（分解能を稼ぐ）だが (a) は必ず満たしている
    detail["S"] = S

    # (d) 桁落ち
    if KiT > 0:
        ki_err = 1.0 / (KiT * (1 << S))
        detail["ki_rel_err"] = ki_err
        if ki_err > 0.02:
            notes.append("Ki の整数化誤差が %.1f%%。A と B がほぼ等しく桁落ちしている。"
                         "Q30/int32 係数か並列形（P と I を別々に持つ）への移行を検討。"
                         % (ki_err * 100))

    a_i, b_i = A * (1 << S), B * (1 << S)
    if abs(a_i) < 1000:
        notes.append("A*2^S = %.0f と小さい。分解能が足りていない可能性がある。" % a_i)
    if b_i < 0:
        notes.append("B が負（%.1f）。Ki*T/2 > Kp なので零点が高すぎる。" % b_i)
    if int16_mac and abs(a_i) < 32767 and abs(b_i) < 32767:
        detail["int16_ok"] = True
    else:
        detail["int16_ok"] = False
        notes.append("係数が int16 に収まらない。16x16 MAC が使えず ISR が遅くなる。")

    return S, notes, detail


def margins(Kp, Ki, plant_gain, L_or_C, Td, kind="current"):
    """一巡伝達関数の位相余裕・ゲイン余裕を数値で求める。

    current : P(s) = Vdc/(s*L)        plant_gain=Vdc, L_or_C=L
    voltage : P(s) = G0/s             plant_gain=G0,  L_or_C=None
    遅れ D(s) = exp(-s*Td) を含める。これを入れないと設計が楽観的になる。
    """
    import cmath
    import math as m

    def Ljw(f):
        w = 2 * m.pi * f
        s_ = 1j * w
        C = Kp + Ki / s_
        P = (plant_gain / (s_ * L_or_C)) if kind == "current" else (plant_gain / s_)
        D = cmath.exp(-s_ * Td)
        return C * P * D

    # 交差周波数（|L|=1）を二分法で
    lo, hi = 1e-2, 1e7
    if abs(Ljw(lo)) < 1 or abs(Ljw(hi)) > 1:
        return None
    for _ in range(200):
        mid = m.sqrt(lo * hi)
        if abs(Ljw(mid)) > 1:
            lo = mid
        else:
            hi = mid
    fc = m.sqrt(lo * hi)
    pm = 180.0 + m.degrees(cmath.phase(Ljw(fc)))

    # 位相が -180deg を横切る点（ゲイン余裕）
    # cmath.phase は (-pi, pi] に折り返すので、そのまま比較すると横断を拾えない。
    # 2*pi 単位で連続化（アンラップ）してから -pi を下回る点を探す。
    gm_db = None
    f_pi = None
    f_prev = fc
    ph_prev = cmath.phase(Ljw(f_prev))
    ph_unwrap_prev = ph_prev
    offset = 0.0
    f = fc
    for _ in range(6000):
        f *= 1.002
        ph_raw = cmath.phase(Ljw(f))
        if ph_raw - ph_prev > m.pi:
            offset -= 2 * m.pi
        elif ph_raw - ph_prev < -m.pi:
            offset += 2 * m.pi
        ph_unwrap = ph_raw + offset
        ph_prev = ph_raw
        if ph_unwrap_prev > -m.pi >= ph_unwrap:
            # 線形補間で横断点を詰める
            t = (ph_unwrap_prev + m.pi) / (ph_unwrap_prev - ph_unwrap)
            f_pi = f_prev * (f / f_prev) ** t
            gm_db = -20 * m.log10(abs(Ljw(f_pi)))
            break
        f_prev, ph_unwrap_prev = f, ph_unwrap

    return {"fc": fc, "pm_deg": pm, "gm_db": gm_db, "f_180": f_pi,
            "delay_margin_Td": (m.radians(pm) / (2 * m.pi * fc) / Td) if Td > 0 else None}


# ----------------------------------------------------------------------
# 出力
# ----------------------------------------------------------------------

def report(args):
    out = []
    w = out.append

    # --- 基本量 ---
    Ts = 1.0 / args.fs
    Vac_pk = args.vac_rms * math.sqrt(2.0)
    adc_full = (2 ** args.adc_bits) - 1
    adc_cnt_per_V = adc_full / args.vref
    adc_cnt_per_A = adc_cnt_per_V * args.isens
    vrec_pk_cnt = args.vline_cnt_per_V * Vac_pk

    w("=" * 68)
    w("基本量")
    w("=" * 68)
    w(f"  Ts              = {Ts*1e6:.3f} us   (fs = {args.fs/1e3:.1f} kHz)")
    w(f"  Vac_pk          = {Vac_pk:.2f} V")
    w(f"  ADC_CNT_PER_V   = {adc_cnt_per_V:.2f}")
    w(f"  ADC_CNT_PER_A   = {adc_cnt_per_A:.2f}   (感度 {args.isens*1000:.0f} mV/A)")
    w(f"  VREC_PK_CNT     = {vrec_pk_cnt:.0f}")

    # センサの測定レンジ確認（よく見落とす）
    sens_range = (args.vref / 2.0 - 0.2) / args.isens
    w(f"  センサ測定可能範囲 = ±{sens_range:.2f} A "
      f"(出力振幅 {args.vref/2:.2f}V ± 、両方向センサ想定)")

    # --- 電流ループ ---
    ci = design_current_loop(args.l, args.vdc, args.fs, args.fc_i)
    cc = to_counts(ci["Kp"], ci["Ki"], Ts, args.period, adc_cnt_per_A)
    e_max_i = float(adc_full)          # 異常時は ADC レールまで振れる
    S_i, notes_i, det_i = pick_shift(cc["A"], cc["B"], e_max_i, args.adc_bits)
    mg_i = margins(ci["Kp"], ci["Ki"], args.vdc, args.l, args.td_ts * Ts, "current")

    w("")
    w("=" * 68)
    w("電流ループ (100% デジタル, 1/fs ごと)")
    w("=" * 68)
    w(f"  交差 fc         = {ci['fc']:.0f} Hz   (fs/{args.fs/ci['fc']:.0f})")
    w(f"  零点 fz         = {ci['fz']:.0f} Hz")
    w(f"  Kp              = {ci['Kp']:.6f} [duty/A]")
    w(f"  Ki              = {ci['Ki']:.2f} [duty/(A*s)]")
    w(f"  Kp_cnt          = {cc['Kp_cnt']:.4f} [PWMcnt/ADCcnt]")
    w(f"  KiT_cnt         = {cc['KiT_cnt']:.4f}")
    if S_i is not None:
        w(f"  SH_I            = {S_i}")
        w(f"  KI_A            = {int(cc['A'] * (1 << S_i))}")
        w(f"  KI_B            = {int(cc['B'] * (1 << S_i))}")
        w(f"  1A の偏差 -> デューティ {cc['Kp_cnt']*adc_cnt_per_A:.0f} カウント "
          f"({cc['Kp_cnt']*adc_cnt_per_A/args.period*100:.2f} %)")
        w("")
        w("  --- シフト量の根拠 ---")
        w(f"  (a) 死にゾーン下限 : Ki*T = {det_i['KiT']:.5f} PWMcnt/サンプル"
          f" -> 小数 {det_i['need_frac_bits']:.1f} bit 必要 -> S >= {det_i['s_lo_deadzone']}")
        w(f"      毎回シフトして丸めると積分が進まない。アキュムレータは 2^S 倍で保持する。")
        w(f"  (b) オーバーフロー : S <= {det_i['s_hi_overflow']}"
          f"  (e は ADC フルスケール {e_max_i:.0f} で評価、余裕 8 倍)")
        w(f"  (c) int16 MAC      : S <= {det_i['s_hi_int16mac']}"
          f"  (|A|*2^S < 32767 で 16x16 MAC が 1 命令)")
        if 'ki_rel_err' in det_i:
            w(f"  (d) 桁落ち         : Ki の整数化誤差 {det_i['ki_rel_err']*100:.3f} %")
        w(f"  -> S = {S_i}"
          f"  (係数 int16 {'に収まる' if det_i.get('int16_ok') else 'に収まらない'})")
        if mg_i:
            w("")
            w("  --- 安定性の検証（遅れ %.1f*Ts を含む） ---" % args.td_ts)
            w(f"  実際の交差   = {mg_i['fc']:.0f} Hz")
            w(f"  位相余裕 PM  = {mg_i['pm_deg']:.1f} deg"
              f"   {'OK' if mg_i['pm_deg'] >= 45 else '[!] 45deg 未満。fc を下げること'}")
            if mg_i['gm_db'] is not None:
                w(f"  ゲイン余裕 GM= {mg_i['gm_db']:.1f} dB"
                  f"   {'OK' if mg_i['gm_db'] >= 6 else '[!] 6dB 未満'}")
            if mg_i['delay_margin_Td']:
                w(f"  遅れ余裕     = 想定遅れの {mg_i['delay_margin_Td']:.2f} 倍まで"
                  f"   {'OK' if mg_i['delay_margin_Td'] >= 2 else '[!] 2倍未満。センスのRCに注意'}")
    for n in notes_i:
        w(f"  [!] {n}")

    # --- 電圧ループ ---
    if args.cout > 0 and args.vo > 0:
        N_vloop = int(round(args.fs / args.f_line))
        T_v = 1.0 / args.f_line
        cv = design_voltage_loop(Vac_pk, args.vo, args.cout, args.f_line, args.fc_v)
        sh_mul = args.sh_mul
        mulconv = adc_cnt_per_A * (1 << sh_mul) / vrec_pk_cnt
        Kp_v_int = cv["Kp"] / args.vo_cnt_per_V * mulconv
        KiT_v_int = cv["Ki"] * T_v / args.vo_cnt_per_V * mulconv
        A_v = Kp_v_int + KiT_v_int / 2.0
        B_v = Kp_v_int - KiT_v_int / 2.0
        e_max_v = args.vo * args.vo_cnt_per_V
        S_v, notes_v, det_v = pick_shift(A_v, B_v, float(adc_full), args.adc_bits)
        mg_v = margins(cv["Kp"], cv["Ki"], cv["G0"], None, T_v, "voltage")
        ipk_rated_mul = args.ipk_rated * mulconv
        ripple = 2.0 * args.io_rated / (PI2 * 2 * args.f_line * args.cout)

        w("")
        w("=" * 68)
        w(f"電圧ループ (1/f_line ごと, 窓幅 N = {N_vloop} サンプル = {T_v*1e3:.1f} ms)")
        w("=" * 68)
        w(f"  交差 fc_v       = {cv['fc']:.2f} Hz   (f_line/{args.f_line/cv['fc']:.0f})")
        w(f"  プラント G0     = {cv['G0']:.2f} [V/(A*s)]")
        w(f"  Kp_v            = {cv['Kp']:.6f} [A/V]")
        w(f"  Ki_v            = {cv['Ki']:.4f} [A/(V*s)]")
        w(f"  SH_MUL          = {sh_mul}")
        w(f"  MULCONV         = {mulconv:.2f}")
        if S_v is not None:
            w(f"  SH_V            = {S_v}")
            w(f"  KV_A            = {int(A_v * (1 << S_v))}")
            w(f"  KV_B            = {int(B_v * (1 << S_v))}")
            w(f"  (a) 死にゾーン  : Ki*T = {det_v['KiT']:.5f} -> S >= {det_v['s_lo_deadzone']}")
            if 'ki_rel_err' in det_v:
                w(f"  (d) 桁落ち      : Ki の整数化誤差 {det_v['ki_rel_err']*100:.3f} %")
        if mg_v:
            w(f"  実際の交差      = {mg_v['fc']:.2f} Hz")
            w(f"  位相余裕 PM     = {mg_v['pm_deg']:.1f} deg"
              f"   {'OK' if mg_v['pm_deg'] >= 45 else '[!] 45deg 未満'}")
        w(f"  IPK_RATED (mul) = {int(ipk_rated_mul)}")
        w(f"  IPK_MAX   (mul) = {int(1.5 * ipk_rated_mul)}  (定格の 1.5 倍)")
        w(f"  出力リプル      = {ripple:.2f} Vpp  "
          f"(Io={args.io_rated:.1f}A 定電流負荷, 2*Io/(w2*C))")
        for n in notes_v:
            w(f"  [!] {n}")

    # --- フィードフォワード ---
    if args.vline_cnt_per_V > 0:
        sh_ff = 12
        kff = args.period / (args.vline_cnt_per_V * args.vdc) * (1 << sh_ff)
        w("")
        w("=" * 68)
        w("フィードフォワード  D_ff = PERIOD*(1 - |vac|/Vdc)")
        w("=" * 68)
        w(f"  SH_FF           = {sh_ff}")
        w(f"  KFF             = {int(kff)}")
        w("  [!] Vdc を実測していないなら、立ち上げ中は無効にすること。")
        w("      定格値を決め打ちすると、低入力時に意味のない最大デューティを出し続ける。")

    # --- 位相シフト ---
    if args.llk > 0 and args.n_tr > 0:
        ps = design_phase_shift(args.vo, args.llk, args.io_rated, Ts, args.vdc, args.n_tr)
        w("")
        w("=" * 68)
        w("位相シフトフルブリッジ")
        w("=" * 68)
        if "error" in ps:
            w(f"  [!] {ps['error']}")
        else:
            w(f"  D1              = {ps['D1']:.4f}  ({ps['deg']:.1f} deg)")
            w(f"  D1_FIX_CNT      = {int(ps['D1'] * args.period)}")
            w(f"  Vdc - n*Vo      = {ps['margin']:.1f} V")
            w(f"  軽負荷では Vdc が n*Vo = {args.n_tr*args.vo:.0f} V に張り付く。")
            if ps["D1"] > 0.5:
                w("  [!] D1 > 0.5（180 度超）。伝送能力が足りない。"
                  "Vdc を上げるか Llk を下げること。")

    # --- アンチワインドアップ ---
    w("")
    w("=" * 68)
    w("アンチワインドアップの上限（構成に合わせること）")
    w("=" * 68)
    w(f"  Duty = D_ff + Dpi  の場合 : ±{int(args.period*0.6)}  (±60%)")
    w(f"  Duty = Dpi のみ    の場合 : 0 〜 {int(args.period*0.95)}  (0〜95%)")
    w("  [!] フィードフォワードを切ったのに ±60% のままにすると、")
    w("      デューティが 60% で頭打ちになり昇圧比 2.5 倍までしか出ない。")
    w("      エラーは出ないので波形を見るまで気づけない。")

    return "\n".join(out)


def emit_c(args):
    """C のヘッダに貼れる形で出す。数値ではなく式の連鎖で書く。"""
    return f"""/* 自動生成: gains.py */

/* --- 実機で校正する値 --- */
#define ISENS_V_PER_A   {args.isens}
#define ISENS_OFS       2048        /* 無電流時の ADC 値。実測で置換 */
#define VLINE_CNT_PER_V {args.vline_cnt_per_V}
#define VO_CNT_PER_V    {args.vo_cnt_per_V}

/* --- 回路定数 --- */
#define L_H             {args.l}
#define VDC_V           {args.vdc}
#define VO_REF_V        {args.vo}
#define C_OUT_F         {args.cout}
#define VAC_RMS_V       {args.vac_rms}
#define F_LINE_HZ       {int(args.f_line)}
#define FS_HZ           {args.fs}
#define PERIODmax       {args.period}
#define ADC_BITS        {args.adc_bits}
#define VREF_V          {args.vref}

/* --- ここから下は自動で決まる（定数畳み込みされる） --- */
#define PI2_            6.28318530718
#define TS_S            (1.0/FS_HZ)
#define VAC_PK_V        (VAC_RMS_V*1.41421356)
#define N_VLOOP         ((int)(FS_HZ/F_LINE_HZ))
#define ADC_CNT_PER_V   (((1L<<ADC_BITS)-1)/VREF_V)
#define ADC_CNT_PER_A   (ADC_CNT_PER_V*ISENS_V_PER_A)
#define VREC_PK_CNT     ((int)(VLINE_CNT_PER_V*VAC_PK_V))

/* 電流ループ */
#define FC_I_HZ         {design_current_loop(args.l, args.vdc, args.fs, args.fc_i)['fc']:.0f}.0
#define KP_I_PHYS       (PI2_*FC_I_HZ*L_H/VDC_V)
#define KI_I_PHYS       (KP_I_PHYS*PI2_*(FC_I_HZ/5.0))
#define KP_I_CNT        (KP_I_PHYS*PERIODmax/ADC_CNT_PER_A)
#define KIT_I_CNT       (KI_I_PHYS*TS_S*PERIODmax/ADC_CNT_PER_A)
#define SH_I            10
#define KI_A            ((int32_t)((KP_I_CNT + KIT_I_CNT/2.0)*(1L<<SH_I)))
#define KI_B            ((int32_t)((KP_I_CNT - KIT_I_CNT/2.0)*(1L<<SH_I)))

/* 乗算器 */
#define SH_MUL          {args.sh_mul}
#define MULCONV         (ADC_CNT_PER_A*(1L<<SH_MUL)/VREC_PK_CNT)

/* 電圧ループ（T は 1/F_LINE であって 1/FS ではない） */
#define FC_V_HZ         {design_voltage_loop(args.vac_rms*math.sqrt(2), args.vo, args.cout, args.f_line, args.fc_v)['fc']:.2f}
#define G_V0            (VAC_PK_V/(2.0*VO_REF_V*C_OUT_F))
#define KP_V_PHYS       (PI2_*FC_V_HZ/G_V0)
#define KI_V_PHYS       (KP_V_PHYS*PI2_*FC_V_HZ)
#define KP_V_CNT        (KP_V_PHYS/VO_CNT_PER_V*MULCONV)
#define KIT_V_CNT       (KI_V_PHYS/(double)F_LINE_HZ/VO_CNT_PER_V*MULCONV)
#define SH_V            14
#define KV_A            ((int32_t)((KP_V_CNT + KIT_V_CNT/2.0)*(1L<<SH_V)))
#define KV_B            ((int32_t)((KP_V_CNT - KIT_V_CNT/2.0)*(1L<<SH_V)))

/* フィードフォワード（Vdc を実測していないなら立ち上げ中は無効に） */
#define USE_FF          0
#define SH_FF           12
#define KFF             ((int32_t)((double)PERIODmax/(VLINE_CNT_PER_V*VDC_V)*(1L<<SH_FF)))
"""


PRESETS = {
    "boost-pfc": dict(l=1e-3, vdc=220.0, fs=100e3, period=9600, isens=0.200,
                      vac_rms=100.0, vo=48.0, cout=7.8e-3, io_rated=10.0,
                      llk=5e-6, n_tr=4.0),
}


def main():
    p = argparse.ArgumentParser(
        description="電力変換器の制御ゲインを回路定数から計算する",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument("--preset", choices=sorted(PRESETS), help="既定の組み合わせを使う")
    p.add_argument("--l", type=float, default=1e-3, help="PFC インダクタンス [H]")
    p.add_argument("--vdc", type=float, default=220.0, help="中間（DCリンク）電圧 [V]")
    p.add_argument("--fs", type=float, default=100e3, help="スイッチング周波数 [Hz]")
    p.add_argument("--period", type=int, default=9600, help="PWM 周期カウント (PTPER)")
    p.add_argument("--f-line", type=float, default=50.0, help="商用周波数 [Hz]")
    p.add_argument("--vac-rms", type=float, default=100.0, help="入力電圧 [Vrms]")
    p.add_argument("--vo", type=float, default=48.0, help="出力電圧 [V]")
    p.add_argument("--cout", type=float, default=7.8e-3, help="出力コンデンサ [F]")
    p.add_argument("--io-rated", type=float, default=10.0, help="定格出力電流 [A]")
    p.add_argument("--ipk-rated", type=float, default=0.0,
                   help="入力電流ピーク定格 [A]（省略時は電力から計算）")
    p.add_argument("--llk", type=float, default=5e-6, help="トランス漏れインダクタンス [H]")
    p.add_argument("--n-tr", type=float, default=4.0, help="巻数比 Np/Ns(片側)")
    p.add_argument("--isens", type=float, default=0.200, help="電流センサ感度 [V/A]")
    p.add_argument("--adc-bits", type=int, default=12, help="ADC 分解能 [bit]")
    p.add_argument("--vref", type=float, default=3.3, help="ADC 基準電圧 [V]")
    p.add_argument("--vline-cnt-per-V", type=float, default=9.83,
                   help="ライン電圧センスの換算 [ADCcount/V]（実測で決める）")
    p.add_argument("--vo-cnt-per-V", type=float, default=77.58,
                   help="出力電圧センスの換算 [ADCcount/V]（実測で決める）")
    p.add_argument("--fc-i", type=float, default=None, help="電流ループ交差周波数 [Hz]")
    p.add_argument("--fc-v", type=float, default=None, help="電圧ループ交差周波数 [Hz]")
    p.add_argument("--sh-mul", type=int, default=11, help="乗算器のシフト量")
    p.add_argument("--td-ts", type=float, default=1.5,
                   help="制御遅れ [Ts単位]。演算1Ts + ZOH 0.5Ts = 1.5 が最低ライン。"
                        "PWM更新がバッファで1周期遅れる構成なら 2.5")
    p.add_argument("--eff", type=float, default=0.93, help="効率（入力電流の見積もり用）")
    p.add_argument("--emit", choices=["report", "c"], default="report",
                   help="report: 読みやすい表 / c: ヘッダに貼れる形")
    # プリセットは「既定値の差し替え」として適用する。先に全部パースしてから
    # 上書きすると、--preset と個別指定を併用したときに個別指定が消える。
    pre, _ = p.parse_known_args()
    if pre.preset:
        p.set_defaults(**PRESETS[pre.preset])
    args = p.parse_args()

    if args.ipk_rated <= 0:
        Pout = args.vo * args.io_rated
        args.ipk_rated = 2.0 * Pout / (args.eff * args.vac_rms * math.sqrt(2.0))

    if args.emit == "c":
        print(emit_c(args))
    else:
        print(report(args))
    return 0


if __name__ == "__main__":
    sys.exit(main())
