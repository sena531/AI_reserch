"""制御ループ安定余裕レポート（HTML、図埋め込み）を生成する。

    python3 simulation/python/loop_bode.py        # ボード線図 PNG を先に作る
    python3 simulation/python/make_loop_report.py # results/loop_stability_report.html

ゲインは firmware/src/main_closed_loop.c の設計式と同じ式で計算している
（GAIN_SAFE = 1 : 立ち上げ用 / 0 : 通常）。ゲインを変えたら下の FC_* を合わせること。
"""
import base64
import html
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, 'results')
PI2 = 2 * np.pi

# ---- 設計値（firmware と同じ） -------------------------------------------
L_NOM, VDC_NOM, TS = 1e-3, 220.0, 1e-5
C_NOM, VO, VAC_NOM, TV = 6e-3, 48.0, 100.0, 0.03
GAINS = {
    'safe':   dict(name='立ち上げ用', fc_i=1000.0, fc_v=1.0, sh_i=14, sh_v=17,
                   ka=(18213, 17986), kv=(15714, 15324)),
    'normal': dict(name='通常', fc_i=1e5 / 30, fc_v=2.0, sh_i=13, sh_v=16,
                   ka=(30798, 29534), kv=(15910, 15129)),
}


def pi_cur(fc):
    kp = PI2 * fc * L_NOM / VDC_NOM
    return kp, kp * PI2 * fc / 5


def pi_vol(fc):
    g0 = VAC_NOM * np.sqrt(2) / (2 * VO * C_NOM)
    kp = PI2 * fc / g0
    return kp, kp * PI2 * fc / 5


def margins(kp, ki, g, td, f):
    s = 1j * PI2 * f
    lo = (kp + ki / s) * g / s * np.exp(-s * td)
    mag = abs(lo)
    ph = np.unwrap(np.angle(lo)) * 180 / np.pi
    i = np.where(np.diff(np.sign(mag - 1)))[0][-1]
    j = np.where(np.diff(np.sign(ph + 180)))[0]
    gm = -20 * np.log10(mag[j[0]]) if len(j) else float('inf')
    return f[i], 180 + ph[i], gm


F_I = np.logspace(1, np.log10(5e4), 20000)
F_V = np.logspace(-3, np.log10(25), 20000)


def cur(key, vdc=VDC_NOM, L=L_NOM, td=1.5 * TS):
    kp, ki = pi_cur(GAINS[key]['fc_i'])
    return margins(kp, ki, vdc / L, td, F_I)


def vol(key, C=C_NOM, vac=VAC_NOM, td=TV):
    kp, ki = pi_vol(GAINS[key]['fc_v'])
    return margins(kp, ki, vac * np.sqrt(2) / (2 * VO * C), td, F_V)


# ---- 計算 ------------------------------------------------------------------
VCS_GRID = [190, 205, 220, 235, 250]
L_GRID = [0.7e-3, 0.85e-3, 1.0e-3, 1.15e-3, 1.3e-3]
C_GRID = [4.8e-3, 5.4e-3, 6.0e-3, 6.6e-3, 7.2e-3]
VAC_GRID = [85, 100, 115]

res = {}
for k in GAINS:
    res[k] = dict(
        cur_nom=cur(k), cur_worst=cur(k, 250, 0.7e-3, 2.5 * TS),
        vol_nom=vol(k), vol_worst=vol(k, 4.8e-3, 115),
        cur_grid=[[cur(k, v, L, 2.5 * TS)[1] for v in VCS_GRID] for L in L_GRID],
        vol_grid=[[vol(k, C, va)[1] for C in C_GRID] for va in VAC_GRID],
    )

# ---- 判定（色だけに頼らず記号とラベルも付ける） ------------------------
def pm_status(pm):
    if pm >= 60: return ('good', '✓', '余裕大')
    if pm >= 45: return ('warn', '△', '目安クリア')
    return ('crit', '✕', '要注意')


def gm_status(gm):
    if gm >= 10: return ('good', '✓', '余裕大')
    if gm >= 6: return ('warn', '△', '目安クリア')
    return ('crit', '✕', '要注意')


esc = html.escape

# ---- 部品 ------------------------------------------------------------------
def tile(title, sub, pm, gm):
    st, ic, lb = pm_status(pm)
    return f'''<div class="tile {st}">
  <div class="t-title">{esc(title)}</div><div class="t-sub">{esc(sub)}</div>
  <div class="t-num">{pm:.1f}<span>°</span></div>
  <div class="t-lab">位相余裕（調べた範囲の最小）</div>
  <div class="t-foot"><span class="badge {st}">{ic} {lb}</span> ゲイン余裕（最小） {gm:.1f} dB</div>
</div>'''


def bar_chart(rows, vmax, guide, unit, title, guide_label):
    """rows: (label, value, color_var, dashed)。横棒。"""
    W, lw, rh, gap = 900, 210, 28, 10
    pw = W - lw - 110
    H = len(rows) * (rh + gap) + 40
    x = lambda v: lw + pw * min(v, vmax) / vmax
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{esc(title)}" class="chart">']
    for t in range(0, int(vmax) + 1, 15 if vmax > 40 else 5):
        out.append(f'<line x1="{x(t):.1f}" x2="{x(t):.1f}" y1="8" y2="{H-26}" class="grid"/>'
                   f'<text x="{x(t):.1f}" y="{H-8}" class="tick" text-anchor="middle">{t}</text>')
    for i, (lab, v, col, hatch) in enumerate(rows):
        y = 12 + i * (rh + gap)
        st = pm_status(v)[0] if unit == '°' else gm_status(v)[0]
        ic = pm_status(v)[1] if unit == '°' else gm_status(v)[1]
        fill = f'var({col})'
        pat = f' fill="url(#hatch{col[2:]})"' if hatch else f' fill="{fill}"'
        out.append(f'<g class="bar"><title>{esc(lab)}: {v:.1f}{unit}</title>'
                   f'<text x="{lw-10}" y="{y+rh/2+5}" class="lab" text-anchor="end">{esc(lab)}</text>'
                   f'<rect x="{lw}" y="{y}" width="{x(v)-lw:.1f}" height="{rh}" rx="4"{pat} stroke="{fill}" stroke-width="2"/>'
                   f'<text x="{x(v)+8:.1f}" y="{y+rh/2+5}" class="val">{v:.1f}{unit} <tspan class="ic-{st}">{ic}</tspan></text></g>')
    out.append(f'<line x1="{x(guide):.1f}" x2="{x(guide):.1f}" y1="4" y2="{H-26}" class="guide"/>'
               f'<text x="{x(guide)+4:.1f}" y="{H-30}" class="guide-t">{esc(guide_label)}</text>')
    out.append('<defs>' + ''.join(
        f'<pattern id="hatch{c}" patternUnits="userSpaceOnUse" width="7" height="7" patternTransform="rotate(45)">'
        f'<rect width="7" height="7" fill="var(--bg)"/><line x1="0" y1="0" x2="0" y2="7" stroke="var(--{c})" stroke-width="4"/></pattern>'
        for c in ('blue', 'orange')) + '</defs></svg>')
    return '\n'.join(out)


def heat(grid, rows, cols, rlab, clab, caption):
    h = ['<table class="heat"><caption>' + esc(caption) + '</caption><tr><th class="corner">'
         + esc(rlab) + ' ＼ ' + esc(clab) + '</th>'
         + ''.join(f'<th>{esc(c)}</th>' for c in cols) + '</tr>']
    for r, row in zip(rows, grid):
        h.append(f'<tr><th>{esc(r)}</th>')
        for v in row:
            st, ic, lb = pm_status(v)
            h.append(f'<td class="{st}" title="{esc(r)} / {lb}">{v:.0f}° <span class="ic">{ic}</span></td>')
        h.append('</tr>')
    return ''.join(h) + '</table>'


def img(name):
    with open(os.path.join(RES, name), 'rb') as f:
        return 'data:image/png;base64,' + base64.b64encode(f.read()).decode()


S, N = res['safe'], res['normal']
for r in (S, N):
    r['cur_pm_min'] = min(r['cur_worst'][1], min(min(x) for x in r['cur_grid']))
    r['vol_pm_min'] = min(r['vol_worst'][1], min(min(x) for x in r['vol_grid']))
dv = lambda fc: 1.0 / (PI2 * fc * C_NOM)   # 1A ステップでの電圧の振れ [V]

block = f'''<svg viewBox="0 0 900 230" class="chart block" role="img" aria-label="制御構成のブロック図">
<defs><marker id="ar" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto">
<path d="M0,0 L10,5 L0,10 z" fill="var(--ink2)"/></marker></defs>
<g class="blk">
 <rect x="10" y="70" width="110" height="56" rx="8"/><text x="65" y="94">出力電圧指令</text><text x="65" y="114" class="s">Vref = 48 V</text>
 <rect x="160" y="60" width="150" height="76" rx="8" class="hl"/><text x="235" y="86">電圧ループ PI</text><text x="235" y="106" class="s">50 Hz 更新</text><text x="235" y="124" class="s b">fc {GAINS['safe']['fc_v']:.0f} Hz / {GAINS['normal']['fc_v']:.0f} Hz</text>
 <circle cx="360" cy="98" r="18"/><text x="360" y="104" class="big">×</text>
 <rect x="410" y="60" width="160" height="76" rx="8" class="hl"/><text x="490" y="86">電流ループ PI</text><text x="490" y="106" class="s">100 kHz</text><text x="490" y="124" class="s b">fc {GAINS['safe']['fc_i']/1000:.0f} kHz / {GAINS['normal']['fc_i']/1000:.1f} kHz</text>
 <rect x="610" y="70" width="110" height="56" rx="8"/><text x="665" y="94">PWM</text><text x="665" y="114" class="s">Q1 / Q2</text>
 <rect x="760" y="60" width="130" height="76" rx="8"/><text x="825" y="86">PFC 回路</text><text x="825" y="106" class="s">L = 1 mH</text><text x="825" y="124" class="s">C = 6 mF</text>
 <text x="360" y="40" class="s">|vac|（入力電圧の形）</text>
</g>
<g class="wire">
 <line x1="120" y1="98" x2="158" y2="98" marker-end="url(#ar)"/><line x1="310" y1="98" x2="340" y2="98" marker-end="url(#ar)"/>
 <line x1="360" y1="48" x2="360" y2="78" marker-end="url(#ar)"/>
 <line x1="378" y1="98" x2="408" y2="98" marker-end="url(#ar)"/><line x1="570" y1="98" x2="608" y2="98" marker-end="url(#ar)"/>
 <line x1="720" y1="98" x2="758" y2="98" marker-end="url(#ar)"/>
 <polyline points="825,136 825,172 490,172 490,138" fill="none" marker-end="url(#ar)"/>
 <polyline points="855,136 855,206 235,206 235,138" fill="none" marker-end="url(#ar)"/>
 <text x="380" y="88" class="s l">Iref</text><text x="130" y="88" class="s l">Ipk</text>
 <text x="660" y="166" class="s">インダクタ電流（内側ループ・速い）</text>
 <text x="540" y="200" class="s">出力電圧の 20 ms 移動平均（外側ループ・遅い）</text>
</g></svg>'''
# Ipk label belongs between voltage PI and multiplier
block = block.replace('<text x="130" y="88" class="s l">Ipk</text>', '<text x="325" y="88" class="s l">Ipk</text>')

compare = f'''<table class="cmp">
<tr><th></th><th><span class="sw" style="--c:var(--blue)"></span>立ち上げ用<br><code>GAIN_SAFE = 1</code></th><th><span class="sw" style="--c:var(--orange)"></span>通常<br><code>GAIN_SAFE = 0</code></th></tr>
<tr><th>電流ループの帯域</th><td>1 kHz</td><td>3.3 kHz</td></tr>
<tr><th>電圧ループの帯域</th><td>1 Hz</td><td>2 Hz</td></tr>
<tr><th>電流ループ 位相余裕（最悪）</th><td class="good">{S['cur_pm_min']:.1f}° ✓</td><td class="crit">{N['cur_pm_min']:.1f}° ✕</td></tr>
<tr><th>電流ループ ゲイン余裕（最悪）</th><td class="good">{S['cur_worst'][2]:.1f} dB ✓</td><td class="crit">{N['cur_worst'][2]:.1f} dB ✕</td></tr>
<tr><th>電圧ループ 位相余裕（最悪）</th><td class="good">{S['vol_pm_min']:.1f}° ✓</td><td class="warn">{N['vol_pm_min']:.1f}° △</td></tr>
<tr><th>負荷 1 A 急変時の電圧の振れ（目安）</th><td>約 {dv(1.0):.0f} V</td><td>約 {dv(2.0):.0f} V</td></tr>
<tr><th>零クロス付近の電流歪み</th><td>やや大きい</td><td>小さい</td></tr>
<tr><th>使いどころ</th><td>初通電・校正前</td><td>校正後の通常運転</td></tr>
</table>'''

settings = f'''<table class="cmp">
<tr><th>ループ</th><th>項目</th><th>立ち上げ用</th><th>通常</th></tr>
<tr><th rowspan="3">電流</th><td>交差 / ゼロ</td><td>1 kHz / 200 Hz</td><td>3.33 kHz / 667 Hz</td></tr>
<tr><td>シフト量 SH_I</td><td>14</td><td>13</td></tr>
<tr><td>KI_A / KI_B</td><td>{GAINS['safe']['ka'][0]} / {GAINS['safe']['ka'][1]}</td><td>{GAINS['normal']['ka'][0]} / {GAINS['normal']['ka'][1]}</td></tr>
<tr><th rowspan="3">電圧</th><td>交差 / ゼロ</td><td>1 Hz / 0.2 Hz</td><td>2 Hz / 0.4 Hz</td></tr>
<tr><td>シフト量 SH_V</td><td>17</td><td>16</td></tr>
<tr><td>KV_A / KV_B</td><td>{GAINS['safe']['kv'][0]} / {GAINS['safe']['kv'][1]}</td><td>{GAINS['normal']['kv'][0]} / {GAINS['normal']['kv'][1]}</td></tr>
</table>'''

pm_rows = [
    ('電流・立ち上げ用（公称）', S['cur_nom'][1], '--blue', False),
    ('電流・立ち上げ用（最悪）', S['cur_worst'][1], '--blue', True),
    ('電流・通常（公称）', N['cur_nom'][1], '--orange', False),
    ('電流・通常（最悪）', N['cur_worst'][1], '--orange', True),
    ('電圧・立ち上げ用（公称）', S['vol_nom'][1], '--blue', False),
    ('電圧・立ち上げ用（最悪）', S['vol_worst'][1], '--blue', True),
    ('電圧・通常（公称）', N['vol_nom'][1], '--orange', False),
    ('電圧・通常（最悪）', N['vol_worst'][1], '--orange', True),
]
gm_rows = [(a, {0: S['cur_nom'], 1: S['cur_worst'], 2: N['cur_nom'], 3: N['cur_worst'],
                4: S['vol_nom'], 5: S['vol_worst'], 6: N['vol_nom'], 7: N['vol_worst']}[i][2], c, h)
           for i, (a, _, c, h) in enumerate(pm_rows)]

vcs_lab = [f'{v} V' for v in VCS_GRID]
l_lab = [f'{L*1e3:.2f} mH' for L in L_GRID]
c_lab = [f'{C*1e3:.1f} mF' for C in C_GRID]
vac_lab = [f'{v} Vrms' for v in VAC_GRID]

doc = f'''<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>制御ループ安定余裕</title>
<style>
:root{{--bg:#fcfcfb;--card:#ffffff;--ink:#0b0b0b;--ink2:#52514e;--ink3:#8a887f;--line:#e4e2dc;
--blue:#2a78d6;--orange:#eb6834;--good:#0ca30c;--warn:#fab219;--crit:#d03b3b;
--good-bg:#e3f3e1;--warn-bg:#fdf1d2;--crit-bg:#f8dede;--hl:#eaf2fc;}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{--bg:#1a1a19;--card:#232321;--ink:#f0efec;--ink2:#c3c2b7;--ink3:#8f8d85;--line:#3a3936;
--blue:#3987e5;--orange:#d95926;--good-bg:#1f3a1f;--warn-bg:#3d3216;--crit-bg:#432222;--hl:#1e2c3d;}}}}
:root[data-theme="dark"]{{--bg:#1a1a19;--card:#232321;--ink:#f0efec;--ink2:#c3c2b7;--ink3:#8f8d85;--line:#3a3936;
--blue:#3987e5;--orange:#d95926;--good-bg:#1f3a1f;--warn-bg:#3d3216;--crit-bg:#432222;--hl:#1e2c3d;}}
*{{box-sizing:border-box}}
body{{margin:0;background:var(--bg);color:var(--ink);font-family:"Hiragino Sans","Yu Gothic UI","Meiryo","Noto Sans JP",sans-serif;line-height:1.7}}
main{{max-width:1000px;margin:0 auto;padding:32px 16px 64px}}
h1{{font-size:1.7rem;margin:0 0 4px}}
h2{{font-size:1.25rem;margin:48px 0 6px}} h2+.lead{{margin-top:0}}
.lead{{color:var(--ink2);margin:0 0 16px}}
code{{font-family:ui-monospace,Consolas,monospace;font-size:.88em;background:color-mix(in srgb,var(--line) 60%,transparent);padding:1px 5px;border-radius:4px}}
.card{{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px 20px;margin:14px 0}}
.tiles{{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,200px),1fr));gap:12px;margin:18px 0}}
.tile{{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px 16px;border-top:5px solid var(--good)}}
.tile.warn{{border-top-color:var(--warn)}} .tile.crit{{border-top-color:var(--crit)}}
.t-title{{font-weight:700}} .t-sub{{color:var(--ink2);font-size:.85rem}}
.t-num{{font-size:2.4rem;font-weight:700;line-height:1.1;margin-top:8px;font-variant-numeric:tabular-nums}} .t-num span{{font-size:1.3rem}}
.t-lab{{color:var(--ink2);font-size:.85rem}} .t-foot{{margin-top:8px;font-size:.85rem;color:var(--ink2)}}
.badge{{display:inline-block;padding:1px 8px;border-radius:99px;font-weight:700;font-size:.8rem;color:var(--ink);margin-right:6px}}
.badge.good{{background:var(--good-bg)}} .badge.warn{{background:var(--warn-bg)}} .badge.crit{{background:var(--crit-bg)}}
.verdict{{display:flex;flex-wrap:wrap;gap:4px 12px;align-items:flex-start}} .verdict > b{{white-space:nowrap}} .verdict div{{flex:1 1 260px;min-width:0}}
.chart{{width:100%;height:auto;display:block}}
.chart .grid{{stroke:var(--line);stroke-width:1}} .chart .tick{{fill:var(--ink3);font-size:12px}}
.chart .lab{{fill:var(--ink);font-size:14px}} .chart .val{{fill:var(--ink);font-size:14px;font-weight:600}}
.chart .guide{{stroke:var(--ink2);stroke-width:1.5;stroke-dasharray:5 4}} .chart .guide-t{{fill:var(--ink2);font-size:13px}}
.chart .bar:hover rect{{filter:brightness(1.1)}}
.ic-good{{fill:var(--good)}} .ic-warn{{fill:#b07800}} .ic-crit{{fill:var(--crit)}}
.block .blk rect,.block .blk circle{{fill:var(--card);stroke:var(--ink2);stroke-width:1.5}} .block .blk rect.hl{{fill:var(--hl);stroke:var(--blue);stroke-width:2}}
.block text{{fill:var(--ink);font-size:14px;text-anchor:middle}} .block text.s{{font-size:12px;fill:var(--ink2)}} .block text.b{{fill:var(--ink);font-weight:700}}
.block text.big{{font-size:22px}} .block .wire line,.block .wire polyline{{stroke:var(--ink2);stroke-width:1.6}}
.block text.l{{font-weight:700;fill:var(--ink)}}
.two{{display:grid;grid-template-columns:repeat(auto-fit,minmax(min(100%,430px),1fr));gap:16px}}
.tblwrap{{overflow-x:auto}}
table{{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}}
.cmp th,.cmp td{{padding:9px 12px;border-bottom:1px solid var(--line);text-align:center;font-size:.93rem}}
.cmp th:first-child{{text-align:left;color:var(--ink2);font-weight:600}} .cmp tr:first-child th{{color:var(--ink);font-weight:700;vertical-align:bottom}}
td.good{{background:var(--good-bg)}} td.warn{{background:var(--warn-bg)}} td.crit{{background:var(--crit-bg)}}
.heat caption{{text-align:left;font-weight:700;padding:0 0 8px}}
.heat th,.heat td{{padding:8px 6px;text-align:center;font-size:.9rem;border:2px solid var(--card)}}
.heat th{{color:var(--ink2);font-weight:600;background:var(--card)}} .heat th.corner{{font-size:.75rem}}
.heat td{{border-radius:6px;font-weight:600;white-space:nowrap}} .heat .ic{{font-size:.8em}}
.legend{{display:flex;flex-wrap:wrap;gap:14px;font-size:.88rem;color:var(--ink2);margin:8px 0}}
.legend i{{display:inline-block;width:14px;height:14px;border-radius:4px;vertical-align:-2px;margin-right:5px}}
.sw{{display:inline-block;width:20px;height:0;border-top:3px solid var(--c);vertical-align:middle;margin-right:6px}}
.sw.d{{border-top-style:dashed}}
figure{{margin:12px 0}} figure img{{width:100%;height:auto;border-radius:10px;border:1px solid var(--line);background:#fcfcfb}}
figcaption{{color:var(--ink2);font-size:.88rem;margin-top:6px}}
.howto{{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:12px}}
.howto div{{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px;font-size:.92rem}}
.howto b{{display:block;margin-bottom:4px}}
ul{{padding-left:1.3em}} li{{margin:4px 0}}
.foot{{color:var(--ink3);font-size:.85rem;margin-top:40px}}
</style></head><body><main>

<h1>制御ループの安定余裕</h1>
<p class="lead">ブリッジレス PFC ファーム <code>firmware/src/main_closed_loop.c</code> の 2 つのゲイン設定（<code>GAIN_SAFE = 1</code> 立ち上げ用 / <code>0</code> 通常）について、回路定数がずれたときにも発振しないかを比較した。</p>

<div class="card verdict"><b>結論</b><div>
立ち上げ用ゲインは、最悪条件でも <b>位相余裕 66° 以上・ゲイン余裕 15 dB 以上</b> で、発振の心配はない。
通常ゲインは電流ループが最悪条件で <b>位相余裕 {N['cur_worst'][1]:.0f}°・ゲイン余裕 {N['cur_worst'][2]:.1f} dB</b> まで下がるので、校正が済むまでは立ち上げ用で回す。</div></div>

<div class="tiles">
{tile('電流ループ', '立ち上げ用（1 kHz）', S['cur_pm_min'], S['cur_worst'][2])}
{tile('電圧ループ', '立ち上げ用（1 Hz）', S['vol_pm_min'], S['vol_worst'][2])}
{tile('電流ループ', '通常（3.3 kHz）', N['cur_pm_min'], N['cur_worst'][2])}
{tile('電圧ループ', '通常（2 Hz）', N['vol_pm_min'], N['vol_worst'][2])}
</div>
<div class="legend"><span><i style="background:var(--good)"></i>✓ 余裕大（位相余裕 60° 以上 / ゲイン余裕 10 dB 以上）</span><span><i style="background:var(--warn)"></i>△ 目安クリア（45° 以上 / 6 dB 以上）</span><span><i style="background:var(--crit)"></i>✕ 要注意（45° 未満 / 6 dB 未満）</span></div>

<h2>1. 制御の構成</h2>
<p class="lead">外側の電圧ループが電流の振幅 Ipk を決め、入力電圧の形 |vac| を掛けて電流指令 Iref を作る。内側の電流ループがその指令にインダクタ電流を追従させる。太字が各ループの交差周波数（立ち上げ用 / 通常）。</p>
<div class="card">{block}</div>

<h2>2. 立ち上げ用と通常の比較</h2>
<p class="lead">安定性と応答の速さはトレードオフ。立ち上げ用は余裕を取るかわりに応答が遅い。</p>
<div class="card tblwrap">{compare}</div>

<h2>3. 安定余裕の比較</h2>
<p class="lead">塗りつぶし = 公称条件、斜線 = 最悪条件。点線が一般的な目安（位相余裕 45°、ゲイン余裕 6 dB）。</p>
<div>
<div class="card"><b>位相余裕 [°]</b>{bar_chart(pm_rows, 90, 45, '°', '位相余裕', '目安 45°')}</div>
<div class="card"><b>ゲイン余裕 [dB]</b>{bar_chart(gm_rows, 30, 6, ' dB', 'ゲイン余裕', '目安 6 dB')}</div>
</div>
<div class="howto">
<div><b>位相余裕とは</b>ループゲインが 1（0 dB）になる周波数で、位相が −180° まであと何度あるか。小さいほど応答が振動的になり、0° で発振する。</div>
<div><b>ゲイン余裕とは</b>位相が −180° になる周波数で、ゲインをあと何 dB 上げると 0 dB に届くか。回路定数のずれでゲインが増えたときの余裕。</div>
<div><b>最悪条件とは</b>電流ループは Vcs 250 V・L 0.7 mH・遅れ 2.5 Ts、電圧ループは C 4.8 mF・入力 115 Vrms。ループゲインと遅れが最も大きくなる組み合わせ。</div>
</div>

<h2>4. 回路定数がずれたときの位相余裕</h2>
<p class="lead">マス目ごとに条件を変えて位相余裕を計算した。数値と記号が判定（色は補助）。電流ループは遅れを大きめの 2.5 Ts で評価。</p>
<div class="two">
<div class="card tblwrap">{heat(S['cur_grid'], l_lab, vcs_lab, 'L', 'Vcs', '電流ループ・立ち上げ用')}</div>
<div class="card tblwrap">{heat(N['cur_grid'], l_lab, vcs_lab, 'L', 'Vcs', '電流ループ・通常')}</div>
<div class="card tblwrap">{heat(S['vol_grid'], vac_lab, c_lab, '入力', 'C', '電圧ループ・立ち上げ用')}</div>
<div class="card tblwrap">{heat(N['vol_grid'], vac_lab, c_lab, '入力', 'C', '電圧ループ・通常')}</div>
</div>
<p class="lead">通常ゲインの電流ループは「Vcs が高い・L が小さい」側（右上）ほど余裕が減る。立ち上げ用はどのマスでも 60° 以上。</p>

<h2>5. ボード線図</h2>
<p class="lead"><span class="sw" style="--c:var(--blue)"></span>立ち上げ用（公称）　<span class="sw" style="--c:var(--orange)"></span>通常（公称）　<span class="sw d" style="--c:var(--blue)"></span>立ち上げ用（最悪）　<span class="sw d" style="--c:var(--orange)"></span>通常（最悪）<br>
上段がループゲイン、下段が位相。● が交差周波数で、下段の縦線の長さ（−180° から ● まで）が位相余裕。</p>
<figure><img src="{img('bode_current.png')}" alt="電流ループのボード線図"><figcaption>電流ループ（100 kHz）。通常ゲインは最悪条件で交差周波数が 5.4 kHz まで上がり、位相が急に落ちる帯域に入る。</figcaption></figure>
<figure><img src="{img('bode_voltage.png')}" alt="電圧ループのボード線図"><figcaption>電圧ループ（50 Hz 更新）。どちらのゲインも位相の山の付近で交差していて、余裕は十分。</figcaption></figure>

<h2>6. ゲインの設定値</h2>
<div class="card tblwrap">{settings}</div>

<h2>7. 計算の前提</h2>
<div class="card"><ul>
<li>一巡伝達関数 L(s) = (Kp + Ki/s) · P(s) · e<sup>−s·td</sup>。PI のゼロは交差周波数の 1/5。</li>
<li>電流ループ：P(s) = Vcs / (s·L)。公称 Vcs 220 V・L 1 mH・遅れ 1.5 Ts（Ts = 10 µs）。</li>
<li>電圧ループ：P(s) = Vac_pk / (2·Vo·C·s)。公称 入力 100 Vrms・Vo 48 V・C 6 mF（3 mF × 2）・遅れ 30 ms（20 ms 移動平均 ＋ 更新 1 周期分）。</li>
<li>連続時間モデルにむだ時間を掛けて離散化の遅れを表した。交差周波数付近の評価は十分正確（ナイキスト周波数の近くでは少しずれる）。</li>
<li>負荷急変時の電圧の振れは ΔV ≈ ΔIo / (2π·fc·C) による目安。</li>
</ul></div>

<p class="foot">生成：<code>simulation/python/loop_bode.py</code>（ボード線図） → <code>simulation/python/make_loop_report.py</code>（このページ）</p>
</main></body></html>'''

out = os.path.join(RES, 'loop_stability_report.html')
with open(out, 'w', encoding='utf-8') as f:
    f.write(doc)
print(out)
for k in res:
    print(k, 'pm_min', round(res[k].get('cur_pm_min',0),1), round(res[k].get('vol_pm_min',0),1), 'cur', [round(x, 1) for x in res[k]['cur_worst']], 'vol', [round(x, 2) for x in res[k]['vol_worst']])
