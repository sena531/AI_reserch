"""電流ループ・PFC電圧ループの一巡伝達関数（ボード線図）を描く。

firmware/src/main_closed_loop.c の GAIN_SAFE=1（立ち上げ用）と GAIN_SAFE=0（通常）を、
公称条件と最悪条件で比較する。出力は results/bode_current.png, results/bode_voltage.png。
"""
import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
plt.rcParams.update({'font.family':'IPAGothic','font.size':10,'axes.edgecolor':'#9a9890',
  'axes.labelcolor':'#52514e','xtick.color':'#52514e','ytick.color':'#52514e','text.color':'#0b0b0b',
  'figure.facecolor':'#fcfcfb','axes.facecolor':'#fcfcfb','axes.spines.top':False,'axes.spines.right':False})
BLUE,ORANGE='#2a78d6','#eb6834'; GRID='#e4e2dc'; INK2='#52514e'
PI2=2*np.pi

def loop(f,Kp,Ki,G,td):
    s=1j*PI2*f; return (Kp+Ki/s)*G/s*np.exp(-s*td)
def analyze(f,L):
    mag=20*np.log10(abs(L)); ph=np.unwrap(np.angle(L))*180/np.pi
    i=np.where(np.diff(np.sign(mag)))[0][-1]; fc=f[i]; pm=180+ph[i]
    j=np.where(np.diff(np.sign(ph+180)))[0]
    fg=f[j[0]] if len(j) else None; gm=-mag[j[0]] if len(j) else None
    return mag,ph,fc,pm,fg,gm

def cur_gain(fc): Kp=PI2*fc*1e-3/220; return Kp,Kp*PI2*fc/5
def vol_gain(fc): G0=141.42/(2*48*6e-3); Kp=PI2*fc/G0; return Kp,Kp*PI2*fc/5

cases_cur=[('立ち上げ用（公称）',BLUE,'-',cur_gain(1000),220/1e-3,1.5e-5),
           ('通常（公称）',ORANGE,'-',cur_gain(1e5/30),220/1e-3,1.5e-5),
           ('立ち上げ用（最悪条件）',BLUE,'--',cur_gain(1000),250/0.7e-3,2.5e-5),
           ('通常（最悪条件）',ORANGE,'--',cur_gain(1e5/30),250/0.7e-3,2.5e-5)]
cases_vol=[('立ち上げ用（公称）',BLUE,'-',vol_gain(1.0),100*1.41421/(2*48*6e-3),0.03),
           ('通常（公称）',ORANGE,'-',vol_gain(2.0),100*1.41421/(2*48*6e-3),0.03),
           ('立ち上げ用（最悪条件）',BLUE,'--',vol_gain(1.0),115*1.41421/(2*48*4.8e-3),0.03),
           ('通常（最悪条件）',ORANGE,'--',vol_gain(2.0),115*1.41421/(2*48*4.8e-3),0.03)]

def fig(cases,f,title,worst,fname,ylim_mag,ylim_ph):
    fg_,(a1,a2)=plt.subplots(2,1,figsize=(8.6,7.4),sharex=True,gridspec_kw={'hspace':0.12})
    rows=[]
    for name,c,ls,(Kp,Ki),G,td in cases:
        mag,ph,fc,pm,fgx,gm=analyze(f,loop(f,Kp,Ki,G,td))
        a1.semilogx(f,mag,color=c,ls=ls,lw=2,label=name)
        a2.semilogx(f,ph,color=c,ls=ls,lw=2)
        a1.plot(fc,0,'o',ms=8,color=c,mec='#fcfcfb',mew=2,zorder=5)
        ph_c=-180+pm
        a2.plot([fc,fc],[-180,ph_c],color=c,lw=2,ls=ls,alpha=.9)
        a2.plot(fc,ph_c,'o',ms=8,color=c,mec='#fcfcfb',mew=2,zorder=5)
        rows.append((name,fc,pm,gm))
    for a in (a1,a2):
        a.grid(True,which='major',color=GRID,lw=.8); a.grid(True,which='minor',color=GRID,lw=.4,alpha=.6)
    a1.axhline(0,color=INK2,lw=1); a2.axhline(-180,color=INK2,lw=1)
    a1.set_ylabel('ループゲイン [dB]'); a2.set_ylabel('位相 [deg]'); a2.set_xlabel('周波数 [Hz]')
    a1.set_ylim(*ylim_mag); a2.set_ylim(*ylim_ph); a2.set_yticks(np.arange(ylim_ph[0],ylim_ph[1]+1,45))
    a1.set_xlim(f[0],f[-1])
    a1.set_title(title,loc='left',fontsize=13,pad=26)
    a1.text(0,1.02,worst,transform=a1.transAxes,fontsize=9,color=INK2,va='bottom')
    a1.legend(loc='lower left',frameon=False,fontsize=9)
    a2.text(0.01,0.03,'● = 交差周波数。縦線の長さ（−180° からの距離）が位相余裕',transform=a2.transAxes,fontsize=9,color=INK2)
    # table
    txt='\n'.join(f'{n:<14s} 交差 {(f'{fc:,.0f}' if fc>=100 else f'{fc:.2f}'):>6s} Hz   位相余裕 {pm:5.1f}°   ゲイン余裕 {gm:5.1f} dB' for n,fc,pm,gm in rows)
    a2.text(0.99,0.97,txt,transform=a2.transAxes,ha='right',va='top',fontsize=8.5,color='#0b0b0b',
            family='IPAGothic',bbox=dict(fc='#fcfcfb',ec=GRID,boxstyle='round,pad=0.5'))
    fg_.savefig(fname,dpi=200,bbox_inches='tight'); print(fname); [print(r) for r in rows]

import os
OUT=os.path.join(os.path.dirname(os.path.abspath(__file__)),'results')
fig(cases_cur,np.logspace(1,np.log10(5e4),4000),'電流ループ（100 kHz）一巡伝達関数',
    '最悪条件 = Vcs 250 V・L 0.7 mH・遅れ 2.5 Ts（ループゲインが最大で遅れも最大）',
    os.path.join(OUT,'bode_current.png'),(-40,60),(-270,-45))
fig(cases_vol,np.logspace(-2,np.log10(25),4000),'PFC 電圧ループ（50 Hz 更新）一巡伝達関数',
    '最悪条件 = C 4.8 mF（−20%）・入力 115 Vrms（ループゲインが最大）。遅れ 30 ms',
    os.path.join(OUT,'bode_voltage.png'),(-40,80),(-270,-45))
