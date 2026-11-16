/* ===========================================================================
 *  ブリッジレスPFC + 位相シフトフルブリッジ  閉ループ制御
 *
 *  ターゲット : dsPIC33EP128GS808 DEVELOPMENT BOARD (DM330026)
 *               MPLAB X + XC16
 *  電源基板   : 回路レイアウト/kicad/buck_pfc.2/buck_pfc.2.kicad_sch
 *
 *  Fcy = 60MHz / Fosc = 119.7MHz / スイッチング 100kHz (PTPER = 9600)
 *
 *  制御方式は 制御＿シミュレーション/closed_loop_buck_pfc.sciprc（Scideam）で
 *  検証したものをそのまま移植している。出典は 修論（林さん）.docx 4.2 節。
 *
 *  --- 動作モード（SW1 = RE4 を押して離すと切り替わる）--------------------
 *   開ループ（起動時。LED1 消灯）
 *     基板上のボリューム R12（AN2）で「入力電流の振幅指令」を手で与える。
 *     電流ループとフィードフォワードは動いたままなので、入力電流は
 *     正弦波の形を保ったまま振幅だけを増減できる。
 *     ★昇圧形なのでデューティを直に固定する開ループは危険（電流が発散する）。
 *       そのため「電流指令を手で与える」形にしてある。
 *   閉ループ（LED1 点灯）
 *     フォトカプラ帰還（AN2）の出力電圧ループが振幅指令を作る。
 *   切り替え時は Q1/Q2 のゲートを 20ms 止め、電圧ループの積分器を
 *   そのときの Ipk に合わせてから入るのでバンプが出ない。
 *
 *  --- 3ループ構成 -------------------------------------------------------
 *   (1) 電流ループ      100 kHz  平均電流制御 -> Q1/Q2 のデューティ
 *   (2) PFC電圧ループ    50 Hz   出力電圧 -> 入力電流の振幅指令
 *   (3) 位相シフト       50 Hz   D1_FIX 固定（既定）／低速トリム（任意）
 *
 *  ★Scideam 検証で分かった重要点（closed1.c から変えたところ）
 *   ・PFC電圧ループの帰還は「商用1周期（20ms）の移動平均」にする。
 *     瞬時値のままだと出力リプル（100Hz）がそのまま指令に乗る。
 *     窓幅を商用1周期ちょうどにすると 50/100Hz が原理的に完全除去される。
 *   ・位相シフトは高速に動かさない。Scideam で瞬時 Vo を帰還にしたところ、
 *     出力リプル ±14V に対して P項だけで dD1 = ±3.7 を要求し、
 *     可動域 0〜0.5 の位相が毎周期 0 と 0.5 に叩きつけられて
 *     入力電流が大きく歪んだ。Cs1/Cs2 は脈動エネルギーを蓄えないので
 *     D1 を固定しても Vcs が自然に変動して電力を通す。まず固定で回すこと。
 *   ・電流ループに昇圧則のフィードフォワードを併用する。
 *     D_ff = PTPER*(1 - |vac|/VCS_FF)。PI の負担が減り歪みが小さくなる。
 *
 *  --- ハード対応 --------------------------------------------------------
 *   ADC（すべて PWM4 トリガ、AN2 完了割り込みで制御を実行）
 *     ADCBUF0 = AN0  VL_sense   分圧 + MCP6021 バッファ
 *     ADCBUF1 = AN1  VN_sense   分圧 + MCP6021 バッファ
 *     ADCBUF2 = AN11 Vo_sense   TLP291 フォトカプラ絶縁帰還
 *                    J22 の真ん中のピンに入れる（JP22 のジャンパは外す）
 *                    （AN2 = 基板上ボリューム R12。ADCON4H.C2CHS で切替）
 *                    ※ C2CHS で AN11 を選んでも結果は ADCBUF2 に入り、
 *                      _ADCAN2Interrupt が鳴る。トリガは TRGSRC2 と
 *                      TRGSRC11 の両方を PWM4 にしてある。
 *     ADCBUF3 = AN3  ILsense    TMCS1107A3 ホール電流センサ（双方向）
 *
 *     VL_sense と VN_sense はどちらも GND2 基準の片側だけが立つ。
 *       正の半周期 : VL = |vac|, VN = 0     （Q2 のボディダイオードでクランプ）
 *       負の半周期 : VL = 0,     VN = |vac| （Q1 のボディダイオードでクランプ）
 *     したがって
 *       Vac  = ADCBUF0 - ADCBUF1  -> 符号付き（極性判定）
 *       Vrec = ADCBUF0 + ADCBUF1  -> |vac| 全波整流（乗算器とFFに使う）
 *
 *   PWM（PTPER = 9600 = 100kHz、1 count = 1.0417ns）
 *     PWM1  PDC1    Q1  ブリッジレスPFC 短絡スイッチ（正の半周期にPWM）
 *     PWM2  PDC2    Q2  同上（負の半周期にPWM）
 *     PWM6  PHASE6  レグA : PWM6H = Q3(ハイ) / PWM6L = Q4(ロー)  中点 Tr+
 *     PWM7  PHASE7  レグB : PWM7H = Q6(ハイ) / PWM7L = Q5(ロー)  中点 Tr-
 *       PDC6 = PDC7 = PTPER/2 固定（50% 相補）
 *       デッドタイムは DTR6/7 = ALTDTR6/7 = DEADTIME（600 = 625ns）
 *       位相シフトは PHASEx で与える。遅れ側は商用半周期ごとに入れ替える。
 *         正の半周期 : レグA 基準（PHASE6=0） / レグB 遅れ
 *         負の半周期 : レグB 基準（PHASE7=0） / レグA 遅れ
 *
 *  ★★ 実機で必ず校正する定数 ★★
 *     下の「ハードウェア校正定数」ブロックの 3 つ。
 *     ここがずれるとゲインが全部ずれる。手順はブロック内のコメント参照。
 * ===========================================================================
 */

#include "p33Exxxx.h"
#include "init.h"
#include <xc.h>
#include <stdint.h>


// DSPIC33EP128GS808 Configuration Bit Settings

// 'C' source line config statements

// FSEC
#pragma config BWRP = OFF               // Boot Segment Write-Protect bit (Boot Segment may be written)
#pragma config BSS = DISABLED           // Boot Segment Code-Protect Level bits (No Protection (other than BWRP))
#pragma config BSEN = OFF               // Boot Segment Control bit (No Boot Segment)
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = DISABLED           // General Segment Code-Protect Level bits (No Protection (other than GWRP))
#pragma config CWRP = OFF               // Configuration Segment Write-Protect bit (Configuration Segment may be written)
#pragma config CSS = DISABLED           // Configuration Segment Code-Protect Level bits (No Protection (other than CWRP))
#pragma config AIVTDIS = OFF            // Alternate Interrupt Vector Table bit (Disabled AIVT)

// FBSLIM
#pragma config BSLIM = 0x1FFF           // Boot Segment Flash Page Address Limit bits (Boot Segment Flash page address  limit)

// FSIGN

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Source Selection (Fast RC Oscillator with divide-by-N with PLL module (FRCPLL) )
#pragma config IESO = ON                // Two-speed Oscillator Start-up Enable bit (Start up device with FRC, then switch to user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = ON            // OSC2 Pin Function bit (OSC2 is general purpose digital I/O pin)
#pragma config IOL1WAY = ON             // Peripheral pin select configuration bit (Allow only one reconfiguration)
#pragma config FCKSM = CSDCMD           // Clock Switching Mode bits (Both Clock switching and Fail-safe Clock Monitor are disabled)
#pragma config PLLKEN = ON_PLL          // PLL Lock Enable Bit (Clock switch to PLL source will wait until the PLL lock signal is valid)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config WDTEN = OFF              // Watchdog Timer Enable bits (WDT and SWDTEN disabled)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config WDTWIN = WIN25           // Watchdog Timer Window Select bits (WDT Window is 25% of WDT period)

// FPOR

// FICD
#pragma config ICS = PGD3               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)
#pragma config BTSWP = OFF              // BOOTSWP Instruction Enable/Disable bit (BOOTSWP instruction is disabled)

// FDEVOPT
#pragma config PWMLOCK = OFF            // PWMx Lock Enable bit (PWM registers may be written without key sequence)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 Pin bit (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 Pin bit (I2C2 mapped to SDA2/SCL2 pins)
#pragma config DBCC = OFF               // DACx Output Cross Connection bit (No Cross Connection between DAC outputs)

// FALTREG
#pragma config CTXT1 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 1 bits (Not Assigned)
#pragma config CTXT2 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT3 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)
#pragma config CTXT4 = OFF              // Specifies Interrupt Priority Level (IPL) Associated to Alternate Working Register 2 bits (Not Assigned)

// FBTSEQ
#pragma config BSEQ = 0xFFF             // Relative value defining which partition will be active after device Reset; the partition containing a lower boot number will be active (Boot Sequence Number bits)
#pragma config IBSEQ = 0xFFF            // The one's complement of BSEQ; must be calculated by the user and written during device programming. (Inverse Boot Sequence Number bits)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

/* =======================================================================
 *  ハードウェア校正定数  ★実機で必ず確認すること★
 * ======================================================================= */

/* --- (1) 電流センサ TMCS1107A3BQDRQ1 -----------------------------------
 *  TMCS1107 の感度（データシート）: A1B=50  A2B=100  A3B=200  A4B=400 mV/A
 *  A3B なので 200mV/A。感度は電源電圧に依存しない（非レシオメトリック）。
 *  無電流時の出力は 0.5*VS（双方向 B 品）:
 *      VS=3.3V -> 1.65V = 2048 counts   VS=5V -> 2.5V = 3103 counts
 *  ★VS=5V だと 3.3V の ADC では +4A で頭打ちになるので VS は 3.3V にすること。
 *  【確認方法】入力を切った状態で ADCBUF3 を読み、その値を ISENS_OFS にする。 */
#define ISENS_V_PER_A   0.200
#define ISENS_OFS       2015    /* 実測 2000..2030 の中央値（VS=3.3V, AC OFF） */

/* --- (2) ライン電圧センス（分圧 + MCP6021 バッファ）---------------------
 *  VL_sense が 1V 上がるのに ADC が何カウント上がるか。
 *  【確認方法】AC を切って VL に既知の直流（例 50V）を加え、ADCBUF0 を読む。
 *              VLINE_CNT_PER_V = (読んだ値) / 50
 *  下の値は 150k / 1.2k 分圧を仮定した暫定値。必ず実測で置き換えること。 */
#define VLINE_CNT_PER_V 9.83

/* --- (3) 出力電圧センス（TLP291 フォトカプラ）--------------------------
 *  TLP291 の CTR は個体差が非常に大きい（50〜600%）ので計算では決まらない。
 *  【確認方法】オープンループで出力を立て、Vo を電圧計で測りながら
 *              ADCBUF2 を読む。2 点取って傾きを出す。
 *                VO_CNT_PER_V = (cnt2 - cnt1) / (V2 - V1)
 *                VO_REF_CNT   = 48V のときのカウント値
 *  下の値は「48V で 3.0V 出力」を仮定した暫定値。 */
#define VO_CNT_PER_V    77.58
#define VO_REF_CNT      3724

/* =======================================================================
 *  回路定数（Scideam の値と一致させる）
 * ======================================================================= */
#define VO_REF_V        48.0        /* 出力電圧指令 [V] */
#define IO_RATED_A      10.0        /* 定格負荷電流 [A] */
#define PO_RATED_W      480.0       /* 定格出力 [W] */
#define VAC_PK_V        141.42      /* AC100Vrms のピーク [V] */
#define N_TR            4.0         /* トランス巻数比 Np/Ns(片側) */
#define L_LK_H          5.0e-6      /* トランス漏れインダクタンス [H] */
#define VCS_DES_V       200.0       /* 位相シフト設計点の中間電圧 [V] */
#define VCS_FF_V        220.0       /* フィードフォワード用の中間電圧 [V]
                                     * ★Scideam 実測の平均値。実機でも
                                     *   Vcs1 を測って合わせること。ここが
                                     *   ずれると電圧ピーク付近が歪む。 */

/* =======================================================================
 *  タイミング定数
 * ======================================================================= */
#define PERIODmax       9600        /* PTPER : 100kHz (1count = 1.0417ns) */
#define FS_HZ           100000.0
#define TS_S            (1.0/FS_HZ)
#define F_LINE_HZ       50
#define N_VLOOP         (int)(FS_HZ/F_LINE_HZ)   /* 2000 = 商用1周期 */
#define DEADTIME        600         /* DTR6/7, ALTDTR6/7 : 625ns */
#define PHASEmax        (PERIODmax/2)            /* 4800 = 180deg */
#define PHASEmin        0

#define DUTYmax         9120        /* Q1/Q2 デューティ上限 95% */
#define DUTYmin         0

/* 零クロス近傍のゲート停止しきい値（ADC カウント）
 * Scideam の PO_HYST = 0.005*VAC_PK に相当 */
#define VAC_HYST        ((int)(0.005*VAC_PK_V*VLINE_CNT_PER_V) + 2)

/* =======================================================================
 *  ADC 換算係数
 * ======================================================================= */
#define ADC_CNT_PER_V   (4095.0/3.3)                    /* 1241.0 */
#define ADC_CNT_PER_A   (ADC_CNT_PER_V*ISENS_V_PER_A)   /* 248.2  */
#define VREC_PK_CNT     ((int)(VLINE_CNT_PER_V*VAC_PK_V))  /* |vac| ピーク */

/* =======================================================================
 *  制御ゲイン（Scideam の物理ゲインから固定小数点へ換算）
 *
 *  PI は双一次（Tustin）の増分形で実装する。
 *      u[n] = u[n-1] + A*e[n] - B*e[n-1]
 *      A = Kp + Ki*T/2 ,  B = Kp - Ki*T/2
 *  すべて浮動小数の #define からコンパイル時に整数へ畳み込まれる。
 *  校正定数を直せばゲインも自動追従する。
 * ======================================================================= */

/* =======================================================================
 *  過電流保護
 *
 *  ハード : CMP1（入力 CMP1D = AN3 = ILsense）-> PWM の Fault 入力。
 *           FCLCON1/2/6/7 の FLTSRC=13(CMP1), FLTMOD=0(ラッチ) が設定済み。
 *           FLTDAT=0b00 なので、トリップすると Q1〜Q6 が全部オフになる。
 *           ★非反転比較なので「しきい値より上」でしかトリップしない。
 *             負の半周期（センサ出力が中点より下）は守れない。
 *  ソフト : 制御 ISR で |Iin| を見て両方向を守る。遅れは最大 10us、
 *           L=1mH / 141V なら 1.4A しか進まないので実用上十分。
 *
 *  ★★ センサの測定上限に注意 ★★
 *    TMCS1107A3B (200mV/A) を VS=3.3V で使うと線形範囲は +-7.25A。
 *    出力が 3.1V で頭打ちになるので、それ以上の電流は「見えない」。
 *    一方 480W 定格での入力電流ピークは 7.3A なので、
 *    定格運転すると保護しきい値が通常動作域に入ってしまう。
 *      -> 立ち上げ中（低電力）は 7.0A で問題ない。
 *      -> 定格まで上げるときは I_OCP_A を上げる（保護が実質無効になる）か、
 *         感度の低い TMCS1107A2B (100mV/A, +-14.5A) に交換すること。
 * ======================================================================= */
#define I_OCP_A         7.0         /* ★トリップ電流 [A]（上記の上限に注意） */
#define I_OCP_CNT       ((int)(I_OCP_A*ADC_CNT_PER_A))
#define CMP1DAC_OCP     ((int)(ISENS_OFS + I_OCP_A*ADC_CNT_PER_A))

/* --- センサの健全性チェック -------------------------------------------
 *  センサが未給電だと ADCBUF3 = 0 になり、Iin = -ISENS_OFS という
 *  「大きな負電流」に見えて過電流保護が誤作動する。そこで
 *    起動時   : |Iin| が SENS_OK_TOL 以内の状態が 20ms 続いたら「健全」
 *    運転中   : ADCBUF3 が SENS_FLOOR 未満になったら「喪失」
 *  とする。VS=3.3V のセンサは出力が 0.1V 程度までしか下がらないので、
 *  0V 近辺は「電源が無い」とみなしてよい。
 *  健全でない間は Q1/Q2 を回さない（フルブリッジは動かしたまま。
 *  無通電でのゲート波形確認ができるようにするため）。 */
#define SENS_OK_TOL     400         /* 起動時の許容幅 [ADCカウント] 約±1.6A
                                     * 実測ノイズが ±133 なので 3 倍の余裕を取る */
#define SENS_FLOOR      80          /* これ未満はセンサ喪失（約0.06V）      */

/* ソフト過電流保護の有効／無効。★診断用の逃げ道★
 *   0 にするとソフト側の過電流ラッチを止める（記録だけ残す）。
 *   ハードの CMP1 は生きたままなので、正方向の本当の過電流は守られる。
 *   誤検出かどうかを切り分けるときだけ 0 にすること。 */
#define OCP_SOFT_EN     1

/* フォルト表示 LED（LD2 赤 = RE10。開発ボード回路図で確認済み） */
#define LED2_LAT        LATEbits.LATE10

/* --- 電流ループ（100kHz、交差 約2.3kHz）-------------------------------
 *  Scideam : KP_I = 0.028508 [duty/A] , KI_I = 1053.66 [duty/(A*s)]
 *  カウント系へ : x PERIODmax [PWMcount/duty] / ADC_CNT_PER_A [ADCcount/A] */
#define KP_I_PHYS       0.028508
#define KI_I_PHYS       1053.66
#define KP_I_CNT        (KP_I_PHYS*PERIODmax/ADC_CNT_PER_A)
#define KIT_I_CNT       (KI_I_PHYS*TS_S*PERIODmax/ADC_CNT_PER_A)
#define SH_I            10
#define KI_A            ((int32_t)((KP_I_CNT + KIT_I_CNT/2.0)*(1L<<SH_I)))
#define KI_B            ((int32_t)((KP_I_CNT - KIT_I_CNT/2.0)*(1L<<SH_I)))

/* --- PFC電圧ループ（50Hz、交差 4Hz、ゼロ 5Hz）-------------------------
 *  Scideam : KP_V = 0.1331 [A/V] （C2 = 7.8mF のとき。C2 を変えたら
 *            KP_V も比例して変える。Scideam 側は自動計算している）
 *  出力 Ipk は「乗算器の単位」で持つ。Iref = (Vrec * Ipk) >> SH_MUL
 *  なので、Ipk[A] -> mul単位 の換算は
 *      x ADC_CNT_PER_A x (1<<SH_MUL) / VREC_PK_CNT                      */
#define KP_V_PHYS       0.1331
#define KI_V_PHYS       (KP_V_PHYS*2.0*3.14159265*5.0)   /* ゼロ 5Hz */
#define SH_MUL          11
#define MULCONV         (ADC_CNT_PER_A*(1L<<SH_MUL)/VREC_PK_CNT)
#define KP_V_CNT        (KP_V_PHYS/VO_CNT_PER_V*MULCONV)
#define KIT_V_CNT       (KI_V_PHYS/(double)F_LINE_HZ/VO_CNT_PER_V*MULCONV)
#define SH_V            14
#define KV_A            ((int32_t)((KP_V_CNT + KIT_V_CNT/2.0)*(1L<<SH_V)))
#define KV_B            ((int32_t)((KP_V_CNT - KIT_V_CNT/2.0)*(1L<<SH_V)))

/* 電流振幅指令の上限（定格の 1.5 倍） */
#define IPK_RATED_A     (2.0*PO_RATED_W/(0.93*VAC_PK_V))     /* 7.30A */
#define IPK_MAX_MUL     ((int32_t)(1.5*IPK_RATED_A*MULCONV))

/* --- フィードフォワード -----------------------------------------------
 *  D_ff = PERIODmax*(1 - |vac|/VCS_FF_V)
 *       = PERIODmax - (Vrec * KFF >> SH_FF)                              */
#define SH_FF           12
#define KFF             ((int32_t)((double)PERIODmax/(VLINE_CNT_PER_V*VCS_FF_V)*(1L<<SH_FF)))

/* --- 位相シフト --------------------------------------------------------
 *  静特性 式(4.46) を D1 について解いた値
 *      D1 = sqrt( Vo*Llk*Io / (Ts*Vcs*(Vcs - n*Vo)) ) = 0.387
 *  位相角にすると 139deg。PWM カウントでは D1 * PERIODmax。            */
/* ★位相シフト角はここで度数で指定する（0〜180度）★
 *   設計値 139.4 度 = D1 0.387。式(4.46) を D1 について解いた値で、
 *     Vcs=200V / Vo=48V / Io=10A / Llk=5uH / Ts=10us
 *   の動作点に合わせてある。
 *
 *   角度を大きくする -> ブリッジの伝送能力が上がる -> 同じ負荷なら Vcs が下がる
 *   角度を小さくする -> 伝送能力が下がる           -> Vcs が上がる
 *   ただし無負荷では電力の行き先が無いので、角度を変えても Vcs の上昇は
 *   止まらない（Vcs は n*Vo に追随する）。負荷を繋ぐことが先。 */
#define D1_FIX_DEG      139.4
#define D1_FIX_CNT      ((int)(D1_FIX_DEG/360.0*PERIODmax))
#define D1_TRIM_CNT     768         /* 低速トリムの範囲 +-0.08*9600 */
#define PH_MODE         0           /* 0: D1_FIX 固定（推奨）
                                     * 1: 出力電圧で低速トリム       */
/* フィードフォワード D_ff = PTPER*(1 - |vac|/VCS_FF_V)
 *   ★立ち上げ中は 0 にすること★
 *   VCS_FF_V = 220V 前提なので、入力が数 V しかない段階では
 *   「220V まで昇圧せよ」という意味の デューティ 95% を出し続ける。
 *   実際の Vcs は 20V 程度なので指令として無意味で、電流ループが
 *   積分で押し戻すまでの間に電流が流れてしまう（Ipk=0 でも流れる）。
 *   100V で安定して回るようになってから 1 に戻すと歪みが減る。 */
#define USE_FF          0

/* =======================================================================
 *  動作モード（SW1 = RE4 を押して離すと切り替わる）
 *
 *   MODE_OPEN   開ループ  LED1 消灯
 *       ADCBUF2 = AN2（基板上ボリューム R12。R36 経由で直結）
 *       ボリュームで「入力電流の振幅指令 Ipk」を直接与える。
 *       電流ループとフィードフォワードは動いたままなので、入力電流は
 *       正弦波の形を保ったまま振幅だけを手で増減できる。
 *       ★昇圧形なのでデューティを直に固定するのは危険（電流が発散する）。
 *         そのため「電流指令を手で与える」形の開ループにしてある。
 *
 *   MODE_CLOSED 閉ループ  LED1 点灯
 *       ADCBUF2 = AN11（Vo_sense フォトカプラ。J22 の真ん中のピン）
 *       出力電圧ループが Ipk を作る。
 *
 *   起動時は安全側の開ループ。ボリュームを絞った状態から始めること。
 * ======================================================================= */
#define MODE_OPEN       0
#define MODE_CLOSED     1

/* モード切替直後にゲートを止める期間。ADC のチャネル切替が落ち着くまでと、
 * 移動平均の窓が埋まるまでを兼ねて商用1周期ぶん取る。 */
#define MODE_BLANK      N_VLOOP

/* SW1 の読み取り周期（制御 ISR の回数。2000 = 20ms）。
 * CN 割り込みはチャタリングで押下中に「離した」エッジを拾ってしまい、
 * 押す・離すで 2 回トグルして元に戻ることがあるため使わない。
 * 20ms ごとにポーリングし、2 回連続で同じ状態なら確定とする。 */
#define SW_SAMPLE       N_VLOOP

/* SW1(RE4) の論理。開発ボードは 10k プルアップ + 押下で GND なので 1（確認済み） */
#define SW1_ACTIVE_LOW  1

/* モード表示 LED（開発ボードのユーザーズガイド DS50002632A 回路図で確認済み）
 *   LED1 = LD1（緑）= RE11 ,  LED2 = LD2（赤）= RE10
 *   使わないなら USE_MODE_LED を 0 にする。 */
#define USE_MODE_LED    1
#define LED1_LAT        LATEbits.LATE11

/* 開ループでボリュームを全開にしたときの電流振幅指令（= 閉ループの上限と同じ） */
#define IPK_OPEN_MAX    IPK_MAX_MUL

/* PHASE レジスタは「遅らせたいカウント数」をそのまま書く。
 * 2026-09-25 実測で確認：PHASE6=0, PHASE7=5882 としたとき
 * PWM7H は PWM6H の 6006 カウント後に立ち上がった（= PHASE7 とほぼ一致）。
 * したがって PERIODmax から引く必要はない。wPhase をそのまま書く。 */

/* =======================================================================
 *  グローバル変数
 * ======================================================================= */
    /* センシング */
    int         Vac      = 0;   /* ADCBUF0 - ADCBUF1  符号付きライン電圧 */
    int         Vrec     = 0;   /* ADCBUF0 + ADCBUF1  |vac| 全波整流     */
    int         Vo_adc   = 0;   /* ADCBUF2  出力電圧                     */
    int         Iin      = 0;   /* ADCBUF3 - ISENS_OFS  符号付き入力電流 */
    int         Ir       = 0;   /* 整流座標のインダクタ電流              */
    int         half     = 0;   /* 1 = 正の半周期                        */

    /* PFC電圧ループ（50Hz） */
    int32_t     vo_sum   = 0;   /* 移動平均の積算                        */
    int         scan_cnt = 0;
    int         Vo_avg   = 0;   /* 商用1周期平均後の出力電圧             */
    int32_t     Ev       = 0, Ev_1 = 0;
    int32_t     Uv       = 0;
    int32_t     Ipk      = 0;   /* 電流振幅指令（乗算器の単位）          */

    /* 電流ループ（100kHz） */
    int32_t     Ei       = 0, Ei_1 = 0;
    int32_t     Ui       = 0;
    int         Iref     = 0;
    int         Dpi      = 0;   /* 電流PI 出力                           */
    int         Dff      = 0;   /* フィードフォワード                    */
    int         Wout     = 0;   /* Q1/Q2 デューティ（PWMカウント）       */

    /* 位相シフト */
    int32_t     Ep       = 0, Ep_1 = 0;
    int32_t     Up       = 0;
    int         wPhase   = D1_FIX_CNT;

    /* ソフトスタート */
    int         Vref     = 0;   /* 出力電圧指令（カウント、ランプする）  */
    int         vref_init = 1;  /* 1 = 次の平均値から指令を立ち上げる    */

    /* 動作モード */
    int         mode       = MODE_OPEN;
    int         mode_blank = 0; /* >0 の間は Q1/Q2 のゲートを止める      */
    int         sw_tick    = 0; /* SW1 サンプリング用カウンタ            */
    int         sw_last    = 0; /* 前回サンプル（1 = 押されている）      */
    int         sw_stable  = 0; /* 同じ値が続いた回数                    */
    int         sw_state   = 0; /* 確定した状態（1 = 押されている）      */
    uint16_t    isr_cnt    = 0; /* デバッグ用：制御 ISR の実行回数       */

    /* 過電流フォルト */
    int         fault      = 0; /* 1 = ソフト過電流でラッチ中            */
    int         fault_hw   = 0; /* デバッグ用：ハード Fault のラッチ状態 */

    /* 起動時の RCON（リセット原因）。main の先頭で保存する。
     *   bit0 POR  電源投入      bit1 BOR  電源電圧低下
     *   bit2 IDLE bit3 SLEEP    bit4 WDTO ウォッチドッグ
     *   bit6 EXTR MCLR          bit7 SWR  ソフトリセット
     *   bit9 IOPUWR 不正命令    bit15 TRAPR トラップ
     * 0x0003 なら通常の電源投入。それ以外はリセットの手がかりになる。 */
    uint16_t    rcon_start = 0;
    uint16_t    reset_cnt  = 0; /* リセットのたびに増える（RAM は保持されない
                                 * ので、実際には毎回 0 に戻る点に注意）     */
    int         ocp_cnt    = 0; /* ソフト過電流が連続成立した回数        */

    /* --- 過電流の原因調査用 ------------------------------------------
     *  iin_trip  : ラッチした瞬間の Iin（正負と大きさが分かる）
     *  a3_trip   : 同じ瞬間の ADCBUF3 の生値
     *  ocp_hits  : しきい値を超えた回数の累計（1回きりか連発かが分かる）
     *  iin_min/max : 直近1周期の Iin の振れ幅。平常時の余裕が分かる   */
    int         iin_trip   = 0;
    int         a3_trip    = 0;
    uint16_t    ocp_hits   = 0;
    uint16_t    cmp1_hits  = 0;
    uint16_t    pol_err_hits = 0;   /* 極性判定が怪しかった回数 */
    int         pol_err      = 0;   /* 1 = いま極性がおかしい   */ /* CMP1 が閾値を超えた回数（ノイズの指標） */
    int         iin_min    = 0,  iin_min_run = 32767;
    int         iin_max    = 0,  iin_max_run = -32768;

    /* --- Pause 用の保持値（リセットするまで消えない）------------------
     *  デバッガで Pause すると制御 ISR は止まるが PWM はハードなので
     *  動き続ける。電流ループが効かなくなって過電流になるので、
     *    ① 目的の電圧で安定させる
     *    ② 交流電源の出力を OFF   ← 電流が流れなくなる
     *    ③ Pause して下の *_hold を読む
     *  という手順が取れるように、20ms ごとの値とは別に最大値を保持する。
     *  次の点を測る前に、Pause 中に Watches でこれらを 0 に書き換えること。 */
    int         wout_pk       = 0,  wout_pk_run  = 0;
    int         iref_pk       = 0,  iref_pk_run  = 0;
    int         wout_hold     = 0;
    int         iref_hold     = 0;
    int         vrec_hold     = 0;  /* |vac| ピークの最大値（校正用）   */
    int         vo11_hold     = 0;  /* Vo_sense 平均の最大値（校正用）  */
    int         iin_hold_max  = 0;  /* Iin の最大値                     */
    int         iin_hold_min  = 0;  /* Iin の最小値                     */
    int         sensor_ok  = 0; /* 1 = 電流センサが健全                  */
    int         sens_cnt   = 0; /* 健全判定用カウンタ                    */

    /* --- 校正用モニタ（制御には使わない。デバッガで読むためだけ） ---
     *  vrec_pk  : |vac| のピーク [ADCカウント]。20ms ごとに更新。
     *             VLINE_CNT_PER_V = vrec_pk / (入力電圧のピーク[V])
     *  vo11_avg : Vo_sense を 20ms 平均した値 [ADCカウント]。
     *             開ループ（C2CHS=0）では AN11 は共有 ADC コアが変換して
     *             ADCBUF11 に入るので、モードによらず Vo が読める。
     *             VO_CNT_PER_V = (cnt2-cnt1)/(V2-V1)                    */
    int         vrec_pk     = 0;
    int         vrec_pk_run = 0;
    int         vl_pk       = 0;    /* ADCBUF0 のピーク（VL 側）         */
    int         vl_pk_run   = 0;
    int         vn_pk       = 0;    /* ADCBUF1 のピーク（VN 側）         */
    int         vn_pk_run   = 0;
    int         vo11_avg    = 0;
    int32_t     vo11_sum    = 0;
    int         cal_cnt     = 0;

    /* デバッグ・その他 */
    int         adcresult = 0;
    int         Deadtime  = DEADTIME;
    int         sw = 1;

/*	レジスタ初期化	*/
#define	reg_CLEAR	(0x0000)

/*	=================	*/
/*	メイン関数		    */
/*	=================	*/
//初期設定してるだけだからこの関数の内容はガン無視していい
int main(void)
{
    /* ★リセット原因を最初に保存する（initClock より前） */
    rcon_start = RCON;
    RCON = 0;
    reset_cnt++;

    initClock();            // Initialize Primary and Auxiliary oscillators
//    initPGA(); 
    initCMP();
    ClrWdt();
    initADC();              // Configures ADC for reading input voltage, output voltages,
                            // load currents, and temperature
    ClrWdt();
//    initTimer();
//    initOC(); 
    initIO();
    initCLC(); 
   ClrWdt();
   
    initPWM();                // PWM Setup
    PTCONbits.PTEN = 1;       // Enable the PWM
    //    PTCONbits.SEVTPS = 1       //
    ClrWdt();
    
//    for(CH1_Loop0=0; Loop0<6000; Loop0++  ){
//        ClrWdt();
//   }

	/*	変数初期化	*/
    CMP1DAC = CMP1DAC_OCP;      // 過電流保護のしきい値（ISENS_OFS + I_OCP_A）
    LED2_LAT = 0;               // フォルト表示 LED 消灯

    /* 起動時は安全側の開ループ。ボリュームを絞った状態から始めること。
     * SW1（RE4）を押して離すと閉ループに切り替わる。 */
    mode = MODE_OPEN;
    ADCON4Hbits.C2CHS = 0;      // AN2 : 基板上ボリューム R12
#if (USE_MODE_LED == 1)
    LED1_LAT = 0;               // LED1 消灯 = 開ループ
#endif
    //================================================================
    /*　※デバッグモード使うときは下2行のコメントを外して　*/
    //ADCON4Hbits.C2CHS   = 1;    // 0-AN2, 1-AN11, 2-VREF, 3-Resirve
    //sw = 0;
    //================================================================
    
//  Soft Start
    
    IOCON1bits.PENH  = 1;       // PWM1H is controlled by IO
    IOCON1bits.PENL  = 1;       // PWM1L is controlled by IO
    IOCON2bits.PENH  = 1;       // PWM2H is controlled by PWM module
    IOCON2bits.PENL  = 1;       // PWM2L is controlled by PWM module
    IOCON3bits.PENH  = 1;       // PWM3H is controlled by PWM module
    IOCON3bits.PENL  = 1;       // PWM3L is controlled by PWM module
    IOCON4bits.PENH  = 1;       // PWM4H is controlled by PWM module
    IOCON4bits.PENL  = 1;       // PWM4L is controlled by PWM module
    IOCON5bits.PENH  = 1;       // PWM5H is controlled by IO
    IOCON5bits.PENL  = 1;       // PWM5L is controlled by IO
    IOCON6bits.PENH  = 1;       // PWM6H is controlled by IO
    IOCON6bits.PENL  = 1;       // PWM6L is controlled by IO
    IOCON7bits.PENH  = 1;       // PWM7H is controlled by PWM module
    IOCON7bits.PENL  = 1;       // PWM7L is controlled by PWM module
    /* PWM8 は使わないので GPIO のままにする（PENH/PENL = 0）。
     * 開発ボードでは PWM8L -> Q2A(N-ch, GND側) / PWM8H -> Q2B(P-ch, 3.3V側) が
     * J8（AN0 = VL_sense の入力ノード）に 470R でぶら下がっている。PWM8 を
     * 動かすと VL_sense が周期的に引っ張られる。RE0 = 0 / RE1 = 1 で両方 OFF。 */
    IOCON8bits.PENH  = 0;
    IOCON8bits.PENL  = 0;
    
//    for(Loop0=0 ; Loop0<6000 ; Loop0++ ){  ClrWdt();  }
    
	    
    /* SW1 は制御 ISR 内でポーリングするので CN 割り込みは使わない */
    CNENEbits.CNIEE4 = 0;
    IEC1bits.CNIE = 0;
    IFS1bits.CNIF = 0;

    IFS7bits.ADCAN2IF = 0;		// Clear ADCAN2 interrupt flag.
    IEC7bits.ADCAN2IE = 1;	    // Enable ADC AN2 interrupt      

    IFS6bits.PWM5IF = 0;		// Clear PWM5 interrupt flag.
    IEC6bits.PWM5IE = 1;	    // Enable PWM5 interrupt 
   
    
//    IFS0bits.T2IF     = 0;      // Clear Timer2 interrupt flag
//    IEC0bits.T2IE     = 1;      // Enable Timer2 interrupt
	while(1)
	{
      ClrWdt();
//    LATDbits.LATD6 = CLC4CONLbits.LCOUT;
//    LATDbits.LATD6 = 1;
    }
}

/*　以下2つの関数が実質的にメインの処理になる　*/
//==============================================================================
//モード切り替え（SW1 を押したときに制御 ISR から呼ばれる）
static void toggle_mode(void)
{
    if (mode == MODE_CLOSED) {
        /* ---- 閉ループ -> 開ループ ---- */
        mode = MODE_OPEN;
        ADCON4Hbits.C2CHS = 0;      // AN2 : 基板上ボリューム R12
    } else {
        /* ---- 開ループ -> 閉ループ ---- */
        mode = MODE_CLOSED;
        ADCON4Hbits.C2CHS = 1;      // AN11 : Vo_sense（フォトカプラ, J22 中央）

        /* バンプレス移行
         *   電圧ループの積分器を「今出ている Ipk」に合わせておくと、
         *   切り替えた瞬間に電流指令が飛ばない。
         *   指令 Vref は次に移動平均が出たところで実測値から立ち上げる。 */
        Uv        = (int32_t)Ipk << SH_V;
        Ev_1      = 0;
        Up        = 0;
        Ep_1      = 0;
        vref_init = 1;
        vo_sum    = 0;
        scan_cnt  = 0;
    }

    /* ADC のチャネル切替が落ち着くまで Q1/Q2 を止める */
    mode_blank = MODE_BLANK;

#if (USE_MODE_LED == 1)
    LED1_LAT = mode;        // 閉ループで点灯
#endif
}

//==============================================================================
//RE4 の CN 割り込み（使っていない。念のためフラグだけ落とす）
void __attribute__ ((__interrupt__, no_auto_psv)) _CNInterrupt(void)
{
    IFS1bits.CNIF = 0; // Clear CN interrupt
}

/* ===========================================================================
 *  制御本体
 *  ADC（AN2）変換完了割り込み = スイッチング周期ごと（100kHz）に呼ばれる
 * ===========================================================================
 */
void __attribute__((__interrupt__, no_auto_psv))_ADCAN2Interrupt()
{
    int32_t lim;

    /* ---------------------------------------------------------------
     *  (1) センシング
     *      VL_sense / VN_sense はどちらも片側だけが立つので
     *        差 = 符号付きライン電圧、和 = |vac| 全波整流 になる
     * --------------------------------------------------------------- */
    Vac    = ADCBUF0 - ADCBUF1;
    Vrec   = ADCBUF0 + ADCBUF1;
    Vo_adc = ADCBUF2;
    adcresult = Vo_adc;                 /* デバッグ表示用 */
    Iin    = ADCBUF3 - ISENS_OFS;       /* 双方向センサ。中点を引いて符号付きに */

    /* ---------------------------------------------------------------
     *  (2) 極性判定（ヒステリシス付き。零クロスでのチャタリング防止）
     * --------------------------------------------------------------- */
    if (half) {
        if (Vac < -VAC_HYST) { half = 0; }
    } else {
        if (Vac >  VAC_HYST) { half = 1; }
    }

    /* 整流座標のインダクタ電流（負の半周期は符号反転） */
    if (half) { Ir =  Iin; } else { Ir = -Iin; }

    /* ★極性判定の誤りを検出する安全ネット★
     *  昇圧形なのでダイオードが逆流を阻止する。したがって整流座標の電流
     *  Ir が大きく負になることは物理的にありえない。もしそうなっていたら
     *  half（極性判定）が実際の半周期と合っていない。
     *  このまま回すと Ei = Iref - Ir が正に増え続けて正帰還になり、
     *  デューティが上限に張り付いて電流が発散する。
     *  そうなる前に積分器をクリアしてゲートを止める。 */
    if (Ir < -SENS_OK_TOL) {
        pol_err = 1;
        pol_err_hits++;
        Ui   = 0;
        Ei_1 = 0;
    } else if (Ir > 0) {
        pol_err = 0;
    }

    /* ---------------------------------------------------------------
     *  ソフト過電流保護（両極性）
     *      CMP1 は非反転なので負の半周期を守れない。ここで両方向を見る。
     *      一度ラッチしたら SW1 を押すまで全ゲートを止めたままにする。
     * --------------------------------------------------------------- */
    /* ハード側の状態は診断用に見るだけ。ここで fault は立てない。
     * PWM1/2 はサイクルバイサイクルなので FLTSTAT は「いま制限中」の意味。 */
    fault_hw = PWMCON1bits.FLTSTAT | PWMCON2bits.FLTSTAT;

    /* センサの健全性（未給電を過電流と誤認しないため）
     *  「2000回連続で範囲内」だと、20ms の間に単発ノイズが 1 回でも入ると
     *  振り出しに戻って永久に成立しない。アップダウンカウンタにして
     *  たまのノイズでは少し戻るだけにする。 */
    if (sensor_ok == 0) {
        if ((Iin > -SENS_OK_TOL) && (Iin < SENS_OK_TOL)) {
            if (++sens_cnt >= N_VLOOP) { sensor_ok = 1; }
        } else {
            sens_cnt -= 8;                      /* 単発ノイズでは全消ししない */
            if (sens_cnt < 0) { sens_cnt = 0; }
        }
    } else {
        if (Iin < (SENS_FLOOR - ISENS_OFS)) { sensor_ok = 0; sens_cnt = 0; }
    }

    /* 過電流保護（センサが健全なときだけ判定する）
     *  単発のノイズでラッチしないよう、2 回連続（20us）で成立したときだけ
     *  ラッチする。20us の間に進む電流は 141V/1mH で 2.8A なので許容範囲。
     *  本当に速い事象はハードの CMP1 が連続監視で捕まえる。 */
    if (sensor_ok && ((Iin > I_OCP_CNT) || (Iin < -I_OCP_CNT))) {
        ocp_hits++;
        if (++ocp_cnt >= 2) {
            if (fault == 0) {           /* 最初にラッチした瞬間だけ記録 */
                iin_trip = Iin;
                a3_trip  = ADCBUF3;
            }
#if (OCP_SOFT_EN == 1)
            fault = 1;
#endif
        }
    } else {
        ocp_cnt = 0;
    }

    if (fault) {
        /* ラッチ。SW1 を押すまで全ゲートを止めたままにする */
        PDC1 = 0;
        PDC2 = 0;
        IOCON6bits.OVRDAT = 0b00;   IOCON7bits.OVRDAT = 0b00;
        IOCON6bits.OVRENH = 1;      IOCON6bits.OVRENL = 1;
        IOCON7bits.OVRENH = 1;      IOCON7bits.OVRENL = 1;
        Ui = 0; Ei_1 = 0;
        Uv = 0; Ev_1 = 0;
        Ipk = 0;
        /* SW1 のポーリングは続ける必要があるのでここでは return しない */
    }

    /* 赤 LED : 過電流ラッチ中、またはセンサが未確立のとき点灯 */
    LED2_LAT = (fault || (sensor_ok == 0));

    /* ---------------------------------------------------------------
     *  (3) 動作モード
     *      開ループ : ADCBUF2 = AN11（ボリューム）-> 電流振幅指令を直接与える
     *      閉ループ : ADCBUF2 = AN2（Vo_sense）  -> 電圧ループが指令を作る
     *      切り替えは SW1（RE4）を押して離す。
     * --------------------------------------------------------------- */
    isr_cnt++;                          /* デバッグ用：回っていれば増える */

    /* ---------------------------------------------------------------
     *  校正用モニタ（制御には影響しない）
     *      商用1周期ごとに |vac| のピークと Vo_sense の平均を確定させる。
     *      Pause したときにランダムな点を掴まないようにするため。
     * --------------------------------------------------------------- */
    if (Wout > wout_pk_run) { wout_pk_run = Wout; }
    if (Iref > iref_pk_run) { iref_pk_run = Iref; }
    if (Iin < iin_min_run) { iin_min_run = Iin; }
    if (Iin > iin_max_run) { iin_max_run = Iin; }
    if (Vrec    > vrec_pk_run) { vrec_pk_run = Vrec;    }
    if (ADCBUF0 > vl_pk_run  ) { vl_pk_run   = ADCBUF0; }
    if (ADCBUF1 > vn_pk_run  ) { vn_pk_run   = ADCBUF1; }
    vo11_sum += ADCBUF11;
    if (++cal_cnt >= N_VLOOP) {
        cal_cnt     = 0;
        vrec_pk     = vrec_pk_run;   vrec_pk_run = 0;
        IFS6bits.PWM5IF = 0;         /* CMP1 通知割り込みを戻す（50Hz） */
        IEC6bits.PWM5IE = 1;
        wout_pk     = wout_pk_run;   wout_pk_run = 0;
        iref_pk     = iref_pk_run;   iref_pk_run = 0;
        if (wout_pk > wout_hold) { wout_hold = wout_pk; }
        if (iref_pk > iref_hold) { iref_hold = iref_pk; }
        iin_min     = iin_min_run;   iin_min_run = 32767;
        iin_max     = iin_max_run;   iin_max_run = -32768;

        /* Pause 用の保持値を更新（下がらない） */
        if (vrec_pk  > vrec_hold   ) { vrec_hold    = vrec_pk;  }
        if (vo11_avg > vo11_hold   ) { vo11_hold    = vo11_avg; }
        if (iin_max  > iin_hold_max) { iin_hold_max = iin_max;  }
        if (iin_min  < iin_hold_min) { iin_hold_min = iin_min;  }
        vl_pk       = vl_pk_run;     vl_pk_run   = 0;
        vn_pk       = vn_pk_run;     vn_pk_run   = 0;
        vo11_avg    = (int)(vo11_sum / N_VLOOP);
        vo11_sum    = 0;
    }

    /* SW1 のポーリング（20ms ごと、2 回連続一致で確定。押した瞬間にトグル） */
    if (++sw_tick >= SW_SAMPLE) {
        int sw_now;
        sw_tick = 0;
#if (SW1_ACTIVE_LOW == 1)
        sw_now = (PORTEbits.RE4 == 0);
#else
        sw_now = (PORTEbits.RE4 != 0);
#endif
        if (sw_now == sw_last) { if (sw_stable < 2) { sw_stable++; } }
        else                   { sw_stable = 0; }
        sw_last = sw_now;

        if ((sw_stable >= 1) && (sw_now != sw_state)) {
            sw_state = sw_now;
            if (sw_state) {
                if (fault) {
                    /* フォルト中の押下はリセット（モードは変えない）。
                     * 原因を取り除いてから押すこと。 */
                    fault   = 0;
                    ocp_cnt = 0;
                    IOCON6bits.OVRENH = 0;  IOCON6bits.OVRENL = 0;
                    IOCON7bits.OVRENH = 0;  IOCON7bits.OVRENL = 0;
                    PWMCON1bits.FLTSTAT = 0;    /* ハード Fault のラッチも解除 */
                    PWMCON2bits.FLTSTAT = 0;
                    PWMCON5bits.FLTSTAT = 0;
                    PWMCON6bits.FLTSTAT = 0;
                    PWMCON7bits.FLTSTAT = 0;
                    IFS6bits.PWM5IF = 0;        /* フォルト通知割り込みを戻す  */
                    IEC6bits.PWM5IE = 1;
                    LED2_LAT   = 0;
                    mode_blank = MODE_BLANK;
                } else {
                    toggle_mode();              /* 押下確定でトグル */
                }
            }
        }
    }

    if (mode == MODE_OPEN) {
        /* ボリューム 0..4095 -> 電流振幅指令 0..IPK_OPEN_MAX
         * 電流ループとフィードフォワードは動いたままなので、
         * 入力電流は正弦波の形のまま振幅だけが変わる。 */
        Ipk = ((int32_t)Vo_adc * IPK_OPEN_MAX) >> 12;
        vo_sum   = 0;
        scan_cnt = 0;
        Vo_avg   = 0;
    }
    else {
    /* ---------------------------------------------------------------
     *  (4) 出力電圧の移動平均（商用1周期ちょうど = 20ms）
     *      窓幅が線周期の整数倍なので 50Hz / 100Hz リプルが完全に消える。
     *      その平均値で PFC電圧ループ（50Hz）を回し、電流振幅指令 Ipk を作る。
     *      修論 4.2：出力電圧を検出 -> 基準と比較 -> コントローラ
     * --------------------------------------------------------------- */
    vo_sum += Vo_adc;
    scan_cnt++;
    if (scan_cnt >= N_VLOOP) {
        scan_cnt = 0;
        Vo_avg = (int)(vo_sum / N_VLOOP);
        vo_sum = 0;

        /* 閉ループに入った直後は、そのときの出力電圧から指令を立ち上げる
         * （いきなり 48V を指令すると電流が飛ぶ） */
        if (vref_init) { Vref = Vo_avg; vref_init = 0; }

        /* ソフトスタート：指令を 300V/s 相当でランプアップ */
        if (Vref < VO_REF_CNT) {
            Vref += (int)(300.0/F_LINE_HZ*VO_CNT_PER_V);
            if (Vref > VO_REF_CNT) { Vref = VO_REF_CNT; }
        }

        Ev = (int32_t)Vref - (int32_t)Vo_avg;
        Uv = Uv + KV_A*Ev - KV_B*Ev_1;
        Ev_1 = Ev;

        /* アンチワインドアップ */
        lim = (int32_t)IPK_MAX_MUL << SH_V;
        if (Uv > lim) { Uv = lim; }
        if (Uv < 0)   { Uv = 0;   }
        Ipk = Uv >> SH_V;

        /* ---------------------------------------------------------
         *  位相シフト量の決定
         *   PH_MODE = 0 : D1_FIX 固定（推奨。まずこれで回す）
         *     Cs1/Cs2 は 100Hz の脈動エネルギーを蓄えないので、
         *     D1 を固定しても Vcs が自然に変動して電力を通す。
         *   PH_MODE = 1 : 出力電圧で D1_FIX の周りを低速トリム
         * --------------------------------------------------------- */
#if (PH_MODE == 0)
        wPhase = D1_FIX_CNT;
#else
        Ep = (int32_t)Vref - (int32_t)Vo_avg;
        Up = Up + KV_A*Ep - KV_B*Ep_1;      /* 電圧ループと同じ特性でよい */
        Ep_1 = Ep;
        lim = (int32_t)D1_TRIM_CNT << SH_V;
        if (Up >  lim) { Up =  lim; }
        if (Up < -lim) { Up = -lim; }
        wPhase = D1_FIX_CNT + (int)(Up >> SH_V);
#endif
        if (wPhase > PHASEmax) { wPhase = PHASEmax; }
        if (wPhase < PHASEmin) { wPhase = PHASEmin; }
    }
    }   /* end of mode == MODE_CLOSED */

    /* ---------------------------------------------------------------
     *  (5) 電流指令（乗算器）
     *      修論 4.2：制御した値に入力電圧の絶対値を掛けて基準電流を作る
     * --------------------------------------------------------------- */
    Iref = (int)(((int32_t)Vrec * Ipk) >> SH_MUL);

    /* ---------------------------------------------------------------
     *  (6) 電流ループ（100kHz）= 平均電流制御
     * --------------------------------------------------------------- */
    Ei = (int32_t)Iref - (int32_t)Ir;
    Ui = Ui + KI_A*Ei - KI_B*Ei_1;
    Ei_1 = Ei;

    /* アンチワインドアップ
     *  USE_FF=1 : フィードフォワードが大半を出すので PI は +-60% の補正でよい。
     *  USE_FF=0 : PI 出力がそのままデューティになるので、0〜DUTYmax の
     *             全域を使えないと必要な昇圧比に届かない。
     *             （20V入力で Vo=48V を出すにはデューティ 85% が要る） */
#if (USE_FF == 1)
    /* ★XC16 の int は 16bit。PERIODmax*6 = 57600 は int16 の範囲(32767)を
     * 超えるので、キャストを括弧の外に置くと定数式の段階で桁あふれする。
     * 先に int32_t へ上げてから掛けること。 */
    lim = ((int32_t)PERIODmax*6/10) << SH_I;
    if (Ui >  lim) { Ui =  lim; }
    if (Ui < -lim) { Ui = -lim; }
#else
    lim = (int32_t)DUTYmax << SH_I;
    if (Ui >  lim) { Ui = lim; }
    if (Ui <  0  ) { Ui = 0;   }
#endif
    Dpi = (int)(Ui >> SH_I);

    /* ---------------------------------------------------------------
     *  (7) フィードフォワード（昇圧則）
     *      Vcs = |vac|/(1-D)  ->  D = 1 - |vac|/Vcs
     *      定常解を先に与えるので PI の負担が減り、電圧ピーク付近の
     *      追従遅れによる歪みが小さくなる。
     * --------------------------------------------------------------- */
#if (USE_FF == 1)
    Dff = PERIODmax - (int)(((int32_t)Vrec * KFF) >> SH_FF);
    if (Dff < 0) { Dff = 0; }
#else
    Dff = 0;
#endif

    Wout = Dff + Dpi;
    if (Wout > DUTYmax) { Wout = DUTYmax; }
    if (Wout < DUTYmin) { Wout = DUTYmin; }

    /* モード切替直後は Q1/Q2 を止める。
     * ADC のチャネルが切り替わった直後の値で電流を流さないため。
     * フルブリッジ（PWM6/7）は動かしたままにする。 */
    if (mode_blank > 0) {
        mode_blank--;
        Wout = 0;
        Ui   = 0;
        Ei_1 = 0;
    }

    /* ---------------------------------------------------------------
     *  (8) ゲート配分
     *      正の半周期 : Q1 が PWM、Q2 はオフ
     *                   レグA(PWM6) が基準、レグB(PWM7) を位相遅れ
     *      負の半周期 : Q2 が PWM、Q1 はオフ
     *                   レグB(PWM7) が基準、レグA(PWM6) を位相遅れ
     *      零クロス近傍は Q1/Q2 を止める（極性が確定しないため）
     * --------------------------------------------------------------- */
    if (fault) {
        PDC1 = 0;
        PDC2 = 0;
    }
    else if (Vac >= VAC_HYST) {
        PDC1   = Wout;
        PDC2   = 0;
        PHASE6 = 0;
        PHASE7 = wPhase;
    }
    else if (Vac <= -VAC_HYST) {
        PDC1   = 0;
        PDC2   = Wout;
        PHASE7 = 0;
        PHASE6 = wPhase;
    }
    else {
        /* 零クロス近傍：PFC は停止、フルブリッジは直前の割り当てを維持 */
        PDC1 = 0;
        PDC2 = 0;
        if (half) { PHASE6 = 0;                     PHASE7 = wPhase; }
        else      { PHASE6 = wPhase;   PHASE7 = 0;                   }
    }

    /* ゲートを止めている間は電流ループの積分器も止める。
     * そうしないと「指令は出ているのに電流が返ってこない」状態で
     * 積分器が上限まで巻き上がり、ゲートが復活した瞬間にデューティ 95% から
     * 動き出して大きな電流サージになる。 */
    if (sensor_ok == 0) { Ui = 0; Ei_1 = 0; }

    /* 電流センサが健全でない間は PFC を回さない。
     * フルブリッジ（PWM6/7）はそのまま動かすので、センサに電源を入れずに
     * ゲート波形の確認ができる。 */
    if (sensor_ok == 0) { PDC1 = 0; PDC2 = 0; }

    /* 極性がおかしい間は Q1/Q2 を止める（フルブリッジはそのまま） */
    if (pol_err) { PDC1 = 0; PDC2 = 0; }

    IFS7bits.ADCAN2IF = 0;
}

//==============================================================================

void __attribute__((__interrupt__, no_auto_psv))_ADCAN0Interrupt()
//void __attribute__((__interrupt__, no_auto_psv, shadow))_ADCAN0Interrupt()
{
//    LATEbits.LATE7  = 1;  // Debug  ++++++++++
    (Nop());
    (Nop());
    (Nop());  
    (Nop());
    (Nop());
    (Nop());  
    (Nop());
    (Nop());
    (Nop());   
    IFS6bits.ADCAN0IF = 0;		// Clear ADCAN0 interrupt flag.
}

void __attribute__((__interrupt__, no_auto_psv))_ADCAN1Interrupt()
//void __attribute__((__interrupt__, no_auto_psv, shadow))_ADCAN1Interrupt()
{
//    LATEbits.LATE7  = 1;  // Debug  ++++++++++
    (Nop());
    (Nop());
    (Nop());  
    (Nop());
    (Nop());
    (Nop());  
    (Nop());
    (Nop());
    (Nop());   
    IFS6bits.ADCAN1IF = 0;		// Clear ADCAN1 interrupt flag.
}

void __attribute__((__interrupt__, no_auto_psv))_ADCAN3Interrupt()
{
//    LATEbits.LATE8  = 1;  // Debug  ++++++++++
    (Nop());
    (Nop());
    (Nop());  
    (Nop());
    (Nop());
    (Nop());  
    (Nop());
    (Nop());
    (Nop());   
    IFS7bits.ADCAN3IF = 0;		// Clear ADCAN3 interrupt flag.
}

/* CMP1（過電流）のフォルト通知。FCLCON5.FLTSRC=13, PWMCON5.FLTIEN=1 経由。
 *
 * ★2026-09-28 修正
 *   元のコード（closed1.c 由来）はここで while(1) して止まっていた。
 *   その結果、CMP1 が一度でもトリップすると
 *     ・制御 ISR が二度と回らない
 *     ・PWM は全部ラッチ停止したまま（フルブリッジの信号が消える）
 *     ・割り込みフラグが未クリアのまま固まるのでデバッガが繋がらず
 *       次回 Debug 時に "Fatal error: 0xc04" になる
 *     ・電源を抜き差しするまで復帰しない
 *   という状態になっていた。フラグを立てて即座に抜けるようにする。 */
void __attribute__((__interrupt__, no_auto_psv))_PWM5Interrupt()
{
    /* CMP1 が閾値を超えたという「通知」だけ。ここでは何も止めない。
     * ・Q1/Q2 はハードがサイクルバイサイクルで既に絞っている
     * ・本物の過電流は制御 ISR のソフト側がラッチする
     * 連打を避けるため割り込みを止め、制御 ISR が商用1周期ごとに戻す。 */
    cmp1_hits++;
    PWMCON5bits.FLTSTAT = 0;
    IEC6bits.PWM5IE = 0;
    IFS6bits.PWM5IF = 0;
}


void __attribute__((__interrupt__, no_auto_psv))_T2Interrupt()
{
//        LATEbits.LATE7 = 1-  LATEbits.LATE7; // Debug  ++++++++++ 
    (Nop());
    (Nop());
    (Nop());
    (Nop());
    (Nop());
    (Nop());  
    (Nop());
    (Nop());
    (Nop());  
    IFS0bits.T2IF = 0;
}

void initClock(void)
{
   // Configure Oscillator to operate the device at 60 MHz
   // Fosc = Fin*M/(N1*N2), Fcy = Fosc/2
   // Fosc = 7.37*(65)/(2*2)= ~119.7Mhz for Fosc, Fcy = 60 MHz 

   // Configure PLL prescaler, PLL postscaler, PLL divisor
   PLLFBD             = 63;   // M = PLLFBD + 2
   CLKDIVbits.PLLPOST =  0;   // N1 = 2
   CLKDIVbits.PLLPRE  =  0;   // N2 = 2

   __builtin_write_OSCCONH(0x01);     // New Oscillator selection FRC w/ PLL
   __builtin_write_OSCCONL(0x01);     // Enable Switch

   while(OSCCONbits.COSC != 0b001);    // Wait for Osc. to switch to FRC w/ PLL
   while(OSCCONbits.LOCK != 1);        // Wait for PLL to Lock

   // Setup the ADC and PWM clock for 120MHz
   // ((FRC * 16) / APSTSCLR ) = (7.37MHz * 16) / 1 = 117.9MHz

   ACLKCONbits.FRCSEL   = 1;	  // FRC provides input for Auxiliary PLL (x16)
   ACLKCONbits.SELACLK  = 1;	  // Aux Osc. provides clock source for PWM & ADC
   ACLKCONbits.APSTSCLR = 7;	  // Divide Auxiliary clock by 1
   ACLKCONbits.ENAPLL   = 1;	  // Enable Auxiliary PLL

   while(ACLKCONbits.APLLCK != 1);  // Wait for Auxiliary PLL to Lock
}

void initIO(void)
{
    TRISA  = 0xFFFF;
    LATA   = 0x0000;
    ODCA   = 0x0000;
    CNENA  = 0x0000;
    CNPUA  = 0x0000;
    CNPDA  = 0x0000;
    ANSELA = 0x0007;

    TRISB  = 0xFFEF;
    LATB   = 0x0000;
    ODCB   = 0x0000;
    CNENB  = 0x0000;
    CNPUB  = 0x0000;
    CNPDB  = 0x0000;
    ANSELB = 0x0001;

    TRISC  = 0xBFFF;
    LATC   = 0x0000;
    ODCC   = 0x0000;
    CNENC  = 0x0000;
    CNPUC  = 0x0000;
    CNPDC  = 0x0000;
    ANSELC = 0x0200;
    
    TRISD  = 0xFFFF;
    LATD   = 0x0000;
    ODCD   = 0x0000;
    CNEND  = 0x0000;
    CNPUD  = 0x0000;
    CNPDD  = 0x0000;
    ANSELD = 0x0000;

    TRISE  = 0xC010;
    LATE   = 0x002A;    /* RE0(PWM8L)=0: Q2A OFF, RE1(PWM8H)=1: Q2B OFF (開発ボードの負荷FET) */
    ODCE   = 0x0028;
    CNENE  = 0x0000;
    CNPUE  = 0x0000;
    CNPDE  = 0x0000;
    ANSELE = 0x0000;


//***************************
// Assign CLCINA To Pin RP67
    RPINR45bits.CLCINAR   = 67; // CLCINA
//***************************
    // Assign CLC1 To Pin RP36
    RPOR4bits.RP36R   = 0x3F; // CLC1(011 1111)
    // Assign CLC1 To Pin RP33
    RPOR3bits.RP33R   = 0x3F; // CLC1(011 1111)
    
// Assign CLC2 To Pin RP76
    RPOR23bits.RP76R   = 0x40; // CLC2(100 0000)
// Assign CLC2 To Pin RP34
    RPOR3bits.RP34R   = 0x40; // CLC2(100 0000)
    
// Assign CLC3 To Pin R** RD14
//    RPOR23bits.RP**R   = 0x41;  // CLC3(100 0001)

// Assign CLC4 To Pin RP74 RD10
//    RPOR22bits.RP74R   = 0x42;  // CLC4(100 0010)
//***************************
// Assign PWM4L To Pin RP62
//    RPOR16bits.RP62R   = 0x34; // PWM4L(011 0100)
//***************************

}

void initPWM(void)
{
// Setup PWM resolution and Period
    
    // (9600*1)/(119.7MHz*8) = 100kHz
    PTCON2bits.PCLKDIV = 0; // PWM Resolution(x1)  ++++++++++++ SWf = 100KHz       周波数変更 ここを修正
    PTCONbits.EIPU = 0;
    PTPER = PERIODmax;    // Switching Frequency

    
    IOCON1bits.PENH = 0;            // GPIO controls I/O port
    IOCON1bits.PENL = 0;           

    IOCON1bits.PMOD = 0;            // Complementary Output mode
    
    IOCON1bits.POLH = 0;            // Drive signals are active-high
    IOCON1bits.POLL = 0;            // Drive signals are active-high

    IOCON1bits.OVRENH = 0;          // Override disabled
    IOCON1bits.OVRENL = 0;	  
    IOCON1bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON1bits.FLTDAT = 0b00;       // If fault occurs:
                                    // PWMH = 0 & PWML = 1
    IOCON1bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON1bits.DTC   = 0;          // Positive Deadtime enabled
    DTR1    = 0;
    ALTDTR1 = 0;
				  
    PWMCON1bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON1bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON1bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON1bits.IFLTMOD = 0;        //
    FCLCON1bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON1bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON1bits.CLMOD = 0;          // Current-Limit Disenable
    
    FCLCON1bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON1bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON1bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON1bits.FLTMOD = 1;         // Fault cycle by cycle
    /* ★Q1 は「サイクルバイサイクル制限」にする。
     * ラッチにすると ILsense に乗る細いスパイク 1 発で装置全体が止まる。
     * この設定なら該当する 1 周期だけゲートが落ちて次から復帰する。
     * 本物の過電流は ADC でサンプリングするソフト側がラッチする。 */
    FCLCON1bits.FLTMOD = 1;         // Fault (cycle-by-cycle)
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON1bits.PHF      = 0;       // 
    LEBCON1bits.PLF      = 0;       // 
    LEBCON1bits.PHR      = 0;       // 
    LEBCON1bits.PLR      = 0;
    LEBCON1bits.FLTLEBEN = 0;
    LEBCON1bits.CLLEBEN  = 0;
    LEBDLY1              = 0 ;

    TRGCON1bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON1bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration

    TRIG1   = 0;                      // Set Initial Trigger location
    PHASE1  = 0;
    SPHASE1 = 0;
    PDC1    = 0;
    SDC1    = 0;

    
    IOCON2bits.PENH = 0;            // GPIO controls I/O port
    IOCON2bits.PENL = 0;           

    IOCON2bits.PMOD = 0;            // Complementary Output mode
    
    IOCON2bits.POLH = 0;            // Drive signals are active-high
    IOCON2bits.POLL = 0;            // Drive signals are active-high

    IOCON2bits.OVRENH = 0;          // Override disabled
    IOCON2bits.OVRENL = 0;	  
    IOCON2bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON2bits.FLTDAT = 0b00;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON2bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON2bits.DTC   = 0;          // Positive Deadtime enabled
    DTR2    = 0;
    ALTDTR2 = 0;
    
    PWMCON2bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON2bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON2bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON2bits.IFLTMOD = 0;        //
    FCLCON2bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON2bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON2bits.CLMOD = 0;          // Current-Limit Disenable

    FCLCON2bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON2bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON2bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON2bits.FLTMOD = 1;         // Fault cycle by cycle
    /* ★Q2 は「サイクルバイサイクル制限」にする。
     * ラッチにすると ILsense に乗る細いスパイク 1 発で装置全体が止まる。
     * この設定なら該当する 1 周期だけゲートが落ちて次から復帰する。
     * 本物の過電流は ADC でサンプリングするソフト側がラッチする。 */
    FCLCON2bits.FLTMOD = 1;         // Fault (cycle-by-cycle)
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON2bits.PHF      = 0;       // 
    LEBCON2bits.PLF      = 0;       // 
    LEBCON2bits.PHR      = 0;       // 
    LEBCON2bits.PLR      = 0;
    LEBCON2bits.FLTLEBEN = 0;       // 
    LEBCON2bits.CLLEBEN  = 0;       // 
    LEBDLY2              = 0;

    TRGCON2bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON2bits.TRGSTRT = 0;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration

    TRIG2   = 0;                      // Set Initial Trigger location
    PHASE2  = 0;
    SPHASE2 = 0;
    PDC2    = 0;
    SDC2    = 0;

    IOCON3bits.PENH = 0;            // GPIO controls I/O port
    IOCON3bits.PENL = 0;           

    IOCON3bits.PMOD = 3;            // Independent Output mode
    
    IOCON3bits.POLH = 0;            // Drive signals are active-high
    IOCON3bits.POLL = 0;            // Drive signals are active-high

    IOCON3bits.OVRENH = 0;          // Override disabled
    IOCON3bits.OVRENL = 0;	  
    IOCON3bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON3bits.FLTDAT = 0b11;       // If fault occurs:
                                    // PWMH = 0 & PWML = 1
    IOCON3bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON3bits.DTC   = 0;          // Positive Deadtime enabled
    DTR3    = 0;
    ALTDTR3 = 0;
				  
    PWMCON3bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON3bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON3bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON3bits.IFLTMOD = 0;        //
    FCLCON3bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON3bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON3bits.CLMOD = 0;          // Current-Limit Disenable

    FCLCON3bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON3bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON3bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON3bits.FLTMOD = 1;         // Fault cycle by cycle
    FCLCON3bits.FLTMOD = 0;         // Fault (latched condition)
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON3bits.PHF      = 0;
    LEBCON3bits.PLF      = 0;
    LEBCON3bits.PHR      = 0;
    LEBCON3bits.PLR      = 0;
    LEBCON3bits.FLTLEBEN = 0;
    LEBCON3bits.CLLEBEN  = 0;
    LEBDLY3              = 0 ;

    TRGCON3bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON3bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration

    TRIG3   = 0;                      // Set Initial Trigger location
    PDC3    = 0;
    SDC3    = 0;
    PHASE3  = 0;
    SPHASE3 = 0;
    
    IOCON4bits.PENH = 0;            // GPIO controls I/O port
    IOCON4bits.PENL = 0;           

    IOCON4bits.PMOD = 0;            // Complementary Output mode
    
    IOCON4bits.POLH = 0;            // Drive signals are active-high
    IOCON4bits.POLL = 0;            // Drive signals are active-high

    IOCON4bits.OVRENH = 0;          // Override disabled
    IOCON4bits.OVRENL = 0;	  
    IOCON4bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON4bits.FLTDAT = 0b00;       // If fault occurs:
                                    // PWMH = 0 & PWML = 1
    IOCON4bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON4bits.DTC   = 0;          // Positive Deadtime enabled
    DTR4    = 0;
    ALTDTR4 = 0;
				  
    PWMCON4bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON4bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON4bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON4bits.IFLTMOD = 0;        //
    FCLCON4bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON4bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON4bits.CLMOD = 0;          // Current-Limit Disenable

    FCLCON4bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON4bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON4bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON4bits.FLTMOD = 1;         // Fault cycle by cycle
    FCLCON4bits.FLTMOD = 0;         // Fault (latched condition)
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON4bits.PHF      = 0;
    LEBCON4bits.PLF      = 0;
    LEBCON4bits.PHR      = 0;
    LEBCON4bits.PLR      = 0;
    LEBCON4bits.FLTLEBEN = 0;
    LEBCON4bits.CLLEBEN  = 0;
    LEBDLY4              = 0 ; 

    TRGCON4bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON4bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration

    /* ★ADC のサンプリング位置。ここで全チャネルが同時サンプリングされる。
     * 0 にすると「PWM6H ターンオン／PWM1H ターンオン／周期ロールオーバー」が
     * 重なる瞬間になり、リンギングを電流値として拾ってしまう。
     * エッジの位置は 0 / PHASE7(3718) / PDC6(4800) / 8518 / Wout なので、
     * それらから離れた 2400（周期の25%）でサンプリングする。
     * 平均電流制御としてもオン期間の中ほどに当たるので都合がよい。 */
    TRIG4   = 2400;                   // Set Initial Trigger location
    PHASE4  = 0;
    SPHASE4 = 0;
    PDC4    = 0;
    SDC4    = 0;

    
    IOCON5bits.PENH = 0;            // GPIO controls I/O port
    IOCON5bits.PENL = 0;           

    IOCON5bits.PMOD = 0;            // Complementary Output mode
    
    IOCON5bits.POLH = 0;            // Drive signals are active-high
    IOCON5bits.POLL = 0;            // Drive signals are active-high

    IOCON5bits.OVRENH = 0;          // Override disabled
    IOCON5bits.OVRENL = 0;	  
    IOCON5bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON5bits.FLTDAT = 0b00;       // If fault occurs:
                                    // PWMH = 0 & PWML = 1
    IOCON5bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON5bits.DTC   = 0;          // Positive Deadtime enabled
    DTR5    = Deadtime;
    ALTDTR5 = Deadtime;
				  
    PWMCON5bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON5bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON5bits.ITB = 0;            // Select Primary Timebase mode
    PWMCON5bits.FLTIEN = 1;            // Fault interrupt is enabled
    
    FCLCON5bits.IFLTMOD = 0;        //
    FCLCON5bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON5bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON5bits.CLMOD = 0;          // Current-Limit Disenable

    FCLCON5bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON5bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON5bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON5bits.FLTMOD = 1;         // Fault cycle by cycle
    FCLCON5bits.FLTMOD = 0;         // Fault (latched condition)
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON5bits.PHF      = 0;
    LEBCON5bits.PLF      = 0;
    LEBCON5bits.PHR      = 0;
    LEBCON5bits.PLR      = 0;
    LEBCON5bits.FLTLEBEN = 0;
    LEBCON5bits.CLLEBEN  = 0;
    LEBDLY5              = 0 ; 

    TRGCON5bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON5bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration

    TRIG5   = 0;                      // Set Initial Trigger location
    PHASE5  = 0;
    SPHASE5 = 0;
    PDC5    = 4800;
    SDC5    = 0;

    
    IOCON6bits.PENH = 0;            // GPIO controls I/O port
    IOCON6bits.PENL = 0;           

    IOCON6bits.PMOD = 0;            // Complementary Output mode
    
    IOCON6bits.POLH = 0;            // Drive signals are active-high
    IOCON6bits.POLL = 0;            // Drive signals are active-high

    IOCON6bits.OVRENH = 0;          // Override disabled
    IOCON6bits.OVRENL = 0;	  
    IOCON6bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON6bits.FLTDAT = 0b00;       // If fault occurs:
                                    // PWMH = 0 & PWML = 1
    IOCON6bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON6bits.DTC   = 0;          // Positive Deadtime enabled
    DTR6    = Deadtime;
    ALTDTR6 = Deadtime;
    
    PWMCON6bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON6bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON6bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON6bits.IFLTMOD = 0;        //
    FCLCON6bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON6bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON6bits.CLMOD = 0;          // Current-Limit Disenable

    FCLCON6bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON6bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON6bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON6bits.FLTMOD = 1;         // Fault cycle by cycle
    /* ★フルブリッジは CMP1 で止めない。
     * ILsense は「入力電流」なので、入力側の過電流でブリッジまで止めると
     * Cs に溜まったエネルギーの逃げ場が無くなって過電圧になる。
     * 正しい動作は「Q1/Q2 を絞って入力を止め、ブリッジは回して Cs を抜く」。
     * 本当に全停止すべき事象はソフト側が OVREN で止める。 */
    FCLCON6bits.FLTMOD = 3;         // Fault disabled

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON6bits.PHF      = 0;       // 
    LEBCON6bits.PLF      = 0;       // 
    LEBCON6bits.PHR      = 0;       // 
    LEBCON6bits.PLR      = 0;
    LEBCON6bits.FLTLEBEN = 0;
    LEBCON6bits.CLLEBEN  = 0;
    LEBDLY6              = 0 ;

    TRGCON6bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON6bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration

    TRIG6   = 0;                      // Set Initial Trigger location
    PHASE6  = 0;
    SPHASE6 = 0;
    PDC6    = 4800;
    SDC6    = 0;

    
    IOCON7bits.PENH = 0;            // GPIO controls I/O port
    IOCON7bits.PENL = 0;           

    IOCON7bits.PMOD = 0;            // Independent Output mode
    
    IOCON7bits.POLH = 0;            // Drive signals are active-high
    IOCON7bits.POLL = 0;            // Drive signals are active-high

    IOCON7bits.OVRENH = 0;          // Override disabled
    IOCON7bits.OVRENL = 0;	  
    IOCON7bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON7bits.FLTDAT = 0b00;       // If fault occurs:
                                    // PWMH = 0 & PWML = 1
    IOCON7bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON7bits.DTC   = 0;          // Positive Deadtime enabled
    DTR7    = Deadtime;
    ALTDTR7 = Deadtime;
				  
    PWMCON7bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON7bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON7bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON7bits.IFLTMOD = 0;        //
    FCLCON7bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON7bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON7bits.CLMOD = 0;          // Current-Limit Disenable

    FCLCON7bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON7bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON7bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON7bits.FLTMOD = 1;         // Fault cycle by cycle
    /* ★フルブリッジは CMP1 で止めない。
     * ILsense は「入力電流」なので、入力側の過電流でブリッジまで止めると
     * Cs に溜まったエネルギーの逃げ場が無くなって過電圧になる。
     * 正しい動作は「Q1/Q2 を絞って入力を止め、ブリッジは回して Cs を抜く」。
     * 本当に全停止すべき事象はソフト側が OVREN で止める。 */
    FCLCON7bits.FLTMOD = 3;         // Fault disabled
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON7bits.PHF      = 0;       // 
    LEBCON7bits.PLF      = 0;       // 
    LEBCON7bits.PHR      = 0;       // 
    LEBCON7bits.PLR      = 0;
    LEBCON7bits.FLTLEBEN = 0;
    LEBCON7bits.CLLEBEN  = 0;
    LEBDLY7              = 0 ;
    
    TRGCON7bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON7bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration
    TRIG7   = 0;                      // Set Initial Trigger location
    PDC7    = 4800;
    SDC7    = 0;
    PHASE7  = 0;
    SPHASE7 = 0;

    
    IOCON8bits.PENH = 0;            // GPIO controls I/O port
    IOCON8bits.PENL = 0;           

    IOCON8bits.PMOD = 0;            // Complementary Output mode
    
    IOCON8bits.POLH = 0;            // Drive signals are active-high
    IOCON8bits.POLL = 0;            // Drive signals are active-high

    IOCON8bits.OVRENH = 0;          // Override disabled
    IOCON8bits.OVRENL = 0;	  
    IOCON8bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON8bits.FLTDAT = 0b00;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON8bits.CLDAT = 0b00;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON8bits.DTC   = 0;          // Positive Deadtime enabled
    DTR8    = 0;
    ALTDTR8 = 0;
				  
    PWMCON8bits.XPRES = 0;            // External pins do not affect the PWMx time base
    PWMCON8bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON8bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON8bits.IFLTMOD = 0;        //
    FCLCON8bits.CLSRC = 0;          // Current-Limit Reserved
    FCLCON8bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON8bits.CLMOD = 0;          // Current-Limit Disenable

    FCLCON8bits.FLTSRC = 13;        // Fault Source is CMP1
    FCLCON8bits.FLTPOL = 0;         // Fault Signal is active-high
//    FCLCON8bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON8bits.FLTMOD = 1;         // Fault cycle by cycle
    FCLCON8bits.FLTMOD = 0;         // Fault (latched condition)
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON8bits.PHF      = 0;       // 
    LEBCON8bits.PLF      = 0;       // 
    LEBCON8bits.PHR      = 0;       // 
    LEBCON8bits.PLR      = 0;
    LEBCON8bits.FLTLEBEN = 0;       // 
    LEBCON8bits.CLLEBEN  = 0;       // 
    LEBDLY8              = 0;

    TRGCON8bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON8bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDCx depending on configuration

    TRIG8   = 0;                      // Set Initial Trigger location
    PHASE8  = 0;
    SPHASE8 = 0;
    PDC8    = DUTYmax;
    SDC8    = 0;

}

void initOC(void)
{
    OC1CON1 = 0;    // It is a good practice to initially clear the control bits
    OC1CON2 = 0;
    OC1CON1bits.OCTSEL = 0x01;  // clock T3clock 70M/8
//    OC1R = 12500;                // Duty 50% 
//    OC1R = 2500;                 // Duty 10%
//    OC1R  = 25000;                 // Duty 100%
    OC1R  = 0;
    OC1RS = 25000;               // Determines the Period  (300Hz)
    OC1CON1bits.OCM = 6;        // This selects the Edge Aligned PWM mode
    OC1CON2bits.SYNCSEL = 0x1F; // No trigger or sync source is selected
    
    //***************************
    // Assign OC1 To Pin RP48
//    RPOR8bits.RP176R  = 0x10;
    RPOR10bits.RP48R   = 0x10; // RP48 / RC0
}


void initCLC(void)
{
    // CLC1 -------------------------------------------------  
    CLC1CONLbits.LCOE = 1; // CLC1 port pin output is enabled
    CLC1CONLbits.MODE = 0; // Four-Input AND-OR

    CLC1CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC1CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC1CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC1CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC1SELbits.DS1 = 0; // DS1 = CLCINA(4L)    (0x000)
    CLC1SELbits.DS2 = 0; // DS2 =         		(0x000)
    CLC1SELbits.DS3 = 0; // DS3 =           	(0x000)
    CLC1SELbits.DS4 = 0; // DS4 =               (0x000)	
 
    CLC1GLSLbits.G1D1T = 1; // G1 = DS1-T
    CLC1GLSLbits.G2D1T = 1; // G2 = DS1-T
    CLC1GLSHbits.G3D1T = 1; // G3 = DS1-T
    CLC1GLSHbits.G4D1T = 1; // G4 = DS1-T

    CLC1CONLbits.LCEN = 1; // CLC1 is enabled
//    CLC1CONLbits.LCEN = 0; // CLC1 is Disenable
    
 // CLC2 -------------------------------------------------   
    CLC2CONLbits.LCOE = 1; // CLC2 port pin output is enabled
    CLC2CONLbits.MODE = 0; // Four-Input AND-OR

    CLC2CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC2CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC2CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC2CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC2SELbits.DS1 = 0; // DS1 =         		(0x000)
    CLC2SELbits.DS2 = 6; // DS2 = PWM3L 		(0x110) 
    CLC2SELbits.DS3 = 6; // DS3 = PWM8L         (0x110) 	
    CLC2SELbits.DS4 = 0; // DS4 =         		(0x000)
 
    CLC2GLSLbits.G1D2N = 1; // G1 = DS2-N
    CLC2GLSLbits.G2D3T = 1; // G2 = DS3-T
    CLC2GLSHbits.G3D2N = 1; // G3 = DS2-N
    CLC2GLSHbits.G4D3T = 1; // G4 = DS3-T

    CLC2CONLbits.LCEN = 1; // CLC2 is enabled
    
 // CLC3 -------------------------------------------------  
    CLC3CONLbits.LCOE = 1; // CLC3 port pin output is enabled
    CLC3CONLbits.MODE = 0; // Four-Input AND-OR

    CLC3CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC3CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC3CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC3CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC3SELbits.DS1 = 0; // DS1 =   			(0x000) 
    CLC3SELbits.DS2 = 5; // DS2 = PWM3H     	(0x101)	
    CLC3SELbits.DS3 = 0; // DS3 =           	(0x000)
    CLC3SELbits.DS4 = 5; // DS4 = PWM4H     	(0x101)

    CLC3GLSLbits.G1D2N = 1; // G1 = DS2-N
    CLC3GLSLbits.G2D4T = 1; // G2 = DS4-T
    CLC3GLSHbits.G3D2N = 1; // G3 = DS2-N
    CLC3GLSHbits.G4D4T = 1; // G4 = DS4-T

    CLC3CONLbits.LCEN = 1; // CLC3 is enabled

 // CLC4 -------------------------------------------------  
    CLC4CONLbits.LCOE = 1; // CLC4 port pin output is enabled
    CLC4CONLbits.MODE = 0; // Four-Input AND-OR

    CLC4CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC4CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC4CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC4CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC4SELbits.DS1 = 1; // DS1 = PWM7H 		(0x001) 
    CLC4SELbits.DS2 = 0; // DS2 =               (0x000)
    CLC4SELbits.DS3 = 0; // DS3 =               (0x000)
    CLC4SELbits.DS4 = 5; // DS4 = PWM2H         (0x101)
    
    CLC4GLSLbits.G1D1N = 1; // G1 = DS1-N
    CLC4GLSLbits.G2D4T = 1; // G2 = DS4-T
    CLC4GLSHbits.G3D1N = 1; // G3 = DS1-N
    CLC4GLSHbits.G4D4T = 1; // G4 = DS4-T

    CLC4CONLbits.LCEN = 1; // CLC4 is enabled

}


void initPGA(void)
{
//    ADMOD0Hbits.DIFF11   = 1;    // 1 = Channel is differential
    PGA1CONbits.PGAEN = 0; //Disable PGA1
    PGA1CONbits.SELPI = 0; //PGA1P1 as positive input
    PGA1CONbits.SELNI = 0; //GND as Negative input
    PGA1CONbits.GAIN =  4; //16x PGA Gain
    // DEVICE CALIBRATION
    TBLPAG = 0x80;
    PGA1CAL = __builtin_tblrdl (0x0E48);    // PGA1
    
//    PGA1CONbits.PGAEN = 1; //Enable PGA1
    PGA1CONbits.PGAEN = 0; //Disenable PGA1 
    
    PGA2CONbits.PGAEN = 0; //Disable PGA2
    PGA2CONbits.SELPI = 0; //PGA2P1 as positive input
    PGA2CONbits.SELNI = 0; //GND as Negative input
    PGA2CONbits.GAIN =  4; //16x PGA Gain
    // DEVICE CALIBRATION
    TBLPAG = 0x80 ;
    PGA2CAL = __builtin_tblrdl (0x0E4C);    // PGA2
    
//    PGA2CONbits.PGAEN = 1; //Enable PGA2
    PGA2CONbits.PGAEN = 0; //Disenable PGA2
}

void initADC(void)
{
    // Setup ADC Clock Input Max speed of 70 MHz --> Fosc = 140 MHz
    ADCON3Hbits.CLKSEL  = 1;    // 0-Fsys, 1-Fosc, 2-FRC, 3-APLL
    ADCON3Hbits.CLKDIV  = 0;    // Global Clock divider (1:1)
    ADCORE0Hbits.ADCS   = 0;    // Core 0 clock divider (1:2)
    ADCORE1Hbits.ADCS   = 0;    // Core 1 clock divider (1:2)
    ADCON2Lbits.SHRADCS = 0;    // 1/2 clock divider

    ADCON1Hbits.FORM    = 0;    // Integer format
    ADCON3Lbits.REFSEL  = 0;    // AVdd as voltage reference

     // ADC Cores in 12-bit resolution mode
    ADCON1Hbits.SHRRES  = 3;    // Shared ADC Core in 12-bit resolution mode
    ADCORE0Hbits.RES    = 3;    // Core 0 ADC Core in 12-bit resolution mode
    ADCORE1Hbits.RES    = 3;    // Core 1 ADC Core in 12-bit resolution mode
    ADCORE2Hbits.RES    = 3;    // Core 2 ADC Core in 12-bit resolution mode
    ADCORE3Hbits.RES    = 3;    // Core 3 ADC Core in 12-bit resolution mode
    ADCON2Hbits.SHRSAMC = 2;    // Shared ADC Core sample time 4Tad
    //ここでadcbufの場所の設定
    ADCON4Hbits.C0CHS   = 0;    // 0-AN0, 1-AN7, 2-PGA1, 3-AN0ALT
    ADCON4Hbits.C1CHS   = 0;    // 0-AN1, 1-AN18, 2-PGA2, 3-AN1ALT    
    ADCON4Hbits.C2CHS   = 0;    // 0-AN2(ボリューム R12), 1-AN11(Vo_sense, J22 中央)
                                // 起動時は開ループなので 0。SW1 で 1 に切り替わる。
    ADCON4Hbits.C3CHS   = 0;    // 0-AN3, 1-AN15, 2-Resirve, 3-Resirve    
    
    // Configure ANx for unsigned format and single ended (0,0)
    ADMOD0L = 0x0000;

    calibrateADC();
    
    ADTRIG0Lbits.TRGSRC0 = 0x08;     // ADC AN0 triggered by PWM4
    ADTRIG0Lbits.TRGSRC1 = 0x08;     // ADC AN1 triggered by PWM4
    ADTRIG0Hbits.TRGSRC2 = 0x08;     // ADC AN2 triggered by PWM4
    ADTRIG0Hbits.TRGSRC3 = 0x08;     // ADC AN3 triggered by PWM4
//    ADTRIG1Lbits.TRGSRC4 = 0x05;     // ADC AN4 triggered by PWM1
//    ADTRIG1Lbits.TRGSRC5 = 0x05;     // ADC AN5 triggered by PWM1
//    ADTRIG1Hbits.TRGSRC6 = 0x05;     // ADC AN6 triggered by PWM1
//    ADTRIG1Hbits.TRGSRC7 = 0x05;     // ADC AN7 triggered by PWM1
//    ADTRIG2Lbits.TRGSRC8 = 0x05;     // ADC AN8 triggered by PWM1
//    ADTRIG2Lbits.TRGSRC9 = 0x05;      // ADC AN9 triggered by PWM1
//    ADTRIG2Hbits.TRGSRC10 = 0x05;     // ADC AN10 triggered by PWM1
    ADTRIG2Hbits.TRGSRC11 = 0x08;    // ADC AN11 triggered by PWM4
//    ADTRIG3Lbits.TRGSRC12 = 0x05;    // ADC AN12 triggered by PWM1
//    ADTRIG3Hbits.TRGSRC13 = 0x05;    // ADC AN13 triggered by PWM1
//    ADTRIG3Hbits.TRGSRC14 = 0x05;    // ADC AN14 triggered by PWM1
//    ADTRIG3Lbits.TRGSRC15 = 0x05;    // ADC AN15 triggered by PWM1
//    ADTRIG4Hbits.TRGSRC16 = 0x05;    // ADC AN16 triggered by PWM1
//    ADTRIG4Lbits.TRGSRC17 = 0x05;    // ADC AN17 triggered by PWM1
//    ADTRIG4Hbits.TRGSRC18 = 0x05;    // ADC AN18 triggered by PWM1
//    ADTRIG4Lbits.TRGSRC19 = 0x05;    // ADC AN19 triggered by PWM1
//    ADTRIG5Hbits.TRGSRC20 = 0x05;    // ADC AN20 triggered by PWM1
//    ADTRIG5Lbits.TRGSRC21 = 0x05;    // ADC AN21 triggered by PWM1

    IFS7bits.ADCAN2IF  = 0;      // Clear ADC interrupt flag
    IPC28bits.ADCAN2IP = 6;      // Set ADC interrupt priority to 7
    ADIELbits.IE2     = 1;      // Enable ADC AN2 interrupt 
}

void calibrateADC(void)
{
    // Power Up delay: 2048 Core Clock Source Periods (TCORESRC) for all ADC cores
    // (~14.6 us)
    _WARMTIME = 11;

    // Turn on ADC module
    ADCON1Lbits.ADON  = 1;

    // Turn on analog power for dedicated core 0
    ADCON5Lbits.C0PWR = 1;
    while(ADCON5Lbits.C0RDY == 0);
    ADCON3Hbits.C0EN  = 1;   // Enable ADC core 0

    // Turn on analog power for dedicated core 1
    ADCON5Lbits.C1PWR = 1;
    while(ADCON5Lbits.C1RDY == 0);
    ADCON3Hbits.C1EN  = 1;   // Enable ADC core 1

    // Turn on analog power for dedicated core 2
    ADCON5Lbits.C2PWR = 1;
    while(ADCON5Lbits.C2RDY == 0);
    ADCON3Hbits.C2EN  = 1;   // Enable ADC core 2

    // Turn on analog power for dedicated core 3
    ADCON5Lbits.C3PWR = 1;
    while(ADCON5Lbits.C3RDY == 0);
    ADCON3Hbits.C3EN  = 1;   // Enable ADC core 3

    // Turn on analog power for shared core
    ADCON5Lbits.SHRPWR = 1;
    while(ADCON5Lbits.SHRRDY == 0);
    ADCON3Hbits.SHREN  = 1;  // Enable shared ADC core

    // Enable calibration for the dedicated core 0
    ADCAL0Lbits.CAL0EN   = 1;
    ADCAL0Lbits.CAL0DIFF = 0;         // Single-ended input calibration
    ADCAL0Lbits.CAL0RUN  = 1;         // Start Cal
    while(ADCAL0Lbits.CAL0RDY == 0);
    ADCAL0Lbits.CAL0EN   = 0;         // Cal complete

    // Enable calibration for the dedicated core 1
    ADCAL0Lbits.CAL1EN   = 1;
    ADCAL0Lbits.CAL1DIFF = 0;         // Single-ended input calibration
    ADCAL0Lbits.CAL1RUN  = 1;         // Start Cal
    while(ADCAL0Lbits.CAL1RDY == 0);
    ADCAL0Lbits.CAL1EN   = 0;         // Cal complete

    // Enable calibration for the dedicated core 2
    ADCAL0Hbits.CAL2EN   = 1;
    ADCAL0Hbits.CAL2DIFF = 0;         // Single-ended input calibration
    ADCAL0Hbits.CAL2RUN  = 1;         // Start Cal
    while(ADCAL0Hbits.CAL2RDY == 0);
    ADCAL0Hbits.CAL2EN   = 0;         // Cal complete

    // Enable calibration for the dedicated core 3
    ADCAL0Hbits.CAL3EN   = 1;
    ADCAL0Hbits.CAL3DIFF = 0;         // Single-ended input calibration
    ADCAL0Hbits.CAL3RUN  = 1;         // Start Cal
    while(ADCAL0Hbits.CAL3RDY == 0);
    ADCAL0Hbits.CAL3EN   = 0;         // Cal complete

    // Enable calibration for the shared core
    ADCAL1Hbits.CSHREN   = 1;
    ADCAL1Hbits.CSHRDIFF = 0;        // Single-ended input calibration
    ADCAL1Hbits.CSHRRUN  = 1;        // Start calibration cycle
    while(ADCAL1Hbits.CSHRRDY == 0); // while calibration is still in progress

    ADCAL1Hbits.CSHREN   = 0;        // Calibration is complete
}

void initTimer(void)
{
    // Configure Timer 2 for soft start
    PR2 = 9375;
    T2CONbits.TCKPS = 2;            // 1:64
    T2CONbits.TCS = 0;              // Clock source is internal Fcy
    T2CONbits.TGATE = 0;
    T2CONbits.TON = 1;
    
    // Configure Timer 3 for soft start
    PR3 = 0xAD00;
    T3CONbits.TCKPS = 1;            // 1:8
    T3CONbits.TCS = 0;              // Clock source is internal Fcy
    T3CONbits.TON = 1;
}

void initCMP(void)
{
    CMP1CONbits.CMPON = 0;  //Disable CMP1
    (Nop());
    CMP1CON =0;
    (Nop());
    CMP1CONbits.FLTREN  = 1; //Digital filter is enabled
    CMP1CONbits.INSEL  = 3; //Selects CMP1D
    CMP1CONbits.ALTINP = 0; //Selects CMP1D
    CMP1CONbits.CMPPOL = 0; //Output is non-inverted
    CMP1CONbits.RANGE = 1;
    
    CMP1DAC = CMP1DAC_OCP;  // 過電流保護のしきい値
   
    CMP1CONbits.CMPON = 1;  //Enable CMP1
    
    CMP2CONbits.CMPON = 0;  //Disable CMP2
    (Nop());
    CMP2CON =0;
    
    CMP3CONbits.CMPON = 0;  //Disable CMP3
    (Nop());
    CMP3CON =0;

    CMP4CONbits.CMPON = 0;  //Disable CMP4
    (Nop());
    CMP4CON =0;
}