/* ===========================================================================
 *  closed_loop_buck_pfc.c
 *
 *  ブリッジレスPFC（Q1/Q2） ＋ 位相シフトフルブリッジ（Q3〜Q6）  閉ループ制御
 *
 *  ターゲット : dsPIC33EP128GS808 DEVELOPMENT BOARD (DM330026)
 *               MPLAB X + XC16 v1.70
 *  対象回路   : 制御＿シミュレーション/scideam/Buck PFC_closed.scicir
 *  制御の出典 : 制御＿シミュレーション/scideam/closed_loop_buck_pfc.sciprc
 *               （修論（林さん）.docx 4.2節「提案回路の制御方式」）
 *  土台       : mplab/dsPIC33EP128GS808_BuckPFC/Buck_OPENPI.c（開ループ）
 *               周辺初期化（initClock/initIO/initPWM/initCLC/initADC/initCMP）は
 *               そのまま引き継いでいる。差分は「★開ループからの変更点」を参照。
 *
 *  Fcy = 60MHz / Fosc = 119.7MHz / スイッチング 100kHz（PTPER = 9600）
 *  AC100Vrms 50Hz  ->  DC48V / 10A（480W）
 *
 * ---------------------------------------------------------------------------
 *  【動作モード】SW1（RE4）を押すと切り替わる
 *
 *   開ループ（起動時。LED1 消灯）
 *     ADCBUF2 = AN2 = 基板上ボリューム R12
 *     ★電流センサに給電していないと LED2（赤）が点灯し、Q1/Q2 は回らない。
 *       これは仕様（センサ未確立で電流を流さない）。フルブリッジだけは
 *       動くので、無通電でゲート波形の確認ができる。
 *     OPEN_MODE_DUTY = 1 : ツマミがそのままデューティ比（添付の開ループと同じ挙動）
 *     OPEN_MODE_DUTY = 0 : ツマミが入力電流の振幅指令 Ipk（昇圧形では安全側）
 *
 *   閉ループ（LED1 点灯）
 *     ADCBUF2 = AN11 = Vo_sense（TLP291 フォトカプラ。J22 の真ん中のピン）
 *     出力電圧ループ（50Hz）が Ipk を作り、電流ループ（100kHz）がデューティを作る。
 *
 *   切り替えの瞬間は Q1/Q2 を 20ms 止め、電圧ループの積分器を「いま出ている Ipk」に
 *   合わせてから渡す（バンプレス移行）。これをやらないと指令が飛ぶ。
 *   フォルト中に押した場合はモード切替ではなくフォルトリセットになる。
 *
 * ---------------------------------------------------------------------------
 *  【3ループ構成】
 *   (1) 電流ループ      100 kHz  平均電流制御 -> Q1/Q2 のデューティ
 *   (2) PFC電圧ループ    50 Hz   出力電圧（商用1周期の移動平均）-> 電流の振幅指令
 *   (3) 位相シフト       50 Hz   D1_FIX 固定（既定）／低速トリム（PH_MODE = 1）
 *
 * ---------------------------------------------------------------------------
 *  ★★ ゲインは sciprc の値をそのまま使っていない ★★
 *
 *  sciprc の電流ループは KP_I = 0.028508 / KI_I = 1053.66 で、PI ゼロが 5882Hz。
 *  これは交差周波数 2528Hz より上にあるため、交差で PI がまだ積分器として
 *  振る舞う（-90度）。プラント Vdc/(sL) の -90度 と合わせて -180度 に張り付き、
 *  遅れ 1.5*Ts を入れた位相余裕は 9.6度 しかない。
 *  理想プラントのシミュレータでは「安定」と出るが、実機で電流センスにわずかな
 *  遅れが乗るだけで振動する。
 *
 *  PFC電圧ループも同じ誤りで、sciprc の FC_V = 4Hz / ゼロ 5Hz は
 *  50Hz 更新 + 20ms 移動平均の遅れ（1.5*Tv = 30ms）を入れると位相余裕 -11.3度、
 *  つまり発散する。
 *
 *  そこで本ファイルでは PI ゼロを交差の 1/5 に置き直した。
 *
 *      ループ        交差        ゼロ       位相余裕   ゲイン余裕
 *      電流        3333 Hz     667 Hz      60.5度     13.7 dB
 *      PFC電圧        2 Hz     0.4 Hz      56.9度       --
 *
 *  ※出力電圧が指令に届かないとき、FC_V を上げてはいけない（発散する方向）。
 *    まず wout_pk と iref_pk を見て、DUTYmax / IPK_MAX_MUL に張り付いていないかを
 *    確かめること。張り付いていれば入力電力が足りないだけで、帯域の問題ではない。
 *
 * ---------------------------------------------------------------------------
 *  【ハード対応】
 *   ADC（すべて PWM4 トリガ。AN2 の変換完了割り込みで制御を実行）
 *     ADCBUF0  = AN0   VL_sense   分圧 + MCP6021 バッファ
 *     ADCBUF1  = AN1   VN_sense   分圧 + MCP6021 バッファ
 *     ADCBUF2  = AN2  （開ループ：ボリューム R12）
 *              = AN11 （閉ループ：Vo_sense フォトカプラ）  ADCON4H.C2CHS で切替
 *     ADCBUF3  = AN3   ILsense    TMCS1107A3B ホール電流センサ（双方向）
 *     ADCBUF11 = AN11  共有コアでも変換しておく（開ループ中でも Vo を読める）
 *
 *     VL_sense と VN_sense はどちらも GND2 基準の片側だけが立つ。
 *       正の半周期 : VL = |vac| , VN = 0     （Q2 のボディダイオードでクランプ）
 *       負の半周期 : VL = 0     , VN = |vac| （Q1 のボディダイオードでクランプ）
 *     したがって
 *       Vac  = ADCBUF0 - ADCBUF1  -> 符号付き（極性判定に使う）
 *       Vrec = ADCBUF0 + ADCBUF1  -> |vac| 全波整流（乗算器とFFに使う）
 *     ※このクランプは電流が流れていないと成立しない。無負荷での校正値を
 *       そのまま定格に使うと合わないことがある。
 *
 *   PWM（PTPER = 9600 = 100kHz、1count = 1.0417ns）
 *     PWM1  PDC1    Q1  ブリッジレスPFC（正の半周期に PWM）
 *     PWM2  PDC2    Q2  同        （負の半周期に PWM）
 *     PWM6  PHASE6  レグA : PWM6H = Q3(ハイ) / PWM6L = Q4(ロー)   中点 Tr+
 *     PWM7  PHASE7  レグB : PWM7H = Q6(ハイ) / PWM7L = Q5(ロー)   中点 Tr-
 *       PDC6 = PDC7 = PTPER/2 固定（50% 相補）
 *       デッドタイム DTR6/7 = ALTDTR6/7 = DEADTIME（600 = 625ns）
 *       位相シフトは PHASEx に「遅らせたいカウント数」をそのまま書く。
 *       遅れ側は商用半周期ごとに入れ替える（PS_SWAP = 1、修論 4.2）。
 *     PWM4 は変調しないが ADC トリガ源として残す（TRIG4 = 2400）。
 *     PWM3 / PWM5 / PWM8 は未使用（PENH/PENL = 0 でピンに出さない）。
 *
 *   ★CLC について★
 *     initCLC は添付の開ループのまま残してあるが、CLC2 は PWM3L / PWM8L、
 *     CLC3 は PWM3H / PWM4H、CLC4 は PWM7H / PWM2H を入力にしている。
 *     Q1/Q2 を PWM1/PWM2 に移したので、PWM3 / PWM8 を変調しなくなり、
 *     また PWM2H が新たに変調されるようになる。
 *     => CLC1/CLC2 の出力ピン（RP36 / RP33 / RP76 / RP34）を使っているなら、
 *        通電前にスコープで波形を確認すること。
 *        使っていないなら USE_CLC を 0 にして CLC を止めてよい。
 *
 * ---------------------------------------------------------------------------
 *  ★★ 実機で必ず校正する定数 ★★
 *     下の「(1) ハードウェア校正定数」ブロックの 5 つ。
 *     ここを直せばゲインは全部自動で追従する（#define の連鎖で導いてある）。
 *     手順は各定数のコメントを参照。
 * ===========================================================================
 */

#include "p33Exxxx.h"
#include "init.h"
#include <xc.h>
#include <stdint.h>

static void toggle_mode(void);

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

/* =======================================================================
 *  (1) ハードウェア校正定数   ★実機で測って置き換える★
 *
 *  ここを直せば下のゲインは全部自動で追従する。数値をベタ書きしないのは
 *  そのため（コンパイラが定数畳み込みするので実行時コストはゼロ）。
 * ======================================================================= */

/* --- 電流センサ TMCS1107A3B（双方向、VS = 3.3V）---------------------------
 *  【測り方】AC を切った状態（無電流）で ADCBUF3 を読む。その値が ISENS_OFS。
 *            何度か読んで中央値を取る。 */
#define ISENS_V_PER_A   0.200       /* センサ感度 [V/A]（データシート値）   */
#define ISENS_OFS       2015        /* 無電流時の ADC 値（実測 2000..2030） */

/* --- ライン電圧センス（分圧 + MCP6021 バッファ）--------------------------
 *  【測り方】AC を切って VL に既知の直流（例 50V）を加え、ADCBUF0 を読む。
 *            VLINE_CNT_PER_V = (読んだ値) / 50
 *  下の値は 150k / 1.2k 分圧を仮定した暫定値。必ず実測で置き換えること。   */
#define VLINE_CNT_PER_V 9.83        /* [ADCカウント/V]                      */

/* --- 出力電圧センス（TLP291 フォトカプラ）-------------------------------
 *  TLP291 の CTR は個体差が非常に大きい（50〜600%）ので計算では決まらない。
 *  【測り方】開ループで出力を立て、Vo を電圧計で測りながら ADCBUF2 を読む。
 *            2 点取って傾きを出す。
 *              VO_CNT_PER_V = (cnt2 - cnt1) / (V2 - V1)
 *              VO_REF_CNT   = 48V のときのカウント値
 *  下の値は「48V で 3.0V 出力」を仮定した暫定値。                          */
#define VO_CNT_PER_V    77.58       /* [ADCカウント/V]                      */
#define VO_REF_CNT      3724        /* 48V のカウント = (4095*3.0)/3.3      */


/* =======================================================================
 *  (2) 回路定数（Scideam / closed_loop_buck_pfc.sciprc と一致させる）
 * ======================================================================= */
#define VAC_RMS_V       100.0       /* 入力電圧 [Vrms]                      */
#define VAC_PK_V        (VAC_RMS_V*1.41421356)      /* 141.42 V             */
#define VO_REF_V        48.0        /* 出力電圧指令 [V]                     */
#define IO_RATED_A      10.0        /* 定格負荷電流 [A]（IDC = 定電流負荷） */
#define PO_RATED_W      (VO_REF_V*IO_RATED_A)       /* 480 W                */
#define ETA             0.93        /* 想定効率（電流上限の見積りに使うだけ）*/
#define L_H             1.0e-3      /* PFC インダクタ L [H]                 */
#define C_OUT_F         7.8e-3      /* 出力コンデンサ C2 [F]                */
#define N_TR            4.0         /* トランス巻数比 Np/Ns(片側)           */
#define L_LK_H          5.0e-6      /* トランス漏れインダクタンス L1 [H]    */

/* 中間（DCリンク）電圧 Vcs。プラント Vdc/(sL) の Vdc であり、同時に
 * フィードフォワードの分母でもある。★実機で Vcs1 を測って合わせること。
 * ここがずれると電流ループのゲインと、電圧ピーク付近の歪みに効く。 */
#define VDC_V           220.0
#define VCS_FF_V        VDC_V       /* FF 用（= 実測の Vcs 平均）           */
#define VCS_DES_V       200.0       /* 位相シフト D1 の設計点（下記参照）   */


/* =======================================================================
 *  (3) タイミング定数
 * ======================================================================= */
#define PI2             6.28318530718
#define PERIODmax       9600        /* PTPER : 100kHz（1count = 1.0417ns）  */
#define FS_HZ           100000.0
#define TS_S            (1.0/FS_HZ)
#define F_LINE_HZ       50          /* ★商用周波数。60Hz 地域ではここを直す
                                     *   N_VLOOP が自動追従する（移動平均の
                                     *   窓幅が商用1周期でなくなるとリプルが
                                     *   素通りして電圧ループが揺れる）     */
#define N_VLOOP         ((int)(FS_HZ/F_LINE_HZ))    /* 2000 = 商用1周期     */
#define DEADTIME        600         /* DTR6/7, ALTDTR6/7 : 625ns            */
#define PHASEmax        (PERIODmax/2)               /* 4800 = 180deg        */
#define PHASEmin        0
#define DUTYmax         9120        /* Q1/Q2 デューティ上限 95%             */
#define DUTYmin         0

/* 零クロス近傍で Q1/Q2 を止める幅。sciprc の PO_HYST = 0.005*VAC_PK 相当。
 * オペアンプのオフセットとノイズより十分大きく取る。小さすぎると入力が
 * 無いのにオフセットだけで極性が決まってしまう。 */
#define VAC_HYST        ((int)(0.005*VAC_PK_V*VLINE_CNT_PER_V) + 2)


/* =======================================================================
 *  (4) ADC 換算係数
 * ======================================================================= */
#define ADC_FS          4095
#define ADC_CNT_PER_V   (4095.0/3.3)                    /* 1240.9           */
#define ADC_CNT_PER_A   (ADC_CNT_PER_V*ISENS_V_PER_A)   /*  248.2           */
#define VREC_PK_CNT     ((int)(VLINE_CNT_PER_V*VAC_PK_V))   /* 1390         */

/* --- 移動平均の除算を掛け算 + シフトに置き換える ------------------------
 *  N_VLOOP = 2000 は 2 のべき乗ではないので、そのまま割ると XC16 の
 *  32bit 除算ライブラリ（___divsi3）が制御 ISR の中で呼ばれる。100 cycle
 *  以上かかるので、2000 回に 1 回だけ ISR が予算（60MIPS * 10us =
 *  600 cycle）に近づいてしまう。掛け算 + シフトなら数 cycle で済む。
 *
 *  AVG_SH の上限は N_VLOOP に依らない：
 *      vo_sum の最大 = ADC_FS * N_VLOOP
 *      積の最大      = ADC_FS * N_VLOOP * (2^AVG_SH / N_VLOOP)
 *                    = ADC_FS * 2^AVG_SH  < 2^31
 *      -> 2^AVG_SH < 2^31/4095 = 2^19   -> AVG_SH <= 18
 *  F_LINE_HZ を 60 に変えても成立する。
 *
 *  AVG_MUL = 131 のとき 131*2000/2^18 = 0.99945 なので利得誤差 -0.055%。
 *  定数倍の誤差なので VO_CNT_PER_V の校正に吸収される。                   */
#define AVG_SH          18
#define AVG_MUL         ((int32_t)((1L<<AVG_SH)/(double)N_VLOOP))   /* 131  */
#define AVG_RND         (1L<<(AVG_SH-1))
#define BOXCAR(sum)     ((int)((((sum)*AVG_MUL) + AVG_RND) >> AVG_SH))


/* =======================================================================
 *  (5) 制御ゲイン
 *
 *  PI は双一次（Tustin）の増分形で実装する。
 *      u[n] = u[n-1] + A*e[n] - B*e[n-1]
 *      A = Kp + Ki*T/2 ,  B = Kp - Ki*T/2
 *
 *  ★アキュムレータは 2^S 倍のまま int32 で保持する。分解能稼ぎではない。
 *    偏差 1 カウントあたりの積分増分は 1 PWM カウント未満（電流ループで
 *    0.154）なので、毎回シフトして整数に丸めると増分が 0 に切り捨てられ、
 *    積分が永久に効かなくなる（P項だけの比例制御になり定常偏差が残る）。
 *    エラーも警告も出ないので、波形を見るまで気づけない種類のバグになる。
 * ======================================================================= */

/* --- 電流ループ（100kHz、毎スキャン）-----------------------------------
 *  プラント P(s) = Vdc/(s*L)   [A/duty]
 *    fc = fs/30 = 3333Hz、ゼロ = fc/5 = 667Hz
 *    遅れ 1.5*Ts を入れて 位相余裕 60.5度 / ゲイン余裕 13.7dB
 *  単位の鎖： [duty/A] -> [PWMcnt/ADCcnt] は x PERIODmax / ADC_CNT_PER_A  */
#define FC_I_HZ         (FS_HZ/30.0)    /* 3333 Hz */
#define FZ_I_HZ         (FC_I_HZ/5.0)
#define KP_I_PHYS       (PI2*FC_I_HZ*L_H/VDC_V)                 /* 0.09520  */
#define KI_I_PHYS       (KP_I_PHYS*PI2*FZ_I_HZ)                 /* 398.9    */
#define KP_I_CNT        (KP_I_PHYS*PERIODmax/ADC_CNT_PER_A)     /* 3.6825   */
#define KIT_I_CNT       (KI_I_PHYS*TS_S*PERIODmax/ADC_CNT_PER_A)/* 0.15425  */
/* シフト量の根拠
 *   (a) 死にゾーン   : Ki*T = 0.154 -> 小数 2.7bit 必要 -> S >= 9
 *   (b) オーバーフロー: e をフルスケール 4095 で評価、余裕 8 倍 -> S <= 14
 *   (c) int16 係数   : |A|*2^S < 32767 -> S <= 13
 *   -> 9 <= S <= 13 の範囲で分解能を取り 13。FC_I_HZ を上げると (c) を破って
 *      KI_A が int16 を超えるので、そのときは S を下げること。            */
#define SH_I            13
#define KI_A            ((int32_t)((KP_I_CNT + KIT_I_CNT/2.0)*(1L<<SH_I)))  /* 30798 */
#define KI_B            ((int32_t)((KP_I_CNT - KIT_I_CNT/2.0)*(1L<<SH_I)))  /* 29534 */

/* --- 乗算器 Iref = (Vrec * Ipk) >> SH_MUL ------------------------------
 *  Vrec が入力ピークのとき Iref が電流ピーク指令になるようにスケールする。
 *  電圧ループの出力 Ipk をこの単位で持つので、ISR で余計な変換が要らない。*/
#define SH_MUL          11
#define MULCONV         (ADC_CNT_PER_A*(1L<<SH_MUL)/VREC_PK_CNT)    /* 365.6 */

/* --- PFC電圧ループ（50Hz = 移動平均が確定したときだけ）-----------------
 *  プラント P(s) = Vac_pk/(2*Vo*C*s)   [V/(Ipk の単位)]
 *    fc = 2Hz、ゼロ = fc/5 = 0.4Hz   -> 位相余裕 56.9度
 *  ★離散化の T は 1/F_LINE（20ms）であって 1/FS ではない。
 *  ★C_OUT_F を変えたらゲインは自動で追従する（C を 2 倍にすればプラント
 *    利得が半分になるので、ゲインを 2 倍しないと帯域が下がる）。
 *  ★出力が指令に届かないとき FC_V_HZ を上げてはいけない（発散する方向）。*/
#define FC_V_HZ         2.0
#define FZ_V_HZ         (FC_V_HZ/5.0)
#define G_V0            (VAC_PK_V/(2.0*VO_REF_V*C_OUT_F))      /* 188.86    */
#define KP_V_PHYS       (PI2*FC_V_HZ/G_V0)                     /* 0.066537  */
#define KI_V_PHYS       (KP_V_PHYS*PI2*FZ_V_HZ)                /* 0.1672    */
#define KP_V_CNT        (KP_V_PHYS/VO_CNT_PER_V*MULCONV)       /* 0.31357   */
#define KIT_V_CNT       (KI_V_PHYS/(double)F_LINE_HZ/VO_CNT_PER_V*MULCONV) /* 0.015757 */
#define SH_V            16          /* (a) Ki*T=0.0158 -> S>=12 / (c) S<=16 */
#define KV_A            ((int32_t)((KP_V_CNT + KIT_V_CNT/2.0)*(1L<<SH_V)))  /* 21066 */
#define KV_B            ((int32_t)((KP_V_CNT - KIT_V_CNT/2.0)*(1L<<SH_V)))  /* 20034 */

/* --- 電流振幅指令の上限 ------------------------------------------------ */
#define IPK_RATED_A     (2.0*PO_RATED_W/(ETA*VAC_PK_V))     /*  7.30 A      */
#define IPK_MAX_A       (1.5*IPK_RATED_A)                   /* 10.95 A      */
#define IPK_MAX_MUL     ((int32_t)(IPK_MAX_A*MULCONV))      /* 4003         */
#define IPK_OPEN_MAX    IPK_MAX_MUL     /* 開ループでツマミ全開のときの Ipk */

/* --- ソフトスタート（出力電圧指令のランプ）---------------------------- */
#define VREF_SLEW_VPS   300.0                               /* [V/s]        */
#define VREF_STEP_CNT   ((int)(VREF_SLEW_VPS/F_LINE_HZ*VO_CNT_PER_V))

/* --- フィードフォワード（昇圧則）--------------------------------------
 *  Vcs = |vac|/(1-D)  ->  D = 1 - |vac|/Vcs
 *  D_ff = PERIODmax - ((Vrec * KFF) >> SH_FF)
 *
 *  ★★ 立ち上げ中は 0 にすること ★★
 *    VCS_FF_V = 220V 決め打ちなので、入力が数 V しかない段階で有効にすると
 *    「220V まで昇圧せよ」= デューティ 95% を出し続ける。実際の Vcs は
 *    数十 V なので指令として意味がなく、電流ループが積分で押し戻すまでの
 *    間に制御外の電流が流れる。
 *    100V で安定して回るようになってから 1 に戻すと歪みが減る。          */
#define USE_FF          0
#define SH_FF           12
#define KFF             ((int32_t)((double)PERIODmax/(VLINE_CNT_PER_V*VCS_FF_V)*(1L<<SH_FF)))  /* 18183 */

/* --- 位相シフト -------------------------------------------------------
 *  静特性 式(4.46) を D1 について解いた値
 *      D1 = sqrt( Vo*Llk*Io / (Ts*Vcs*(Vcs - n*Vo)) )
 *
 *  ★(Vcs - n*Vo) が分母にあるので、Vcs の仮定で D1 が倍半分変わる★
 *      Vcs = 200V -> D1 = 0.387 -> 139.4deg -> PHASE 3717 カウント  <- 既定
 *      Vcs = 220V -> D1 = 0.197 ->  71.1deg -> PHASE 1894 カウント
 *    n*Vo = 192V に近づくほど伝送能力が落ちて必要な位相が発散する。
 *    裏を返すと軽負荷では Vcs が n*Vo に張り付く（ブリッジが出力電圧の
 *    n 倍にクランプする）。実機で Vcs1 を測ってから決めること。
 *
 *  角度を大きくする -> 伝送能力が上がる -> 同じ負荷なら Vcs が下がる
 *  角度を小さくする -> 伝送能力が下がる -> Vcs が上がる
 *  ただし無負荷では電力の行き先が無いので、角度を変えても Vcs の上昇は
 *  止まらない。負荷を繋ぐことが先。                                      */
#define D1_FIX_DEG      139.4
#define D1_FIX_CNT      ((int)(D1_FIX_DEG/360.0*PERIODmax))  /* 3717        */

/* PH_MODE = 0 : D1_FIX 固定（推奨。まずこれで回す）
 *               Cs1/Cs2 は 100Hz の脈動エネルギーを蓄えないので、D1 を
 *               固定しても Vcs が自然に変動して脈動電力を通す。出力電圧は
 *               PFC電圧ループ（入力電流の振幅）だけで制御できる。
 * PH_MODE = 1 : D1_FIX を中心に +-D1_TRIM_CNT だけ低速トリムする。
 *               帰還は Vo_avg なのでリプルには反応しない。
 *
 *  トリムのゲイン（プラントから設計）
 *    dIo/dD1 = 2*Io/D1 = 51.7 A/D1
 *    等価負荷 dIo/dVo = Io*Vcs/(Vo*(Vcs-n*Vo)) = 5.21 A/V
 *    -> 直流利得 = 9.92 V per unit D1、極は 691Hz（Co 起因）
 *    交差 4.6Hz を積分項主導で取り、遅れ 1.5*Tv を入れて 位相余裕 70度
 *  ★sciprc では「電圧ループと同じ係数でよい」と流用していたが、単位が
 *    [D1/V] と [A/V] で違うので、ここでは専用に換算している。            */
#define PH_MODE         0
#define D1_TRIM_CNT     768         /* +-0.08 * PERIODmax                   */
#define KP_PH_PHYS      0.05        /* [D1/V]                               */
#define KI_PH_PHYS      2.53        /* [D1/(V*s)]                           */
#define KP_PH_CNT       (KP_PH_PHYS*PERIODmax/VO_CNT_PER_V)         /* 6.187 */
#define KIT_PH_CNT      (KI_PH_PHYS/(double)F_LINE_HZ*PERIODmax/VO_CNT_PER_V) /* 6.261 */
#define SH_PH           11          /* |A|*2^S < 32767 -> S <= 11           */
#define KPH_A           ((int32_t)((KP_PH_CNT + KIT_PH_CNT/2.0)*(1L<<SH_PH))) /* 19081 */
#define KPH_B           ((int32_t)((KP_PH_CNT - KIT_PH_CNT/2.0)*(1L<<SH_PH))) /*  6261 */

/* 位相の遅れ側を商用半周期ごとに入れ替えるか（修論 4.2）
 *   1 : 正の半周期はレグB遅れ、負の半周期はレグA遅れ
 *   0 : 常にレグB を遅らせる。電力伝送窓の長さは同じなので特性は変わらない。
 *       零クロスでの位相反転が無くなるので、50Hz の非対称が疑わしいときは
 *       こちらを試す。                                                     */
#define PS_SWAP         1


/* =======================================================================
 *  (6) 保護
 *
 *  一つの保護で全部を賄おうとすると、速さとノイズ耐性のどちらかを必ず失う。
 *  それぞれが得意なことだけをやる 3 層に分ける。
 *
 *   第1層 アナログ比較器 CMP1（入力 CMP1D = AN3 = ILsense）
 *         サブ us。ノイズ耐性は低いので ★サイクルバイサイクル★ にする。
 *         ラッチにするとノイズ 1 発で装置全体が止まる。サイクルバイサイクル
 *         なら該当周期だけ削れて次から復帰するので、保護の役目は果たしつつ
 *         運転は続く。PWM1/PWM2 の FLTMOD = 1 にしてある。
 *         ★非反転比較なので「しきい値より上」でしかトリップしない。
 *           負の半周期（センサ出力が中点より下）は守れない。
 *
 *   第2層 ADC サンプリング + ソフト判定
 *         1 周期（10us）。サンプル点で平均化されるのでノイズ耐性が高い。
 *         ★ラッチ停止はこちら★。両極性を見る。単発ノイズでラッチしない
 *         よう 2 回連続（20us）で成立したときだけラッチする。
 *         20us で進む電流は 141V/1mH で 1.4A なので許容範囲。
 *
 *   第3層 妥当性チェック
 *         センサの生死、極性判定の誤り。制御器にゴミを食わせない。
 *
 *  ★★ 段をまたいで止めない ★★
 *    入力側（Q1/Q2）の過電流で後段のフルブリッジまで止めてはいけない。
 *    中間コンデンサ Cs1/Cs2 に溜まったエネルギーの逃げ場がなくなって
 *    過電圧になる。正しい動作は「前段を絞って入力を断ち、後段は回したまま
 *    中間電圧を出力へ抜く」。そのため既定では PWM6/7 を止めない。
 *
 *  ★★ 電流センサの測定上限に注意 ★★
 *    TMCS1107A3B（200mV/A）を VS = 3.3V で使うと線形範囲は +-7.25A。
 *    出力が 3.1V で頭打ちになるので、それ以上の電流は「見えない」。
 *    一方 480W 定格での入力電流ピークは 7.3A なので、定格運転すると
 *    保護しきい値が通常動作域に入ってしまう。
 *      -> 立ち上げ中（低電力）は 7.0A で問題ない。
 *      -> 定格まで上げるときは感度の低い TMCS1107A2B（100mV/A, +-14.5A）に
 *         交換すること。I_OCP_A を上げて逃げると保護が実質無効になる。
 * ======================================================================= */
#define I_OCP_A         7.0         /* トリップ電流 [A]（上記の上限に注意） */
#define I_OCP_CNT       ((int)(I_OCP_A*ADC_CNT_PER_A))      /* 1737         */
#define CMP1DAC_OCP     ((int)(ISENS_OFS + I_OCP_A*ADC_CNT_PER_A))  /* 3752 */
#define OCP_SOFT_EN     1           /* 0 でソフト過電流ラッチを止める
                                     * （記録だけ残す。誤検出の切り分け用。
                                     *   ハードの CMP1 は生きたまま）      */
#define USE_CMP_OCP     1           /* ハード比較器 CMP1 による過電流保護
                                     * ★添付の開ループでは CMPON = 0 で
                                     *   無効のままだったので、ここで初めて
                                     *   有効にする。
                                     * ★要確認★ initCMP は INSEL = 3
                                     *   （CMP1D）を選んでいる。CMP1D が
                                     *   本当に AN3（ILsense）かをデータシート
                                     *   とピン配置で確認すること。入力の
                                     *   選択を間違えると、常時トリップして
                                     *   デューティが出ない（サイクル毎モード
                                     *   なので壊れはしないが原因が分かり
                                     *   にくい）。疑わしいときは 0 にして
                                     *   ソフト保護だけで動かし、切り分ける。*/
#define FAULT_STOP_BRIDGE 0         /* 1 にすると過電流でフルブリッジも
                                     * 止める。中間電圧が過電圧になりうる
                                     * ので既定は 0。                      */

/* --- センサの健全性チェック -------------------------------------------
 *  センサが未給電だと ADCBUF3 = 0 になり、Iin = -ISENS_OFS という
 *  「大きな負電流」に見えて過電流保護が誤作動する。そこで
 *    起動時 : |Iin| が SENS_OK_TOL 以内の状態が 20ms 続いたら「健全」
 *    運転中 : ADCBUF3 が SENS_FLOOR 未満になったら「喪失」
 *  VS = 3.3V のセンサは出力が 0.1V 程度までしか下がらないので、0V 近辺は
 *  「電源が無い」とみなしてよい。
 *  健全でない間は Q1/Q2 を回さない（フルブリッジは動かしたままなので、
 *  センサに電源を入れずにゲート波形の確認ができる）。
 *
 *  ★判定は「N 回連続」ではなく「積算」で行う★
 *    「2000 回連続で範囲内」だと、20ms の間に単発ノイズが 1 回入るだけで
 *    振り出しに戻り、ノイズ余裕が小さいと永久に成立しない。              */
#define SENS_OK_TOL     400         /* 約 +-1.6A（実測ノイズ +-133 の 3 倍）*/
#define SENS_FLOOR      80          /* これ未満はセンサ喪失（約 0.06V）    */


/* =======================================================================
 *  (7) 動作モードと入出力
 * ======================================================================= */
#define MODE_OPEN       0
#define MODE_CLOSED     1

/* 開ループでボリューム R12（AN2）が操作する量
 *   1 : デューティ比を直に与える（添付の開ループと同じ挙動）
 *       電流ループと FF はバイパスされる。波形の比較がしやすい。
 *       ★電流は制限されない（CMP1 とソフト OCP だけが頼り）。
 *         低い入力電圧から、ツマミを絞った状態で始めること。
 *   0 : 入力電流の振幅指令 Ipk を与える
 *       電流ループと FF は生きたままなので、入力電流は正弦波の形を保った
 *       まま振幅だけが変わる。昇圧形でデューティを直に固定すると電流が
 *       発散しうるので、定格に近づける段ではこちらに切り替える。
 *
 *  進め方： まず 1 で添付と同じ波形を確認し、確認が済んだら 0 に移る。
 *
 *  ※添付の開ループはツマミ R20（AN3）で位相シフト角も調整していたが、
 *    この基板では AN3 が電流センサ ILsense なので再現していない。
 *    位相角は D1_FIX_DEG で決める。                                      */
#define OPEN_MODE_DUTY  1

/* SW1(RE4) の論理。開発ボードは 10k プルアップ + 押下で GND */
#define SW1_ACTIVE_LOW  1

/* SW1 の読み取り周期（制御 ISR の回数。2000 = 20ms）
 *  ★CN 割り込みは使わない★
 *    チャタリングで押下中に「離した」エッジを拾ってしまい、押す・離すで
 *    2 回トグルして元に戻ることがある。20ms ごとにポーリングし、2 回連続で
 *    同じ状態なら確定とする。                                            */
#define SW_SAMPLE       N_VLOOP

/* モード切替直後にゲートを止める期間。ADC のチャネル切替が落ち着くまでと、
 * 移動平均の窓が埋まるまでを兼ねて商用1周期ぶん取る。 */
#define MODE_BLANK      N_VLOOP

/* モード表示 LED（開発ボードのユーザーズガイド DS50002632A 回路図より）
 *   LED1 = LD1（緑）= RE11 : 閉ループで点灯
 *   LED2 = LD2（赤）= RE10 : フォルト／センサ未確立で点灯                */
#define USE_MODE_LED    1
#define LED1_LAT        LATEbits.LATE11
#define LED2_LAT        LATEbits.LATE10

/* CLC を使うか。ヘッダの「★CLC について★」を参照。
 * CLC 出力ピンを使っていないなら 0 にして止めてよい。 */
#define USE_CLC         1

/* ISR の実行時間をスコープで測るとき 1 にする（RE7 が ISR 中 Hi）。
 * 予算は 60MIPS * 10us = 600 命令サイクル。 */
#define ISR_TIME_PIN    0
#define DBG_PIN_LAT     LATEbits.LATE7


/* =======================================================================
 *  グローバル変数
 * ======================================================================= */
    /* --- initPWM が参照する（添付の開ループと同じ名前で残す）--- */
    int         Deadtime = DEADTIME;
    long        Loop0    = 0;       /* ソフトスタートの待ちループ           */

    /* --- センシング --- */
    int         Vac      = 0;   /* ADCBUF0 - ADCBUF1  符号付きライン電圧    */
    int         Vrec     = 0;   /* ADCBUF0 + ADCBUF1  |vac| 全波整流        */
    int         Vo_adc   = 0;   /* ADCBUF2（モードで中身が変わる）          */
    int         Iin      = 0;   /* ADCBUF3 - ISENS_OFS  符号付き入力電流    */
    int         Ir       = 0;   /* 整流座標のインダクタ電流                 */
    int         half     = 0;   /* 1 = 正の半周期                           */

    /* --- PFC電圧ループ（50Hz）--- */
    int32_t     vo_sum   = 0;   /* 移動平均の積算                           */
    int         scan_cnt = 0;
    int         Vo_avg   = 0;   /* 商用1周期平均後の出力電圧 [ADCカウント]  */
    int32_t     Ev       = 0, Ev_1 = 0;
    int32_t     Uv       = 0;   /* 2^SH_V 倍で保持                          */
    int32_t     Ipk      = 0;   /* 電流振幅指令（乗算器の単位）             */
    int         Vref     = 0;   /* 出力電圧指令 [ADCカウント]               */
    int         vref_init = 1;  /* 1 = 次の平均確定で実測値から立ち上げる   */

    /* --- 電流ループ（100kHz）--- */
    int32_t     Ei       = 0, Ei_1 = 0;
    int32_t     Ui       = 0;   /* 2^SH_I 倍で保持                          */
    int         Iref     = 0;
    int         Dpi      = 0;
    int         Dff      = 0;
    int         Wout     = 0;   /* Q1/Q2 デューティ [PWMカウント]           */

    /* --- 位相シフト --- */
    int32_t     Ep       = 0, Ep_1 = 0;
    int32_t     Up       = 0;   /* 2^SH_PH 倍で保持                         */
    int         wPhase   = D1_FIX_CNT;

    /* --- モードと SW1 --- */
    int         mode       = MODE_OPEN;
    int         mode_blank = 0;
    int         sw_tick    = N_VLOOP/4;
    int         sw_now     = 0;
    int         sw_last    = 0;
    int         sw_stable  = 0;
    int         sw_state   = 0;

    /* --- 保護 --- */
    int         fault      = 0;     /* ソフト過電流ラッチ                   */
    int         fault_hw   = 0;     /* CMP1 がいま制限中か（診断用）        */
    int         ocp_cnt    = 0;
    int         sensor_ok  = 0;
    int         sens_cnt   = 0;
    int         pol_err    = 0;
    int         iin_trip   = 0;     /* ラッチした瞬間の電流                 */
    int         a3_trip    = 0;     /* ラッチした瞬間の ADCBUF3 生値        */
    unsigned int ocp_hits     = 0;
    unsigned int pol_err_hits = 0;
    unsigned int pwm5_hits    = 0;
    unsigned int isr_cnt      = 0;  /* 回っていれば増える                   */

    /* --- 校正・観測用（制御には影響しない）------------------------------
     *  商用周波数の系を瞬時値で観測しても何も分からない。デバッガで停止した
     *  瞬間は正弦波のランダムな位相であって代表値ではない。そこで商用1周期
     *  ごとに確定させる。*_hold はリセットするまで下がらない保持値
     *  （電源を切ってから安全に停止して読むため）。
     *
     *  ★wout_pk が DUTYmax のままなら、ループは制御していない。飽和している
     *    だけ。iref_pk と並べて見ると追従しているかが一目で分かる。      */
    int         cal_cnt    = N_VLOOP/2;
    /* ★3 つの「1/N_VLOOP 回」分岐を同じ回に重ねない★
     *   電圧ループ（scan_cnt）、観測値の確定（cal_cnt）、SW1 のポーリング
     *   （sw_tick）はどれも N_VLOOP 回に 1 回だけ重い処理をする。初期値を
     *   揃えると 3 つが必ず同じ回に集まり、その回だけ ISR が予算
     *   （60MIPS * 10us = 600 命令サイクル）に近づく。
     *   初期値をずらしておけば、重い回が 3 回に分散する。
     *   （観測値の窓が電圧ループの窓とずれるが、観測専用なので問題ない）*/
    int         vrec_pk = 0, vrec_pk_run = 0, vrec_hold = 0;
    int         vl_pk   = 0, vl_pk_run   = 0;
    int         vn_pk   = 0, vn_pk_run   = 0;
    int         wout_pk = 0, wout_pk_run = 0, wout_hold = 0;
    int         iref_pk = 0, iref_pk_run = 0, iref_hold = 0;
    int         iin_max = 0, iin_max_run = -32768, iin_hold_max = 0;
    int         iin_min = 0, iin_min_run =  32767, iin_hold_min = 0;
    int32_t     vo11_sum = 0;
    int         vo11_avg = 0, vo11_hold = 0;

/* レジスタ初期化 */
#define reg_CLEAR   (0x0000)


/* ===========================================================================
 *  メイン関数
 *    初期化の順番は添付の開ループと同じ。そのあとに
 *      ・保護の層を作り直す（比較器をサイクルバイサイクルにする）
 *      ・使わない PWM チャネルをピンに出さない
 *      ・フルブリッジを 50% 相補 + 位相差で回し始める
 *    を追加している。
 * ===========================================================================
 */
int main(void)
{
    initClock();                // Primary / Auxiliary オシレータ
    initCMP();                  // CMP1（この中では CMPON = 0 のまま）
    ClrWdt();
    initADC();                  // AN0/AN1/AN2(AN11)/AN3 を PWM4 トリガで変換
    ClrWdt();
    initIO();
#if (USE_CLC == 1)
    initCLC();
#endif
    ClrWdt();

    initPWM();                  // PTPER = PERIODmax、8ch 分の設定
    PTCONbits.PTEN = 1;         // PWM タイムベース開始
    ClrWdt();

    /* -------------------------------------------------------------------
     *  保護の層を作り直す
     *
     *  initPWM は全チャネルを FLTSRC = 13（CMP1）/ FLTMOD = 0（ラッチ）で
     *  設定している。しかし比較器をラッチにするとノイズ 1 発で装置全体が
     *  止まってしまう。比較器は連続監視でスパイクも拾うので、
     *  ★サイクルバイサイクル（FLTMOD = 1）にして該当周期だけ削る★のが正しい。
     *  ラッチ停止は ADC サンプリングを見るソフト側に持たせる。
     *
     *  また、入力側の過電流で後段のフルブリッジを止めると Cs1/Cs2 の
     *  エネルギーの逃げ場が無くなって過電圧になるので、PWM6/7 は
     *  Fault 無効（FLTMOD = 3）にして回し続ける。
     * ------------------------------------------------------------------- */
    FCLCON1bits.FLTMOD = 1;     // PWM1 (Q1) : サイクルバイサイクル
    FCLCON2bits.FLTMOD = 1;     // PWM2 (Q2) : 同
    FCLCON5bits.FLTMOD = 1;     // PWM5      : 通知専用（ラッチさせない）
    FCLCON6bits.FLTMOD = 3;     // PWM6 レグA : Fault 無効（段をまたいで止めない）
    FCLCON7bits.FLTMOD = 3;     // PWM7 レグB : 同

    /* 過電流しきい値を電流センサの中点基準で与える（添付は 1000 固定だった）*/
#if (USE_CMP_OCP == 1)
    CMP1DAC = CMP1DAC_OCP;
    CMP1CONbits.CMPON = 1;      // ★添付では無効のままだった。ここで有効化する
#else
    CMP1CONbits.CMPON  = 0;     // ハード保護を切る（ソフト保護だけで動かす）
    FCLCON1bits.FLTMOD = 3;     // Fault 無効。比較器の誤トリップで PWM が
    FCLCON2bits.FLTMOD = 3;     // 止まらないようにする
#endif

    /* -------------------------------------------------------------------
     *  デバッグモードで AN11（Vo_sense）を直接見たいときは、下の 2 行の
     *  コメントを外して閉ループ相当で起動する。通常は不要。
     * ------------------------------------------------------------------- */
    //ADCON4Hbits.C2CHS = 1;    // 0-AN2(ボリューム R12), 1-AN11(Vo_sense)
    //mode = MODE_CLOSED;

    /* ソフトスタート（電源が落ち着くまで待つ） */
    for (Loop0 = 0; Loop0 < 6000; Loop0++) { ClrWdt(); }

    /* -------------------------------------------------------------------
     *  PWM 出力ピンの有効化
     *    使うのは PWM1 / PWM2（Q1/Q2）と PWM6 / PWM7（フルブリッジ）だけ。
     *    PWM3 / PWM4 / PWM5 / PWM8 はピンに出さない（PEN = 0）。
     *    ★PEN はピンへの出力を切るだけで、PWM 生成ロジックは動いたまま。
     *      したがって PWM4 は ADC トリガ源として使えるし、CLC は内部信号
     *      として PWM3L / PWM4H / PWM8L を参照し続ける。
     * ------------------------------------------------------------------- */
    IOCON1bits.PENH  = 1;       // PWM1H = Q1 ゲート
    IOCON1bits.PENL  = 1;
    IOCON2bits.PENH  = 1;       // PWM2H = Q2 ゲート
    IOCON2bits.PENL  = 1;
    IOCON3bits.PENH  = 0;       // 未使用
    IOCON3bits.PENL  = 0;
    IOCON4bits.PENH  = 0;       // 未使用（ADC トリガ源としてのみ使う）
    IOCON4bits.PENL  = 0;
    IOCON5bits.PENH  = 0;       // 未使用（フォルト通知にのみ使う）
    IOCON5bits.PENL  = 0;
    IOCON6bits.PENH  = 1;       // PWM6H = Q3 / PWM6L = Q4（レグA）
    IOCON6bits.PENL  = 1;
    IOCON7bits.PENH  = 1;       // PWM7H = Q6 / PWM7L = Q5（レグB）
    IOCON7bits.PENL  = 1;
    IOCON8bits.PENH  = 0;       // 未使用
    IOCON8bits.PENL  = 0;

    /* フルブリッジは 50% 相補で回し続け、位相差だけを動かす */
    PDC6   = PERIODmax/2;
    PDC7   = PERIODmax/2;
    PHASE6 = 0;
    PHASE7 = D1_FIX_CNT;
    wPhase = D1_FIX_CNT;

    /* Q1/Q2 は止めた状態から始める */
    PDC1 = 0;
    PDC2 = 0;

    /* 表示 LED の初期状態。起動時は安全側の開ループ */
    LED2_LAT = 0;
#if (USE_MODE_LED == 1)
    LED1_LAT = 0;               // 消灯 = 開ループ
#endif
#if (ISR_TIME_PIN == 1)
    DBG_PIN_LAT = 0;
#endif

    /* -------------------------------------------------------------------
     *  RE4 の CN 割り込みは使わない（チャタリングで押下中に「離した」
     *  エッジを拾い、押す・離すで 2 回トグルして元に戻ることがある）。
     *  SW1 は制御 ISR の中で 20ms ごとにポーリングする。
     * ------------------------------------------------------------------- */
    CNENEbits.CNIEE4 = 0;
    IEC1bits.CNIE    = 0;
    IFS1bits.CNIF    = 0;

    IFS7bits.ADCAN2IF = 0;      // 制御 ISR（AN2 変換完了 = 100kHz）
    IEC7bits.ADCAN2IE = 1;

    IFS6bits.PWM5IF = 0;        // CMP1 トリップの通知
    IEC6bits.PWM5IE = 1;

    while (1)
    {
        ClrWdt();
    }
}


/* ===========================================================================
 *  モード切り替え（SW1 の押下が確定したときに制御 ISR から呼ばれる）
 * ===========================================================================
 */
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
         *   電圧ループの積分器を「いま出ている Ipk」に合わせておくと、
         *   切り替えた瞬間に電流指令が飛ばない。
         *   指令 Vref は次に移動平均が確定したところで実測値から立ち上げる。
         *   ★OPEN_MODE_DUTY = 1 の開ループでは Ipk を使っていないので
         *     Ipk = 0 から始まる。ツマミで出していたデューティはいったん
         *     捨てられ、電圧ループが実測の出力電圧から積み上げ直す。     */
        Uv        = Ipk << SH_V;
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
    LED1_LAT = mode;            // 閉ループで点灯
#endif
}


/* ===========================================================================
 *  RE4 の CN 割り込み（使っていない。念のためフラグだけ落とす）
 * ===========================================================================
 */
void __attribute__((__interrupt__, no_auto_psv)) _CNInterrupt(void)
{
    IFS1bits.CNIF = 0;
}


/* ===========================================================================
 *  制御本体
 *    AN2（または AN11）の変換完了割り込み = PWM4 トリガ = 100kHz ごと
 *
 *  並びは固定する。
 *    (1) センシング   (2) 極性判定   (3) 保護   (4) 外側ループ
 *    (5) 乗算器       (6) 電流PI     (7) FF加算・クランプ   (8) ゲート配分
 *
 *  ★(3) を (6) より前に置く★
 *    壊れた入力で制御器を回すと積分器が異常値を記憶する。一度巻き上がった
 *    積分器は、原因を取り除いたあともしばらく悪さをする。
 *
 *  ★途中で return しない★
 *    抜けると前回のデューティがレジスタに残り続ける。ハードウェアは
 *    止まらないので「制御は止まったのにスイッチングは続く」最悪の状態に
 *    なる。保護がかかったときもフラグを立てて最後まで流し、(8) で 0 を書く。
 * ===========================================================================
 */
void __attribute__((__interrupt__, no_auto_psv)) _ADCAN2Interrupt(void)
{
    int32_t lim;                /* アンチワインドアップの制限値 */
#if (USE_FF == 1)
    int32_t tmp;                /* フィードフォワードの一時変数 */
#endif
    int     use_loop;

#if (ISR_TIME_PIN == 1)
    DBG_PIN_LAT = 1;            /* 実行時間をスコープで測る（予算 600 cycle）*/
#endif
    isr_cnt++;                  /* 回っていれば増える                       */

    /* -------------------------------------------------------------------
     *  (1) センシング
     *      VL_sense / VN_sense はどちらも片側だけが立つので
     *        差 = 符号付きライン電圧、和 = |vac| 全波整流 になる。
     *      ※このクランプは電流が流れていないと成立しない。
     * ------------------------------------------------------------------- */
    Vac    = (int)ADCBUF0 - (int)ADCBUF1;
    Vrec   = (int)ADCBUF0 + (int)ADCBUF1;
    Vo_adc = ADCBUF2;                   /* モードで中身が変わる            */
    Iin    = (int)ADCBUF3 - ISENS_OFS;       /* 双方向センサ。中点を引いて符号付 */

    /* -------------------------------------------------------------------
     *  (2) 極性判定（ヒステリシス付き。零クロスでのチャタリング防止）
     * ------------------------------------------------------------------- */
    if (half) {
        if (Vac < -VAC_HYST) { half = 0; }
    } else {
        if (Vac >  VAC_HYST) { half = 1; }
    }

    /* 整流座標のインダクタ電流（負の半周期は符号反転）
     *  ★ここが符号規律の要★
     *    Iref は常に正（整流座標）なので Ir も同じ座標でなければならない。
     *    極性フラグが実際の半周期とずれると Ei = Iref - Ir の符号が反転し、
     *    負帰還が正帰還に変わる。 */
    if (half) { Ir =  Iin; } else { Ir = -Iin; }

    /* ★極性判定の誤りを検出する安全ネット★
     *  昇圧形はダイオードが逆流を阻止するので、整流座標の電流が大きく負に
     *  なることは物理的にありえない。そうなっていたら half が実際の半周期と
     *  合っていない。このまま回すとデューティが上限に張り付いて電流が
     *  発散するので、正帰還になる前に積分器をクリアしてゲートを止める。
     *  （「ゲインが高すぎる」とは全く違う壊れ方。デューティが上限で
     *    動かなくなるのが特徴。） */
    if (Ir < -SENS_OK_TOL) {
        pol_err = 1;
        pol_err_hits++;
        Ui   = 0;
        Ei_1 = 0;
    } else if (Ir > 0) {
        pol_err = 0;
    }

    /* -------------------------------------------------------------------
     *  (3) 保護
     * ------------------------------------------------------------------- */
    /* ハード側の状態は診断用に見るだけ（ここで fault は立てない）。
     * PWM1/2 はサイクルバイサイクルなので FLTSTAT は「いま制限中」の意味。*/
    fault_hw = PWMCON1bits.FLTSTAT | PWMCON2bits.FLTSTAT;

    /* センサの健全性（未給電を過電流と誤認しないため）。
     * 「N 回連続」ではなくアップダウンカウンタにして、たまのノイズでは
     * 少し戻るだけにする。 */
    if (sensor_ok == 0) {
        if ((Iin > -SENS_OK_TOL) && (Iin < SENS_OK_TOL)) {
            if (++sens_cnt >= N_VLOOP) { sensor_ok = 1; }
        } else {
            sens_cnt -= 8;
            if (sens_cnt < 0) { sens_cnt = 0; }
        }
    } else {
        if (ADCBUF3 < SENS_FLOOR) { sensor_ok = 0; sens_cnt = 0; }
    }

    /* ソフト過電流保護（両極性）。CMP1 は非反転なので負の半周期を守れない。
     * 単発ノイズでラッチしないよう 2 回連続（20us）で成立したときだけ
     * ラッチする。20us で進む電流は 141V/1mH で 1.4A なので許容範囲。
     * 本当に速い事象はハードの CMP1 が連続監視で捕まえる。 */
    if (sensor_ok && ((Iin > I_OCP_CNT) || (Iin < -I_OCP_CNT))) {
        ocp_hits++;
        if (++ocp_cnt >= 2) {
            if (fault == 0) {           /* 最初にラッチした瞬間だけ記録    */
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
        /* ラッチ。SW1 を押すまで Q1/Q2 を止めたままにする。
         * 積分器は全部クリアしておく（復帰した瞬間に最大デューティから
         * 動き出さないように）。 */
        Ui  = 0;  Ei_1 = 0;
        Uv  = 0;  Ev_1 = 0;
        Ipk = 0;
#if (FAULT_STOP_BRIDGE == 1)
        /* ★段をまたいで止めると Cs1/Cs2 のエネルギーの逃げ場が無くなって
         *   過電圧になる。既定ではここは無効。 */
        IOCON6bits.OVRDAT = 0b00;   IOCON7bits.OVRDAT = 0b00;
        IOCON6bits.OVRENH = 1;      IOCON6bits.OVRENL = 1;
        IOCON7bits.OVRENH = 1;      IOCON7bits.OVRENL = 1;
#endif
    }

    /* 赤 LED : 過電流ラッチ中、またはセンサが未確立のとき点灯 */
    LED2_LAT = (fault || (sensor_ok == 0));

    /* -------------------------------------------------------------------
     *  SW1 のポーリング（20ms ごと、2 回連続一致で確定、押下でトグル）
     *  フォルト中の押下はモード切替ではなくフォルトリセットにする。
     * ------------------------------------------------------------------- */
    if (++sw_tick >= SW_SAMPLE) {
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
            if (sw_state) {                 /* 押下の確定                  */
                if (fault) {
                    /* 原因を取り除いてから押すこと */
                    fault   = 0;
                    ocp_cnt = 0;
                    IOCON6bits.OVRENH = 0;  IOCON6bits.OVRENL = 0;
                    IOCON7bits.OVRENH = 0;  IOCON7bits.OVRENL = 0;
                    PWMCON1bits.FLTSTAT = 0;    /* ハード側のラッチも解除  */
                    PWMCON2bits.FLTSTAT = 0;
                    PWMCON5bits.FLTSTAT = 0;
                    IFS6bits.PWM5IF = 0;
                    IEC6bits.PWM5IE = 1;
                    LED2_LAT   = 0;
                    mode_blank = MODE_BLANK;
                } else {
                    toggle_mode();
                }
            }
        }
    }

    /* -------------------------------------------------------------------
     *  (4) 動作モードと外側ループ
     * ------------------------------------------------------------------- */
    use_loop = 1;

    if (mode == MODE_OPEN) {
        /* 閉ループ側の状態は持ち越さない */
        vo_sum   = 0;
        scan_cnt = 0;
        wPhase   = D1_FIX_CNT;

#if (OPEN_MODE_DUTY == 1)
        /* ツマミ 0..4095 -> デューティ 0..DUTYmax（添付の開ループと同じ）。
         * 電流ループと FF はバイパスし、積分器は止めておく。 */
        use_loop = 0;
        Wout = (int)(((int32_t)Vo_adc * DUTYmax) >> 12);
        Iref = 0;
        Dpi  = 0;
        Dff  = 0;
        Ipk  = 0;
        Ui   = 0;  Ei_1 = 0;
        Uv   = 0;  Ev_1 = 0;
#else
        /* ツマミ 0..4095 -> 電流振幅指令 0..IPK_OPEN_MAX。
         * 電流ループと FF は生きたままなので、入力電流は正弦波の形を保った
         * まま振幅だけが変わる。 */
        Ipk  = ((int32_t)Vo_adc * IPK_OPEN_MAX) >> 12;
        Uv   = 0;  Ev_1 = 0;
#endif
    }
    else {
        /* ---------------------------------------------------------------
         *  出力電圧の移動平均（窓幅 = 商用1周期ちょうど = 20ms）
         *    窓幅が線周期の整数倍のとき、その周波数とすべての高調波で
         *    ゲインが厳密に 0 になる。一次 LPF で同じ減衰を得ようとすると
         *    はるかに大きな遅れを払うことになる。
         *    積算と間引きを同じカウンタでやるので、窓幅とループ周期が
         *    自動的に一致する（別々に持つとずれたときにリプルが漏れる）。
         * --------------------------------------------------------------- */
        vo_sum += Vo_adc;
        if (++scan_cnt >= N_VLOOP) {
            scan_cnt = 0;
            Vo_avg   = BOXCAR(vo_sum);
            vo_sum   = 0;

            /* 閉ループに入った直後は、そのときの出力電圧から指令を立ち上げる
             * （いきなり 48V を指令すると電流が飛ぶ） */
            if (vref_init) { Vref = Vo_avg; vref_init = 0; }

            /* ソフトスタート：指令を VREF_SLEW_VPS でランプアップ */
            if (Vref < VO_REF_CNT) {
                Vref += VREF_STEP_CNT;
                if (Vref > VO_REF_CNT) { Vref = VO_REF_CNT; }
            }

            /* --- PFC電圧ループ（50Hz）-> 電流の振幅指令 Ipk --- */
            Ev   = (int32_t)Vref - (int32_t)Vo_avg;
            Uv   = Uv + KV_A*Ev - KV_B*Ev_1;
            Ev_1 = Ev;

            /* アンチワインドアップ：出せる範囲は 0 〜 IPK_MAX_MUL */
            lim = IPK_MAX_MUL << SH_V;
            if (Uv > lim) { Uv = lim; }
            if (Uv < 0)   { Uv = 0;   }
            Ipk = Uv >> SH_V;

            /* --- 位相シフト量 --- */
#if (PH_MODE == 0)
            wPhase = D1_FIX_CNT;
#else
            Ep   = (int32_t)Vref - (int32_t)Vo_avg;
            Up   = Up + KPH_A*Ep - KPH_B*Ep_1;
            Ep_1 = Ep;
            lim = (int32_t)D1_TRIM_CNT << SH_PH;
            if (Up >  lim) { Up =  lim; }
            if (Up < -lim) { Up = -lim; }
            wPhase = D1_FIX_CNT + (int)(Up >> SH_PH);
#endif
            if (wPhase > PHASEmax) { wPhase = PHASEmax; }
            if (wPhase < PHASEmin) { wPhase = PHASEmin; }
        }
    }

    if (use_loop) {
        /* ---------------------------------------------------------------
         *  (5) 乗算器 -> 電流指令
         *      修論 4.2：制御した値に入力電圧の絶対値を掛けて基準電流を作る。
         *      入力電流を入力電圧と相似にする = 力率を 1 にする仕掛け。
         * --------------------------------------------------------------- */
        Iref = (int)(((int32_t)Vrec * Ipk) >> SH_MUL);
        if (Iref > ADC_FS) { Iref = ADC_FS; }

        /* ---------------------------------------------------------------
         *  (6) 電流ループ（100kHz）= 平均電流制御
         * --------------------------------------------------------------- */
        Ei = (int32_t)Iref - (int32_t)Ir;
        /* 偏差はフルスケールに収める（異常時はレールまで振れる） */
        if (Ei >  ADC_FS) { Ei =  ADC_FS; }
        if (Ei < -ADC_FS) { Ei = -ADC_FS; }

        Ui   = Ui + KI_A*Ei - KI_B*Ei_1;
        Ei_1 = Ei;

        /* アンチワインドアップ ★USE_FF と必ず連動させる★
         *  片方だけ変えるとデューティが途中で頭打ちになる。エラーも警告も
         *  出ないので、波形を見るまで気づけない種類のバグになる。 */
#if (USE_FF == 1)
        /* FF が定常分を出すので PI は補正分だけ受け持つ。
         * ★キャストは溢れる演算より前に置く★
         *   XC16 の int は 16bit。(int32_t)(PERIODmax*6/10) と書くと
         *   9600*6 = 57600 が int16 で桁あふれして -7936 に化け、lim が
         *   負になって PI が何も出力できなくなる。 */
        lim = ((int32_t)PERIODmax*6/10) << SH_I;
        if (Ui >  lim) { Ui =  lim; }
        if (Ui < -lim) { Ui = -lim; }
#else
        /* FF が無いと PI 出力がそのままデューティになるので、0 〜 DUTYmax の
         * 全域を使えないと必要な昇圧比に届かない
         * （20V 入力で 48V を出すにはデューティ 85% が要る）。 */
        lim = (int32_t)DUTYmax << SH_I;
        if (Ui > lim) { Ui = lim; }
        if (Ui < 0)   { Ui = 0;   }
#endif
        Dpi = (int)(Ui >> SH_I);

        /* ---------------------------------------------------------------
         *  (7) フィードフォワード（昇圧則）
         *      D = 1 - |vac|/Vcs  を先に与えると PI の負担が減り、
         *      電圧ピーク付近の追従遅れによる歪みが小さくなる。
         * --------------------------------------------------------------- */
#if (USE_FF == 1)
        tmp = ((int32_t)Vrec * KFF) >> SH_FF;
        if (tmp > PERIODmax) { tmp = PERIODmax; }
        Dff = PERIODmax - (int)tmp;
#else
        Dff = 0;
#endif
        Wout = Dff + Dpi;
    }

    if (Wout > DUTYmax) { Wout = DUTYmax; }
    if (Wout < DUTYmin) { Wout = DUTYmin; }

    /* モード切替直後は Q1/Q2 を止める（ADC のチャネルが切り替わった直後の
     * 値で電流を流さないため）。フルブリッジは動かしたままにする。 */
    if (mode_blank > 0) {
        mode_blank--;
        Wout = 0;
        Ui   = 0;  Ei_1 = 0;
    }

    /* 零クロス近傍は極性が確定しないので PFC を止める */
    if ((Vac < VAC_HYST) && (Vac > -VAC_HYST)) { Wout = 0; }

    /* ゲートを止める理由が立っているなら積分も止める。
     * 「指令は出しているのに電流が返ってこない」状態で積分を続けると
     * 上限まで巻き上がり、ゲートが復活した瞬間に最大デューティから
     * 動き出して大きなサージになる。 */
    if (fault || (sensor_ok == 0) || pol_err) {
        Wout = 0;
        Ui   = 0;  Ei_1 = 0;
    }

    /* -------------------------------------------------------------------
     *  (8) ゲート配分
     *      正の半周期 : Q1 が PWM、Q2 はオフ / レグA 基準・レグB 遅れ
     *      負の半周期 : Q2 が PWM、Q1 はオフ / レグB 基準・レグA 遅れ
     *
     *      ★極性判定 (2) と同じ half から導く★
     *        別々の条件式で書くと、零クロス付近で「ゲートは正の半周期なのに
     *        電流の符号は負の半周期」という瞬間が生まれる。
     *
     *      ★相補モード（PMOD = 0）なので PDC = 0 のときロー側が 100% オン
     *        になる。休ませたい側のスイッチを常時オフにしたいなら、ロー側の
     *        出力を配線してはいけない（配線すると同期整流動作になる）。
     *        スコープで「信号が出ていない」と判断するときも、ハイ側だけで
     *        なくロー側も見ること。
     * ------------------------------------------------------------------- */
    if (half) {
        PDC1 = Wout;
        PDC2 = 0;
    } else {
        PDC1 = 0;
        PDC2 = Wout;
    }

#if (PS_SWAP == 1)
    if (half) { PHASE6 = 0;       PHASE7 = wPhase; }   /* レグB が位相遅れ */
    else      { PHASE7 = 0;       PHASE6 = wPhase; }   /* レグA が位相遅れ */
#else
    PHASE6 = 0;
    PHASE7 = wPhase;
#endif

    /* -------------------------------------------------------------------
     *  校正・観測用（制御には影響しない）
     *    商用1周期ごとに代表値を確定させる。瞬時値を読んでも意味が無い。
     * ------------------------------------------------------------------- */
    if (Wout > wout_pk_run)      { wout_pk_run = Wout;    }
    if (Iref > iref_pk_run)      { iref_pk_run = Iref;    }
    if (Iin  < iin_min_run)      { iin_min_run = Iin;     }
    if (Iin  > iin_max_run)      { iin_max_run = Iin;     }
    if (Vrec > vrec_pk_run)      { vrec_pk_run = Vrec;    }
    if ((int)ADCBUF0 > vl_pk_run) { vl_pk_run  = ADCBUF0; }
    if ((int)ADCBUF1 > vn_pk_run) { vn_pk_run  = ADCBUF1; }
    vo11_sum += ADCBUF11;       /* 開ループ中でも Vo を読めるようにする    */

    if (++cal_cnt >= N_VLOOP) {
        cal_cnt  = 0;
        vrec_pk  = vrec_pk_run;   vrec_pk_run = 0;
        vl_pk    = vl_pk_run;     vl_pk_run   = 0;
        vn_pk    = vn_pk_run;     vn_pk_run   = 0;
        wout_pk  = wout_pk_run;   wout_pk_run = 0;
        iref_pk  = iref_pk_run;   iref_pk_run = 0;
        iin_max  = iin_max_run;   iin_max_run = -32768;
        iin_min  = iin_min_run;   iin_min_run =  32767;
        vo11_avg = BOXCAR(vo11_sum);            vo11_sum = 0;

        /* リセットするまで下がらない保持値 */
        if (vrec_pk  > vrec_hold)    { vrec_hold    = vrec_pk;  }
        if (wout_pk  > wout_hold)    { wout_hold    = wout_pk;  }
        if (iref_pk  > iref_hold)    { iref_hold    = iref_pk;  }
        if (vo11_avg > vo11_hold)    { vo11_hold    = vo11_avg; }
        if (iin_max  > iin_hold_max) { iin_hold_max = iin_max;  }
        if (iin_min  < iin_hold_min) { iin_hold_min = iin_min;  }

        /* CMP1 トリップ通知を戻す（ISR 内で自分を止めているので） */
        IFS6bits.PWM5IF = 0;
        IEC6bits.PWM5IE = 1;
    }

#if (ISR_TIME_PIN == 1)
    DBG_PIN_LAT = 0;
#endif
    IFS7bits.ADCAN2IF = 0;
}


/* ===========================================================================
 *  使っていない ADC 割り込み（フラグだけ落とす）
 * ===========================================================================
 */
void __attribute__((__interrupt__, no_auto_psv)) _ADCAN0Interrupt(void)
{
    IFS6bits.ADCAN0IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _ADCAN1Interrupt(void)
{
    IFS6bits.ADCAN1IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _ADCAN3Interrupt(void)
{
    IFS7bits.ADCAN3IF = 0;
}

/* ===========================================================================
 *  CMP1 トリップの通知（PWM5 の Fault 割り込み）
 *
 *  ★添付の開ループはここで while(1) に入って停止していた★
 *    無限ループに入ると ClrWdt も SW1 のポーリングも効かなくなり、
 *    原因の確認もリセットもできない。ここでは回数を数えるだけにして、
 *    自分の割り込みだけ止める（毎周期鳴って ISR を埋めないように）。
 *    50Hz ごとに制御 ISR が戻してくれる。
 *    止めるかどうかの判断はソフト過電流保護（第2層）が行う。
 * ===========================================================================
 */
void __attribute__((__interrupt__, no_auto_psv)) _PWM5Interrupt(void)
{
    pwm5_hits++;
    IEC6bits.PWM5IE = 0;
    IFS6bits.PWM5IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
    IFS0bits.T2IF = 0;
}


/* ===========================================================================
 *  ここから下は周辺初期化。添付の開ループ（Buck_OPENPI.c）から引き継いでいる。
 *
 *  ★開ループからの変更点（このファイル全体）★
 *
 *   項目                 開ループ            このファイル       理由
 *   -------------------  ------------------  -----------------  ----------------
 *   Q1/Q2 のデューティ   PDC3 / PDC4         PDC1 / PDC2        実機の配線
 *   PWM3/PWM4            PFC デューティ      変調しない         PWM4 は ADC
 *                                                               トリガ源として残す
 *   TRIG4                0                   2400 (周期の25%)   周期境界はリンギング
 *                                                               を読む最悪点
 *   DTR6/7, ALTDTR6/7    Deadtime = 400      DEADTIME = 600     ドライバの 500ns に
 *                                                               合わせる (625ns)
 *   PDC8                 DUTYmax             0                  PWM8 未使用
 *   PENH/PENL            全 ch = 1           1/2/6/7 のみ 1     未使用 ch をピンに
 *                                                               出さない
 *   CMP1                 CMPON = 0 (無効)    CMPON = 1          ★開ループでは
 *                                                               ハード過電流保護が
 *                                                               効いていなかった
 *   CMP1DAC              1000 (固定値)       ISENS_OFS + I_OCP  センサ中点基準
 *   FCLCON1/2 FLTMOD     0 (ラッチ)          1 (サイクル毎)     ノイズ1発で装置が
 *                                                               止まらないように
 *   FCLCON6/7 FLTMOD     0 (ラッチ)          3 (無効)           段をまたいで
 *                                                               止めない
 *   _PWM5Interrupt       while(1) で停止     回数を数えるだけ   停止すると ClrWdt も
 *                                                               SW1 も効かなくなる
 *   _CNInterrupt (RE4)   C2CHS を 0 に戻す   フラグだけ落とす   SW1 はポーリング
 *   ADCBUF2 の中身       AN2 固定            モードで AN2/AN11  閉ループで Vo_sense
 *
 *  ★CLC は添付のまま（USE_CLC = 1）★
 *    CLC2 は PWM3L / PWM8L、CLC3 は PWM3H / PWM4H、CLC4 は PWM7H / PWM2H を
 *    入力にしている。Q1/Q2 を PWM1/PWM2 に移したことで、PWM3 / PWM8 は
 *    変調されなくなり、逆に PWM2H が新たに変調されるようになる。
 *    => CLC1/CLC2 の出力ピン（RP36 / RP33 / RP76 / RP34）を使っているなら、
 *       通電前にスコープで波形を確認すること。使っていなければ
 *       USE_CLC を 0 にして CLC を止めてよい。
 * ===========================================================================
 */

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
    LATE   = 0x002B;
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
    FCLCON1bits.FLTMOD = 0;         // Fault (latched condition)
    

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
    FCLCON2bits.FLTMOD = 0;         // Fault (latched condition)
    

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
    DTR3    = 200;
    ALTDTR3 = 200;
				  
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

    TRIG4   = 2400;                   // ★ADC トリガ位置を周期の 25% に置く
                                      //   周期境界(0)はターンオンのリンギングを
                                      //   電流値として読む最悪点。オン期間の
                                      //   中ほどなら CCM で平均電流の良い近似。
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
    FCLCON6bits.FLTMOD = 0;         // Fault (latched condition)

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
    FCLCON7bits.FLTMOD = 0;         // Fault (latched condition)
    

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
    DTR8    = Deadtime;
    ALTDTR8 = Deadtime;
				  
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
    PDC8    = 0;                      // ★PWM8 は未使用（ピンにも出さない）
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

    ADCON4Hbits.C0CHS   = 0;    // 0-AN0, 1-AN7, 2-PGA1, 3-AN0ALT
    ADCON4Hbits.C1CHS   = 0;    // 0-AN1, 1-AN18, 2-PGA2, 3-AN1ALT    
    ADCON4Hbits.C2CHS   = 0;    // 0-AN2, 1-AN11, 2-VREF, 3-Resirve
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
    
    CMP1DAC = CMP1DAC_OCP;  // ★電流センサの中点基準でしきい値を与える
   
    CMP1CONbits.CMPON = 0;  //Disenable CMP1
    //CMP1CONbits.CMPON = 1;  //Enable CMP1
    
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

