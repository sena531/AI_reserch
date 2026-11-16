
/*
 *  開発環境 : MPLAB X IDE v5.35
 *             XC16 (v1.41)
 *             dsPIC33EP128GS808 (DM330026)
 * 
 *  動作電圧 3.3V
 *  動作周波数 Fcy = 60MHz,  Fosc =119.7MHz
 *  スイッチング周波数 100kHz (10us)
 *  PWM出力はPWM2ピン
 * 
 *  開ループモードと閉ループモードの2つのモードがある
 *  モードの切り替えはSW1を押す（正確には押して離す）ことで行う
 *  
 *  1.開ループモード (LED1は消灯)
 *  　R12のツマミを回すことでduty比を調整
 * 
 *  2.閉ループモード (LED1が点灯)
 *  　12bitADC(J22の真ん中のピン AN11)を使う
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

	/*	グローバル変数宣言	*/
    int     Vref = 3724 ;    //目標電圧 (4095*3)/3.3
    int     DUTYmax = 4600 ;    //50%
    int     DUTYmin = 480 ;     //5%
    int     PHASEmax = 4800 ;    //180°
    int     PHASEmin = 0 ;       //0°
    int     adcresult = 0;
    int     PERIODmax  = 9600;  // 100KHz
    
    int32_t     Et1 = 0;    /* 電圧今回偏差 */
    int32_t     Et2 = 0;    /* 電圧前回偏差 */
    int32_t     Et3 = 0;    /* 電流今回偏差 */
    int32_t     Et4 = 0;    /* 電流前回偏差 */
    int32_t     Et5 = 0;    /* 位相今回偏差 */
    int32_t     Et6 = 0;    /* 位相前回偏差 */
    int32_t     Ut1 = 0;    /* 電圧今回制御量 */
    int32_t     Ut2 = 0;    /* 電圧前回制御量 */
    int32_t     Ut3 = 0;    /* 電流今回制御量 */
    int32_t     Ut4 = 0;    /* 電流前回制御量 */
    int32_t     Ut5 = 0;    /* 位相今回制御量 */
    int32_t     Ut6 = 0;    /* 位相前回制御量 */

    int32_t     Kp = 344;   //Kp << 8
    int32_t     Kp1 = 3;
    int32_t     K1 = 347;   //{(Ki*T)/2 + Kp} << 20
    int32_t     K2 = 343;   //{(Ki*T)/2 - Kp} << 20
    int32_t     K3 = 67;    //{(Ki*T)/2 + Kp} << 10
    int32_t     K4 = 65;    //{(Ki*T)/2 - Kp} << 10
    int32_t     K5 = 100;   //{(Ki*T)/2 + Kp} << 20
    int32_t     K6 = 99;    //{(Ki*T)/2 - Kp} << 20
    
    int     Wout;   //PFC制御出力
    int     wPhase; //位相シフト出力
    int     Vac;    //AC入力電圧
    int     Iin;    //入力電流
    int     Iref;   //参照電流
    int     Vdc;    //全波整流
    int     Vout;
    int     Deadtime = 600 ;    //Dead Time
    int     sw = 1;
    
    
    
/*	レジスタ初期化	*/
#define	reg_CLEAR	(0x0000)

/*	=================	*/
/*	メイン関数		    */
/*	=================	*/
//初期設定してるだけだからこの関数の内容はガン無視していい
int main(void)
{
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
    CMP1DAC =  1000;            // 過電流保護
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
    IOCON8bits.PENH  = 1;       // PWM8H is controlled by PWM module
    IOCON8bits.PENL  = 1;       // PWM8L is controlled by PWM module
    
//    for(Loop0=0 ; Loop0<6000 ; Loop0++ ){  ClrWdt();  }
    
	    
    CNENEbits.CNIEE4 = 1;
    IEC1bits.CNIE = 1;
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
//RE4の割り込み
//モード切り替えを行う
void __attribute__ ((__interrupt__, no_auto_psv)) _CNInterrupt(void)
{
    ADCON4Hbits.C2CHS   = 0;    // 0-AN2, 1-AN11
    
    IFS1bits.CNIF = 0; // Clear CN interrupt
}

//ADCが変換終了後に発生する割り込み
//2つのモードの処理が書かれている
void __attribute__((__interrupt__, no_auto_psv))_ADCAN2Interrupt()
//void __attribute__((__interrupt__, no_auto_psv, shadow))_ADCAN0Interrupt()
{
    /* Vin */
    Vac = ADCBUF0 - ADCBUF1 ;   //AC入力電圧
    Vdc = ADCBUF0 + ADCBUF1 ;   //全波整流
    
    /* Iin */
    if( ADCBUF3 >= 2048 ){  Iin = ADCBUF3 -2048 ;   }
    else{   Iin = -ADCBUF3 + 2048 ; }
    
    /*演算*/
    //出力電圧ループ
    adcresult = ADCBUF2;    //AN2, 11を使用
    Et1=Vref-adcresult;
    Ut1 = Ut2 + K1*Et1 - K2*Et2;
    Ut2 = Ut1;
    Et2 = Et1;
    Vout = Ut1>>20;
    //電流ループ
    Iref = Vdc * Vout ;
    Et3 = Iref - Iin ;
    Ut3 = Ut4 + K3*Et3 - K4*Et4;
    Ut4 = Ut3;
    Et4 = Et3;
    Wout = Ut3>>16;
    //位相シフト用        
    Et5=Vref-adcresult;
    Ut5 = Ut6 + K5*Et5 - K6*Et6;
    Ut6 = Ut5;
    Et6 = Et5;
    wPhase = Ut5>>20;
    /* リミットチェック */
    if( Wout >= DUTYmax){ Wout = DUTYmax; }
    if( Wout <= DUTYmin){ Wout = DUTYmin; }
    if( wPhase >= PHASEmax){ wPhase = PHASEmax ;}
    if( wPhase <= PHASEmin){ wPhase = PHASEmin ;}
    
    if( Vac >= 20 ){
        PDC1 = Wout ;
        PDC2 = 0 ;
        PHASE6 = 0 ;
        PHASE7 = 9424 - wPhase;
    }
    else if( Vac < 20 && Vac >= 0 ){
        PDC1 = 0 ;
        PDC2 = 0 ;
        PHASE6 = 0 ;
        PHASE7 = 9424 - wPhase;
    }
    else if( Vac <= -20 ){
        PDC1 = 0 ;
        PDC2 = Wout ;
        PHASE7 = 0 ;
        PHASE6 = 9424 - wPhase;
    }
    else{
        PDC1 = 0 ;
        PDC2 = 0 ;
        PHASE7 = 0 ;
        PHASE6 = 9424 - wPhase;
        }    
 
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

void __attribute__((__interrupt__, no_auto_psv))_PWM5Interrupt()
{
//        LATEbits.LATE7 = 1-  LATEbits.LATE7; // Debug  ++++++++++ 

    LATEbits.LATE10 = 1;    // LED2 : RE10= Hi
    
    while(1);
    
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

    TRIG4   = 0;                      // Set Initial Trigger location
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
    ADCON4Hbits.C2CHS   = 1;    // 0-AN2, 1-AN11, 2-VREF, 3-Resirve
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
    
    CMP1DAC = 4000;
   
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