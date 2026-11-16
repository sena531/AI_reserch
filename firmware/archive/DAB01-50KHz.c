
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
	int     wPhase = 0;
    int     delay;
	int     delay1;
	long	CH1_Loop0 = 0 ;	/*	カウンター	*/
  	long	CH2_Loop0 = 0 ;	/*	カウンター	*/
    
    int     OUTphase;
	unsigned int     VIN;
	unsigned int     IIN;
    unsigned int     D6 = 0;

    
/*	レジスタ初期化	*/
#define	reg_CLEAR	(0x0000)

/*	=================	*/
/*	メイン関数		    */
/*	=================	*/
int main(void)
{
    initClock();            // Initialize Primary and Auxiliary oscillators
    initPGA(); 
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
    PTCONbits.PTEN = 1;        // Enable the PWM
//    PTCONbits.SEVTPS = 1       //

    
//    for(CH1_Loop0=0; CH1_Loop0<6000; CH1_Loop0++  ){
//        ClrWdt();
//   }
    
    IOCON1bits.PENH  = 1;       // PWM1H is controlled by IO
    IOCON1bits.PENL  = 1;       // PWM1L is controlled by IO
    IOCON2bits.PENH  = 0;       // PWM2H is controlled by IO
    IOCON2bits.PENL  = 0;       // PWM2L is controlled by IO
    IOCON3bits.PENH  = 0;       // PWM3H is controlled by IO
    IOCON3bits.PENL  = 0;       // PWM3L is controlled by IO
    IOCON4bits.PENH  = 1;       // PWM4H is controlled by IO
    IOCON4bits.PENL  = 1;       // PWM4L is controlled by IO
    IOCON5bits.PENH  = 1;       // PWM5H is controlled by PWM module
    IOCON5bits.PENL  = 1;       // PWM5L is controlled by PWM module
    IOCON6bits.PENH  = 1;       // PWM6H is controlled by PWM module
    IOCON6bits.PENL  = 1;       // PWM6L is controlled by PWM module
    IOCON7bits.PENH  = 1;       // PWM7H is controlled by PWM module
    IOCON7bits.PENL  = 1;       // PWM7L is controlled by PWM module
    IOCON8bits.PENH  = 1;       // PWM8H is controlled by PWM module
    IOCON8bits.PENL  = 1;       // PWM8L is controlled by PWM module
    
	/*	変数初期化	*/

	CH1_Loop0 = 0 ;
	CH2_Loop0 = 0 ;


    IFS7bits.ADCAN2IF = 0;		// Clear ADCAN0 interrupt flag.
    IEC7bits.ADCAN2IE = 1;	    // Enable ADC AN0 interrupt      

//    IFS0bits.T2IF     = 0;      // Clear Timer2 interrupt flag
//    IEC0bits.T2IE     = 1;      // Enable Timer2 interrupt
    IFS8bits.CLC4IF     = 1;      // Clear CLC4 interrupt flag
    IEC8bits.CLC4IE     = 1;      // Enable CLC4 interrupt    
//    LATEbits.LATE8 = 1;
    

	while(1)
	{
    ClrWdt();
//    LATDbits.LATD6 = CLC4CONLbits.LCOUT;
//    LATDbits.LATD6 = 1;
    }
}

void __attribute__((__interrupt__, no_auto_psv))_ADCAN2Interrupt()
//void __attribute__((__interrupt__, no_auto_psv, shadow))_ADCAN0Interrupt()
{
//   LATEbits.LATE15  = 1;  // Debug  ++++++++++
    IFS7bits.ADCAN2IF = 0;		// Clear ADCAN0 interrupt flag.
    
    OUTphase  = ( ADCBUF2 << 2 ) - 8192 ;

    PHASE6 = 0 ;
    wPhase = 19200 - OUTphase;
        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
    PHASE7 = wPhase;

//    if( OUTphase >= 4800 ){
        PHASE5 = 0;
        SPHASE1 = 0;
        PDC5 = 0;
        PDC1 = 0;
        SPHASE5 = 0;
        SDC5 =  0;
        
        PHASE8 = 0;
        PDC8 = 0;
        SPHASE8 = 0;
        SDC8 =  0;
//    }
    
        if(0 <= OUTphase && OUTphase <= 4800){
        PDC5 = 9600 - OUTphase - OUTphase ;
        PDC1 = 9600 - OUTphase + 480;
        SDC5 = 9600 - OUTphase - OUTphase ;
        PDC8 = 9600 - OUTphase - OUTphase ;
        PDC4 = 9600  + 480;
        SDC8 = 9600 - OUTphase - OUTphase ;
        PHASE5 = 19200 - OUTphase - 480;
        DTR1 =           OUTphase + 480;
        SPHASE5 = 9600 - OUTphase - 480;
        PHASE8 = 19200 - OUTphase - 480 - OUTphase;
        DTR4 =           OUTphase + OUTphase + 480 ;
        wPhase = 19200 + 9600 - OUTphase - 480 - OUTphase;
        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
        SPHASE8 = wPhase ;
    }

    if(-4800 <= OUTphase && OUTphase <= 0){
        wPhase = 19200 + 9600 + OUTphase + OUTphase ;
        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
        PDC5 = wPhase;
        PDC1 = 9600 + OUTphase + 480;
        SDC5 = wPhase;
        PDC8 = wPhase;
        SDC8 = wPhase;
        wPhase = 9600 + OUTphase + OUTphase + OUTphase + 480;
        if ( wPhase<= 0 ){ wPhase = 0; }
        PDC4 = wPhase;
        
        wPhase = 19200 + OUTphase - 480;
//        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
        PHASE5 = wPhase;
        DTR1= -OUTphase + 480;
        wPhase = 9600 + OUTphase - 480;
//        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
        SPHASE5 = wPhase;
        wPhase = 19200 + OUTphase - 480 - OUTphase;
//        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
        PHASE8 =wPhase;
        wPhase = 9600 + OUTphase - 480 - OUTphase;
        DTR4= -OUTphase + 480 ;
//        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
        wPhase = 9600  - 480 ;
        if ( wPhase>= 19200 ){ wPhase = wPhase-19200; }
        SPHASE8 = wPhase ;
    }
/*     
    if(OUTphase <= 4800){
        PHASE5 = 0;
        PDC5 = 0;
        SPHASE5 = 0;
        SDC5 =  0;
        
        PHASE8 = 0;
        PDC8 = 0;
        SPHASE8 = 0;
        SDC8 =  0;
    }
*/
    
//    LATEbits.LATE15  = 0;        // Debug  ++++++++++ 
//    ADSTATL = 0;
    IFS7bits.ADCAN2IF = 0;		// Clear ADCAN0 interrupt flag.
}

/*
void __attribute__((__interrupt__, no_auto_psv))_ADCAN1Interrupt()
//void __attribute__((__interrupt__, no_auto_psv, shadow))_ADCAN1Interrupt()
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
    IFS6bits.ADCAN1IF = 0;		// Clear ADCAN1 interrupt flag.
}
*/

void __attribute__((__interrupt__, no_auto_psv))_CLC4Interrupt()
{
//    LATEbits.LATE15  = 1;  // Debug  ++++++++++
    LATEbits.LATE15 = CLC4CONLbits.LCOUT;

//    LATEbits.LATE15  = 0;  // Debug  ++++++++++
    IFS8bits.CLC4IF = 0;
}

void __attribute__((__interrupt__, no_auto_psv))_T2Interrupt()
{
//        LATEbits.LATE8 = 1-  LATEbits.LATE8; // Debug  ++++++++++ 
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

    TRISB  = 0xFFFF;
    LATB   = 0x0000;
    ODCB   = 0x0000;
    CNENB  = 0x0000;
    CNPUB  = 0x0000;
    CNPDB  = 0x0000;
    ANSELB = 0x0001;
 
    TRISC  = 0xFFFF;
    LATC   = 0x0000;
    ODCC   = 0x0000;
    CNENC  = 0x0000;
    CNPUC  = 0x0000;
    CNPDC  = 0x0000;
    ANSELC = 0x0200;
    
//    TRISD  = 0xFFFF;
    TRISD  = 0xFFEF;
    LATD   = 0x0000;
    ODCD   = 0x0000;
    CNEND  = 0x0000;
    CNPUD  = 0x0000;
    CNPDD  = 0x0000;
    ANSELD = 0x0000;

    TRISE  = 0x7FFF;
    LATE   = 0x0000;
    ODCE   = 0x0000;
    CNENE  = 0x0000;
    CNPUE  = 0x0000;
    CNPDE  = 0x0000;
    ANSELE = 0x0000;

//***************************
// Assign CLC1 To Pin RP62
    RPOR16bits.RP62R   = 0x3F; // RP62 / CLC1(011 1111)

// Assign CLC3 To Pin RP19
    RPOR1bits.RP19R   = 0x41; // RP19 / CLC3(100 0001)

// Assign CLC2 To Pin RP45
    RPOR8bits.RP45R   = 0x40; // RP45 / CLC2(100 0000)

// Assign CLC4 To Pin RP46
    RPOR9bits.RP46R   = 0x42; // RP46 / CLC4(100 0010)

// Assign PWM8H To Pin RP51
    RPOR11bits.RP51R   = 0x3B; // RP51 / PWM8H(011 1101)



//***************************    
// Assign FLT3 To Pin RP20
//    RPINR13bits.FLT3R   = 20; // FLT3  /RP20
    
// Assign CLCINA To Pin RP19
//    RPINR45bits.CLCINAR   = 19; // CLCINA  /RP19
    
// Assign FLT4 To Pin RP19
//    RPINR13bits.FLT4R   = 19; // FLT4  /RP19
    
// Assign FLT1 To Pin RP67
//    RPINR12bits.FLT1R   = 67; // FLT1  /RP67
    
// Assign FLT2 To Pin RP44
//    RPINR12bits.FLT2R   = 44; // FLT3  /RP44

// Assign FLT5 To Pin RP45
//    RPINR42bits.FLT5R   = 45; // FLT5  /RP45

// Assign FLT6 To Pin RP46
//    RPINR42bits.FLT6R   = 46; // FLT6  /RP46

// Assign FLT7 To Pin RP176
//    RPINR43bits.FLT7R   = 68; // FLT7  /RP176 PWM6H

// Assign FLT8 To Pin RP60
//    RPINR43bits.FLT8R   = 70; // FLT8  /RP60 PWM5Hpin    
// Assign CLCINB To Pin RP68
//    RPINR46bits.CLCINBR   = 68; // CLCINB  /RP68 PWM6H


}

void initPWM(void)
{
// Setup PWM resolution and Period
    PTCON2bits.PCLKDIV = 0; // PWM Resolution
    PTCONbits.EIPU = 0;   
    PTPER = 19200;    // Switching Frequency
    
    IOCON1 = 0;
    IOCON2 = 0;
    IOCON3 = 0;
    IOCON4 = 0;
    
    IOCON6bits.PENH = 0;            // GPIO controls I/O port
    IOCON6bits.PENL = 0;           

    IOCON6bits.PMOD = 0;            // Complementary Mode
    
    IOCON6bits.POLH = 0;            // Drive signals are active-high
    IOCON6bits.POLL = 0;            // Drive signals are active-high

    IOCON6bits.OVRENH = 0;          // Override disabled
    IOCON6bits.OVRENL = 0;	  
    IOCON6bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON6bits.FLTDAT = 0b01;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON6bits.CLDAT = 0b10;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON6bits.DTC   = 0;          // Positive Deadtime enabled
    DTR6    = 960;
    ALTDTR6 = 960;
				  
    PWMCON6bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON6bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON6bits.IFLTMOD = 0;        //
    FCLCON6bits.CLSRC = 0;          // Current-Limit FALT1
    FCLCON6bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON6bits.CLMOD = 0;          // Current-Limit Disenable
//    FCLCON6bits.CLMOD = 1;          // Current-Limit enable
    
    FCLCON6bits.FLTSRC = 0;        // Fault Source is FALT1
    FCLCON6bits.FLTPOL = 0;         // Fault Signal is active-high
    FCLCON6bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON6bits.FLTMOD = 1;         // Fault cycle by cycle    
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON6bits.PHF      = 1;       // 
    LEBCON6bits.PLF      = 1;       // 
    LEBCON6bits.FLTLEBEN = 0;       // 
    LEBDLY6              = 0;      // 

    TRGCON6bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON6bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDC1 depending on configuration

    TRIG6   = 0;                      // Set Initial Trigger location
    PHASE6  = 0;
    PDC6    = 9600;


    IOCON7bits.PENH = 0;            // GPIO controls I/O port
    IOCON7bits.PENL = 0;           

    IOCON7bits.PMOD = 0;            // Complementary Mode
    
    IOCON7bits.POLH = 0;            // Drive signals are active-high
    IOCON7bits.POLL = 0;            // Drive signals are active-high

    IOCON7bits.OVRENH = 0;          // Override disabled
    IOCON7bits.OVRENL = 0;	  
    IOCON7bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON7bits.FLTDAT = 0b01;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON7bits.CLDAT = 0b10;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON7bits.DTC   = 0;          // Positive Deadtime enabled
    DTR7    = 960;
    ALTDTR7 = 960;
				  
    PWMCON7bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON7bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON7bits.IFLTMOD = 0;        //
    FCLCON7bits.CLSRC = 0;          // Current-Limit FALT1
    FCLCON7bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON7bits.CLMOD = 0;          // Current-Limit Disenable
//    FCLCON7bits.CLMOD = 1;          // Current-Limit enable
    
    FCLCON7bits.FLTSRC = 0;        // Fault Source is FALT1
    FCLCON7bits.FLTPOL = 0;         // Fault Signal is active-high
    FCLCON7bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON7bits.FLTMOD = 1;         // Fault cycle by cycle    
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON7bits.PHF      = 1;       // 
    LEBCON7bits.PLF      = 1;       // 
    LEBCON7bits.FLTLEBEN = 0;       // 
    LEBDLY7              = 0;      // 

    TRGCON7bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON7bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDC1 depending on configuration

    TRIG7   = 0;                      // Set Initial Trigger location
    PHASE7  = 0;
    PDC7    = 9600;


    IOCON5bits.PENH = 0;            // GPIO controls I/O port
    IOCON5bits.PENL = 0;           

    IOCON5bits.PMOD = 3;            // Independent Output mode
    
    IOCON5bits.POLH = 0;            // Drive signals are active-high
    IOCON5bits.POLL = 0;            // Drive signals are active-high

    IOCON5bits.OVRENH = 0;          // Override disabled
    IOCON5bits.OVRENL = 0;	  
    IOCON5bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON5bits.FLTDAT = 0b01;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON5bits.CLDAT = 0b10;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON5bits.DTC   = 0;          // Positive Deadtime enabled
    DTR5    = 0;
    ALTDTR5 = 0;
				  
    PWMCON5bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON5bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON5bits.IFLTMOD = 0;        //
    FCLCON5bits.CLSRC = 0;          // Current-Limit FALT1
    FCLCON5bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON5bits.CLMOD = 0;          // Current-Limit Disenable
//    FCLCON5bits.CLMOD = 1;          // Current-Limit enable
    
    FCLCON5bits.FLTSRC = 0;        // Fault Source is FALT1
    FCLCON5bits.FLTPOL = 0;         // Fault Signal is active-high
    FCLCON5bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON5bits.FLTMOD = 1;         // Fault cycle by cycle    
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON5bits.PHF      = 1;       // 
    LEBCON5bits.PLF      = 1;       // 
    LEBCON5bits.FLTLEBEN = 0;       // 
    LEBDLY5              = 0;      // 

    TRGCON5bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON5bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDC1 depending on configuration

    TRIG5   = 0;                      // Set Initial Trigger location
    PHASE5  = 0;
    SPHASE5  = 0;
    PDC5    = 9600;
    SDC5    = 9600;


    IOCON8bits.PENH = 0;            // GPIO controls I/O port
    IOCON8bits.PENL = 0;           

    IOCON8bits.PMOD = 3;            // Independent Output mode
    
    IOCON8bits.POLH = 0;            // Drive signals are active-high
    IOCON8bits.POLL = 0;            // Drive signals are active-high

    IOCON8bits.OVRENH = 0;          // Override disabled
    IOCON8bits.OVRENL = 0;	  
    IOCON8bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON8bits.FLTDAT = 0b01;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON8bits.CLDAT = 0b10;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON8bits.DTC   = 0;          // Positive Deadtime enabled
    DTR8    = 0;
    ALTDTR8 = 0;
				  
    PWMCON8bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON8bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON8bits.IFLTMOD = 0;        //
    FCLCON8bits.CLSRC = 0;          // Current-Limit FALT1
    FCLCON8bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON8bits.CLMOD = 0;          // Current-Limit Disenable
//    FCLCON8bits.CLMOD = 1;          // Current-Limit enable
    
    FCLCON8bits.FLTSRC = 0;        // Fault Source is FALT1
    FCLCON8bits.FLTPOL = 0;         // Fault Signal is active-high
    FCLCON8bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON8bits.FLTMOD = 1;         // Fault cycle by cycle    
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON8bits.PHF      = 1;       // 
    LEBCON8bits.PLF      = 1;       // 
    LEBCON8bits.FLTLEBEN = 0;       // 
    LEBDLY8              = 0;      // 

    TRGCON8bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON8bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDC1 depending on configuration

    TRIG8   = 0;                      // Set Initial Trigger location
    PHASE8  = 0;
    SPHASE8  = 0;
    PDC8    = 9600;
    SDC8    = 9600;

    IOCON1bits.PENH = 0;            // GPIO controls I/O port
    IOCON1bits.PENL = 0;           

    IOCON1bits.PMOD = 0;            // Complementary Mode
    
    IOCON1bits.POLH = 0;            // Drive signals are active-high
    IOCON1bits.POLL = 0;            // Drive signals are active-high

    IOCON1bits.OVRENH = 0;          // Override disabled
    IOCON1bits.OVRENL = 0;	  
    IOCON1bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON1bits.FLTDAT = 0b01;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON1bits.CLDAT = 0b10;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON1bits.DTC   = 0;          // Positive Deadtime enabled
    DTR1    = 0;
    ALTDTR1 = 0;
				  
    PWMCON1bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON1bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON1bits.IFLTMOD = 0;        //
    FCLCON1bits.CLSRC = 0;          // Current-Limit FALT1
    FCLCON1bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON1bits.CLMOD = 0;          // Current-Limit Disenable
//    FCLCON1bits.CLMOD = 1;          // Current-Limit enable
    
    FCLCON1bits.FLTSRC = 0;        // Fault Source is FALT1
    FCLCON1bits.FLTPOL = 0;         // Fault Signal is active-high
    FCLCON1bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON1bits.FLTMOD = 1;         // Fault cycle by cycle    
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON1bits.PHF      = 1;       // 
    LEBCON1bits.PLF      = 1;       // 
    LEBCON1bits.FLTLEBEN = 0;       // 
    LEBDLY1              = 0;      // 

    TRGCON1bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON1bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDC1 depending on configuration

    TRIG1   = 0;                      // Set Initial Trigger location
    PHASE1  = 0;
    SPHASE1  = 0;
    PDC1    = 9600;
    SDC1    = 9600;

    IOCON4bits.PENH = 0;            // GPIO controls I/O port
    IOCON4bits.PENL = 0;           

    IOCON4bits.PMOD = 0;            // Complementary Mode
    
    IOCON4bits.POLH = 0;            // Drive signals are active-high
    IOCON4bits.POLL = 0;            // Drive signals are active-high

    IOCON4bits.OVRENH = 0;          // Override disabled
    IOCON4bits.OVRENL = 0;	  
    IOCON4bits.OVRDAT = 0b00;       // Override data PWMH and PWML
    IOCON4bits.FLTDAT = 0b01;       // If fault occurs:
                                    // PWMH = 1 & PWML = 0
    IOCON4bits.CLDAT = 0b10;        // If Current Limit occurs:
                                    // PWMH = 1 & PWML = 0

    PWMCON4bits.DTC   = 0;          // Positive Deadtime enabled
    DTR4    = 0;
    ALTDTR4 = 0;
				  
    PWMCON4bits.IUE = 0;            // Immediate duty cycle updates
    PWMCON4bits.ITB = 0;            // Select Primary Timebase mode

    FCLCON4bits.IFLTMOD = 0;        //
    FCLCON4bits.CLSRC = 0;          // Current-Limit FALT1
    FCLCON4bits.CLPOL = 0;          // Current-Limit Signal is active-High
    FCLCON4bits.CLMOD = 0;          // Current-Limit Disenable
//    FCLCON4bits.CLMOD = 1;          // Current-Limit enable
    
    FCLCON4bits.FLTSRC = 0;        // Fault Source is FALT1
    FCLCON4bits.FLTPOL = 0;         // Fault Signal is active-high
    FCLCON4bits.FLTMOD = 3;         // Fault Disenable
//    FCLCON4bits.FLTMOD = 1;         // Fault cycle by cycle    
    

// In order to block the sensed current spike at the
// turn on edge of the HS FET, use Leading Edge blanking.
    LEBCON4bits.PHF      = 1;       // 
    LEBCON4bits.PLF      = 1;       // 
    LEBCON4bits.FLTLEBEN = 0;       // 
    LEBDLY4              = 0;      // 

    TRGCON4bits.TRGDIV  = 0;        // Trigger interrupt generated every 1 PWM cycles

    TRGCON4bits.TRGSTRT = 1;        // Trigger generated after waiting 1 PWM cycles

    // Initialize PDC1 depending on configuration

    TRIG4   = 0;                      // Set Initial Trigger location
    PHASE4  = 0;
    SPHASE4  = 0;
    PDC4    = 9600;
    SDC4    = 9600;    
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
    CLC1CONLbits.MODE = 2; // Four-Input AND

    CLC1CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC1CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC1CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC1CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC1SELbits.DS1 = 3; // DS1 = PWM1H
    CLC1SELbits.DS2 = 7; // DS2 = PWM5H
    CLC1SELbits.DS3 = 0; // DS3 = 
    CLC1SELbits.DS4 = 7; // DS4 = PWM6H
 
    CLC1GLSLbits.G1D1N = 1; // G1 = DS1-N
    CLC1GLSLbits.G2D1N = 1; // G2 = DS1-N
//    CLC1GLSLbits.G1D2N = 1; // G1 = DS2-N
//    CLC1GLSLbits.G2D2N = 1; // G2 = DS2-N
    CLC1GLSHbits.G3D4T = 1; // G3 = DS4-T
    CLC1GLSHbits.G4D4T = 1; // G4 = DS4-T

    CLC1CONLbits.LCEN = 1; // CLC1 is enabled
    
 // CLC2 -------------------------------------------------   
    CLC2CONLbits.LCOE = 1; // CLC1 port pin output is enabled
    CLC2CONLbits.MODE = 2; // Four-Input AND

    CLC2CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC2CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC2CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC2CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC2SELbits.DS1 = 0; // DS1 = 
    CLC2SELbits.DS2 = 7; // DS2 = PWM7H
    CLC2SELbits.DS3 = 5; // DS3 = PWM4H
    CLC2SELbits.DS4 = 7; // DS4 = PWM8H
 
    CLC2GLSLbits.G1D2T = 1; // G1 = DS2-T
    CLC2GLSLbits.G2D2T = 1; // G2 = DS2-T
    CLC2GLSHbits.G3D3N = 1; // G3 = DS3-N
    CLC2GLSHbits.G4D3N = 1; // G4 = DS3-N
//    CLC2GLSHbits.G3D4N = 1; // G3 = DS4-N
//    CLC2GLSHbits.G4D4N = 1; // G4 = DS4-N
    CLC2CONLbits.LCEN = 1; // CLC2 is enabled
    
 // CLC3 -------------------------------------------------  
    CLC3CONLbits.LCOE = 1; // CLC3 port pin output is enabled
    CLC3CONLbits.MODE = 2; // Four-Input AND

    CLC3CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC3CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC3CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC3CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC3SELbits.DS1 = 0; // DS1 = 
    CLC3SELbits.DS2 = 3; // DS2 = PWM5L
    CLC3SELbits.DS3 = 0; // DS3 = 
    CLC3SELbits.DS4 = 3; // DS4 = PWM6L
 
    CLC3GLSLbits.G1D2N = 1; // G1 = DS2-N
    CLC3GLSLbits.G2D2N = 1; // G2 = DS2-N
    CLC3GLSHbits.G3D4T = 1; // G3 = DS4-T
    CLC3GLSHbits.G4D4T = 1; // G4 = DS4-T

    CLC3CONLbits.LCEN = 1; // CLC3 is enabled


 // CLC4 -------------------------------------------------  
    CLC4CONLbits.LCOE = 1; // CLC1 port pin output is enabled
    CLC4CONLbits.MODE = 2; // Four-Input AND

    CLC4CONHbits.G1POL = 0; // Channel 1 logic output is not inverted
    CLC4CONHbits.G2POL = 0; // Channel 2 logic output is not inverted
    CLC4CONHbits.G3POL = 0; // Channel 3 logic output is not inverted
    CLC4CONHbits.G4POL = 0; // Channel 4 logic output is not inverted

    CLC4SELbits.DS1 = 0; // DS1 = 
    CLC4SELbits.DS2 = 3; // DS2 = PWM7L
    CLC4SELbits.DS3 = 0; // DS3 = 
    CLC4SELbits.DS4 = 3; // DS4 = PWM8L
 
    CLC4GLSLbits.G1D2T = 1; // G1 = DS2-T
    CLC4GLSLbits.G2D2T = 1; // G2 = DS2-T
    CLC4GLSHbits.G3D4N = 1; // G3 = DS4-N
    CLC4GLSHbits.G4D4N = 1; // G4 = DS4-N

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
    
    PGA1CONbits.PGAEN = 1; //Enable PGA1    
    
    PGA2CONbits.PGAEN = 0; //Disable PGA2
    PGA2CONbits.SELPI = 0; //PGA2P1 as positive input
    PGA2CONbits.SELNI = 0; //GND as Negative input
    PGA2CONbits.GAIN =  4; //16x PGA Gain
    // DEVICE CALIBRATION
    TBLPAG = 0x80 ;
    PGA2CAL = __builtin_tblrdl (0x0E4C);    // PGA2
    
    PGA2CONbits.PGAEN = 1; //Enable PGA2
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

    ADCON4Hbits.C0CHS   = 2;    // 0-AN0, 1-AN7, 2-PGA1, 3-AN0ALT
    ADCON4Hbits.C1CHS   = 2;    // 0-AN1, 1-AN18, 2-PGA2, 3-AN1ALT    
    ADCON4Hbits.C2CHS   = 0;    // 0-AN2, 1-AN11, 2-VREF, 3-Resirve
    ADCON4Hbits.C3CHS   = 0;    // 0-AN3, 1-AN15, 2-Resirve, 3-Resirve    
    
    // Configure ANx for unsigned format and single ended (0,0)
    ADMOD0L = 0x0000;

    calibrateADC();
    
    ADTRIG0Lbits.TRGSRC0 = 0x0A;     // ADC AN0 triggered by PWM6
    ADTRIG0Lbits.TRGSRC1 = 0x0A;     // ADC AN1 triggered by PWM6
    ADTRIG0Hbits.TRGSRC2 = 0x0A;     // ADC AN2 triggered by PWM6
    ADTRIG0Hbits.TRGSRC3 = 0x0A;     // ADC AN3 triggered by PWM6
//    ADTRIG1Lbits.TRGSRC4 = 0x05;     // ADC AN4 triggered by PWM1
//    ADTRIG1Lbits.TRGSRC5 = 0x05;     // ADC AN5 triggered by PWM1
//    ADTRIG1Hbits.TRGSRC6 = 0x05;     // ADC AN6 triggered by PWM1
//    ADTRIG1Hbits.TRGSRC7 = 0x05;     // ADC AN7 triggered by PWM1
//    ADTRIG2Lbits.TRGSRC8 = 0x05;     // ADC AN8 triggered by PWM1
//    ADTRIG2Lbits.TRGSRC9 = 0x05;     // ADC AN9 triggered by PWM1
//    ADTRIG2Hbits.TRGSRC10 = 0x05;    // ADC AN10 triggered by PWM1
    ADTRIG2Hbits.TRGSRC11 = 0x0A;    // ADC AN11 triggered by PWM6        
    
    IFS7bits.ADCAN2IF  = 0;      // Clear ADC interrupt flag
    IPC28bits.ADCAN2IP = 6;      // Set ADC interrupt priority to 7
    ADIELbits.IE2     = 1;      // Enable ADC AN0 interrupt 
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
    CMP1CONbits.INSEL  = 1; //Selects CMP1B
    CMP1CONbits.ALTINP = 0; //Selects CMP1B
    CMP1CONbits.CMPPOL = 0; //Output is inverted
    CMP1CONbits.RANGE = 1;
    
    CMP1DAC = 2000;
   
//    CMP1CONbits.CMPON = 0;  //Disenable CMP1
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
