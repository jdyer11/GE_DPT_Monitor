# GE_DPT_Monitor
Double Pulse Test PWM Code w/ Monitoring Feature


//###########################################################################
// GE_DPT_Monitor_v1
// Description: Double Pulse Test PWM Code w/ Monitoring Feature.
// Author: Jacob Dyer
// Date: 5-07-2018
// Version: 6_3 RAM
// Last version: 6_2 RAM
//
// Note:
// 1. PWM Generation
// 2. Monitoring
// 3. SCI Interface
//
// Modification:
// 1. ...
//
//===========================================================================
// History:
//---------------------------------------------------------------------------
//
//###########################################################################



//===========================================================================
// Include header files used in the main function.
//===========================================================================
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File
#include "math.h"
//#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
//#include "DSP2833x_EPwm_defines.h" 	// useful defines for initialization
#include "SFO_V5.h"  // SFO V5 library headerfile - required to use SFO library functions
#include "DC_Mag.h"
#include "rtwtypes.h"
#include "DC_Mag_Private.h"

//===========================================================================
// Data type definition.
//===========================================================================
// EPWM configuration
typedef struct
{
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwm_CMPA_Direction;
   Uint16 EPwm_CMPB_Direction;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmMaxCMPA;
   Uint16 EPwmMinCMPA;
   Uint16 EPwmMaxCMPB;
   Uint16 EPwmMinCMPB;
}EPWM_INFO;


// UART communication between inverter and PC
uint8_T ReceivedByte_PC = 0;
uint8_T recv_cnt_PC = 0;
uint8_T recv_syn_PC = 0;
int8_T  sign_PC = 0;
real32_T Vdc_ref_PC=0;
uint8_T Vdc_refH_PC=0;
uint8_T Vdc_refL_PC=0;
real32_T P_ref_PC=0;
uint8_T P_refH_PC=0;
uint8_T P_refL_PC=0;
real32_T I_ref_PC=0;
uint8_T I_refH_PC=0;
uint8_T I_refL_PC=0;
real32_T Charge_ref_PC=0;
uint8_T Charge_refH_PC=0;
uint8_T Charge_refL_PC=0;
real32_T Run_ref_PC=0;
uint8_T Run_refH_PC=0;
uint8_T Run_refL_PC=0;
real32_T Stop_ref_PC=0;
uint8_T Stop_refH_PC=0;
uint8_T Stop_refL_PC=0;
real32_T Reset_ref_PC=0;
uint8_T Reset_refH_PC=0;
uint8_T Reset_refL_PC=0;
real32_T Estop_ref_PC=0;
uint8_T Estop_refH_PC=0;
uint8_T Estop_refL_PC=0;
real32_T Backup_ref_PC=0;
uint8_T Backup_refH_PC=0;
uint8_T Backup_refL_PC=0;

//**************************************************************************//


void HRPWM_Config(int);
void error (void);
void enable_interrupts(void);

// General System nets - Useful for debug
//Uint16 nMepChannel;

//====================================================================
// The following declarations are required in order to use the SFO
// library functions:
//
int MEP_ScaleFactor[PWM_CH]; // Global array used by the SFO library

// Array of pointers to EPwm register structures:
// *ePWM[0] is defined as dummy value not used in the example
volatile struct EPWM_REGS *ePWM[PWM_CH] =
 			 {  &EPwm1Regs, &EPwm1Regs, &EPwm2Regs,	&EPwm3Regs, &EPwm4Regs,	&EPwm5Regs,	&EPwm6Regs};

//===========================================================================
// Prototype statements for functions found within this file.
//===========================================================================
void InitEPwm1Example(void);
void InitEPwm2Example(void);
void InitEPwm3Example(void);

void Gpio_select(void);

void EPWMOutDisable(void);
void EPWMOutEnable(void);
double angle_limit(double w1); // angle limit in 2*pi

void SFO_MepDis(int i);

interrupt void epwm1_isr(void);
interrupt void SCIRXINTB_isr(void);
void init_SCI(void);
void init_SCI_GPIO(void);

#define DETECT_H_DVFD_FB GpioDataRegs.GPCDAT.bit.GPIO75		//Output of Monitoring board
#define DETECT_H_GVTD_FB GpioDataRegs.GPCDAT.bit.GPIO74		//Output of Monitoring board
#define DETECT_H_DVTD_FB GpioDataRegs.GPCDAT.bit.GPIO73		//Output of Monitoring board

#define DETECT_L_DVFD_FB GpioDataRegs.GPCDAT.bit.GPIO79		//Output of Monitoring board
#define DETECT_L_GVTD_FB GpioDataRegs.GPCDAT.bit.GPIO78		//Output of Monitoring board
#define DETECT_L_DVTD_FB GpioDataRegs.GPCDAT.bit.GPIO77		//Output of Monitoring board

//===========================================================================
// Global definitions found within this file.
//===========================================================================
// Configure the period for each timer
#define EPWM_TIMER_TBPRD  15000  // Period register   // 12000(6.25kHz)  5000(15kHz)  15000(5khz)  7500(10khz)
#define EPWM_DUTY       3000
#define EPWM_DTIME       100  //150 //30(0.4us)  150(2us)  100(1.3us)

#define EPWM1_TIMER_TBPRD  15000  // Period register
#define EPWM1_DUTY       3000
#define EPWM1_DTIME       100  //150 //30

#define EPWM2_TIMER_TBPRD  15000  // Period register
#define EPWM2_DUTY       3000
#define EPWM2_DTIME      100  // 150 //30

#define EPWM3_TIMER_TBPRD  15000  // Period register
#define EPWM3_DUTY       3000
#define EPWM3_DTIME      100  // 150 //30


// Configure the period for each timer
#define EPWM1_MAX_CMPA     1950        // CW: Because the period of EPWM is changing, these limitations are not used.
#define EPWM1_MIN_CMPA       50
#define EPWM1_MAX_CMPB     1950
#define EPWM1_MIN_CMPB       50

#define EPWM2_MAX_CMPA     1950
#define EPWM2_MIN_CMPA       50
#define EPWM2_MAX_CMPB     1950
#define EPWM2_MIN_CMPB       50

#define EPWM3_MAX_CMPA     1950      //? Why different?
#define EPWM3_MIN_CMPA       50
#define EPWM3_MAX_CMPB     1950
#define EPWM3_MIN_CMPB       50

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0

// Running status LED
// Make this long enough so that we can see an LED toggle
#define DELAY 500000L    // running status
#define DELAY_ERROR 100000L    // error status

//===========================================================================
// Global Variable Declaration
//===========================================================================
// Global variables used in this example
EPWM_INFO epwm1_info;
EPWM_INFO epwm2_info;
EPWM_INFO epwm3_info;

// AD sample values
double Vdc = 0;
double Idc = 0;

// Interrupt counters
Uint16 BackTicker = 0;         // Counter of cycling times in the main functions
Uint16 MainIsrTicker = 0;      // Counter of the main interrupt times


// Error Signs
int ErrorFlag = 0;                //flag1=1,2,3: Over current in phase a, b, c

double Tn = 0;
double Tn1 = 0;
double test_data_12 = 0;
double sw_per_2nd = 0;
double Tn_2nd = 0;

int pll_count = 0;
int pll_count_1 = 0;

// For button
int PLUS_DT=0;
int Preset_PLUS_DT=0;
int Last_GT_Flag=0;
int Last_DT_Flag=0;
int Last_DF_Flag=0;
int Near_GT_Flag=0;
int Near_DT_Flag=0;
int Near_DF_Flag=0;
double t_GT=0;
double t_DT=0;
double t_DF=0;
double t_TurnOff=0.0;
short int i_monitor=0;
short int i_monitor_L=0;
short int i_monitor_H=0;
double t_delay_off=0.0;

int DTT_Step=0;
int DPT_Count_1=4000; // 400 us delay
int DPT_Count_2=24; // 1 s delay
int DPT_Count_a=0;
int DPT_Count_b=0;
double sw_per_fix = 1500;

int Q_Point=0;
int timeArray[4];
int t_Avg=0;

// System flag
int PWMOutEnFlag = 0;   // Enable PWM output. 0: Disable, 1: Enable
int PWMOutEnFlag_ADAPTIVE = 0;
int PWMOutEnFlag_ACTIVE = 0;
int k_full_2=0;
double	sw_per_2=0; // time (us) * 150
double	Tn_2=0; // first pulse duration
double	Tn1_2=0;//Duration for phase B & C

//double Ts;
double sw_per = 15000;  // Period register   // 12000(6.25kHz)  5000(15kHz)  15000(5khz)  7500(10khz)  37500(2khz) 50000(1.5kHz);

int k_full=0;
int pll_count_2=0;
double sw_per_full=60000;

int i;

void HRPWM_Config(period)
{
	Uint16 j;
// ePWM channel register configuration with HRPWM
// ePWMxA toggle low/high with MEP control on Rising edge

	for (j=1;j<PWM_CH;j++)
	{
		(*ePWM[j]).TBCTL.bit.PRDLD = TB_IMMEDIATE;	        // set Immediate load
		(*ePWM[j]).TBPRD = period-1;		                // PWM frequency = 1 / period
		(*ePWM[j]).CMPA.half.CMPA = period / 2;             // set duty 50% initially
		(*ePWM[j]).CMPA.half.CMPAHR = (1 << 8);             // initialize HRPWM extension
		(*ePWM[j]).CMPB = period / 2;	                    // set duty 50% initially
		(*ePWM[j]).TBPHS.all = 0;
		(*ePWM[j]).TBCTR = 0;

		(*ePWM[j]).TBCTL.bit.CTRMODE = TB_COUNT_UP;
		(*ePWM[j]).TBCTL.bit.PHSEN = TB_DISABLE;
		(*ePWM[j]).TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
		(*ePWM[j]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
		(*ePWM[j]).TBCTL.bit.CLKDIV = TB_DIV1;
		(*ePWM[j]).TBCTL.bit.FREE_SOFT = 11;

		(*ePWM[j]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
		(*ePWM[j]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
		(*ePWM[j]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		(*ePWM[j]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;


		(*ePWM[j]).AQCTLA.bit.ZRO = AQ_SET;               // PWM toggle high/low
		(*ePWM[j]).AQCTLA.bit.CAU = AQ_CLEAR;
		(*ePWM[j]).AQCTLB.bit.ZRO = AQ_SET;
		(*ePWM[j]).AQCTLB.bit.CBU = AQ_CLEAR;

		EALLOW;
		(*ePWM[j]).HRCNFG.all = 0x0;
		(*ePWM[j]).HRCNFG.bit.EDGMODE = HR_FEP;			 // MEP control on falling edge
		(*ePWM[j]).HRCNFG.bit.CTLMODE = HR_CMP;
		(*ePWM[j]).HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
		EDIS;
	}
}

void main(void)
{
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the DSP2833x_SysCtrl.c file.
	InitSysCtrl();

// Step 2. Initalize GPIO: 
// This example function is found in the DSP2833x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();  // Skipped for this example  

// For this case just init GPIO pins for ePWM1, ePWM2, ePWM3
// These functions are in the DSP2833x_EPwm.c file
	InitEPwm1Gpio();
	InitEPwm2Gpio();
	InitEPwm3Gpio();


	//Initialize the GPIO for monitoring.
	Gpio_select();


	init_SCI_GPIO();
	init_SCI();


// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts 
	DINT;

// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.  
// This function is found in the DSP2833x_PieCtrl.c file.
	InitPieCtrl();
   
// Disable CPU interrupts and clear all CPU interrupt flags:
	IER = 0x0000;
	IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt 
// Service Routines (ISR).  
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in DSP2833x_DefaultIsr.c.
// This function is found in DSP2833x_PieVect.c.
	InitPieVectTable();

	//enable_interrupts();
// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.  
	EALLOW;  // This is needed to write to EALLOW protected registers
	PieVectTable.EPWM1_INT = &epwm1_isr;
	EDIS;    // This is needed to disable write to EALLOW protected registers


	EALLOW;
	PieVectTable.SCIRXINTB = &SCIRXINTB_isr;// Hook interrupt to the ISR
	EDIS;
	PieCtrlRegs.PIEIER9.bit.INTx3 = 1;   // Enable interrupt SCIRXINTB
	IER |= M_INT9;


// Step 4. Initialize all the Device Peripherals:
// This function is found in DSP2833x_InitPeripherals.c
// InitPeripherals();  // Not required for this example

// For this example, only initialize the ePWM

	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
	EDIS;

//  MEP_ScaleFactor variables intialization for SFO library functions
	for(i=0;i<PWM_CH;i++)
	{
	    MEP_ScaleFactor[i] =0;
	}

//  MEP_ScaleFactor variables intialization using SFO_MepDis_V5 library function.
	EALLOW;
	for(i=1;i<PWM_CH;i++)
	{
	
	    (*ePWM[i]).HRCNFG.bit.EDGMODE = 1;            // Enable HRPWM logic for channel prior to calibration
	    while ( SFO_MepDis_V5(i) == SFO_INCOMPLETE ); //returns "0" when cal. incomplete for channel
	}
	EDIS;

// 	Initialize a common seed variable MEP_ScaleFactor[0] required for all SFO functions
	MEP_ScaleFactor[0] = MEP_ScaleFactor[1];

//  Some useful Period vs Frequency values
//  SYSCLKOUT =     150MHz         100 MHz
//  -----------------------------------------
//	Period	        Frequency      Frequency
//	1000			150 kHz		   100 KHz
//	800				187 kHz		   125 KHz
//	600				250 kHz		   167 KHz
//	500				300 kHz		   200 KHz
//	250				600 kHz		   400 KHz
//	200				750 kHz		   500 KHz


	InitEPwm1Example();    
	InitEPwm2Example();
	InitEPwm3Example();


	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
	EDIS;


	for(i=0; i<1000; i++)   // Wait until the first AD conversion is finished
	{
		asm("          NOP");
	}


// Step 5. User specific code, enable interrupts:


// Enable CPU INT3 which is connected to EPWM1-3 INT:
	IER |= M_INT3;

// Enable EPWM INTn in the PIE: Group 3 interrupt 1
	PieCtrlRegs.PIEIER3.bit.INTx1 = 1;


// Enable global Interrupts and higher priority real-time debug events:
	EINT;   // Enable Global interrupt INTM
	ERTM;   // Enable Global realtime interrupt DBGM

// Step 6. IDLE loop. Just sit and loop forever (optional):

// DSP Running status LED
	EALLOW;
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;
	EDIS;


	//PC_scib_xmit(0xAA);

	for(;;)
	{
		// asm("          NOP");
		
		// Counter
		BackTicker ++;
		
		// Running status LED toggle
		// This loop will be interrupted, so the overall
		// delay between pin toggles will be longer.
		DELAY_US(DELAY);
		GpioDataRegs.GPBTOGGLE.bit.GPIO32 = 1;
	}

} 


void Gpio_select()
{

    EALLOW;
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pullup on GPIO16
	GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pullup on GPIO17
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0x00000000;  // Only GPIO16
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0x00000000;  // Only GPIO17
//	GpioCtrlRegs.GPACTRL.bit.QUALPRD2 = 0xFF; //Sampling time for GPIO16 to GPIO23 is 510*Tsysclk
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 0x2; //Qualification using 6 samples, sampling window = 510*Tsysclk*5 = 17us
//	GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 0x2; //Qualification using 6 samples, sampling window = 510*Tsysclk*5 = 17us
	GpioCtrlRegs.GPAQSEL2.bit.GPIO16 =0;  // Synchronize to SYSCLKOUT only
	GpioCtrlRegs.GPAQSEL2.bit.GPIO17 =0;  // Synchronize to SYSCLKOUT only
    GpioCtrlRegs.GPADIR.bit.GPIO16 = 0;   // GPIO16 input, detect the start/stop signal from push button
	GpioDataRegs.GPASET.bit.GPIO16 = 1;   // Load output latch
    GpioCtrlRegs.GPADIR.bit.GPIO17 = 0;   // GPIO17 input, detect the output trigger from push button
	GpioDataRegs.GPASET.bit.GPIO17 = 1;   // Load output latch

    GpioCtrlRegs.GPAPUD.bit.GPIO27 = 0;   // Enable pullup on GPIO27
	GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pullup on GPIO21
	GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO23
	GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0x00000000;  // Only GPIO27
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0x00000000;  // Only GPIO21
	GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0x00000000;  // Only GPIO23
	GpioCtrlRegs.GPAQSEL2.bit.GPIO27 =0;  // Synchronize to SYSCLKOUT only
	GpioCtrlRegs.GPAQSEL2.bit.GPIO21 =0;  // Synchronize to SYSCLKOUT only
	GpioCtrlRegs.GPAQSEL2.bit.GPIO23 =0;  // Synchronize to SYSCLKOUT only
    GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;   // GPIO27 input, detect the start/stop signal from push button
	GpioDataRegs.GPASET.bit.GPIO27 = 1;   // Load output latch
    GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;   // GPIO21 input, detect the output trigger from push button
	GpioDataRegs.GPASET.bit.GPIO21 = 1;   // Load output latch
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;   // GPIO23 input, detect the start/stop signal from push button
	GpioDataRegs.GPASET.bit.GPIO23 = 1;   // Load output latch

    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO27
	GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pullup on GPIO21
	GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pullup on GPIO23
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0x00000000;  // Only GPIO27
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0x00000000;  // Only GPIO21
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0x00000000;  // Only GPIO23
	GpioCtrlRegs.GPAQSEL2.bit.GPIO20 =0;  // Synchronize to SYSCLKOUT only
	GpioCtrlRegs.GPAQSEL2.bit.GPIO19 =0;  // Synchronize to SYSCLKOUT only
	GpioCtrlRegs.GPAQSEL2.bit.GPIO18 =0;  // Synchronize to SYSCLKOUT only
    GpioCtrlRegs.GPADIR.bit.GPIO20 = 0;   // GPIO27 input, detect the start/stop signal from push button
	GpioDataRegs.GPASET.bit.GPIO20 = 1;   // Load output latch
    GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;   // GPIO21 input, detect the output trigger from push button
	GpioDataRegs.GPASET.bit.GPIO19 = 1;   // Load output latch
    GpioCtrlRegs.GPADIR.bit.GPIO18 = 0;   // GPIO23 input, detect the start/stop signal from push button
	GpioDataRegs.GPASET.bit.GPIO18 = 1;   // Load output latch

	GpioCtrlRegs.GPCPUD.bit.GPIO79 = 1;   // Enable pullup
	GpioCtrlRegs.GPCPUD.bit.GPIO78 = 1;   // Enable pullup
	GpioCtrlRegs.GPCPUD.bit.GPIO77 = 1;   // Enable pullup
	GpioCtrlRegs.GPCPUD.bit.GPIO75 = 1;   // Enable pullup
	GpioCtrlRegs.GPCPUD.bit.GPIO74 = 1;   // Enable pullup
	GpioCtrlRegs.GPCPUD.bit.GPIO73 = 1;   // Enable pullup
	GpioCtrlRegs.GPCMUX1.bit.GPIO79 = 0x00000000;  // Only GPIO27
	GpioCtrlRegs.GPCMUX1.bit.GPIO78 = 0x00000000;  // Only GPIO21
	GpioCtrlRegs.GPCMUX1.bit.GPIO77 = 0x00000000;  // Only GPIO23
	GpioCtrlRegs.GPCMUX1.bit.GPIO75 = 0x00000000;  // Only GPIO23
	GpioCtrlRegs.GPCMUX1.bit.GPIO74 = 0x00000000;  // Only GPIO23
	GpioCtrlRegs.GPCMUX1.bit.GPIO73 = 0x00000000;  // Only GPIO23
	//GpioCtrlRegs.GPCQSEL1.bit.GPIO79 =0;  // Synchronize to SYSCLKOUT only
	//GpioCtrlRegs.GPCQSEL1.bit.GPIO78 =0;  // Synchronize to SYSCLKOUT only
	//GpioCtrlRegs.GPCQSEL1.bit.GPIO77 =0;  // Synchronize to SYSCLKOUT only
	GpioCtrlRegs.GPCDIR.bit.GPIO79 = 0;   // GPIO27 input, detect the start/stop signal from push button
	//GpioDataRegs.GPCSET.bit.GPIO79 = 1;   // Load output latch
	GpioCtrlRegs.GPCDIR.bit.GPIO78 = 0;   // GPIO21 input, detect the output trigger from push button
	//GpioDataRegs.GPCSET.bit.GPIO78 = 1;   // Load output latch
	GpioCtrlRegs.GPCDIR.bit.GPIO77 = 0;   // GPIO23 input, detect the start/stop signal from push button
	//GpioDataRegs.GPCSET.bit.GPIO77 = 1;   // Load output latch
	GpioCtrlRegs.GPCDIR.bit.GPIO75 = 0;   // GPIO23 input, detect the start/stop signal from push button
	GpioCtrlRegs.GPCDIR.bit.GPIO74 = 0;   // GPIO23 input, detect the start/stop signal from push button
	GpioCtrlRegs.GPCDIR.bit.GPIO73 = 0;   // GPIO23 input, detect the start/stop signal from push button
	
    EDIS;
}


void InitEPwm1Example()
{

  //duty_var=100;

   // Setup TBCLK
   EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;           // Set timer period // CW: Intial Ts is 12000*2/150000000=0.00016s (6.25kHz)
   EPwm1Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm1Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   //EPwm1Regs.CMPA.half.CMPA = EPWM_DUTY;     // Set compare A value
   //EPwm1Regs.CMPB = EPWM_DUTY; EPWM1_DTIME              // Set Compare B value

   // Test
//   EPwm1Regs.CMPA.half.CMPA = EPWM1_DUTY;     // Set compare A value
//   EPwm1Regs.CMPB = EPWM1_DUTY + 2 * EPWM1_DTIME; // Set Compare B value

   // Modify the setting of dead time. Version:3
   EPwm1Regs.CMPA.half.CMPA = EPWM1_DUTY;     // Set compare A value
   EPwm1Regs.CMPB = EPWM1_DUTY - 2 * EPWM1_DTIME; // Set Compare B value


   // Setup counter mode
   EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up and down
   EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;        // Disable phase loading
   EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
   EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;//TB_SYNC_IN; // Sync down-stream module
   EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set actions
//   EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM1A on event A, down count
//   EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;           // Clear PWM1A on event A, up count
//
//   EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
//   EPwm1Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM1B on event B, down count

   // Modify the comparison
   EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;             // Clear PWM1B on event B, up count
   EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;               // Set PWM1B on event B, down count


   // Interrupt where we will change the Compare Values
   EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;      // Select INT on ZERO event
   EPwm1Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
   EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event

   epwm1_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm1_info.EPwmRegHandle = &EPwm1Regs;          // Set the pointer to the ePWM module
	EALLOW;
	EPwm1Regs.HRCNFG.all = 0x0;
	EPwm1Regs.HRCNFG.bit.EDGMODE = HR_REP;	//HR_FEP;			 // MEP control on falling edge
	EPwm1Regs.HRCNFG.bit.CTLMODE = HR_CMP;
	EPwm1Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
	EDIS;

}



void InitEPwm2Example()
{

    // Setup TBCLK
   EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;           // Set timer period
   EPwm2Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
   EPwm2Regs.TBCTR = 0x0000;                      // Clear counter

   // Set Compare values
   //EPwm2Regs.CMPA.half.CMPA = EPWM_DUTY;     // Set compare A value
   //EPwm2Regs.CMPB = EPWM_DUTY;               // Set compare B value

   // Test
//   EPwm2Regs.CMPA.half.CMPA = EPWM2_DUTY;     // Set compare A value
//   EPwm2Regs.CMPB = EPWM2_DUTY + 2 * EPWM2_DTIME; // Set Compare B value

   // Modify the setting of dead time. Version:3
   EPwm2Regs.CMPA.half.CMPA = EPWM2_DUTY;     // Set compare A value
   EPwm2Regs.CMPB = EPWM2_DUTY - 2 * EPWM2_DTIME; // Set Compare B value


   // Setup counter mode
   EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up and down
   EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
   EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
   EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
   EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
   EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

   // Setup shadowing
   EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
   EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
   EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


   // Set actions
//   EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM2A on event A, down count
//   EPwm2Regs.AQCTLA.bit.CAD = AQ_SET;           	// Clear PWM2A on event A, up count
//
//   EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM2B on event B, up count
//   EPwm2Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM2B on event B, down count

   // Modify the comparison
   EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;             // Set PWM1A on event A, up count
   EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

   EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;             // Clear PWM1B on event B, up count
   EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;               // Set PWM1B on event B, down count


   // Interrupt where we will change the Compare Values
//   EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;      // Select INT on Period event
//   EPwm2Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
//   EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event

   epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
   epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the ePWM module
	EALLOW;
	EPwm2Regs.HRCNFG.all = 0x0;
	EPwm2Regs.HRCNFG.bit.EDGMODE = HR_REP;	//HR_FEP;			 // MEP control on falling edge
	EPwm2Regs.HRCNFG.bit.CTLMODE = HR_CMP;
	EPwm2Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
	EDIS;

}


void InitEPwm3Example(void)
{
	// Setup TBCLK
	EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;           // Set timer period
	EPwm3Regs.TBPHS.half.TBPHS = 0x0000;           // Phase is 0
	EPwm3Regs.TBCTR = 0x0000;                      // Clear counter

	// Set Compare values
	//EPwm3Regs.CMPA.half.CMPA = EPWM_DUTY;     // Set compare A value
	//EPwm3Regs.CMPB = EPWM_DUTY;               // Set Compare B value

	// Modify the setting of dead time. Version:3
	EPwm3Regs.CMPA.half.CMPA = EPWM3_DUTY;     // Set compare A value
	EPwm3Regs.CMPB = EPWM3_DUTY - 2 * EPWM3_DTIME; // Set Compare B value

	// Setup counter mode
	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up and down
	EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE;        // Enable phase loading
	EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;       // Clock ratio to SYSCLKOUT
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	// Setup shadowing
	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;  // Load on Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;


	// Set actions
	//   EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM3A on event A, down count
	//   EPwm3Regs.AQCTLA.bit.CAD = AQ_SET;           // Clear PWM3A on event A, up count
	//
	//   EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM3B on event B, up count
	//   EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;           // Clear PWM3B on event B, down count

	// Modify the comparison
	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;             // Set PWM1A on event A, up count
	EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;           // Clear PWM1A on event A, down count

	EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;             // Clear PWM1B on event B, up count
	EPwm3Regs.AQCTLB.bit.CBD = AQ_CLEAR;               // Set PWM1B on event B, down count


	// Interrupt where we will change the Compare Values
	//   EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;      // Select INT on Period event
	//   EPwm3Regs.ETSEL.bit.INTEN = 1;                 // Enable INT
	//   EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;            // Generate INT on 1st event

	epwm3_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
	epwm3_info.EPwmRegHandle = &EPwm3Regs;          // Set the pointer to the ePWM module

	EALLOW;
	EPwm3Regs.HRCNFG.all = 0x0;
	EPwm3Regs.HRCNFG.bit.EDGMODE = HR_REP;	//HR_FEP;			 // MEP control on falling edge
	EPwm3Regs.HRCNFG.bit.CTLMODE = HR_CMP;
	EPwm3Regs.HRCNFG.bit.HRLOAD  = HR_CTR_ZERO;
	EDIS;
//	01 MEP control of rising edge
//	10 MEP control of falling edge
//	11 MEP control of both edges

}



//===========================================================================
// Function: void EPWMOutDisable(void)
// Description: Disable the output of PWM by software-forced tripping.
//===========================================================================
void EPWMOutDisable(void)
{

	// Modify the comparison
// Set the output of EPWMxA and EPWMxB as low.
	EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD;
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
	EPwm1Regs.CMPA.half.CMPA = 0;     // Set PWM1A duty

	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;             // Set PWM1B on event B, up count
	EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;           // Clear PWM1B on event B, down count
	EPwm1Regs.CMPB = 0;         // Set Compare B value

	EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD;
	EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
	EPwm2Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD + 1;     // Set PWM1A duty

	EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
	EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;           // Clear PWM1B on event B, down count
	EPwm2Regs.CMPB = EPWM_TIMER_TBPRD + 1;         // Set Compare B value

	EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD;
	EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
	EPwm3Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD + 1;     // Set PWM1A duty

	EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
	EPwm3Regs.AQCTLB.bit.ZRO = AQ_CLEAR;           // Clear PWM1B on event B, down count
	EPwm3Regs.CMPB = EPWM_TIMER_TBPRD + 1;         // Set Compare B value
}



//===========================================================================
// Function: void EPWMOutDisableClr(void)
// Description: Clear the disable of the output of PWM by clearing the software-forced tripping.
//===========================================================================
void EPWMOutEnable(void)
{

	// Modify the comparison
// Initialize the EPWM
	EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
	EPwm1Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD + 1;     // Set PWM1A duty

	EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
	EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;           // Clear PWM1B on event B, down count
	EPwm1Regs.CMPB = EPWM_TIMER_TBPRD + 1;         // Set Compare B value

	EPwm2Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
	EPwm2Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD + 1;     // Set PWM1A duty

	EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
	EPwm2Regs.AQCTLB.bit.ZRO = AQ_SET;           // Clear PWM1B on event B, down count
	EPwm2Regs.CMPB = EPWM_TIMER_TBPRD + 1;         // Set Compare B value

	EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
	EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
	EPwm3Regs.CMPA.half.CMPA = EPWM_TIMER_TBPRD + 1;     // Set PWM1A duty

	EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;             // Set PWM1B on event B, up count
	EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;           // Clear PWM1B on event B, down count
	EPwm3Regs.CMPB = EPWM_TIMER_TBPRD + 1;         // Set Compare B value
}



//===========================================================================
// Function: epwm1_isr()
// Description: Main interrupt service program, for EPWM1 interrupt.
//===========================================================================
interrupt void epwm1_isr(void)
{

// Counting the interrupt times
	MainIsrTicker ++;


//---------------------------------------------------------------------------
//            AD sampling
//---------------------------------------------------------------------------
	Vdc = 600; //100; //


//---------------------------------------------------------------------------
//            Read I/O
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
//            Fault detection and protection
//---------------------------------------------------------------------------

	ErrorFlag=0;

//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// case 1
	PWMOutEnFlag = Charge_ref_PC;
	PWMOutEnFlag_ACTIVE = Reset_ref_PC;
    PWMOutEnFlag_ADAPTIVE = Run_ref_PC;
//    if (PWMOutEnFlag_ACTIVE==1 && PWMOutEnFlag_ADAPTIVE==1)
//    {
//    	PWMOutEnFlag_ACTIVE = 0;
//   	PWMOutEnFlag_ADAPTIVE = 0;
 //   }

    if (PWMOutEnFlag==0)
    	Preset_PLUS_DT=0;
    else
    {
    	Preset_PLUS_DT=(int) I_ref_PC;
        if (Preset_PLUS_DT<=0)
        	Preset_PLUS_DT=0;
        if (Preset_PLUS_DT>2000)
        	Preset_PLUS_DT=2000;

    }
//---------------------------------------------------------------------------
	if(PWMOutEnFlag == 1)     // If the PWM output is enabled, control the motor.
	{
		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		//            Double pulse parameter setting
		k_full=0; // phaseA: (5A:0) phaseA+B//C: (5A: 1) DPT: (5A:0)
		sw_per= (I_ref_PC/2)*130; // DPT: (5A:724 10A:1296 15A:1838 20A:2363) phaseA: (5A:48997) phaseA+B//C: (5A:37852 )
		Tn=sw_per-150; // DPT: (5A:574 10A:1146 15A:1688 20A:2213) phaseA: (5A:48397) phaseA+B//C: (5A:37252 )
		Tn1=300;// 2 us
		sw_per_2nd=450; // 3 us
		Tn_2nd=300; // 2 us

		//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
		if (pll_count == 0) // first pulse
		{
			if (pll_count_1<k_full)
			{
				// phase A
				EPwm1Regs.TBPRD = sw_per;           // Set timer period
				EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
				EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
				EPwm1Regs.CMPA.half.CMPA = sw_per+1;     // Set PWM1A duty, keep logic high

				EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
				EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm1Regs.CMPB = 0;     // Set PWM1B duty, keep logic low

				// phase B
				EPwm2Regs.TBPRD = sw_per;
				EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
				EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				EPwm2Regs.CMPA.half.CMPA = sw_per+1;     // Set PWM2A duty, keep logic low

				EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
				EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm2Regs.CMPB = sw_per+1;     // Set PWM2B duty

				// phase C
				EPwm3Regs.TBPRD = sw_per;
				EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
				EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				EPwm3Regs.CMPA.half.CMPA = sw_per+1;     // Set PWM3A duty

				EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
				EPwm3Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm3Regs.CMPB = sw_per+1;     // Set PWM2B duty

				pll_count=0;
				pll_count_1=pll_count_1+1;
			}
			else
			{
				// phase A
				EPwm1Regs.TBPRD = sw_per;           // Set timer period
				EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
				EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
				EPwm1Regs.CMPA.half.CMPA = Tn;     // Set PWM1A duty,
				// logic high for Tn duration, then logic low for rest of sw_per

				EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
				EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm1Regs.CMPB = 0;     // Set PWM1B duty, keep logic low

				// phase B
				EPwm2Regs.TBPRD = sw_per;
				EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
				EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				//		   EPwm2Regs.CMPA.half.CMPA = Tn+PLUS_DT;     // Set PWM2A duty
				EPwm2Regs.CMPA.half.CMPA = Tn+(PLUS_DT>>3);     // Set PWM2A duty, logic low for 'Tn+(PLUS_DT>>3)' duration, then logic high the rest
				// test_data_12=EPwm2Regs.CMPA.half.CMPA-Tn;
				EPwm2Regs.CMPA.half.CMPAHR = (((long)(PLUS_DT&7)*MEP_ScaleFactor[2])<<5)+0x180;//0x2000+0x180; //0x180;

//				if (PWMOutEnFlag_ACTIVE==1)
//				{
//					PLUS_DT=Preset_PLUS_DT;
//					EPwm2Regs.CMPA.half.CMPA = Tn+(PLUS_DT>>3);     // Set PWM2A duty
//					EPwm2Regs.CMPA.half.CMPAHR = (((long)(PLUS_DT&7)*MEP_ScaleFactor[2])<<5)+0x180;//0x2000+0x180; //0x180;
//				}

				EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
				EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm2Regs.CMPB = sw_per+1;     // Set PWM2B duty

				// phase C
				EPwm3Regs.TBPRD = sw_per;
				EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
				EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				EPwm3Regs.CMPA.half.CMPA = sw_per+1;     // Set PWM3A duty, keep logic high

				EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
				EPwm3Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm3Regs.CMPB = sw_per+1;     // Set PWM3B duty

				pll_count=1;
				pll_count_1=0;
			}
		}

		else		//pll_count!=0
		{
			if (pll_count == 1) // 2nd pulse
			{
				// phase A
				EPwm1Regs.TBPRD = sw_per_2nd;           // Set timer period
				EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
				EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				EPwm1Regs.CMPA.half.CMPA = 0;     // Set PWM1A duty, keep logic low

				if (PWMOutEnFlag_ACTIVE==1) // intelligent gate drive enabled
				{
					EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
					EPwm1Regs.AQCTLB.bit.ZRO = AQ_SET;
					EPwm1Regs.CMPB = Tn_2nd;     // Set PWM1B duty, logic high for 'Tn_2nd' duration, then low for rest of 'sw_per_2nd'
				}

				else
				{
					EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
					EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
					EPwm1Regs.CMPB = 0;     // Set PWM1B duty, keep logic low
				}

				// phase B
				EPwm2Regs.TBPRD = sw_per_2nd;
				EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
				EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				EPwm2Regs.CMPA.half.CMPA = sw_per_2nd+1;     // Set PWM2A duty, keep logic low

				EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
				EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm2Regs.CMPB = sw_per_2nd+1;     // Set PWM2B duty

				// phase C
				EPwm3Regs.TBPRD = sw_per_2nd;
				EPwm3Regs.AQCTLA.bit.CAU = AQ_CLEAR;
				EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
				EPwm3Regs.CMPA.half.CMPA = Tn_2nd+(PLUS_DT>>3);     // Set PWM2A duty
				EPwm3Regs.CMPA.half.CMPAHR = (((long)(PLUS_DT&7)*MEP_ScaleFactor[3])<<5)+0x180;//0x2000+0x180; //0x180;

				EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
				EPwm3Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
				EPwm3Regs.CMPB = sw_per_2nd+1;     // Set PWM2B duty

				pll_count = 2;
			}
			else		//pll_count>1
			{
				if (pll_count == 2) // 3rd pulse
				{
					EPwm1Regs.TBPRD = sw_per;           // Set timer period
					EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
					EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;
					EPwm1Regs.CMPA.half.CMPA = Tn1;     // Set PWM1A duty, logic high for 'Tn1' duration, then low for rest of 'sw_per'

					EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
					EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
					EPwm1Regs.CMPB = 0;     // Set PWM1B duty, keep logic low

					EPwm2Regs.TBPRD = sw_per;
					EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
					EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
					EPwm2Regs.CMPA.half.CMPA = sw_per+1;     // Set PWM2A duty, keep logic low

					EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
					EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
					EPwm2Regs.CMPB = sw_per+1;     // Set PWM2B duty

					EPwm3Regs.TBPRD = sw_per;
					EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
					EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
					EPwm3Regs.CMPA.half.CMPA = sw_per+1;     // Set PWM2A duty

					EPwm3Regs.AQCTLB.bit.CBU = AQ_SET;
					EPwm3Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
					EPwm3Regs.CMPB = sw_per+1;     // Set PWM2B duty

					pll_count = 3;
				}
				else	//pll_count>2
				{
					// phase A
					EPwm1Regs.TBPRD = sw_per_fix;           // Set timer period
					EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
					EPwm1Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
					EPwm1Regs.CMPA.half.CMPA = 0;     // Set PWM1A duty, keep logic low

					EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
					EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
					EPwm1Regs.CMPB = 0;     // Set PWM1B duty, keep logic low

					// phase B
					EPwm2Regs.TBPRD = sw_per_fix;
					EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
					EPwm2Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
					EPwm2Regs.CMPA.half.CMPA = sw_per_fix+1;     // Set PWM2A duty, keep logic low


					EPwm2Regs.AQCTLB.bit.CBU = AQ_SET;
					EPwm2Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
					EPwm2Regs.CMPB = sw_per_fix+1;     // Set PWM2B duty

					// phase C
					EPwm3Regs.TBPRD = sw_per_fix;
					EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
					EPwm3Regs.AQCTLA.bit.ZRO = AQ_CLEAR;
					EPwm3Regs.CMPA.half.CMPA =  sw_per_fix+1;     // Set PWM2A duty

					EPwm3Regs.AQCTLB.bit.ZRO = AQ_SET;
					EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
					EPwm3Regs.CMPB = sw_per_fix+1;     // Set PWM2B duty

					// set 1s duration for continuous DPT
					if (DPT_Count_a<=DPT_Count_1)
					{
					   DPT_Count_a++;
					}
					else if (DPT_Count_a>DPT_Count_1)
					{
					   DPT_Count_a=0;
					   DPT_Count_b++;
					}

					if (DPT_Count_b>DPT_Count_2)
					{
						DPT_Count_b=0;
						pll_count=0;

						if (PWMOutEnFlag_ADAPTIVE == 0)
						{
							PLUS_DT=0;
							Near_GT_Flag=0;
							Near_DF_Flag=0;
							Near_DT_Flag=0;
							Last_GT_Flag=0;
							Last_DF_Flag=0;
							Last_DT_Flag=0;
							t_GT=0;
							t_DF=0;
							t_DT=0;
							t_TurnOff=0;
							t_delay_off=0;
							DTT_Step=0;
						}
						else if (PWMOutEnFlag_ADAPTIVE == 1)
						{
							if(DTT_Step<10)  //Do this part 10 times for strong accuracy
							{
								//DETECT_L_GVTD_FB DETECT_L_DVTD_FB DETECT_L_DVFD_FB
								if(DETECT_L_GVTD_FB == 0) //this means S_L_aux is lagging GVTD signal
								{
									if(Last_GT_Flag==0 && Near_GT_Flag==0)
										PLUS_DT=PLUS_DT+10;   //Needs a big time jump to get closer to GVTD transition
									else
									{
										Near_GT_Flag=1;
										PLUS_DT=PLUS_DT+1;
									}
								}
								//DETECT_L_GVTD_FB
								else if (DETECT_L_GVTD_FB == 1) //this means S_L_aux is leading GVTD signal
								{
									if(Last_GT_Flag==1 && Near_GT_Flag==0)
										PLUS_DT=PLUS_DT-10;
									else
									{
										Near_GT_Flag=1;
										PLUS_DT=PLUS_DT-1;   //Needs smaller jump to get closer to GVTD transition
									}
								}
								Last_GT_Flag=DETECT_L_GVTD_FB; //DETECT_L_GVTD_FB
							}
							else if(DTT_Step<20)
							{
								if(DETECT_H_DVTD_FB == 0)
								{
									if(Last_DT_Flag==0 && Near_DT_Flag==0)
										PLUS_DT=PLUS_DT+10;
									else
									{
										Near_DT_Flag=1;
										PLUS_DT=PLUS_DT+1;
									}
								}
								else if (DETECT_H_DVTD_FB == 1)
								{
									if(Last_DT_Flag==1 && Near_DT_Flag==0)
										PLUS_DT=PLUS_DT-10;
									else
									{
										Near_DT_Flag=1;
										PLUS_DT=PLUS_DT-1;
									}
								}
								Last_DT_Flag=DETECT_H_DVTD_FB;
							}
							else
							{
								if(DETECT_H_DVFD_FB == 0)
								{
									if(Last_DF_Flag==0 && Near_DF_Flag==0)
										PLUS_DT=PLUS_DT+10;
									else
									{
										Near_DF_Flag=1;
										PLUS_DT=PLUS_DT+1;
									}
								}
								else if (DETECT_H_DVFD_FB == 1)
								{
									if(Last_DF_Flag==1 && Near_DF_Flag==0)
										PLUS_DT=PLUS_DT-10;
									else
									{
										Near_DF_Flag=1;
										PLUS_DT=PLUS_DT-1;
									}
								}
								Last_DF_Flag=DETECT_H_DVFD_FB;
							}
						}
						timeArray[Q_Point]=PLUS_DT;		//*(5.0/6.0) (from datasheet);
						//Just want 4 value array of PLUS_DT values
						Q_Point++;
						if(Q_Point>=4)
							Q_Point=0;
						t_Avg=0;
						for(i=0;i<4;i++)
							t_Avg+=timeArray[i];  //Take the average of the 4 value array
						t_Avg=t_Avg>>2;

						if(Near_GT_Flag)  //Once this flag is triggered, the value is close and increment DTT_Step
							{DTT_Step++;
//							if(DTT_Step>=14)
//							DTT_Step=14;
							}


						if(Near_DT_Flag)
							{DTT_Step++;
//							if(DTT_Step>=29)
//								DTT_Step=29;
							}

						if(DTT_Step<=10)
						{
							t_GT=t_Avg*(5.0/6.0);     //Need t_GT to get delay time calculation
							if(DTT_Step==10)
							{
								Near_GT_Flag=0;
								DTT_Step++;
							}
						}
						else if(DTT_Step<=20)
						{
							t_DT=t_Avg*(5.0/6.0);
							t_delay_off= t_DT-t_GT;   // A final answer!

							if(DTT_Step==20)
							{
								Near_DT_Flag=0;
								DTT_Step++;
							}
						}
						else
						{
							t_DF=t_Avg*(5.0/6.0);
							t_TurnOff=t_DF-t_GT;    // A final answer!

						}
					}
				}
			}
		}
	}	// END OF if(PWMOutEnFlag == 1)

// case 2
//---------------------------------------------------------------------------
	if (PWMOutEnFlag == 0)
	{
		pll_count = 0;
		PLUS_DT=0;
		Near_GT_Flag=0;
		Near_DF_Flag=0;
		Near_DT_Flag=0;
		Last_GT_Flag=0;
		Last_DF_Flag=0;
		Last_DT_Flag=0;
		t_GT=0;
		t_DF=0;
		t_DT=0;
		t_TurnOff=0;
		t_delay_off=0;
		DTT_Step=0;
		EPWMOutDisable();
	}	//END OF if (PWMOutEnFlag == 0)


//---------------------------------------------------------------------------
//            Clear INT flag
//---------------------------------------------------------------------------
	// Clear INT flag for this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}


//=============================================================
// FUNCTION:    error
// DESCRIPTION: An error occurs when the MEP_ScaleFactor [n]
//              calculated from SFO_MEPEn_V5 differs by > +/- 15
//              from the Seed Value in MEP_ScaleFactor[0].
//              SFO_MepEn_V5 returned a "2" (SFO_OUTRANGE_ERROR).
//              The user should:
//              (1) Re-run SFO_MepDis_V5 to re-calibrate
//                  an appropriate seed value.
//              (2) Ensure the code is not calling Mep_En_V5
//                  on a different channel when it is currently
//                  still running on a channel. (Repetitively
//                  call Mep_En_V5 on current channel until an
//                  SFO_COMPLETE ( i.e. 1) is returned.
//              (3) If the out-of-range condition is acceptable
//                  for the application, ignore the "2" and
//                  treat it as a "1" or SFO_COMPLETE.
//
// PARAMETERS:  N/A
// RETURN:      N/A
//=============================================================

void error (void)
{
	ESTOP0;  // Error - MEP_ScaleFactor out of range of Seed - rerun MepDis calibration.
}

void isr_int9pie3_task_fcn(void)
{
	PC_SCIb_recv();

	// communication between Inverter and PC
	// frame beginning
	recv_cnt_PC = recv_cnt_PC + 1;
	if(recv_syn_PC == 0 && ReceivedByte_PC!=0x46) // F
	{
		recv_syn_PC = 0;
		recv_cnt_PC = 0;
	}
	if(recv_syn_PC == 0 && ReceivedByte_PC==0x46) // F
	{
		recv_syn_PC = 1;
		recv_cnt_PC = 1;
	}
	if(recv_syn_PC == 1 && recv_cnt_PC==2 && ReceivedByte_PC==0x45) //// E
	{
		recv_syn_PC = 2;
	}
	if(recv_syn_PC == 1 && recv_cnt_PC==2 && ReceivedByte_PC!=0x45)  // E
	{
		recv_syn_PC = 0;
	}

	if (recv_syn_PC == 2 && recv_cnt_PC==3)
	{
		if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
			I_refH_PC = ReceivedByte_PC-0x30;
		else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
			I_refH_PC = ReceivedByte_PC-0x37;
	}
	if (recv_syn_PC == 2 && recv_cnt_PC==4)
	{
		if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
			I_refL_PC = ReceivedByte_PC-0x30;
		else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
			I_refL_PC = ReceivedByte_PC-0x37;

		I_ref_PC = I_refH_PC * 16 + I_refL_PC;
	}
	if (recv_syn_PC == 2 && recv_cnt_PC==5)
	{
		if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
			Charge_refH_PC = ReceivedByte_PC-0x30;
		else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
			Charge_refH_PC = ReceivedByte_PC-0x37;
	}
	if (recv_syn_PC == 2 && recv_cnt_PC==6)
	{
		if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
			Charge_refL_PC = ReceivedByte_PC-0x30;
		else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
			Charge_refL_PC = ReceivedByte_PC-0x37;

		//Vdc_refL_PC = ReceivedByte_PC;
		Charge_ref_PC = Charge_refH_PC * 16 + Charge_refL_PC; // e.g. 1356 = 13*100+56;

		//P_ref_PC = P_refH_PC * 16 + P_refL_PC; // e.g. 1356 = 13*100+56;
		//I_ref_PC = I_refH_PC * 16 + I_refL_PC;


		//recv_cnt_PC = 0;
		//recv_syn_PC = 0;
	}

	if (recv_syn_PC == 2 && recv_cnt_PC==7)
		{
			if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
				Run_refH_PC = ReceivedByte_PC-0x30;
			else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
				Run_refH_PC = ReceivedByte_PC-0x37;
		}
		if (recv_syn_PC == 2 && recv_cnt_PC==8)
		{
			if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
				Run_refL_PC = ReceivedByte_PC-0x30;
			else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
				Run_refL_PC = ReceivedByte_PC-0x37;

			//Vdc_refL_PC = ReceivedByte_PC;
			Run_ref_PC = Run_refH_PC * 16 + Run_refL_PC; // e.g. 1356 = 13*100+56;

			//P_ref_PC = P_refH_PC * 16 + P_refL_PC; // e.g. 1356 = 13*100+56;
			//I_ref_PC = I_refH_PC * 16 + I_refL_PC;


			//recv_cnt_PC = 0;
			//recv_syn_PC = 0;
		}

		if (recv_syn_PC == 2 && recv_cnt_PC==9)
				{
					if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
						Stop_refH_PC = ReceivedByte_PC-0x30;
					else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
						Stop_refH_PC = ReceivedByte_PC-0x37;
				}
				if (recv_syn_PC == 2 && recv_cnt_PC==10)
				{
					if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
						Stop_refL_PC = ReceivedByte_PC-0x30;
					else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
						Stop_refL_PC = ReceivedByte_PC-0x37;

					//Vdc_refL_PC = ReceivedByte_PC;
					Stop_ref_PC = Stop_refH_PC * 16 + Stop_refL_PC; // e.g. 1356 = 13*100+56;

					//P_ref_PC = P_refH_PC * 16 + P_refL_PC; // e.g. 1356 = 13*100+56;
					//I_ref_PC = I_refH_PC * 16 + I_refL_PC;


					//recv_cnt_PC = 0;
					//recv_syn_PC = 0;
				}

			if (recv_syn_PC == 2 && recv_cnt_PC==11)
					{
						if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
							Reset_refH_PC = ReceivedByte_PC-0x30;
						else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
							Reset_refH_PC = ReceivedByte_PC-0x37;
					}
						if (recv_syn_PC == 2 && recv_cnt_PC==12)
						{
							if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
								Reset_refL_PC = ReceivedByte_PC-0x30;
							else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
								Reset_refL_PC = ReceivedByte_PC-0x37;

									//Vdc_refL_PC = ReceivedByte_PC;
							Reset_ref_PC = Reset_refH_PC * 16 + Reset_refL_PC; // e.g. 1356 = 13*100+56;

									//P_ref_PC = P_refH_PC * 16 + P_refL_PC; // e.g. 1356 = 13*100+56;
									//I_ref_PC = I_refH_PC * 16 + I_refL_PC;


							//recv_cnt_PC = 0;
							//recv_syn_PC = 0;
					}

				if (recv_syn_PC == 2 && recv_cnt_PC==13)
						{
							if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
								Estop_refH_PC = ReceivedByte_PC-0x30;
							else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
								Estop_refH_PC = ReceivedByte_PC-0x37;
						}
							if (recv_syn_PC == 2 && recv_cnt_PC==14)
							{
								if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
									Estop_refL_PC = ReceivedByte_PC-0x30;
								else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
									Estop_refL_PC = ReceivedByte_PC-0x37;

											//Vdc_refL_PC = ReceivedByte_PC;
						Estop_ref_PC = Estop_refH_PC * 16 + Estop_refL_PC; // e.g. 1356 = 13*100+56;

											//P_ref_PC = P_refH_PC * 16 + P_refL_PC; // e.g. 1356 = 13*100+56;
											//I_ref_PC = I_refH_PC * 16 + I_refL_PC;

						PC_scib_xmit(i_monitor_H);
							//recv_cnt_PC = 0;
							//recv_syn_PC = 0;
						}

				if (recv_syn_PC == 2 && recv_cnt_PC==15)
							{
								if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
									Backup_refH_PC = ReceivedByte_PC-0x30;
								else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
									Backup_refH_PC = ReceivedByte_PC-0x37;
							}
								if (recv_syn_PC == 2 && recv_cnt_PC==16)
								{
									if( (ReceivedByte_PC>=0x30) &&  (ReceivedByte_PC<=0x39) ) // 0--9
										Backup_refL_PC = ReceivedByte_PC-0x30;
									else if( (ReceivedByte_PC>=0x41) &&  (ReceivedByte_PC<=0x46) ) // A--F
										Backup_refL_PC = ReceivedByte_PC-0x37;

												//Vdc_refL_PC = ReceivedByte_PC;
							Backup_ref_PC = Backup_refH_PC * 16 + Backup_refL_PC; // e.g. 1356 = 13*100+56;

																		//P_ref_PC = P_refH_PC * 16 + P_refL_PC; // e.g. 1356 = 13*100+56;
																		//I_ref_PC = I_refH_PC * 16 + I_refL_PC;

							PC_scib_xmit(i_monitor_L);
								recv_cnt_PC = 0;
								recv_syn_PC = 0;
							}
							
	// frame end


//	i_t_TurnOff=((int) (6*t_TurnOff)+2);
	if (Estop_ref_PC ==1)
		i_monitor = ((int) (100*t_TurnOff)+2);
	else
		i_monitor = ((int) (100*t_delay_off)+2);
		//i_monitor = t_delay_off;

	i_monitor_H=(i_monitor>>8);
	i_monitor_L=(i_monitor-i_monitor_H*256);


}


interrupt void SCIRXINTB_isr(void)  // with upper PC, lower than BTB communication  int9.3,
{
  volatile unsigned int PIEIER3_stack_save = PieCtrlRegs.PIEIER3.all;
  volatile unsigned int PIEIER9_stack_save = PieCtrlRegs.PIEIER9.all;
  PieCtrlRegs.PIEIER9.all &= ~4;       //disable group9 lower/equal priority interrupts    int9.3
  asm(" RPT #5 || NOP");               //wait 5 cycles
  IFR &= ~257;                         //eventually disable lower/equal priority pending interrupts   // group 1 and 9
  PieCtrlRegs.PIEACK.all = 257;        //ACK to allow other interrupts from the same group to fire
  IER |= 256;                         //group9 int9
  EINT;                                //global interrupt enable
  isr_int9pie3_task_fcn();
  DINT;                                // disable global interrupts during context switch, CPU will enable global interrupts after exiting ISR
  PieCtrlRegs.PIEIER3.all = PIEIER3_stack_save;//restore PIEIER register that was modified
  PieCtrlRegs.PIEIER9.all = PIEIER9_stack_save;//restore PIEIER register that was modified
  EALLOW;
  ScibRegs.SCIFFRX.bit.RXFFINTCLR = 1;

  //add by Bo   11/5/2013
  ScibRegs.SCICTL1.bit.SWRESET = 1;    //Software reset

  EDIS;
  PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;// Acknowledge to receive more interrupts
}


void init_SCI(void)
{                                      /* initialize SCI & FIFO registers */
  EALLOW;

  // SCI-A
	SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol
	SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
	SciaRegs.SCICTL2.bit.RXBKINTENA =1;

	SciaRegs.SCIHBAUD    =0;  // 115200 baud @LSPCLK = 37.5MHz.
	SciaRegs.SCILBAUD    =39;

	SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
//*/

	//SCI-B
	ScibRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
								   // No parity,8 char bits,
								   // async mode, idle-line protocol
	ScibRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
								   // Disable RX ERR, SLEEP, TXWAKE
	ScibRegs.SCICTL2.bit.RXBKINTENA =1;

	ScibRegs.SCIHBAUD    =0;  // 115200 baud @LSPCLK = 37.5MHz.
	ScibRegs.SCILBAUD    =39;

	ScibRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset

  EDIS;
}

void init_SCI_GPIO(void)
{
  EALLOW;
  //SCIA
  GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;  //Enable pull-up for GPIO28
  GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1; //Configure GPIO28 as SCIRXDA
  GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;  //Enable pull-up for GPIO29
  GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1; //Configure GPIO29 as SCITXDA

  //SCIB
  GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;  //Enable pull-up for GPIO11
  GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2; //Configure GPIO11 as SCIRXDB    becaful of here, A, B and C module are not the same
  GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;  //Enable pull-up for GPIO9
  GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2; //Configure GPIO9 as SCITXDB

  EDIS;
}


//===========================================================================
// No more.
//===========================================================================
