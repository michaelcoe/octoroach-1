/*
 * Name: SetupADC.c
 * Desc: Configure the analog to digital converter.
 * Date: 2009-04-02
 * Author: fgb
 */

#include "adc.h"
#include "adc_pid.h"
#include "p33Fxxxx.h"
#include "ports.h"

//Functions
static void adcSetupPeripheral(void);
//DMA related functions
static void initDma0(void);
void __attribute__((__interrupt__)) _DMA0Interrupt(void);


//Variables to store values as they come out of the DMA buffer
static unsigned int adc_bemfL;
static unsigned int adc_bemfR;
static unsigned int adc_battery;
static unsigned int adc_AN3;

void adcSetup(void){
	adcSetupPeripheral();
	initDma0(); //DMA is needed to read multiple values from the ADC core
}

static void adcSetupPeripheral(void) {
    unsigned int AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value,
            AD1PCFGHvalue, AD1PCFGLvalue, AD1CSSHvalue, AD1CSSLvalue,
            AD1CHS0value, AD1CHS123value;

    AD1CON1value = ADC_MODULE_ON & //ADC module is enabled
            ADC_IDLE_CONTINUE & // ADC will continue in idle mode
            ADC_AD12B_10BIT & // ADC in 10 bit mode
            ADC_FORMAT_INTG & // ADC in integer format (CLARIFY)
            ADC_CLK_MPWM & // MCPWM interval ends sampling and starts conversion
            ADC_MULTIPLE & //Sequential sampling
            ADC_ADDMABM_ORDER & //DMA buffers are written in the order of conversion
            ADC_AUTO_SAMPLING_ON & //ADC does not need to be triggered manually
            ADC_SAMP_ON; //sample / hold amplifiers are sampling  (maybe incorrect)
    AD1CON2value = ADC_VREF_AVDD_AVSS & //Vref+ = AVdd , Vref- = AVss
            ADC_SCAN_ON & //Scan through ADC channels
            ADC_SELECT_CHAN_0 & //Only CH0, will scan through all 5 pins
            ADC_ALT_BUF_OFF & //Use one 16 word buffer
            ADC_ALT_INPUT_OFF & // Alternate between MUXA and MUXB
            ADC_DMA_ADD_INC_1; //Increment DMA address after each sample
    AD1CON3value = ADC_CONV_CLK_SYSTEM & //Use System clock, not internal RC osc
            ADC_CONV_CLK_3Tcy & //Tad = 3 * Tcy TODO: Check this
            ADC_SAMPLE_TIME_1; //Sample Time = 1*Tad
    AD1CON4value = ADC_DMA_BUF_LOC_1; //This may be wrong (TODO)


    AD1CHS123value = 0; //Dummy value, CH 1,2,3 unused

    AD1CHS0value = ADC_CH0_NEG_SAMPLEA_VREFN & // Sample A, Vref- = AVss = ground
            ADC_CH0_POS_SAMPLEA_AN0 & // Battery
            ADC_CH0_POS_SAMPLEA_AN8 & // Motor A
            ADC_CH0_POS_SAMPLEA_AN9 & // Motor B
            ADC_CH0_POS_SAMPLEA_AN10& // Motor C
            ADC_CH0_POS_SAMPLEA_AN11; // Motor D


    AD1CSSHvalue = SCAN_NONE_16_31; //Skip AN16-AN131 for Input Scan, not avail on dsPic33
    //Scan: AN0, AN8, AN9, AN10, AN11
    AD1CSSLvalue = SCAN_ALL_0_15 & SKIP_SCAN_AN1 & SKIP_SCAN_AN2 &
            SKIP_SCAN_AN3 & SKIP_SCAN_AN4 & SKIP_SCAN_AN5 &
            SKIP_SCAN_AN6 & SKIP_SCAN_AN7 & SKIP_SCAN_AN12 &
            SKIP_SCAN_AN13 & SKIP_SCAN_AN14 & SKIP_SCAN_AN15; //Leaving: AN0, AN8, AN9, AN10, AN11

    //Set pins to analog inputs; also check init_default.c
    AD1PCFGHvalue = ENABLE_ALL_DIG_16_31; //Shouldn't matter, only AN0-15 on 706A
    AD1PCFGLvalue = ENABLE_AN0_ANA & //Battery
            ENABLE_AN8_ANA & //Motor A
            ENABLE_AN9_ANA & //Motor B
            ENABLE_AN10_ANA & //Motor C
            ENABLE_AN11_ANA; //Motor D

    SetChanADC1(AD1CHS123value, AD1CHS0value);
    OpenADC1(AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value, AD1PCFGLvalue, AD1PCFGHvalue, AD1CSSHvalue, AD1CSSLvalue);

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; //Disable A/D interrupt
}


//For testing purposes, should not be enabled
/*
void __attribute__((interrupt,no_auto_psv)) _ADC1Interrupt(void)
{
	//ADC sync indicator
	if(AD1CON1bits.DONE){
		LATB |= (1<<4);
	}else{
		LATB &= ~(1<<4);
	}
	IFS0bits.AD1IF = 0;
}
*/

//Getters for other modules to access values
unsigned int adcGetBEMFL(){
	return adc_bemfL;
}

unsigned int adcGetBEMFR(){
	return adc_bemfR;
}

unsigned int adcGetVBatt(){
	return adc_battery;
}

unsigned int adcGetAN3(){
	return adc_AN3;
}


//////////////////////////////////////////////////////////////////////
///////////////      DMA Section     /////////////////////////////////
//////////////////////////////////////////////////////////////////////

#define  SAMP_BUFF_SIZE	 		1		// Size of the input buffer per analog input

//Buffers need special attribute to be in DMA memory space
static int  BufferA[4][SAMP_BUFF_SIZE] __attribute__((space(dma)));
static int  BufferB[4][SAMP_BUFF_SIZE] __attribute__((space(dma)));

static unsigned int DmaBuffer = 0;



/*****************************************************************************
* Function Name : initDma0
* Description   : Setup function for DMA0, to read ADC1 into a buffer
* Parameters    : None
* Return Value  : None
*****************************************************************************/
static void initDma0(void)
{
	DMA0CONbits.AMODE = 0;			// Configure DMA for Register Indirect w/ post-increment
	DMA0CONbits.MODE  = 2;			// Configure DMA for Continuous Ping-Pong mode
	
	DMA0PAD=(int)&ADC1BUF0;
	//DMA0CNT = (SAMP_BUFF_SIZE*2)-1;					
	DMA0CNT = 3;  //See dsPIC user's manual. 4 analog reads -> DMA0CNT = 4-1 = 3
	//DMA0CNT = 7;

	DMA0REQ=13; //ADC1 requests

	DMA0STA = __builtin_dmaoffset(BufferA);		
	DMA0STB = __builtin_dmaoffset(BufferB);

	IFS0bits.DMA0IF = 0;			//Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1;			//Set the DMA interrupt enable bit

	DMA0CONbits.CHEN=1;
}

/*****************************************************************************
* Function Name : _DMA0Interrupt
* Description   : Interrupt hander for DMA0 , associated with ADC1 here.
				  Motor BEMF vales are set through setter functions.
* Parameters    : None
* Return Value  : None
*****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	if(DmaBuffer==0) {
		adc_bemfL = 	BufferA[0][0];	//AN11
		adc_battery = 	BufferA[1][0];	//AN0
		adc_bemfR = 	BufferA[2][0];	//AN1
		adc_AN3 = 		BufferA[3][0];	//AN3

	} else {

		adc_bemfL = 	BufferB[0][0];	//AN11
		adc_battery = 	BufferB[1][0];	//AN0
		adc_bemfR = 	BufferB[2][0];	//AN1
		adc_AN3 = 		BufferB[3][0];	//AN3
	}

	DmaBuffer ^= 1;	 //Toggle between buffers
	IFS0bits.DMA0IF = 0;		//Clear the DMA0 Interrupt Flag
}
// End DMA section
