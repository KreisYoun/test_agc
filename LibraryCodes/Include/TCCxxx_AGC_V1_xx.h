#ifndef TCC_AGC_H
#define TCC_AGC_H

#define TC_AGC_VERSION "0.02.01"



// [1] AGC control Parameters
typedef void *	tc_agc_t;
typedef struct tc_agc_params_t
{
	// Input Parameters
	int		m_iFrameSize;		// Frame Size
	int		m_iSampleRate;		// Sample Rate

	int		m_iThrdB;			// AGC Threshold				[default 0]
	int		m_iAttdB;			// AGC Max Attenuation Gain [default 0]
	int		m_iStepSize;		// AGC Attack Step Size		[default 2]
	int		m_iStepSize_r;		// AGC Release Step Size		[default 2]
}tc_agc_params_t;
// AGC Step Size, option(dB) : 0(1dB), 1(0.5), 2(0.25), 3(0.125), 4(0.0625), 5(0.03125), 6(0.015625), 7(0.007813) 



// [2] Return Code
// Init
#define	TC_AGC_INIT_SUCCESS		(0x00000000)
#define	TC_AGC_INIT_INVALID		(0x80000001)  // -2147483647
// Process
#define	TC_AGC_PROC_SUCCESS		(0x00000000)
// Finish
#define	TC_AGC_FNSH_SUCCESS		(0x00000000)
#define	TC_AGC_FNSH_FAIL		(0x80000001)  // -2147483647
// Control
#define	TC_AGC_CTRL_SUCCESS		(0x00000000)
#define	TC_AGC_CTRL_INVALID		(0x80000001)  // -2147483647
#define	TC_AGC_CTRL_FAIL		(0x80000002)  // -2147483646



// [3] AGC Parameter Control
#define	TC_AGC_PARAM_THRESHOLD_DB	(0)
#define	TC_AGC_PARAM_ATTENUATION_DB	(1)
#define	TC_AGC_PARAM_ATTACK_STEP	(2)
#define	TC_AGC_PARAM_RELEASE_STEP	(3)



// [4] AGC Functions
/**********************************************************************
* Breif
*		TC_AGC_Init	: Initializes AGC
* I/O
*		sAGCParams		: AGC parameter structure
* Return
*		If Successful, TC_AGC_Init returns 0. 
*		Otherwise, it returns a negative value. 
**********************************************************************/
int TC_AGC_Init(tc_agc_t *st_agc, tc_agc_params_t sAGCParams);


/**********************************************************************
* Breif
*		TC_AGC_Control: sAGCParams structure member setting
* I/O
*		sAGCParams		: AGC parameter structure
*		Item			: Item index for sAGCParams structure members,
*						  use AGC parameters control #define
*		ControlValue: appropriate value to set
* Return
*		If Successful 0, Otherwise -1(Item) or -2(ControlValue Range).
**********************************************************************/
int TC_AGC_Control(tc_agc_t *st_agc, tc_agc_params_t *sAGCParams, int Item, int ControlValue);


/**********************************************************************
* Breif
*		TC_AGC_Process: Apply AGC to psInPcm using sAGCParams,
*						  then output to psOutPcm
* I/O
*		sAGCParams		: AGC parameter structure
*		psInPcm		: Input pcm buffer
*		psOutPcm		: Output pcm buffer
**********************************************************************/
int TC_AGC_Process(void *st_agc, tc_agc_params_t sAGCParams, short *psInPcm, short *psOutPcm);


/**********************************************************************
* Breif
*		TC_AGC_Finish	: Close AGC structure, must be called to free agc handle
**********************************************************************/
int TC_AGC_Finish(void *st_agc);


#endif
