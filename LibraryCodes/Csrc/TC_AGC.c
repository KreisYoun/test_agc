
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "TCCxxx_AGC_V1_xx.h"
#include "TC_AGC.h"
#include "math.h"

#ifdef ARM_SIM
#include "TC_AGC_debug.h"
#endif


#if !defined (FLOATING_POINT)
#define TCAGC_GAIN_EXT_BIT			(7)		// Extension Bit
#define TCAGC_GAIN_EXT				(1<<TCAGC_GAIN_EXT_BIT) // 128scale (7bit EXT)

// Table for Find the Linear to Log value
// TBL : -50(TCAGC_L2DB_TBL_START) ~ 0dB, step = 0.125dB //
#define TCAGC_L2DB_TBL_SIZE		(401)	// SIZE
#define TCAGC_L2DB_TBL_START		(-50)	// Start = -50dB
#define TCAGC_L2DB_C_SRCH			(20)	// Coarse Search Depth

#define TCAGC_L2DB_TBL_STEP		(8)	// Step = 0.25dB
#define TCAGC_L2DB_TBL_STEP_BIT	(3)	// Bit for TCAGC_L2DB_TBL_STEP
#define TCAGC_L2DB_EXT_BIT			(15)	// Extension Bit

int TCAGC_L2DB_TBL [TCAGC_L2DB_TBL_SIZE] = {
  104,   105,   107,   108,   110,   111,   113,   115,   116,   118, 
  120,   121,   123,   125,   127,   129,   130,   132,   134,   136, 
  138,   140,   142,   144,   146,   148,   151,   153,   155,   157, 
  160,   162,   164,   167,   169,   171,   174,   176,   179,   182, 
  184,   187,   190,   192,   195,   198,   201,   204,   207,   210, 
  213,   216,   219,   222,   225,   229,   232,   235,   239,   242, 
  246,   249,   253,   257,   260,   264,   268,   272,   276,   280, 
  284,   288,   292,   296,   301,   305,   309,   314,   318,   323, 
  328,   332,   337,   342,   347,   352,   357,   362,   368,   373, 
  378,   384,   389,   395,   401,   407,   413,   419,   425,   431, 
  437,   443,   450,   456,   463,   470,   476,   483,   490,   497, 
  505,   512,   519,   527,   535,   542,   550,   558,   566,   574, 
  583,   591,   600,   608,   617,   626,   635,   644,   654,   663, 
  673,   683,   693,   703,   713,   723,   734,   744,   755,   766, 
  777,   788,   800,   811,   823,   835,   847,   859,   872,   885, 
  897,   910,   924,   937,   950,   964,   978,   992,  1007,  1021, 
 1036,  1051,  1066,  1082,  1098,  1114,  1130,  1146,  1163,  1180, 
 1197,  1214,  1232,  1249,  1268,  1286,  1305,  1323,  1343,  1362, 
 1382,  1402,  1422,  1443,  1464,  1485,  1506,  1528,  1550,  1573, 
 1596,  1619,  1642,  1666,  1690,  1715,  1740,  1765,  1790,  1816, 
 1843,  1869,  1896,  1924,  1952,  1980,  2009,  2038,  2068,  2097, 
 2128,  2159,  2190,  2222,  2254,  2287,  2320,  2353,  2388,  2422, 
 2457,  2493,  2529,  2566,  2603,  2641,  2679,  2718,  2757,  2797, 
 2838,  2879,  2920,  2963,  3006,  3049,  3093,  3138,  3184,  3230, 
 3277,  3324,  3372,  3421,  3471,  3521,  3572,  3624,  3677,  3730, 
 3784,  3839,  3894,  3951,  4008,  4066,  4125,  4185,  4246,  4307, 
 4370,  4433,  4497,  4562,  4629,  4696,  4764,  4833,  4903,  4974, 
 5046,  5119,  5193,  5269,  5345,  5423,  5501,  5581,  5662,  5744, 
 5827,  5912,  5997,  6084,  6172,  6262,  6353,  6445,  6538,  6633, 
 6729,  6827,  6925,  7026,  7128,  7231,  7336,  7442,  7550,  7659, 
 7771,  7883,  7997,  8113,  8231,  8350,  8471,  8594,  8719,  8845, 
 8973,  9103,  9235,  9369,  9505,  9643,  9783,  9924, 10068, 10214, 
10362, 10512, 10665, 10819, 10976, 11135, 11297, 11460, 11627, 11795, 
11966, 12139, 12315, 12494, 12675, 12859, 13045, 13234, 13426, 13621, 
13818, 14018, 14222, 14428, 14637, 14849, 15064, 15283, 15504, 15729, 
15957, 16188, 16423, 16661, 16902, 17147, 17396, 17648, 17904, 18164, 
18427, 18694, 18965, 19240, 19519, 19802, 20089, 20380, 20675, 20975, 
21279, 21587, 21900, 22218, 22540, 22867, 23198, 23534, 23875, 24221, 
24573, 24929, 25290, 25657, 26029, 26406, 26789, 27177, 27571, 27970, 
28376, 28787, 29205, 29628, 30057, 30493, 30935, 31383, 31838, 32300, 
32768};
#endif //#if !defined (FLOATING_POINT)




// ==========================================//
// ========= [1] TC_AGC_Interface ===========//
// ==========================================//
int TC_AGC_Init(tc_agc_t *st_agc, tc_agc_params_t sAGCParams)
{
	int i;
	// AGC Handler
	struct agc_handle *st;
	
#if defined (TCAGC_GAIN_TBL_GEN) && !defined (FLOATING_POINT)
	TCAGC_L2DB_table_gen_fix(TCAGC_L2DB_TBL_STEP);
#endif

	// Invalid Range Detector
	if(sAGCParams.m_iFrameSize == 0)
		return TC_AGC_INIT_INVALID;
	if(sAGCParams.m_iSampleRate == 0)
		return TC_AGC_INIT_INVALID;
	if(sAGCParams.m_iThrdB < AGC_THR_MAX || sAGCParams.m_iThrdB > 0)
		return TC_AGC_INIT_INVALID;
	if(sAGCParams.m_iAttdB < AGC_ATT_MAX || sAGCParams.m_iAttdB > 0)
		return TC_AGC_INIT_INVALID;


	*st_agc = (struct agc_handle *)malloc(sizeof(struct agc_handle));
	st = (struct agc_handle *)*st_agc;

    if(st == NULL)
        return TC_AGC_INIT_INVALID;

	// [1] Control Parameter Store
	st->frame_size  = sAGCParams.m_iFrameSize;
	st->sample_rate = sAGCParams.m_iSampleRate;
	st->thr_dB = sAGCParams.m_iThrdB;
	st->att_dB = sAGCParams.m_iAttdB;
	st->step_dB	  = sAGCParams.m_iStepSize;
	st->step_dB_r = sAGCParams.m_iStepSize_r;

	// [2] VAD Parameter Initialization	
	st->vad_params.m_iCount = -1;
	st->vad_params.m_iMinAverage = 0x7fffffff;
	st->vad_params.m_iTempAverage = 0x7fffffff;

	for(i = 0 ; i < AGC_IIR_NUM+1 ; i++)
	{
		st->vad_params.m_iPastSampley[i] = 0;
		st->vad_params.m_iPastSamplex[i] = 0;
	}
	
	// [3] Peak Detector Paramter Initialization
	st->pd_params.m_iCount = -1;
	st->pd_params.m_iSlowAverage = 0;
	st->pd_params.m_iPartAverage = 0 ;
	st->pd_params.m_iSubSamp = st->frame_size/4;


	// [4] Gain Control Parameter Initialization
#if defined (FLOATING_POINT)
	st->thr_linear = (float)pow(10.f, ((float)st->thr_dB)/20.f) * (float)pow(2.f, 15.f);
	
	st->gain_params.m_iCount = 0;
	st->gain_params.m_Apply_Gain = 0.f;
	
	/********** Attack & Release Step **********
	step_dB = 0	  ==> Gain_step = 1;		[dB]
	step_dB = 1	  ==> Gain_step = 0.5;
	step_dB = 2	  ==> Gain_step = 0.25;
	step_dB = 3	  ==> Gain_step = 0.125;
	step_dB = 4	  ==> Gain_step = 0.0625;
	step_dB = 5	  ==> Gain_step = 0.03125;
	step_dB = 6	  ==> Gain_step = 0.015625;
	step_dB = 7	  ==> Gain_step = 0.007812;
	step_dB = other ==> Gain_step = 0.25;
	*******************************************/
	// [4-1] Attack Step
	if(st->step_dB > 7)		
		st->step_dB = 2;	
	st->gain_params.m_Gain_step = 1.f / (float)pow(2.f, (float)st->step_dB);

	// [4-2] Release Step
	if(st->step_dB_r > 7)		
		st->step_dB_r = 2;	
	st->gain_params.m_Gain_step_r = 1.f / (float)pow(2.f, (float)st->step_dB_r);	
#else
	st->thr_linear = TCAGC_log2lin(st->thr_dB<<TCAGC_L2DB_TBL_STEP_BIT);
	
	st->gain_params.m_iCount = 0;
	st->gain_params.m_Apply_Gain = 0;	
	
	/********** Attack & Release Step **********
	step_dB = 0	  ==> Gain_step = 128; [dB ]
	step_dB = 1	  ==> Gain_step = 64;  TCAGC_GAIN_EXT
	step_dB = 2	  ==> Gain_step = 32;
	step_dB = 3	  ==> Gain_step = 16;
	step_dB = 4	  ==> Gain_step = 8;
	step_dB = 5	  ==> Gain_step = 4;
	step_dB = 6	  ==> Gain_step = 2;
	step_dB = 7	  ==> Gain_step = 1;
	step_dB = other ==> Gain_step = 32;
	*******************************************/
	// [4-1] Attack Step
	if(st->step_dB > 7)		
		st->step_dB = 2;	
	st->gain_params.m_Gain_step = (128 >> st->step_dB);

	// [4-2] Release Step
	if(st->step_dB_r > 7)		
		st->step_dB_r = 2;	
	st->gain_params.m_Gain_step_r = (128 >> st->step_dB_r);
#endif

	for(i = 0 ; i < 10 ; i++)
		st->gain_params.m_thr_chk_buff[i] = 0;
	st->gain_params.m_thr_chk_cnt = 0;
	st->gain_params.m_thr_chk_thr = 5;
	
	return TC_AGC_INIT_SUCCESS;
}



int TC_AGC_Finish(void *h)
{	
	if(h == NULL)
		return TC_AGC_FNSH_FAIL;
	else
		free(h);

	return TC_AGC_FNSH_SUCCESS;
}



int TC_AGC_Control(tc_agc_t *st_agc, tc_agc_params_t *sAGCParams, int Item, int ControlValue)
{
	struct agc_handle *st;
	st = (struct agc_handle *)st_agc;

#if defined (FLOATING_POINT)
	switch(Item)
	{
	case TC_AGC_PARAM_THRESHOLD_DB:
		if(ControlValue < AGC_THR_MAX || ControlValue > 0)
			return TC_AGC_CTRL_INVALID;
		
		sAGCParams->m_iThrdB = ControlValue;
		st->thr_dB = ControlValue;

		st->thr_linear = (float)pow(10.f, ((float)st->thr_dB)/20.f) * (float)pow(2.f, 15.f);
		break;

	case TC_AGC_PARAM_ATTENUATION_DB:
		if(ControlValue < AGC_ATT_MAX || ControlValue > 0)
			return TC_AGC_CTRL_INVALID;

		sAGCParams->m_iAttdB = ControlValue;
		st->att_dB = ControlValue;

		break;

	case TC_AGC_PARAM_ATTACK_STEP:
		sAGCParams->m_iStepSize = ControlValue;
		st->step_dB = ControlValue;

		if(st->step_dB > 7)		
			st->step_dB = 2;	
		st->gain_params.m_Gain_step = 1.f / (float)pow(2.f, (float)st->step_dB);
		break;

	case TC_AGC_PARAM_RELEASE_STEP:
		sAGCParams->m_iStepSize_r = ControlValue;
		st->step_dB_r = ControlValue;

		if(st->step_dB_r > 7)		
			st->step_dB_r = 2;	
		st->gain_params.m_Gain_step_r = 1.f / (float)pow(2.f, (float)st->step_dB_r);
		break;
	
	default :
		return TC_AGC_CTRL_FAIL;
	}
#else
	switch(Item)
	{
	case TC_AGC_PARAM_THRESHOLD_DB:
		if((ControlValue < AGC_THR_MAX) || (ControlValue > 0))
			return TC_AGC_CTRL_INVALID;
		sAGCParams->m_iThrdB = ControlValue;
		st->thr_dB = ControlValue;

		st->thr_linear = TCAGC_log2lin(st->thr_dB<<TCAGC_L2DB_TBL_STEP_BIT);
		break;

	case TC_AGC_PARAM_ATTENUATION_DB:
		if((ControlValue < AGC_ATT_MAX) || (ControlValue > 0))
			return TC_AGC_CTRL_INVALID;
		sAGCParams->m_iAttdB = ControlValue;
		st->att_dB = ControlValue;
		break;

	case TC_AGC_PARAM_ATTACK_STEP:
		sAGCParams->m_iStepSize = ControlValue;
		st->step_dB = ControlValue;

		if(st->step_dB > 7)		
			st->step_dB = 2;	
		st->gain_params.m_Gain_step = (128 >> st->step_dB);
		break;

	case TC_AGC_PARAM_RELEASE_STEP:
		sAGCParams->m_iStepSize_r = ControlValue;
		st->step_dB_r = ControlValue;		

		if(st->step_dB_r > 7)		
			st->step_dB_r = 2;	
		st->gain_params.m_Gain_step_r = (128 >> st->step_dB_r);
		break;
	
	default :
		return TC_AGC_CTRL_FAIL;
	}

#endif
	return TC_AGC_CTRL_SUCCESS;
}



int TC_AGC_Process(void *h, tc_agc_params_t psAGCHandle, short *psInPcm, short *psOutPcm)
{	
	int i, vad_result;
	struct agc_handle *st;
		
#if	TCAGC_DBG_ENABLE
	static char	pc_temp_filename_buffer[200] = {0,};	
	static int		i_frame_time_msec;
	static int		dbg_FileOpenFlag = 1;
	static FILE *	dbg_File;

	if(dbg_FileOpenFlag) 
	{
		dbg_FileOpenFlag = 0;
		TCAGC_dbg_file_naming(gcFileName, "_final_debug.cvs", pc_temp_filename_buffer);

		dbg_File = fopen(pc_temp_filename_buffer, "wb");
		if(dbg_File) {
			printf("_final debug file debugging open\n");
			fprintf(dbg_File,"Frame/Index\t");
			fprintf(dbg_File,"time\t");
			fprintf(dbg_File,"time1\t");			
			fprintf(dbg_File,"Input\t");
			fprintf(dbg_File,"VAD\t");
			fprintf(dbg_File,"PD_Slow\t");
			fprintf(dbg_File,"PD_Fast\t");
			fprintf(dbg_File,"Applied_Gain\t");
			fprintf(dbg_File,"Output\n");
			//fprintf(dbg_File,"dec_rule\t");
			//fprintf(dbg_File,"dbg_rule\t");
			//fprintf(dbg_File,"frame_vad\t");
			//fprintf(dbg_File,"Out_rptr\t");
			//fprintf(dbg_File,"Out_valid\n");
		}
	}
#endif	

	st = (struct agc_handle *)h;

	// NO AGC
	if(st->thr_dB >= 0 || st->att_dB >= 0)
	{
		for (i = 0 ; i < st->frame_size ; i++)
			psOutPcm[i] = psInPcm[i];
	}	
	else // AGC
	{
		// VAD
		vad_result = TCAGC_detect_speech_activity(&st->vad_params, psInPcm, st->frame_size);

		// Peak Detector
		TCAGC_peak_detector(psInPcm, vad_result, &st->pd_params, st->frame_size);

#if defined (FLOATING_POINT)
		// AGC Gain Calculation	
		TCAGC_gain_control(vad_result, st->pd_params, &st->gain_params, st->thr_linear, st->att_dB);

		// Apply Gain
		TCAGC_gain_apply(psInPcm, psOutPcm, st->gain_params.m_Apply_Gain, st->frame_size);
#else
		// AGC Gain Calculation	
		TCAGC_gain_control_fix(vad_result, st->pd_params, &st->gain_params, st->thr_linear, st->att_dB);

		// Apply Gain
		TCAGC_gain_apply_fix(psInPcm, psOutPcm, st->gain_params.m_Apply_Gain, st->frame_size);

#endif
		
		
		
		
#if	TCAGC_DBG_ENABLE
		for(i = 0 ; i < st->frame_size ; i++)
		{
			i_frame_time_msec = giFrameCounter * 16 * 10 + (int)((double)i*16./(double)st->frame_size*10.) ;
			if(dbg_File) fprintf(dbg_File,"[%03d/%03d]\t", giFrameCounter, i);
			if(dbg_File) fprintf(dbg_File,"[%02d:%02d:%04d]\t", (i_frame_time_msec/(10000*60)), (i_frame_time_msec/10000)%60, i_frame_time_msec%10000);
			if(dbg_File) fprintf(dbg_File,"%02d%02d%04d\t", (i_frame_time_msec/(10000*60)), (i_frame_time_msec/10000)%60, i_frame_time_msec%10000);
			if(dbg_File) fprintf(dbg_File,"%6d\t", psInPcm[i]);
			if(dbg_File) fprintf(dbg_File,"%6d\t", vad_result);
			if(dbg_File) fprintf(dbg_File,"%6d\t", st->pd_params.m_iSlowAverage);
			if(dbg_File) fprintf(dbg_File,"%6d\t", st->pd_params.m_iPartAverage);
#if defined (FLOATING_POINT)
			if(dbg_File) fprintf(dbg_File,"%7.5f\t", st->gain_params.m_Apply_Gain);
#else
			if(dbg_File) fprintf(dbg_File,"%7.5f\t", (double)st->gain_params.m_Apply_Gain/128);
#endif
			if(dbg_File) fprintf(dbg_File,"%6d\n", psOutPcm[i]);
			//if(dbg_File) fprintf(dbg_File,"%6d\t", VAD_VALID_THR1);
			//if(dbg_File) fprintf(dbg_File,"%6d\t", VAD_VALID_THR2);
			//if(dbg_File) fprintf(dbg_File,"%6d\t", dec_rule);
		}
		if(giFrameCounter == TCAGC_DBG_FRAME_COUNT)	
			if(dbg_File) fclose(dbg_File);
#endif
	}

#if	TCAGC_DBG_ENABLE
	if(giFrameCounter == TCAGC_DBG_FRAME_COUNT)	
		if(dbg_File) fclose(dbg_File);
#endif

	return TC_AGC_PROC_SUCCESS;
}



// ==========================================//
// ============ [2] TC_AGC_Core =============//
// ==========================================//


#if defined (FLOATING_POINT)
// ================================================== //
// ================ FLOATING POINT ================== //
// ================================================== //
// AGC Gain Control
void TCAGC_gain_control(int vad_result, agc_peak_det_t PDHandle, agc_gain_ctrl_t* psGainHandle, float AGC_thr_linear, int AGC_attn)
{
	int i;
	int thr_chk, thr_chk_acc, diff_pwr_ref;
	float diff_pwr, pd_slow_avg;


	//diff_pwr_ref = PDHandle.m_iSlowAverage;
	diff_pwr_ref = PDHandle.m_iPartAverage;


	if(vad_result)
	{
		// slow avg convert to log
#ifdef MATH_FUNCTION
		pd_slow_avg = (float)(20.*TCAGC_log10_func((float)diff_pwr_ref / (float)pow(2., 15.)));
#else
		pd_slow_avg = (float)(20.*log10(diff_pwr_ref / pow(2., 15.)));
#endif		

		// calculation different power
		diff_pwr = (float)AGC_attn - pd_slow_avg;
		diff_pwr = TCAGC_round_fl(diff_pwr*5.f)/5.f;			// Round 0.2 scaling

		// check threshold
		if(PDHandle.m_iPartAverage > AGC_thr_linear)
			thr_chk = 1;
		else
			thr_chk = 0;

		// check result store
		psGainHandle->m_thr_chk_buff[psGainHandle->m_thr_chk_cnt] = thr_chk;
		
		psGainHandle->m_thr_chk_cnt++;
		psGainHandle->m_thr_chk_cnt = psGainHandle->m_thr_chk_cnt % psGainHandle->m_thr_chk_thr;

		thr_chk_acc = 0;
		for (i = 0 ; i < psGainHandle->m_thr_chk_thr ; i++)
			thr_chk_acc += psGainHandle->m_thr_chk_buff[i];

		// Apply gain
		if(thr_chk_acc > (psGainHandle->m_thr_chk_thr*6)/10)	// 60% Threshold
		{
			psGainHandle->m_Apply_Gain -= psGainHandle->m_Gain_step;
			if(psGainHandle->m_Apply_Gain < AGC_attn)
				psGainHandle->m_Apply_Gain = (float)AGC_attn;

			if(psGainHandle->m_Apply_Gain < diff_pwr)
			{
				psGainHandle->m_Apply_Gain += psGainHandle->m_Gain_step_r;
				if(psGainHandle->m_Apply_Gain > diff_pwr)
					psGainHandle->m_Apply_Gain = diff_pwr;
			}

			if(psGainHandle->m_Apply_Gain < AGC_attn)
				psGainHandle->m_Apply_Gain = (float)AGC_attn;
		}
		else
		{
			psGainHandle->m_Apply_Gain += psGainHandle->m_Gain_step_r;
			if(psGainHandle->m_Apply_Gain > 0)
				psGainHandle->m_Apply_Gain = 0;
		}
	}
}

// AGC Gain Apply
void TCAGC_gain_apply(short *psInPcm, short *psOutPcm, float apply_gain, int NumSample)
{
	int i;
	float tmp_out, gain_fl;

	gain_fl = (float)(pow(10., apply_gain/20.));

	for (i = 0 ; i < NumSample ; i++)
	{
		tmp_out = psInPcm[i] * gain_fl;

		if(tmp_out > 32767)
			psOutPcm[i] = 32767;
		else if(tmp_out < -32767)
			psOutPcm[i] = -32767;
		else
			psOutPcm[i] = (short)TCAGC_round_fl(tmp_out);
	}

}

int TCAGC_round_fl(float x)
{
	if( x - floor(x) >= 0.5)
		return (int)ceil(x);
	else
		return (int)floor(x);
}



#else // #if defined (FLOATING_POINT)
// ================================================== //
// ================== FIXED POINT =================== //
// ================================================== //
void TCAGC_gain_control_fix(int vad_result, agc_peak_det_t PDHandle, agc_gain_ctrl_t* psGainHandle, int AGC_thr_linear, int AGC_attn)
{
	int i;
	int	thr_chk, thr_chk_acc, diff_pwr, pd_slow_avg;

	int diff_pwr_ref, AGC_attn_scale;

	//diff_pwr_ref = PDHandle.m_iSlowAverage;
	diff_pwr_ref = PDHandle.m_iPartAverage;

	AGC_attn_scale = (AGC_attn<<TCAGC_GAIN_EXT_BIT);

	if(vad_result)
	{			
		pd_slow_avg = TCAGC_lin2log(diff_pwr_ref); // Log Scaling by TCAGC_L2DB_TBL_STEP

		// calculation different power
		diff_pwr = (AGC_attn<<TCAGC_L2DB_TBL_STEP_BIT) - pd_slow_avg;		

		// check threshold
		if(PDHandle.m_iPartAverage > AGC_thr_linear)
			thr_chk = 1;
		else
			thr_chk = 0;

		// check result store
		psGainHandle->m_thr_chk_buff[psGainHandle->m_thr_chk_cnt] = thr_chk;
		
		psGainHandle->m_thr_chk_cnt++;
		psGainHandle->m_thr_chk_cnt = psGainHandle->m_thr_chk_cnt % psGainHandle->m_thr_chk_thr;

		thr_chk_acc = 0;
		for (i = 0 ; i < psGainHandle->m_thr_chk_thr ; i++)
			thr_chk_acc += psGainHandle->m_thr_chk_buff[i];

		// Apply gain
		if(thr_chk_acc * 10 > psGainHandle->m_thr_chk_thr*6)
		{
			// Attack
			psGainHandle->m_Apply_Gain -= psGainHandle->m_Gain_step;
			if(psGainHandle->m_Apply_Gain < AGC_attn_scale)
				psGainHandle->m_Apply_Gain = AGC_attn_scale;

			// Release
			if((psGainHandle->m_Apply_Gain<<TCAGC_L2DB_TBL_STEP_BIT) < (diff_pwr<<TCAGC_GAIN_EXT_BIT))
			{
				psGainHandle->m_Apply_Gain += psGainHandle->m_Gain_step_r;
				if((psGainHandle->m_Apply_Gain<<TCAGC_L2DB_TBL_STEP_BIT) > (diff_pwr<<TCAGC_GAIN_EXT_BIT))
					psGainHandle->m_Apply_Gain = TCAGC_XROUND_TR((diff_pwr<<TCAGC_GAIN_EXT_BIT),TCAGC_L2DB_TBL_STEP_BIT);
			}

			if(psGainHandle->m_Apply_Gain < AGC_attn_scale)
				psGainHandle->m_Apply_Gain = AGC_attn_scale;
		}
		else
		{
			psGainHandle->m_Apply_Gain += psGainHandle->m_Gain_step_r;
			if(psGainHandle->m_Apply_Gain > 0)
				psGainHandle->m_Apply_Gain = 0;
		}
	}
}

void TCAGC_gain_apply_fix(const short *psInPcm, short *psOutPcm, int apply_gain, int NumSample)
{
	int i, tmp_out, gain_fx;

	gain_fx = TCAGC_log2lin( TCAGC_XROUND_TR((apply_gain<<TCAGC_L2DB_TBL_STEP_BIT),TCAGC_GAIN_EXT_BIT));

	for (i = 0 ; i < NumSample ; i++)
	{
		tmp_out = psInPcm[i] * gain_fx;
		tmp_out = tmp_out>>TCAGC_L2DB_EXT_BIT;

		if(tmp_out > 32767)
			psOutPcm[i] = 32767;
		else if(tmp_out < -32767)
			psOutPcm[i] = -32767;
		else
			psOutPcm[i] = tmp_out;
	}
}


// [1] Linear to Log Converter scale Up TCAGC_L2DB_TBL_STEP
int  TCAGC_lin2log(int input)
{
	int i, j, idx, log_value;	

	log_value = 1000;

	idx = 0;
	// Lower Bound
	if(input < TCAGC_L2DB_TBL[0])
		idx = 0;
	// Upper Bound
	else if(input >= TCAGC_L2DB_TBL[TCAGC_L2DB_TBL_SIZE-1])
		idx = TCAGC_L2DB_TBL_SIZE-1;
	// Ohters
	else
	{
		for(i = 0 ; i < TCAGC_L2DB_TBL_SIZE - TCAGC_L2DB_C_SRCH ; i = i+TCAGC_L2DB_C_SRCH)
		{
			if(input >= TCAGC_L2DB_TBL[i] && input < TCAGC_L2DB_TBL[i+TCAGC_L2DB_C_SRCH])
			{
				for(j = 0 ; j < TCAGC_L2DB_C_SRCH ; j++)
				{
					if(input >= TCAGC_L2DB_TBL[i+j] && input < TCAGC_L2DB_TBL[i+j+1])
						idx = i+j;
				}
			}
		}
		for(j = i ; j < TCAGC_L2DB_TBL_SIZE ; j++)
		{
			if(input >= TCAGC_L2DB_TBL[j] && input < TCAGC_L2DB_TBL[j+1])
				idx = j;
		}
	}

	log_value = (TCAGC_L2DB_TBL_START<<TCAGC_L2DB_TBL_STEP_BIT) + idx; // Scaling  TCAGC_GAIN_TBL_STEP

	return log_value;
}

// Log to Linear Scale up TCAGC_GAIN_EXT
int  TCAGC_log2lin(int input)
{
	int idx, output, max_val;

	//TCAGC_L2DB_TBL_STEP Step dB => 1step	
	max_val = TCAGC_L2DB_TBL_START<<TCAGC_L2DB_TBL_STEP_BIT;
	if(input < max_val)
		input = max_val;

	if(input > 0)
		input = 0;
	idx = input + TCAGC_L2DB_TBL_SIZE - 1;
	output = TCAGC_L2DB_TBL[idx];

	return output;
}



#endif // #if defined (FLOATING_POINT)



// ================================================== //
// ======= FLOATING POINT ||  FIXED POINT =========== //
// ================================================== //

// Peak Detector
void TCAGC_peak_detector(const short *psInPcm, int vad_results, agc_peak_det_t* psPDHandle, int NumSample)
{
	int i, j;

	int slow_avg, part_avg, value;

	int	abs_pcm;
	int max_peak;
	
	slow_avg = psPDHandle->m_iSlowAverage;
	part_avg = psPDHandle->m_iPartAverage;
	
	if(vad_results)
	{
		// Find the Peak in Partial Subsample
		for(i = 0 ; i < NumSample ; i = i+psPDHandle->m_iSubSamp)
		{
			max_peak = 0;
			for(j = 0 ; j < psPDHandle->m_iSubSamp ; j++)
			{
				abs_pcm = TCAGC_ABS((int)psInPcm[i+j]);
				if(abs_pcm > max_peak)
					max_peak = abs_pcm;
			}

			// Slow & Fast Average
			if(max_peak > part_avg)
				value = AGC_VAD_FAST_R;
			else
				value = AGC_VAD_FAST_F;
			part_avg = ((((1<<AGC_VAD_FBITS) - value)*max_peak + value*part_avg)>>AGC_VAD_FBITS);


			
			if(max_peak > slow_avg)
				value = AGC_VAD_FAST_R;
			else
				value = AGC_VAD_SLOW_F;
			slow_avg = ((((1<<AGC_VAD_FBITS) - value)*max_peak + value*slow_avg)>>AGC_VAD_FBITS);
		}

		psPDHandle->m_iSlowAverage = slow_avg;
		psPDHandle->m_iPartAverage = part_avg;
	}
}




// [1] Voice Activity Detection for Each samples
int TCAGC_detect_speech_activity(agc_vad_t* psVadHandle, const short* pPcmIn, int NumSample)
{
	int j;
	int vad = 0;	
	int i_max_fast = 0;
	int i_abs_pcm, value, in_slow, in_fast;	
	int in_slow_p, in_slow_cnt, in_fast_p, in_fast_cnt;

	int dec_flag_tot, dec_flag1, dec_flag2, dec_flag3;
	int dec_rule_1, dec_rule_2, mean_pwr;
	
	dec_flag_tot = dec_flag1 = dec_flag2 = dec_flag3 = 0;
	dec_rule_1 = dec_rule_2 = mean_pwr = 0;
	

	if(psVadHandle->m_iCount == -1)
	{
		// IIR Filter
		i_abs_pcm = TCAGC_iir_vad(pPcmIn[0],psVadHandle->m_iPastSamplex,psVadHandle->m_iPastSampley);
		psVadHandle->m_iSlowAverage = (i_abs_pcm<0)? -i_abs_pcm : i_abs_pcm;
		psVadHandle->m_iFastAverage = (i_abs_pcm<0)? -i_abs_pcm : i_abs_pcm;
	}
	
	in_slow = psVadHandle->m_iSlowAverage;
	in_fast = psVadHandle->m_iFastAverage;


	// To find slope
	in_slow_p = in_slow;
	in_fast_p = in_fast;
	in_slow_cnt = in_fast_cnt = 0;
	for(j = 0 ; j < NumSample ; j++)
	{
		// IIR Filter & Abs
		i_abs_pcm = TCAGC_iir_vad(pPcmIn[j],psVadHandle->m_iPastSamplex,psVadHandle->m_iPastSampley);
		i_abs_pcm = (i_abs_pcm<0)? -i_abs_pcm : i_abs_pcm;

		// slow envelope tracker
		if(i_abs_pcm > in_slow)		value = AGC_VAD_SLOW_R;
		else						value = AGC_VAD_SLOW_F;		
		in_slow = ((((1<<AGC_VAD_FBITS) - value)*i_abs_pcm + value*in_slow)>>AGC_VAD_FBITS);


		// fast envelope tracker 
		if(i_abs_pcm > in_fast)		value = AGC_VAD_FAST_R;
		else						value = AGC_VAD_FAST_F;
		in_fast = ((((1<<AGC_VAD_FBITS) - value)*i_abs_pcm + value*in_fast)>>AGC_VAD_FBITS);


		if(in_slow <= in_slow_p)
			in_slow_cnt++;
		if(in_fast <= in_fast_p)
			in_fast_cnt++;

		// Voice Activity Decision Rule [1] - using Slow&Fast 
		dec_flag1 = (int)(in_slow > (int)(psVadHandle->m_iMinAverage * AGC_VAD_SCALER));
		dec_flag2 = (int)(in_slow > AGC_VAD_THR_PWR);
		dec_flag3 = (int)(in_fast > AGC_VAD_THR_PWR);
		dec_flag_tot = dec_flag1 && (dec_flag2 && dec_flag3);

		if(dec_flag_tot) 
			dec_rule_1++;
			
		psVadHandle->m_iSlowAverage = in_slow;
		psVadHandle->m_iFastAverage = in_fast;

		i_max_fast = TCAGC_MAX(i_max_fast,in_fast);

		// Total PCM Power		
		mean_pwr += ((i_abs_pcm * i_abs_pcm)>>2);

		in_slow_p = in_slow;
		in_fast_p = in_fast;		
	}
	mean_pwr = mean_pwr / (NumSample>>2);

	
	if(psVadHandle->m_iCount < AGC_VAD_UPDATE_THR*10)
		psVadHandle->m_iCount++;
	
	if( psVadHandle->m_iCount > AGC_VAD_UPDATE_THR)
	{
		psVadHandle->m_iCount = 0;
		psVadHandle->m_iMinAverage = TCAGC_MIN(psVadHandle->m_iTempAverage,in_fast);
		psVadHandle->m_iTempAverage = in_fast;
		psVadHandle->m_iMinAverage = TCAGC_MAX(100,psVadHandle->m_iMinAverage);
	}
	else
	{
		psVadHandle->m_iMinAverage = TCAGC_MIN(psVadHandle->m_iMinAverage,in_fast);
		psVadHandle->m_iTempAverage = TCAGC_MIN(psVadHandle->m_iTempAverage,in_fast);
		psVadHandle->m_iMinAverage = TCAGC_MAX(100,psVadHandle->m_iMinAverage);
	}
	psVadHandle->m_iMaxAverage = i_max_fast;



	// Voice Activity Decision Rule [2] - using Slope
	dec_rule_2 = 1;
	//if(in_slow_cnt > (int)((float)NumSample * 0.9) && in_fast_cnt > (int)((float)NumSample * 0.75))
	if((in_slow_cnt*10 > NumSample*9) && (in_fast_cnt*100 > NumSample * 75))
		dec_rule_2 = 0;	


	// Final VAD
	if(dec_rule_1 > 1)
		vad = dec_rule_2;

	return vad;
}




int TCAGC_iir_vad(short NewSample, int* pLastx, int* pLasty) 
{
	int n, i_temp_y;

	//shift the old samples
    for(n = AGC_IIR_NUM; n > 0; n--) 
	{
       pLastx[n] = pLastx[n-1];
       pLasty[n] = pLasty[n-1];
    }

    //Calculate the new output
    pLastx[0] = NewSample;
    i_temp_y = AGC_IIR_ACoef[0] * NewSample;
    for(n = 1; n <= AGC_IIR_NUM; n++)
        i_temp_y += AGC_IIR_ACoef[n] * pLastx[n] - AGC_IIR_BCoef[n] * pLasty[n];

    i_temp_y >>= 14;
    pLasty[0] = i_temp_y;
    return i_temp_y;
}









#if defined (FLOATING_POINT) && defined (MATH_FUNCTION)
//=============== LOG10 Function ===============//
float TCAGC_log10_func (float input)
{
	float Est_log = 0.f;

	if(input <= 0)
		return 100000;
	else
	{
		Est_log = TCAGC_log_func(input);
		Est_log = Est_log / (float)TCAGC_LOG_10;
		
		return Est_log;
	}	
}

//=============== LOG Function ===============//
float TCAGC_log_func (float input)
{
	int i, Exp_idx, Int_idx;
	
	float Exp_term, Int_term, Est_term, Est_log;
	float input_1, input_2, input_3;
	float tmp_input;


	if(input <= 0)
	{
		return 100000;
	}
	else
	{
		if(input < TCAGC_log_pow_10[0])
			input = TCAGC_log_pow_10[0];
		if(input > TCAGC_log_pow_10[20])
			input = TCAGC_log_pow_10[20];

		// Find the Exponential
		for (i = 0 ; i < 21 ; i++)
		{
			tmp_input = input * TCAGC_log_pow_10[i];
			if(tmp_input >= 1.f && tmp_input < 10.f)
			{
				Exp_term = TCAGC_log_pow_10[i];
				Exp_idx = i - 10;
			}
		}
		input_1 = input * Exp_term;

		// Find the Integer
		Int_idx = 0;
		for (i = 1 ; i <= 5 ; i++)
		{
			tmp_input = input_1 / (float)i;
			if(tmp_input >= 1.f && tmp_input <= 2.f)
			{
				Int_term = (float)i;			
				Int_idx = i-1;
			}
		}
		input_2 = input_1 / Int_term;


		// Find the Log for abs(input2) < 2
		input_3 = (input_2-1.f) / (input_2+1.f);
		

		Est_term = 0.f;
		for (i = 1 ; i < TCAGC_LOG_EST_LOOP_CNT ; i=i+2)
		{
			tmp_input = TCAGC_pow_func(input_3, i-1);			
			Est_term = Est_term + 2.f*input_3 * (1.f/(float)i) * tmp_input;
		}

		Est_log = Est_term - (float)Exp_idx*(float)TCAGC_LOG_10;
		Est_log = Est_log + TCAGC_log_const_value[Int_idx];

		return Est_log;
	}
}

//=============== Power Function ===============//
float TCAGC_pow_func (float input, int exp)
{
	int i;
	float output;

	output = 1.f;
	for(i = 0 ; i < exp ; i++)
		output = output * input;

	return output;
}
#endif //#if defined (MATH_FUNCTION)




#if defined (TCAGC_GAIN_TBL_GEN) && !defined (FLOATING_POINT)
void TCAGC_L2DB_table_gen_fix(int dB_step)
{	
	FILE *fp;
	int i, idx;
	double input;
	const char filename_L2DB_gen_tbl[] = "./AGC_L2DB_TBL.txt";

	double *input_data, *output_data;
	input_data = malloc(sizeof(double)*1000);
	output_data = malloc(sizeof(double)*1000);

	// [1] Table Generation
	fp = fopen(filename_L2DB_gen_tbl, "w");

	idx = 0;
	for(input = TCAGC_L2DB_TBL_START ; input < 1./(double)TCAGC_L2DB_TBL_STEP ; input += 1./(double)TCAGC_L2DB_TBL_STEP)
	{
		input_data[idx] = input;
		output_data[idx] = TCAGC_round_table((float)pow(10.f, (input)/20.f) * (float)pow(2.f, (double)TCAGC_L2DB_EXT_BIT));

		fprintf(fp, "%8.4f   %8.4f  \n", input_data[idx], output_data[idx]);

		idx++;
	}	
	
	// Input
	fprintf(fp, "\n\n DB2L INPUT : SIZE = %3d \n", idx);	
	for(i = 0 ; i < idx ; i++)
	{
		fprintf(fp, "%5d, ", (int)output_data[i]);		
		if(i%10 == 9)
			fprintf(fp, "\n");
	}
	fclose(fp);

	free(input_data);
	free(output_data);


	
	// [2] TCAGC_L2DB Test
	fp = fopen("./TCAGC_L2DB_test.txt","w");
	fprintf(fp, "Linear to DB Scale\n");
	i = 0;
	fprintf(fp, "%5d %10.4f %10.4f\n", 
		i, (float)(20.*log10((float)1. / pow(2., 15.))), (float)TCAGC_lin2log(i)/(float)TCAGC_L2DB_TBL_STEP);

	for(i = 1 ; i < 32768 ; i++)
		fprintf(fp, "%5d %10.4f %10.4f\n", i, (float)(20.*log10((float)i / pow(2., 15.))), (float)TCAGC_lin2log(i)/(float)TCAGC_L2DB_TBL_STEP);
	
	
	// [3] TCAGC_DB2L Test
	fprintf(fp, "DB to Linear Scale\n");
	for(input = -100  ; input <= 10 ; input += 0.0625)
		fprintf(fp, "%8.4f  %8.4f  %5d\n", input, pow(10., input/20.) * pow(2., 15.), TCAGC_log2lin((int)(input*TCAGC_L2DB_TBL_STEP)) );

	fclose(fp);
	
}

int TCAGC_round_table(float x)
{
	if(x - floor(x) >= 0.5)
		return (int)ceil(x);
	else
		return (int)floor(x);
}


#endif // #if defined (TCAGC_GAIN_TBL_GEN) && !defined (FLOATING_POINT)
