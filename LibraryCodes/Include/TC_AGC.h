
#ifndef TC_AGC_LIB_H
#define TC_AGC_LIB_H



//===== [0] Predefined =====//
//==== Math Function Use ====//
//#define MATH_FUNCTION

//==== Gain Table Generation : For Development ====//
//#define TCAGC_GAIN_TBL_GEN



//===== [1] Constant Parameters =====//
// 0. Initial Threshold
#define	AGC_THR_MAX		(-40)
#define	AGC_ATT_MAX		(-50)

// 1. DC Notch Filter
#define	AGC_IIR_NUM		(2)
const int	AGC_IIR_ACoef[AGC_IIR_NUM+1] = {11726,-23453,11726};
const int	AGC_IIR_BCoef[AGC_IIR_NUM+1] = {16384,-22101,8421}; 

// 2. VAD
#define	AGC_VAD_FBITS		(15)
#define	AGC_VAD_SLOW_R		(32440)		//0.99
#define	AGC_VAD_SLOW_F		(32604)		//0.995
#define	AGC_VAD_FAST_R		(29491)		//0.9
#define	AGC_VAD_FAST_F		(32276)		//0.985

#define	AGC_VAD_SCALER		(2)
#define	AGC_VAD_UPDATE_THR	(100)
#define	AGC_VAD_THR_PWR		(327)	// -40
//#define	AGC_VAD_THR_PWR		(3277)	// -20
//#define	AGC_VAD_THR_PWR		(1036)	// -30
//#define	AGC_VAD_THR_PWR		(104)	// -50



//===== [2] Structure & Control Handler =====//
typedef struct agc_vad_t
{
	int			m_iCount;
	int			m_iMinAverage;
	int			m_iTempAverage;
	int			m_iMaxAverage;
	int			m_iFastAverage;
	int			m_iSlowAverage;
	int 		m_iPastSampley[AGC_IIR_NUM+1]; //output samples
	int 		m_iPastSamplex[AGC_IIR_NUM+1]; //input samples	
}agc_vad_t;

typedef struct agc_peak_det_t
{
	int			m_iCount;
	int			m_iSlowAverage;
	int			m_iPartAverage;
	int			m_iSubSamp;
}agc_peak_det_t;

typedef struct agc_gain_ctrl_t
{
	int			m_iCount;
	int			m_thr_chk_buff[10];
	int			m_thr_chk_cnt;
	int			m_thr_chk_thr;
	
#if defined (FLOATING_POINT)
	float		m_Apply_Gain;
	float		m_Gain_step;
	float		m_Gain_step_r;
#else
	int			m_Apply_Gain;
	int			m_Gain_step;
	int			m_Gain_step_r;
#endif
}agc_gain_ctrl_t;

typedef struct agc_handle
{	
	int			frame_size;			// Frame Size
	int			sample_rate;		// Sample Rate
	int			thr_dB;				// AGC Threshold [dB]
	int			att_dB;				// AGC Attenuation Gain dB
	int			step_dB;			// AGC Step Size (0:0.0625, 1:0.125, 2:0.25 dB)
	int			step_dB_r;			// AGC Step Size (

#if defined (FLOATING_POINT)
	float		thr_linear;			// AGC Threshold [linear]
#else
	int			thr_linear;
#endif

	agc_vad_t		vad_params;		// VAD Parameter
	agc_peak_det_t	pd_params;		// Peak Detector Parameter
	agc_gain_ctrl_t	gain_params;	// Gain Control Parameter
}agc_handle;



//===== [3] Predefined Function =====//
#define	TCAGC_ABS(x) ((x) < 0 ? (-(x)) : (x))   // ABSOLUTE 
#define	TCAGC_MIN(a,b) ((a) < (b) ? (a) : (b))	//< Maximum 32-bit value.   
#define	TCAGC_MAX(a,b) ((a) > (b) ? (a) : (b))	//< Maximum 32-bit value. 
#define	TCAGC_XROUND_TR(input, TRBIT) ((input + (1<<(TRBIT-1)))>>TRBIT)	// TR & ROUND



//===== [4] AGC Functions =====//
int TCAGC_detect_speech_activity(agc_vad_t* psVadHandle, const short* pPcmIn, int NumSample);
int TCAGC_iir_vad(short NewSample, int* pLastx, int* pLasty);
void TCAGC_peak_detector(const short *psInPcm, int vad_results, agc_peak_det_t* psPDHandle, int NumSample);

#if defined (FLOATING_POINT)
void TCAGC_gain_control(int vad_result, agc_peak_det_t PDHandle, agc_gain_ctrl_t* psGainHandle, float AGC_thr, int AGC_attn);
void TCAGC_gain_apply(short *psInPcm, short *psOutPcm, float apply_gain, int NumSample);
int TCAGC_round_fl(float x);
#else
void TCAGC_gain_control_fix(int vad_result, agc_peak_det_t PDHandle, agc_gain_ctrl_t* psGainHandle, int AGC_thr, int AGC_attn);
void TCAGC_gain_apply_fix(const short *psInPcm, short *psOutPcm, int apply_gain, int NumSample);
int  TCAGC_lin2log(int input); // Linear to Log
int  TCAGC_log2lin(int input); // Log to Linear
#endif



//===== [5] Other Functions 
#if defined (FLOATING_POINT) && defined (MATH_FUNCTION)
#define TCAGC_LOG_EST_LOOP_CNT		(15)
#define TCAGC_LOG_10					(2.30258509299405)
const float TCAGC_log_const_value[5]= {0.f,				//log(1)
								0.693147180559945f,	//log(2)
								1.09861228866811f,	//log(3)
								1.38629436111989f,	//log(4)
								1.60943791243410f};	//log(5)
float TCAGC_log_pow_10[21]= { (float)1e-10	,	(float)1e-9,	(float)1e-8	,	(float)1e-7	,	(float)1e-6	,	
						(float)1e-5	,	(float)1e-4,	(float)1e-3	,	(float)1e-2	,	(float)1e-1	,	
						(float)1e0		,	
						(float)1e1		,	(float)1e2	,	(float)1e3		,	(float)1e4		,	(float)1e5		,
						(float)1e6		,	(float)1e7	,	(float)1e8		,	(float)1e9		,	(float)1e10	};

float TCAGC_log10_func(float input);			// Log10 Function
float TCAGC_log_func(float input);				// Log Function
float TCAGC_pow_func (float input, int exp);	// Power Function
#endif

#if defined (TCAGC_GAIN_TBL_GEN) && !defined (FLOATING_POINT)
void TCAGC_L2DB_table_gen_fix(int dB_step);
int TCAGC_round_table(float x);
#endif


#endif	// #ifndef TCAGC_LIB_H
