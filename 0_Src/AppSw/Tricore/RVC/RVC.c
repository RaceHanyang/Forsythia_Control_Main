/*
 * RVC.c
 * Created on 2019. 11. 01
 * Author: Dua
 */

/*
TODO:
    RVC frequency
        - To 10 ms instead of 1ms
    CAN associated functions
        - R2D entry routine display
        - Steering wheel function
        - Parameter load/save
        - Log Data Broadcasting
    Battery Management
        - Charge consumed calculation
    Analog sensor
        - Steering Wheel Analog: need calibration code
    GPIO
        - Charge Enable
    WheelSpeed
        - Wheel speed calculation by motor info
 */

/***************************** Includes ******************************/
#include <math.h>

#include "AdcForceStart.h"
#include "AdcSensor.h"
#include "Beeper_Test_Music.h"
#include "Gpio_Debounce.h"
#include "HLD.h"
#include "IfxPort.h"
#include "PedalMap.h"

#include "RVC.h"
#include "RVC_privateDataStructure.h"
#include "TorqueVectoring/TorqueVectoring.h"

#include "AmkInverter_can.h"
#include "DashBoardCan.h"
#include "MechMsg.h"
#include "SteeringAngleAdc.h"

/**************************** Macro **********************************/
#define PWMFREQ 5000 // PWM frequency in Hz
#define PWMVREF 5.0  // PWM reference voltage (On voltage)

/*
#define OUTCAL_LEFT_MUL 1.06
#define OUTCAL_LEFT_OFFSET 0.015
#define OUTCAL_RIGHT_MUL 1.065
#define OUTCAL_RIGHT_OFFSET 0.0125
 */

#define OUTCAL_LEFT_MUL 1.0f
#define OUTCAL_LEFT_OFFSET 0.0f
#define OUTCAL_RIGHT_MUL 1.0f
#define OUTCAL_RIGHT_OFFSET 0.0f

#define R2D_ONHOLD (3 * 100)  // 3 seconds
#define R2D_OFFHOLD (1 * 100) // 1 seconds
#define R2D_REL (1 * 100)     // 1 seconds
#define R2D_APPS_TH 95
#define R2D_APPS_LO 5
#define R2D_APPS_HOLD (5 * 10) // 0.5 seconds
#define R2D_BPPS_TH 95
#define R2D_BPPS_LO 5
#define R2D_BPPS_HOLD (5 * 10) // 0.5 seconds
#define R2D_TEST               // Test: start button set R2D directly

#define PEDAL_BRAKE_ON_THRESHOLD 10
#define REGEN_MUL 1 // 2

#define POWER_LIM 80000        // 80kW
#define CURRENT_LIM_SET_VAL 10 // 10A

#define TV1PGAIN 0.001

#define REGEN_ON_INIT FALSE //***** Regen is not abailable now!!! ***** //FIXME //TODO: Regen limit

#define LVBAT_LINCAL_A 1.021f
#define LVBAT_LINCAL_B 0
#define LVBAT_LINCAL_D 0

#define VAR_UPDATE_ERROR_LIM 10

#define THROTLE_5V
#define AMK_TEST 1

#define BRAKE_ON_BP
// #define BRAKE_ON_TH_BP1	3.3f
// #define BRAKE_ON_TH_BP2 5.6f
#define BRAKE_ON_TH_BP1 20.0f
#define BRAKE_ON_TH_BP2 20.0f

#define BP_MAX_BAR 172.369f
#define BP_MAX_V 4.5f
#define BP_MIN_V 0.5f

// #define BMS_PDL_ERROR	TRUE
#define BMS_PDL_ERROR FALSE

#define RTDS_TIME (3000) // RTD Sound length in ms.

/*********************** Global Variables ****************************/
static boolean rtds = FALSE;

RVC_t RVC = {
    .readyToDrive = RVC_ReadyToDrive_status_notInitialized,
    .torque.controlled = 0,
    .torque.rearLeft = 0,
    .torque.rearRight = 0,
    .torque.isRegenOn = REGEN_ON_INIT,

    .calibration.leftAcc.mul = 1,
    .calibration.leftAcc.offset = 0,
    .calibration.rightAcc.mul = 1,
    .calibration.rightAcc.offset = 0,
    .calibration.leftDec.mul = 1,
    .calibration.leftDec.offset = 0,
    .calibration.rightDec.mul = 1,
    .calibration.rightDec.offset = 0,

    .power.limit = POWER_LIM,
    .currentLimit.setValue = CURRENT_LIM_SET_VAL,
};

RVC_public_t RVC_public;
/******************* Private Function Prototypes *********************/
IFX_STATIC void RVC_initAdcSensor(void);
IFX_STATIC void RVC_initPwm(void);
IFX_STATIC void RVC_initGpio(void);
IFX_STATIC void RVC_pollGpi(RVC_Gpi_t *gpi);

IFX_INLINE void RVC_updateReadyToDriveSignal(void);
IFX_INLINE void RVC_getTorqueRequired(void);
IFX_INLINE void RVC_torqueSignalGeneration(void);
IFX_INLINE void RVC_updateSharedVariable(void);

/********************* Function Implementation ***********************/
void RVC_init(void)
{
	RVC_initAdcSensor();

	RVC.tvMode = RVC_TorqueVectoring_modeOpen;
	RVC.tcMode = RVC_TractionControl_modeNone;
	RVC_PedalMap_lut_setMode(0);

	RVC_initPwm();

	RVC_initGpio();

	RVC.readyToDrive = RVC_ReadyToDrive_status_initialized;
}

static inline double clamp(double min, double max, double x)
{
	return (x < min) ? min : (x > max ? max : x);
}

static const double power_max = 80000.0; // 80kW
static const double vel_error = 1.0;     // velocity lower bound
#define GEAR_RATIO (1.0)                 // Gear ratio, assuming 1:1 for simplicity
#define RADIUS (0.3)                     // Wheel radius in meters
#define TORQUE_MAX (21.43)               // Maximum torque in Nm, needs to be updated to actual system value
#define FORCE_MAX ((GEAR_RATIO * TORQUE_MAX) / RADIUS) // maximum force [N]
static inline double maxd(double a, double b)
{
	return (a > b) ? a : b;
}
static inline double mind(double a, double b)
{
	return (a < b) ? a : b;
}
static double ff_pwr_lim(double v_x, double u)
{
	RVC.pwr_lim.ff.v_x = v_x;
	RVC.pwr_lim.ff.u = u;

	double f_p_lim = power_max / maxd(v_x, vel_error);

	RVC.pwr_lim.ff.a_ff = mind((4.0 * FORCE_MAX), f_p_lim) / (4.0 * FORCE_MAX);

	return RVC.pwr_lim.ff.a_ff;
}

static const double power_band = 5000.0;
static const double lpf_dt = 1.0;   // 1ms
static const double lpf_tau = 50.0; // 50ms
static inline double lpf(double prev, double input)
{
	double alpha = lpf_dt / (lpf_tau + lpf_dt);
	return (1.0f - alpha) * prev + alpha * input;
}
static double fb_pwr_lim(double v, double i)
{
	const double K_P = 1.0 / power_band;

	RVC.pwr_lim.fb.v = v;
	RVC.pwr_lim.fb.i = i;
	RVC.pwr_lim.fb.p = v * i;

	double e_p = power_max - RVC.pwr_lim.fb.p;
	double a_raw = clamp(0.0, 1.0, K_P * e_p);

	RVC.pwr_lim.fb.a_fb = lpf(RVC.pwr_lim.fb.a_fb, a_raw);

	return RVC.pwr_lim.fb.a_fb;
}

static double pwr_lim(double v_x, double u, double v, double i)
{
	double a = ff_pwr_lim(v_x, u) * fb_pwr_lim(v, i);

	double f_raw = (4 * FORCE_MAX) * u;

	return a * f_raw;
}

// F/R split
static const double pwl_t_0 = 0.1;
static const double pwl_x_1 = 0.2;
static const double pwl_x_2 = 0.6;
static const double pwl_s_1 = 0.4;
static const double pwl_s_2 = 0.1;
static double base_f_r_split(double u)
{
	if(u < pwl_x_1)
	{
		return pwl_t_0 + pwl_s_1 * u;
	}
	if(u < pwl_x_2)
	{
		return ((0.5 + pwl_s_2 * (pwl_x_2 - 1)) - (pwl_t_0 + pwl_s_1 * pwl_x_1)) * (u - pwl_x_1) / (pwl_x_2 - pwl_x_1) +
		       (pwl_t_0 + pwl_s_1 * pwl_x_1);
	}
	return 0.5 + pwl_s_2 * (u - 1);
}

static const double wheel_base = 1.0;
static inline int sgn(double x)
{
	return (x > 0.0) - (x < 0.0); // x>0:1, x<0:-1, x==0:0
}
static double f_r_split(double f_des, double u, double a_x, double r, double r_des)
{
	RVC.torque_vectoring.f_r.base.k_0 = base_f_r_split(u);

	return RVC.torque_vectoring.f_r.base.k_0 - RVC.torque_vectoring.f_r.acc.k_a * a_x / wheel_base +
	       RVC.torque_vectoring.f_r.yaw.k_r * sgn(r_des) * (r_des - r);
}

static inline double f_r_saturation(double f_f, double f_r)
{
	RVC.torque_vectoring.f_r.k_frl = maxd(f_f, f_r) / (2 * FORCE_MAX);

	if(RVC.torque_vectoring.f_r.k_frl > 1.0)
	{
		return 1.0 / RVC.torque_vectoring.f_r.k_frl;
	}
	return 1.0;
}

static const double force_min = 0.0; // minimum force
static double f_i_margin(double f_i)
{
	double f = f_i / (2.0);
	return mind(FORCE_MAX - f, f - force_min);
}

static const double track_width = 0.5; // width of the vehicle in m
static inline double diff_f_conversion(double m_c)
{
	return m_c / track_width;
}

static const double force_margin_error = 50.0;
static double l_r_split(double f_f_margin, double f_r_margin)
{
	double f_margin = f_f_margin + f_r_margin;
	double k_tfr;
	if(f_margin < force_margin_error)
	{
		k_tfr = (0.5) + (f_f_margin - f_r_margin) / (2.0 * force_margin_error);
	}
	k_tfr = f_f_margin / f_margin;

	return lpf(RVC.torque_vectoring.l_r.k_tfr, k_tfr);
}

static inline double absd(double x)
{
	return (x < 0.0) ? -x : x;
}
static double diff_f_c_conversion(double k_tfr, double f_c_0, double f_f_margin, double f_r_margin)
{
	double f_margin = f_f_margin + f_r_margin;
	if(f_margin < absd(f_c_0))
	{
		return k_tfr * sgn(f_c_0) * f_margin;
	}
	return k_tfr * sgn(f_c_0) * f_c_0;
}

static double diff_f_exceed_conversion(double f_c_0_abs, double f_margin)
{
	if(f_margin < f_c_0_abs)
	{
		return 0.0;
	}
	return f_c_0_abs - f_margin;
}

static const double j_left = -1.0;
static inline double drive_left_force_conversion(double f, int s, double f_c, double f_exceed)
{
	return f / 2.0 + s * j_left * f_c - (1.0 - s * j_left) * f_exceed;
}
static const double j_right = 1.0;
static inline double drive_right_force_conversion(double f, int s, double f_c, double f_exceed)
{
	return f / 2.0 + s * j_right * f_c - (1.0 - s * j_right) * f_exceed;
}

static inline signed short force_to_torque(double force)
{
	return (signed short)(force * RADIUS / GEAR_RATIO); // Convert force to torque
}

void RVC_run_1ms(void)
{
	RVC_updateReadyToDriveSignal();

	while(IfxCpu_acquireMutex(&RVC_public.bms.shared.mutex))
		; // Wait for mutex
	{
		RVC_public.bms.data = RVC_public.bms.shared.data;
		IfxCpu_releaseMutex(&RVC_public.bms.shared.mutex);
	}

	while(IfxCpu_acquireMutex(&AmkInverterMonitorPublic.mutex))
		; // wait for the mutex
	{
		RVC.AmkMonitor = AmkInverterMonitorPublic.monitor;
		IfxCpu_releaseMutex(&AmkInverterMonitorPublic.mutex);
	}

	RVC.tv.sta = SDP_SteeringAngleAdc.sta.degree;

	RVC_getTorqueRequired();

	if(RVC.BrakePressure1.value > BRAKE_ON_TH_BP1)
		RVC.brakeOn.bp1 = TRUE;
	else
		RVC.brakeOn.bp1 = FALSE;

	if(RVC.BrakePressure2.value > BRAKE_ON_TH_BP2)
		RVC.brakeOn.bp2 = TRUE;
	else
		RVC.brakeOn.bp2 = FALSE;

	// RVC.brakeOn.tot = RVC.brakeOn.bp1 | RVC.brakeOn.bp2 | RVC.brakePressureOn.value;
	RVC.brakeOn.tot = RVC.brakeOn.bp1 | RVC.brakeOn.bp2;

	// common
	double u = clamp(0, 1, (double)RVC.torque.desired / 100.0); // pedal input

	// power limit
	RVC.pwr_lim.ff.v_x = 0; // <TODO>
	RVC.pwr_lim.ff.u = u;
	RVC.pwr_lim.fb.v = RVC_public.bms.data.voltage;
	RVC.pwr_lim.fb.i = RVC_public.bms.data.current;
	RVC.pwr_lim.f_des = pwr_lim(RVC.pwr_lim.ff.v_x, RVC.pwr_lim.ff.u, RVC.pwr_lim.fb.v, RVC.pwr_lim.fb.i);

	// F/R split
	RVC.torque_vectoring.f_r.base.u = u;
	RVC.torque_vectoring.f_r.acc.a_x = 0;   // <TODO>
	RVC.torque_vectoring.f_r.yaw.r = 0;     // <TODO>
	RVC.torque_vectoring.f_r.yaw.r_des = 0; // <TODO>
	RVC.torque_vectoring.f_r.f_des = RVC.pwr_lim.f_des;
	RVC.torque_vectoring.f_r.k_fr = f_r_split(RVC.torque_vectoring.f_r.f_des, RVC.torque_vectoring.f_r.base.u,
	    RVC.torque_vectoring.f_r.acc.a_x, RVC.torque_vectoring.f_r.yaw.r, RVC.torque_vectoring.f_r.yaw.r_des);
	RVC.torque_vectoring.f_r.f_f = RVC.torque_vectoring.f_r.k_fr * RVC.torque_vectoring.f_r.f_des;
	RVC.torque_vectoring.f_r.f_r = (1.0 - RVC.torque_vectoring.f_r.k_fr) * RVC.torque_vectoring.f_r.f_des;
	RVC.torque_vectoring.f_r.f_r_sat = f_r_saturation(RVC.torque_vectoring.f_r.f_f, RVC.torque_vectoring.f_r.f_r);
	RVC.torque_vectoring.f_r.f_f *= RVC.torque_vectoring.f_r.f_r_sat;
	RVC.torque_vectoring.f_r.f_r *= RVC.torque_vectoring.f_r.f_r_sat;

	// L/R split
	RVC.torque_vectoring.l_r.m_c = 0; // <TODO>
	RVC.torque_vectoring.l_r.f_f = RVC.torque_vectoring.f_r.f_f;
	RVC.torque_vectoring.l_r.f_r = RVC.torque_vectoring.f_r.f_r;
	double f_f_margin = f_i_margin(RVC.torque_vectoring.l_r.f_f);
	double f_r_margin = f_i_margin(RVC.torque_vectoring.l_r.f_r);
	double f_c_0 = diff_f_conversion(RVC.torque_vectoring.l_r.m_c);
	RVC.torque_vectoring.l_r.k_tfr = l_r_split(f_f_margin, f_r_margin);
	double f_c_f = diff_f_c_conversion(RVC.torque_vectoring.l_r.k_tfr, f_c_0, f_f_margin, f_r_margin);
	double f_c_r = diff_f_c_conversion(1.0 - RVC.torque_vectoring.l_r.k_tfr, f_c_0, f_f_margin, f_r_margin);
	double f_exceed = diff_f_exceed_conversion(absd(f_c_0), f_f_margin + f_r_margin);

	int s = sgn(f_c_0); // sign of f_c_0
	RVC.torque_vectoring.l_r.f_fl = drive_left_force_conversion(RVC.torque_vectoring.l_r.f_f, s, f_c_f, f_exceed);
	RVC.torque_vectoring.l_r.f_fr = drive_right_force_conversion(RVC.torque_vectoring.l_r.f_f, s, f_c_f, f_exceed);
	RVC.torque_vectoring.l_r.f_rl = drive_left_force_conversion(RVC.torque_vectoring.l_r.f_r, s, f_c_r, f_exceed);
	RVC.torque_vectoring.l_r.f_rr = drive_right_force_conversion(RVC.torque_vectoring.l_r.f_r, s, f_c_r, f_exceed);

	RVC.torque.frontLeft = force_to_torque(RVC.torque_vectoring.l_r.f_fl);
	RVC.torque.frontRight = force_to_torque(RVC.torque_vectoring.l_r.f_fr);
	RVC.torque.rearLeft = force_to_torque(RVC.torque_vectoring.l_r.f_rl);
	RVC.torque.rearRight = force_to_torque(RVC.torque_vectoring.l_r.f_rr);

	RVC_torqueSignalGeneration();

	/* TODO: Shared variable update */
	RVC_updateSharedVariable();
}

void RVC_run_10ms(void)
{
	RVC_pollGpi(&RVC.airPositive);
	RVC_pollGpi(&RVC.airNegative);
	RVC_pollGpi(&RVC.brakePressureOn);
	RVC_pollGpi(&RVC.brakeSwitch);
	RVC_pollGpi(&RVC.tsalOn);
	RVC_pollGpi(&RVC.sdcSenBspd);
	RVC_pollGpi(&RVC.sdcSenImd);
	RVC_pollGpi(&RVC.sdcSenAms);
	RVC_pollGpi(&RVC.sdcSenFinal);
	RVC_pollGpi(&RVC.bmsOk);
	RVC_pollGpi(&RVC.imdOk);
	RVC_pollGpi(&RVC.bspdOk);
	RVC_pollGpi(&RVC.bmsMpo);
	RVC_pollGpi(&RVC.chargeEn);
	AdcSensor_getData(&RVC.LvBattery_Voltage);
	AdcSensor_getData(&RVC.BrakePressure1);
	AdcSensor_getData(&RVC.BrakePressure2);
}

/****************** Private Function Implementation ******************/
IFX_STATIC void RVC_initAdcSensor(void)
{
	AdcSensor_Config adcConfig;

	/* LV battery voltage */
	adcConfig.adcConfig.channelIn = &(HLD_Vadc_Channel_In){HLD_Vadc_group5, HLD_Vadc_ChannelId_6};

	adcConfig.adcConfig.lpf.activated = TRUE;
	adcConfig.adcConfig.lpf.config.gain = 1;
	adcConfig.adcConfig.lpf.config.cutOffFrequency = 2;
	adcConfig.adcConfig.lpf.config.samplingTime = 10.0e-3;

	adcConfig.isOvervoltageProtected = FALSE;
	adcConfig.linCalConfig.isAct = TRUE;
	adcConfig.linCalConfig.a = LVBAT_LINCAL_A;
	adcConfig.linCalConfig.b = LVBAT_LINCAL_B;
	adcConfig.linCalConfig.d = LVBAT_LINCAL_D;
	adcConfig.tfConfig.a = (130.0f + 20.0f) / 20.0f;
	adcConfig.tfConfig.b = 0.0f;

	AdcSensor_initSensor(&RVC.LvBattery_Voltage, &adcConfig);
	HLD_AdcForceStart(RVC.LvBattery_Voltage.adcChannel.channel.group);

	/* Brake Pressure*/
	adcConfig.adcConfig.lpf.activated = TRUE;
	adcConfig.adcConfig.lpf.config.gain = 1;
	adcConfig.adcConfig.lpf.config.cutOffFrequency = 1 / (2 * IFX_PI * (1e-2f));
	adcConfig.adcConfig.lpf.config.samplingTime = 10.0e-3;

	adcConfig.isOvervoltageProtected = FALSE;
	adcConfig.linCalConfig.isAct = FALSE;
	adcConfig.tfConfig.a = BP_MAX_BAR / (BP_MAX_V - BP_MIN_V);
	adcConfig.tfConfig.b = BP_MAX_BAR / (BP_MAX_V - BP_MIN_V) * (-BP_MIN_V);

	adcConfig.adcConfig.channelIn = &(HLD_Vadc_Channel_In){HLD_Vadc_group4, HLD_Vadc_ChannelId_4};
	AdcSensor_initSensor(&RVC.BrakePressure1, &adcConfig);
	adcConfig.adcConfig.channelIn = &(HLD_Vadc_Channel_In){HLD_Vadc_group0, HLD_Vadc_ChannelId_2};
	AdcSensor_initSensor(&RVC.BrakePressure2, &adcConfig);
	HLD_AdcForceStart(RVC.BrakePressure1.adcChannel.channel.group);
	HLD_AdcForceStart(RVC.BrakePressure2.adcChannel.channel.group);

	/* Steering Angle Analog (Backup function) */
	// TODO
}

IFX_STATIC void RVC_initPwm(void)
{
	/* PWM output initialzation */
	HLD_GtmTom_Pwm_Config pwmConfig;
	pwmConfig.frequency = PWMFREQ;

	pwmConfig.tomOut = &PWMACCL;
	HLD_GtmTomPwm_initPwm(&RVC.out.accel_rearLeft, &pwmConfig);

	pwmConfig.tomOut = &PWMACCR;
	HLD_GtmTomPwm_initPwm(&RVC.out.accel_rearRight, &pwmConfig);

	pwmConfig.tomOut = &PWMDCCL;
	HLD_GtmTomPwm_initPwm(&RVC.out.decel_rearLeft, &pwmConfig);

	pwmConfig.tomOut = &PWMDCCR;
	HLD_GtmTomPwm_initPwm(&RVC.out.decel_rearRight, &pwmConfig);

	/* PWM output calibration */
	RVC.calibration.leftAcc.mul = OUTCAL_LEFT_MUL;
	RVC.calibration.leftAcc.offset = OUTCAL_LEFT_OFFSET;

	RVC.calibration.rightAcc.mul = OUTCAL_RIGHT_MUL;
	RVC.calibration.rightAcc.offset = OUTCAL_RIGHT_OFFSET;
}

IFX_STATIC void RVC_initGpio(void)
{
	/* FWD output config */
	IfxPort_setPinMode(FWD_OUT.port, FWD_OUT.pinIndex, IfxPort_OutputMode_pushPull);
	IfxPort_setPinLow(FWD_OUT.port, FWD_OUT.pinIndex);
	/* R2D signal output config */
	IfxPort_setPinMode(R2DOUT.port, R2DOUT.pinIndex, IfxPort_OutputMode_pushPull);
	IfxPort_setPinHigh(R2DOUT.port, R2DOUT.pinIndex);

	/* Start button config */
	Gpio_Debounce_inputConfig gpioInputConfig;
	Gpio_Debounce_initInputConfig(&gpioInputConfig);
	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &START_BTN;
	Gpio_Debounce_initInput(&RVC.startButton, &gpioInputConfig);

	/* AIR Contact signal input config */
	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &AIR_P_IN;
	Gpio_Debounce_initInput(&RVC.airPositive.debounce, &gpioInputConfig);
	gpioInputConfig.port = &AIR_N_IN;
	Gpio_Debounce_initInput(&RVC.airNegative.debounce, &gpioInputConfig);

	/* Pedalbox signal input config */
	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &BP_IN;
	Gpio_Debounce_initInput(&RVC.brakePressureOn.debounce, &gpioInputConfig);
	gpioInputConfig.port = &BSW_IN;
	Gpio_Debounce_initInput(&RVC.brakeSwitch.debounce, &gpioInputConfig);

	/* TSAL light input config */
	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &TSAL_RED_ON_5V;
	Gpio_Debounce_initInput(&RVC.tsalOn.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &SDC_SEN_BSPD_5V;
	Gpio_Debounce_initInput(&RVC.sdcSenBspd.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &SDC_SEN_IMD_5V;
	Gpio_Debounce_initInput(&RVC.sdcSenImd.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &SDC_SEN_AMS_5V;
	Gpio_Debounce_initInput(&RVC.sdcSenAms.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &SDC_SEN_FINAL_5V;
	Gpio_Debounce_initInput(&RVC.sdcSenFinal.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &BMS_OK;
	Gpio_Debounce_initInput(&RVC.bmsOk.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &IMD_OK;
	Gpio_Debounce_initInput(&RVC.imdOk.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &BSPD_OK;
	Gpio_Debounce_initInput(&RVC.bspdOk.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &BMS_MPO_5V;
	Gpio_Debounce_initInput(&RVC.bmsMpo.debounce, &gpioInputConfig);

	gpioInputConfig.bufferLen = Gpio_Debounce_BufferLength_10;
	gpioInputConfig.inputMode = IfxPort_InputMode_noPullDevice;
	gpioInputConfig.port = &CHARGE_EN;
	Gpio_Debounce_initInput(&RVC.chargeEn.debounce, &gpioInputConfig);
}

IFX_STATIC void RVC_pollGpi(RVC_Gpi_t *gpi)
{
	gpi->value = Gpio_Debounce_pollInput(&gpi->debounce);
}

/***************** Inline Function Implementation ******************/
IFX_INLINE void RVC_updateReadyToDriveSignal(void)
{
	static boolean curR2d = FALSE;
	static boolean pastR2d = FALSE;

	/*Store past AMK State*/
	pastR2d = curR2d;

	/*Get current AMK State*/
	while(IfxCpu_acquireMutex(&AmkInverterPublic.mutex))
		; // Wait for the mutex
	{
		curR2d = AmkInverterPublic.r2d;
		IfxCpu_releaseMutex(&AmkInverterPublic.mutex);
	}

	/*Update RTD state*/
	if(curR2d == TRUE)
	{
		RVC.readyToDrive = RVC_ReadyToDrive_status_run;
	}
	else
	{
		RVC.readyToDrive = RVC_ReadyToDrive_status_initialized;
	}

	/*Invoke RTDS*/
	if((pastR2d != curR2d) && (curR2d == TRUE))
	{
		// Turn off the drivetrain befor entering the ready-to-drive sound
		RVC.torque.controlled = 0;
		// Enter ready-to-drive sound
		rtds = TRUE;
	}

	if(rtds)
	{
		if(RVC.RTDS_Tick < RTDS_TIME)
		{
			RVC.RTDS_Tick++;
			IfxPort_setPinLow(R2DOUT.port, R2DOUT.pinIndex);
			// IfxPort_setPinHigh(FWD_OUT.port, FWD_OUT.pinIndex);
			//			if (RVC.RTDS_Tick == 2000)	CanCommunication_reInit();
			//			CanCommunication_reInit();
		}
		else
		{
			rtds = FALSE;
			RVC.RTDS_Tick = 0;
			IfxPort_setPinHigh(R2DOUT.port, R2DOUT.pinIndex);
			//			CanCommunication_reInit();
			// IfxPort_setPinLow(FWD_OUT.port, FWD_OUT.pinIndex);
		}
	}
}

IFX_INLINE void RVC_getTorqueRequired(void)
{
	if(SDP_PedalBox.apps.isValueOk && !rtds) // APPS Plausibility check
	{
		RVC.torque.controlled = (RVC.torque.desired = RVC_PedalMap_lut_getResult(SDP_PedalBox.apps.pps));
	}
	else
	{
		RVC.torque.controlled = (RVC.torque.desired = 0); // APPS Fail
	}
#ifdef BRAKE_ON_BP // No Regen!	//TODO
	if(RVC.brakeOn.tot == TRUE)
	{
		RVC.torque.controlled = (RVC.torque.desired = 0); // Zero torque signal when brake on.
	}
#else
	if(SDP_PedalBox.bpps.isValueOk) // BPPS Plausibility check
	{
		if(SDP_PedalBox.bpps.pps > PEDAL_BRAKE_ON_THRESHOLD)
		{
			RVC.torque.desired = -(SDP_PedalBox.bpps.pps); // BPPS overide
			if(RVC.torque.isRegenOn)                       // Regen
			{
				RVC.torque.controlled = RVC.torque.desired;
			}
			else
			{
				RVC.torque.controlled = (RVC.torque.desired = 0); // Regen off: Zero torque signal when brake on.
			}
		}
	}
	else // FIXME: BSPD control using Brake Pressure Analog signal. Failsafe for BPPS.
	{
		RVC.torque.controlled = 0; // BPPS Fail
	}
#endif
}

IFX_INLINE void RVC_torqueSignalGeneration(void)
{
	while(IfxCpu_acquireMutex(&AmkInverterPublic.mutex))
		; // Wait for the mutex
	{
		if(RVC.readyToDrive != RVC_ReadyToDrive_status_run)
			AmkInverterPublic.r2d = FALSE;

		AmkInverterPublic.fl = RVC.torque.frontLeft;
		AmkInverterPublic.fr = RVC.torque.frontRight;
		AmkInverterPublic.rl = RVC.torque.rearLeft;
		AmkInverterPublic.rr = RVC.torque.rearRight;

		AmkInverterPublic.brakeOn = RVC.brakeOn.tot;

		AmkInverterPublic.acceleraing = (RVC.torque.desired > 0) ? TRUE : FALSE;

		IfxCpu_releaseMutex(&AmkInverterPublic.mutex);
	}
}

IFX_INLINE void VariableUpdateRoutine_dashboard(void)
{
	// DashBoard_public.shared.data.vcu			= RVC.vcuOk.value;
	DashBoard_public.shared.data.bmsOk = RVC.bmsOk.value;
	DashBoard_public.shared.data.imdOk = RVC.imdOk.value;
	DashBoard_public.shared.data.bspdOk = RVC.bspdOk.value;
	DashBoard_public.shared.data.appsOk = SDP_PedalBox.apps.isValueOk;
	DashBoard_public.shared.data.bppsOk = SDP_PedalBox.bpps.isValueOk;
	DashBoard_public.shared.data.sdcSenFinal = RVC.sdcSenFinal.value;
	DashBoard_public.shared.data.rtdOn = (RVC.readyToDrive == RVC_ReadyToDrive_status_run);
	DashBoard_public.shared.data.brakeOn = RVC.brakeOn.tot;
	DashBoard_public.shared.data.tsalOn = RVC.tsalOn.value;
}

IFX_INLINE void VariableUpdateRoutine_mech_msg(void)
{
	mech_msg_public.shared.data.steering_angle = (sint16)(SDP_SteeringAngleAdc.sta.degree * 100);
	mech_msg_public.shared.data.apps = (uint8)SDP_PedalBox.apps.pps;
	mech_msg_public.shared.data.bpps = (uint8)SDP_PedalBox.bpps.pps;
	mech_msg_public.shared.data.brake_pressure_0 = (uint16)RVC.BrakePressure1.value * 10;
	mech_msg_public.shared.data.brake_pressure_1 = (uint16)RVC.BrakePressure2.value * 10;
}

volatile uint32 updateErrorCount_dashboard = 0;
volatile uint32 updateErrorCount_mech_msg = 0;

IFX_INLINE void RVC_updateSharedVariable(void)
{
	if(IfxCpu_acquireMutex(&DashBoard_public.shared.mutex)) // Do not wait
	{
		VariableUpdateRoutine_dashboard();
		IfxCpu_releaseMutex(&DashBoard_public.shared.mutex);
		updateErrorCount_dashboard = 0;
	}
	else if(updateErrorCount_dashboard < VAR_UPDATE_ERROR_LIM)
	{
		updateErrorCount_dashboard++;
	}
	else
	{
		while(IfxCpu_acquireMutex(&DashBoard_public.shared.mutex))
			;
		{
			VariableUpdateRoutine_dashboard();
			IfxCpu_releaseMutex(&DashBoard_public.shared.mutex);
		}
		updateErrorCount_dashboard = 0;
	}

	if(IfxCpu_acquireMutex(&mech_msg_public.shared.mutex)) // Do not wait
	{
		VariableUpdateRoutine_mech_msg();
		IfxCpu_releaseMutex(&mech_msg_public.shared.mutex);
		updateErrorCount_mech_msg = 0;
	}
	else if(updateErrorCount_mech_msg < VAR_UPDATE_ERROR_LIM)
	{
		updateErrorCount_mech_msg++;
	}
	else
	{
		while(IfxCpu_acquireMutex(&mech_msg_public.shared.mutex))
			;
		{
			VariableUpdateRoutine_mech_msg();
			IfxCpu_releaseMutex(&mech_msg_public.shared.mutex);
		}
		updateErrorCount_mech_msg = 0;
	}
}
