/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2023 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/
/***********************************************************************************************************************
 * File Name   : r_mtr_ics.c
 * Description : Processes of a user interface (tool)
 ***********************************************************************************************************************/
/**********************************************************************************************************************
 * History : DD.MM.YYYY Version
 *         : 23.05.2023 1.00
 ***********************************************************************************************************************/

/***********************************************************************************************************************
 * Includes <System Includes> , "Project Includes"
 ***********************************************************************************************************************/
#include <stdint.h>
#include "mtr_main.h"
#include "r_mtr_ics.h"
#include "ics2_RA6T3.h"
#include "hal_data.h"
#include "r_mtr_motor_parameter.h"
#include "r_mtr_control_parameter.h"
static uint8_t u1_cnt_ics;                     /* Counter for period of calling "scope_watchpoint" */

float    g_f4_id_ref_monitor;                  /* The reference d-axis current value [A] */
float    g_f4_id_ad_monitor;                   /* The d-axis current value [A] */
float    g_f4_iq_ref_monitor;                  /* The reference q-axis current value [A] */
float    g_f4_iq_ad_monitor;                   /* The q-axis current value [A] */
float    g_f4_iu_ad_monitor;                   /* U-phase current value [A] */
float    g_f4_iv_ad_monitor;                   /* V-phase current value [A] */
float    g_f4_iw_ad_monitor;                   /* W-phase current value [A] */
float    g_f4_vdc_ad_monitor;                  /* Main Line Voltage[V] */
float    g_f4_vd_ref_monitor;                  /* The reference d-axis voltage value [V] */
float    g_f4_vq_ref_monitor;                  /* The reference q-axis voltage value [V] */
float    g_f4_refu_monitor;                    /* U-phase reference voltage value [V] */
float    g_f4_refv_monitor;                    /* V-phase reference voltage value [V] */
float    g_f4_refw_monitor;                    /* W-phase reference voltage value [V] */
float    g_f4_ed_monitor;                      /* Estimated d-axis component[V] of flux due to the permanent magnet */
float    g_f4_eq_monitor;                      /* Estimated q-axis component[V] of flux due to the permanent magnet */
float    g_f4_phase_err_monitor;               /* Estimated phase error[rad] from actual rotor electrical angle */
float    g_f4_angle_rad_monitor;               /* Angle of rotor [rad] */
float    g_f4_speed_est_monitor;               /* speed value [rad/s] */
float    g_f4_speed_ref_monitor;               /* command speed value for speed PI control[rad/s] */
uint8_t  g_u1_state_id_ref_monitor;            /* The d-axis current command status */
uint8_t  g_u1_state_iq_ref_monitor;            /* The q-axis current command status */
uint8_t  g_u1_state_speed_ref_monitor;         /* The speed command status */
float    g_f4_speed_kp_monitor;                /* Kp for speed loop */
float    g_f4_speed_ki_monitor;                /* Ki for speed loop */
float    g_f4_current_kp_d_monitor;            /* Kp for d-axis current loop */
float    g_f4_current_ki_d_monitor;            /* Ki for d-axis current loop */
float    g_f4_current_kp_q_monitor;            /* Kp for q-axis current loop */
float    g_f4_current_ki_q_monitor;            /* Ki for q-axis current loop */
float    g_f4_k_e_obs_1_d_monitor;             /* BEMF observer gain 1 for d-axis */
float    g_f4_k_e_obs_2_d_monitor;             /* BEMF observer gain 2 for d-axis */
float    g_f4_k_e_obs_1_q_monitor;             /* BEMF observer gain 1 for q-axis */
float    g_f4_k_e_obs_2_q_monitor;             /* BEMF observer gain 2 for q-axis */
float    g_f4_kp_est_speed_monitor;            /* The proportional gain for PLL */
float    g_f4_ki_est_speed_monitor;            /* The integral gain for PLL*/
float    g_f4_speed_rpm_monitor;

float    com_f4_ref_speed_rpm = 0;             /* Motor speed reference [rpm] (mechanical) */
uint16_t com_u2_mtr_pp;                        /* Pole pairs */
float    com_f4_mtr_r;                         /* Resistance [ohm] */
float    com_f4_mtr_ld;                        /* D-axis inductance [H] */
float    com_f4_mtr_lq;                        /* Q-axis inductance [H] */
float    com_f4_mtr_m;                         /* Permanent magnetic flux [Wb] */
float    com_f4_mtr_j;                         /* Rotor inertia [kgm^2] */
float    com_f4_current_omega;                 /* Natural frequency for current loop [Hz] */
float    com_f4_current_zeta;                  /* Damping ratio for current loop */
float    com_f4_speed_omega;                   /* Natural frequency for speed loop [Hz] */
float    com_f4_speed_zeta;                    /* Damping ratio for speed loop */
float    com_f4_e_obs_omega;                   /* Natural frequency for BEMF observer [Hz] */
float    com_f4_e_obs_zeta;                    /* Damping ratio for BEMF observer */
float    com_f4_pll_est_omega;                 /* Natural frequency for rotor position Phase-Locked Loop [Hz] */
float    com_f4_pll_est_zeta;                  /* Damping ratio for rotor position Phase-Locked Loop */
float    com_f4_ref_id;                        /* Id reference when open loop [A] */
float    com_f4_ol_id_up_step;                 /* The d-axis current reference ramping up rate */
float    com_f4_ol_id_down_step;               /* The d-axis current reference ramping down rate */
float    com_f4_id_down_speed_rpm;             /* The speed threshold to ramp down the d-axis current */
float    com_f4_id_up_speed_rpm;               /* The speed threshold to ramp up the d-axis current */
float    com_f4_max_speed_rpm;                 /* Maximum speed */
float    com_f4_overspeed_limit_rpm;           /* Over speed limit */
float    com_f4_overcurrent_limit;             /* Over current limit */
float    com_f4_iq_limit;                      /* Q-axis current limit */
float    com_f4_limit_speed_change;            /* Limit of speed change */
uint8_t  com_u1_enable_write = 0;              /* ICS write enable flag */
uint8_t  g_u1_enable_write = 0;                /* ICS write enable flag */
uint8_t  g_u1_trig_enable_write;

extern   motor_cfg_t g_user_motor_cfg;
extern   motor_sensorless_extended_cfg_t g_user_motor_sensorless_extended_cfg;
extern   motor_speed_cfg_t g_user_motor_speed_cfg;
extern   motor_speed_extended_cfg_t g_user_motor_speed_extended_cfg;
extern   motor_current_cfg_t g_user_motor_current_cfg;
extern   motor_current_extended_cfg_t g_user_motor_current_extended_cfg;
extern   motor_angle_cfg_t g_user_motor_angle_cfg;
extern   motor_estimate_extended_cfg_t g_user_motor_estimate_extended_cfg;
extern   motor_driver_cfg_t g_user_motor_driver_cfg;
extern   motor_driver_extended_cfg_t g_user_motor_driver_extended_cfg;
extern   uint16_t g_u2_max_speed_rpm;


/* Apostolis extra on-the-fly changing of speed, current gains*/
extern float    com_f4_on_the_fly_speed_omega;
extern float    com_f4_on_the_fly_speed_zeta;
extern float   com_f4_on_the_fly_current_omega;
extern float    com_f4_on_the_fly_current_zeta;
extern float    com_f4_on_the_fly_obs_omega;
extern float    com_f4_on_the_fly_obs_zeta;
extern float    com_f4_on_the_fly_pll_omega;
extern float    com_f4_on_the_fly_pll_zeta;
extern uint8_t   g_u1_on_the_fly_activate;
/* Apostolis extra on-the-fly changing of speed, current gains*/

/***********************************************************************************************************************
 * Function Name : mtr_set_com_variables
 * Description   : Set com variables
 * Arguments     : None
 * Return Value  : None
 ***********************************************************************************************************************/
void mtr_set_com_variables(void)
{
    if (com_u1_enable_write == g_u1_enable_write)
    {
        if (com_f4_ref_speed_rpm > (float)g_u2_max_speed_rpm)
        {
            com_f4_ref_speed_rpm = (float)g_u2_max_speed_rpm;
        }
        else if (com_f4_ref_speed_rpm < -(float)g_u2_max_speed_rpm)
        {
            com_f4_ref_speed_rpm = -(float)g_u2_max_speed_rpm;
        }
        g_motor_sensorless0.p_api->speedSet(g_motor_sensorless0.p_ctrl, com_f4_ref_speed_rpm);

        g_user_motor_sensorless_extended_cfg.f_overspeed_limit   = com_f4_overspeed_limit_rpm;
        g_user_motor_sensorless_extended_cfg.f_overcurrent_limit = com_f4_overcurrent_limit * MTR_SQRT_2 * MTR_OVERCURRENT_MARGIN_MULT;

        g_user_motor_speed_extended_cfg.f_limit_speed_change          = com_f4_limit_speed_change * com_u2_mtr_pp;
        g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_ref         = com_f4_ref_id;
        g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_up_step     = com_f4_ol_id_up_step * com_f4_ref_id;
        g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_down_step   = com_f4_ol_id_down_step * com_f4_ref_id;
        g_user_motor_speed_extended_cfg.ol_param.f4_id_down_speed_rad = com_f4_id_down_speed_rpm * com_u2_mtr_pp;
        g_user_motor_speed_extended_cfg.ol_param.f4_id_up_speed_rad   = com_f4_id_up_speed_rpm * com_u2_mtr_pp;
        g_user_motor_speed_extended_cfg.f_max_speed_rad               = com_f4_max_speed_rpm * com_u2_mtr_pp;
        g_user_motor_speed_extended_cfg.d_param.f_speed_omega         = com_f4_speed_omega;
        g_user_motor_speed_extended_cfg.d_param.f_speed_zeta          = com_f4_speed_zeta;
        g_user_motor_speed_extended_cfg.mtr_param.u2_mtr_pp           = com_u2_mtr_pp;
        g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_r            = com_f4_mtr_r;
        g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_ld           = com_f4_mtr_ld;
        g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_lq           = com_f4_mtr_lq;
        g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_m            = com_f4_mtr_m;
        g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_j            = com_f4_mtr_j;

        g_user_motor_current_extended_cfg.p_motor_parameter->u2_mtr_pp        = com_u2_mtr_pp;
        g_user_motor_current_extended_cfg.p_motor_parameter->f4_mtr_r         = com_f4_mtr_r;
        g_user_motor_current_extended_cfg.p_motor_parameter->f4_mtr_ld        = com_f4_mtr_ld;
        g_user_motor_current_extended_cfg.p_motor_parameter->f4_mtr_lq        = com_f4_mtr_lq;
        g_user_motor_current_extended_cfg.p_motor_parameter->f4_mtr_m         = com_f4_mtr_m;
        g_user_motor_current_extended_cfg.p_motor_parameter->f4_mtr_j         = com_f4_mtr_j;
        g_user_motor_current_extended_cfg.p_design_parameter->f_current_omega = com_f4_current_omega;
        g_user_motor_current_extended_cfg.p_design_parameter->f_current_zeta  = com_f4_current_zeta;

        g_user_motor_estimate_extended_cfg.f_e_obs_omega             = com_f4_e_obs_omega;
        g_user_motor_estimate_extended_cfg.f_e_obs_zeta              = com_f4_e_obs_zeta;
        g_user_motor_estimate_extended_cfg.f_pll_est_omega           = com_f4_pll_est_omega;
        g_user_motor_estimate_extended_cfg.f_pll_est_zeta            = com_f4_pll_est_zeta;
        g_user_motor_estimate_extended_cfg.st_motor_params.u2_mtr_pp = com_u2_mtr_pp;
        g_user_motor_estimate_extended_cfg.st_motor_params.f4_mtr_r  = com_f4_mtr_r;
        g_user_motor_estimate_extended_cfg.st_motor_params.f4_mtr_ld = com_f4_mtr_ld;
        g_user_motor_estimate_extended_cfg.st_motor_params.f4_mtr_lq = com_f4_mtr_lq;
        g_user_motor_estimate_extended_cfg.st_motor_params.f4_mtr_m  = com_f4_mtr_m;
        g_user_motor_estimate_extended_cfg.st_motor_params.f4_mtr_j  = com_f4_mtr_j;
        g_motor_angle0_ctrl.st_bemf_obs.st_motor_params.u2_mtr_pp    = com_u2_mtr_pp;
        g_motor_angle0_ctrl.st_bemf_obs.st_motor_params.f4_mtr_r     = com_f4_mtr_r;
        g_motor_angle0_ctrl.st_bemf_obs.st_motor_params.f4_mtr_ld    = com_f4_mtr_ld;
        g_motor_angle0_ctrl.st_bemf_obs.st_motor_params.f4_mtr_lq    = com_f4_mtr_lq;
        g_motor_angle0_ctrl.st_bemf_obs.st_motor_params.f4_mtr_m     = com_f4_mtr_m;
        g_motor_angle0_ctrl.st_bemf_obs.st_motor_params.f4_mtr_j     = com_f4_mtr_j;
        g_u1_trig_enable_write = MTR_FLG_SET;
        g_u1_enable_write ^= 1;                         /* Change every time 0 and 1 */
    }
}

/***********************************************************************************************************************
 * Function Name : mtr_ics_variables_init
 * Description   : Initialize valiables for Analyzer interface
 * Arguments     : None
 * Return Value  : None
 ***********************************************************************************************************************/
void mtr_ics_variables_init(void)
{
    g_u1_enable_write = 0;
    com_u1_enable_write = 0;

#if MTR_MOTOR_PARAMETER
    com_u2_mtr_pp            = MP_POLE_PAIRS;
    com_f4_mtr_r             = MP_RESISTANCE;
    com_f4_mtr_ld            = MP_D_INDUCTANCE;
    com_f4_mtr_lq            = MP_Q_INDUCTANCE;
    com_f4_mtr_m             = MP_MAGNETIC_FLUX;
    com_f4_mtr_j             = MP_ROTOR_INERTIA;
    com_f4_overcurrent_limit = MP_NOMINAL_CURRENT_RMS*2;
    //com_f4_iq_limit          = MP_NOMINAL_CURRENT_RMS;
    com_f4_iq_limit          = (float)(MP_MAGNETIC_FLUX/MP_D_INDUCTANCE);
#else
    com_u2_mtr_pp            = g_user_motor_speed_extended_cfg.mtr_param.u2_mtr_pp;
    com_f4_mtr_r             = g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_r;
    com_f4_mtr_ld            = g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_ld;
    com_f4_mtr_lq            = g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_lq;
    com_f4_mtr_m             = g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_m;
    com_f4_mtr_j             = g_user_motor_speed_extended_cfg.mtr_param.f4_mtr_j;
    com_f4_overcurrent_limit = g_user_motor_sensorless_extended_cfg.f_overcurrent_limit / (MTR_SQRT_2 * MTR_OVERCURRENT_MARGIN_MULT);
    com_f4_iq_limit          = g_user_motor_speed_extended_cfg.f_iq_limit;
#endif
    com_f4_limit_speed_change = g_user_motor_speed_extended_cfg.f_limit_speed_change / g_user_motor_speed_extended_cfg.mtr_param.u2_mtr_pp;

#if MTR_CONTROL_PARAMETER
    com_f4_current_omega       = CP_CURRENT_OMEGA;
    com_f4_current_zeta        = CP_CURRENT_ZETA;
    com_f4_speed_omega         = CP_SPEED_OMEGA;
    com_f4_speed_zeta          = CP_SPEED_ZETA;
    com_f4_e_obs_omega         = CP_E_OBS_OMEGA;
    com_f4_e_obs_zeta          = CP_E_OBS_ZETA;
    com_f4_pll_est_omega       = CP_PLL_EST_OMEGA;
    com_f4_pll_est_zeta        = CP_PLL_EST_ZETA;
    com_f4_id_down_speed_rpm   = CP_ID_DOWN_SPEED_RPM;
    com_f4_id_up_speed_rpm     = CP_ID_UP_SPEED_RPM;
    com_f4_max_speed_rpm       = CP_MAX_SPEED_RPM;
    com_f4_overspeed_limit_rpm = CP_OVERSPEED_LIMIT_RPM;
    com_f4_ref_id              = CP_OL_ID_REF;
#else
    com_f4_current_omega       = g_user_motor_current_extended_cfg.p_design_parameter->f_current_omega;
    com_f4_current_zeta        = g_user_motor_current_extended_cfg.p_design_parameter->f_current_zeta;
    com_f4_speed_omega         = g_user_motor_speed_extended_cfg.d_param.f_speed_omega;
    com_f4_speed_zeta          = g_user_motor_speed_extended_cfg.d_param.f_speed_zeta;
    com_f4_e_obs_omega         = g_user_motor_estimate_extended_cfg.f_e_obs_omega;
    com_f4_e_obs_zeta          = g_user_motor_estimate_extended_cfg.f_e_obs_zeta;
    com_f4_pll_est_omega       = g_user_motor_estimate_extended_cfg.f_pll_est_omega;
    com_f4_pll_est_zeta        = g_user_motor_estimate_extended_cfg.f_pll_est_zeta;
    com_f4_id_down_speed_rpm   = g_user_motor_speed_extended_cfg.ol_param.f4_id_down_speed_rad / g_user_motor_speed_extended_cfg.mtr_param.u2_mtr_pp;
    com_f4_id_up_speed_rpm     = g_user_motor_speed_extended_cfg.ol_param.f4_id_up_speed_rad / g_user_motor_speed_extended_cfg.mtr_param.u2_mtr_pp;
    com_f4_max_speed_rpm       = g_user_motor_speed_extended_cfg.f_max_speed_rad / g_user_motor_speed_extended_cfg.mtr_param.u2_mtr_pp;
    com_f4_overspeed_limit_rpm = g_user_motor_sensorless_extended_cfg.f_overspeed_limit;
    com_f4_ref_id              = g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_ref;
#endif
    com_f4_ol_id_up_step   = g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_up_step / g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_ref;
    com_f4_ol_id_down_step = g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_down_step / g_user_motor_speed_extended_cfg.ol_param.f4_ol_id_ref;
}

/***********************************************************************************************************************
 * Function Name : mtr_ics_interrupt_process
 * Description   : Process for ICS(Analyzer)
 * Arguments     : None
 * Return Value  : None
 ***********************************************************************************************************************/
void mtr_ics_interrupt_process(void)
{
    u1_cnt_ics++;
    if (MTR_ICS_DECIMATION <= u1_cnt_ics)                      /* Decimation of ICS call */
    {

        /* Apostolis extra on-the-fly changing of speed, current gains*/
        if(g_u1_on_the_fly_activate==1){

            g_user_motor_speed_extended_cfg.d_param.f_speed_omega = com_f4_on_the_fly_speed_omega;
            g_user_motor_speed_extended_cfg.d_param.f_speed_zeta  = com_f4_on_the_fly_speed_zeta;
            g_user_motor_current_extended_cfg.p_design_parameter->f_current_omega = com_f4_on_the_fly_current_omega;
            g_user_motor_current_extended_cfg.p_design_parameter-> f_current_zeta = com_f4_on_the_fly_current_zeta;
            g_user_motor_estimate_extended_cfg.f_e_obs_omega =   com_f4_on_the_fly_obs_omega;
            g_user_motor_estimate_extended_cfg.f_e_obs_zeta =    com_f4_on_the_fly_obs_zeta;
            g_user_motor_estimate_extended_cfg.f_pll_est_omega = com_f4_on_the_fly_pll_omega;
            g_user_motor_estimate_extended_cfg.f_pll_est_zeta=   1.0;


                    g_motor_speed0.p_api->parameterUpdate(g_motor_speed0.p_ctrl, &g_user_motor_speed_cfg);
            g_motor_current0.p_api->parameterUpdate(g_motor_current0.p_ctrl, &g_user_motor_current_cfg);
            g_motor_angle0.p_api->parameterUpdate(g_motor_angle0.p_ctrl, &g_user_motor_angle_cfg);
        }

        else {
            g_user_motor_speed_extended_cfg.d_param.f_speed_omega = CP_SPEED_OMEGA;
            g_user_motor_speed_extended_cfg.d_param.f_speed_zeta  = CP_SPEED_ZETA;
            g_user_motor_current_extended_cfg.p_design_parameter->f_current_omega = CP_CURRENT_OMEGA;
            g_user_motor_current_extended_cfg.p_design_parameter-> f_current_zeta = CP_CURRENT_ZETA;
            g_user_motor_estimate_extended_cfg.f_e_obs_omega =   CP_E_OBS_OMEGA;
            g_user_motor_estimate_extended_cfg.f_e_obs_zeta =    CP_E_OBS_ZETA;
            g_user_motor_estimate_extended_cfg.f_pll_est_omega = CP_PLL_EST_OMEGA;
            g_user_motor_estimate_extended_cfg.f_pll_est_zeta=  CP_PLL_EST_ZETA;

            g_motor_speed0.p_api->parameterUpdate(g_motor_speed0.p_ctrl, &g_user_motor_speed_cfg);
            g_motor_current0.p_api->parameterUpdate(g_motor_current0.p_ctrl, &g_user_motor_current_cfg);
            g_motor_angle0.p_api->parameterUpdate(g_motor_angle0.p_ctrl, &g_user_motor_angle_cfg);
        }
        /* Apostolis extra on-the-fly changing of speed, current gains*/


        g_f4_id_ref_monitor           = g_motor_current0_ctrl.f_id_ref;
        g_f4_id_ad_monitor            = g_motor_current0_ctrl.f_id_ad;
        g_f4_iq_ref_monitor           = g_motor_current0_ctrl.f_iq_ref;
        g_f4_iq_ad_monitor            = g_motor_current0_ctrl.f_iq_ad;
        g_f4_iu_ad_monitor            = g_motor_current0_ctrl.f_iu_ad;
        g_f4_iv_ad_monitor            = g_motor_current0_ctrl.f_iv_ad;
        g_f4_iw_ad_monitor            = g_motor_current0_ctrl.f_iw_ad;
        g_f4_vdc_ad_monitor           = g_motor_current0_ctrl.st_vcomp.f_vdc;
        g_f4_vd_ref_monitor           = g_motor_current0_ctrl.f_vd_ref;
        g_f4_vq_ref_monitor           = g_motor_current0_ctrl.f_vq_ref;
        g_f4_refu_monitor             = g_motor_current0_ctrl.f_refu;
        g_f4_refv_monitor             = g_motor_current0_ctrl.f_refv;
        g_f4_refw_monitor             = g_motor_current0_ctrl.f_refw;
        g_f4_ed_monitor               = g_motor_angle0_ctrl.f4_ed;
        g_f4_eq_monitor               = g_motor_angle0_ctrl.f4_eq;
        g_f4_phase_err_monitor        = g_motor_angle0_ctrl.f4_phase_err_rad;
        g_f4_angle_rad_monitor        = g_motor_angle0_ctrl.f4_angle_rad;
        g_f4_speed_est_monitor        = g_motor_current0_ctrl.f_speed_rad;
        g_f4_speed_ref_monitor        = g_motor_speed0_ctrl.f_ref_speed_rad_ctrl;
        g_u1_state_id_ref_monitor     = g_motor_speed0_ctrl.u1_state_id_ref;
        g_u1_state_iq_ref_monitor     = g_motor_speed0_ctrl.u1_state_iq_ref;
        g_u1_state_speed_ref_monitor  = g_motor_speed0_ctrl.u1_state_speed_ref;

        g_f4_speed_kp_monitor         = g_motor_speed0_ctrl.pi_param.f_kp;
        g_f4_speed_ki_monitor         = g_motor_speed0_ctrl.pi_param.f_ki;
        g_f4_current_kp_d_monitor     = g_motor_current0_ctrl.st_pi_id.f_kp;
        g_f4_current_ki_d_monitor     = g_motor_current0_ctrl.st_pi_id.f_ki;
        g_f4_current_kp_q_monitor     = g_motor_current0_ctrl.st_pi_iq.f_kp;
        g_f4_current_ki_q_monitor     = g_motor_current0_ctrl.st_pi_iq.f_ki;
        g_f4_k_e_obs_1_d_monitor      = g_motor_angle0_ctrl.st_bemf_obs.st_d_axis.f4_k_e_obs_1;
        g_f4_k_e_obs_2_d_monitor      = g_motor_angle0_ctrl.st_bemf_obs.st_d_axis.f4_k_e_obs_2;
        g_f4_k_e_obs_1_q_monitor      = g_motor_angle0_ctrl.st_bemf_obs.st_q_axis.f4_k_e_obs_1;
        g_f4_k_e_obs_2_q_monitor      = g_motor_angle0_ctrl.st_bemf_obs.st_q_axis.f4_k_e_obs_2;
        g_f4_kp_est_speed_monitor     = g_motor_angle0_ctrl.st_pll_est.f4_kp_est_speed;
        g_f4_ki_est_speed_monitor     = g_motor_angle0_ctrl.st_pll_est.f4_ki_est_speed;

        g_f4_speed_rpm_monitor = g_f4_speed_est_monitor * MTR_RAD_RPM / (float)g_user_motor_current_extended_cfg.p_motor_parameter->u2_mtr_pp;

        u1_cnt_ics = 0;
        ics2_watchpoint();                                     /* Call ICS */
    }

    if (MTR_FLG_SET == g_u1_trig_enable_write)
    {
        g_motor_speed0.p_api->parameterUpdate(g_motor_speed0.p_ctrl, &g_user_motor_speed_cfg);
        g_motor_current0.p_api->parameterUpdate(g_motor_current0.p_ctrl, &g_user_motor_current_cfg);
        g_motor_angle0.p_api->parameterUpdate(g_motor_angle0.p_ctrl, &g_user_motor_angle_cfg);
        g_u1_trig_enable_write = MTR_FLG_CLR;
    }
} /* End of function mtr_ics_interrupt_process */
