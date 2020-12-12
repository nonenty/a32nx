#include "FlyByWire.h"
#include "FlyByWire_private.h"

const uint8_T FlyByWire_IN_InAir = 1U;
const uint8_T FlyByWire_IN_NO_ACTIVE_CHILD = 0U;
const uint8_T FlyByWire_IN_OnGround = 2U;
const uint8_T FlyByWire_IN_Flare_Reduce_Theta_c = 1U;
const uint8_T FlyByWire_IN_Flare_Set_Rate = 2U;
const uint8_T FlyByWire_IN_Flare_Store_Theta_c_deg = 3U;
const uint8_T FlyByWire_IN_Flight_High = 4U;
const uint8_T FlyByWire_IN_Flight_Low = 5U;
const uint8_T FlyByWire_IN_Ground = 6U;
const uint8_T FlyByWire_IN_frozen = 1U;
const uint8_T FlyByWire_IN_running = 2U;
const uint8_T FlyByWire_IN_Flight = 1U;
const uint8_T FlyByWire_IN_FlightToGroundTransition = 2U;
const uint8_T FlyByWire_IN_Ground_p = 3U;
const uint8_T FlyByWire_IN_automatic = 1U;
const uint8_T FlyByWire_IN_manual = 2U;
const uint8_T FlyByWire_IN_reset = 3U;
const uint8_T FlyByWire_IN_tracking = 4U;
const uint8_T FlyByWire_IN_flight_clean = 1U;
const uint8_T FlyByWire_IN_flight_flaps = 2U;
const uint8_T FlyByWire_IN_ground = 3U;
const uint8_T FlyByWire_IN_FlightMode = 1U;
const uint8_T FlyByWire_IN_GroundMode = 2U;
const fbw_output FlyByWire_rtZfbw_output = {
  {
    {
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    }
  },

  {
    {
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      false,
      false,
      0.0,
      false,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0
    },

    {
      0.0
    },

    {
      0.0
    },

    {
      0.0,
      0.0
    }
  },

  {
    {
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0,
      0.0,
      0.0
    },

    {
      0.0,
      0.0
    }
  },

  {
    0.0,
    0.0,
    false,
    0.0,
    0.0,
    0.0
  }
} ;

const fbw_input FlyByWire_rtZfbw_input = { { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, { 0.0,
    0.0, 0.0 } };

real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[], uint32_T maxIndex)
{
  real_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  return (table[iLeft + 1U] - table[iLeft]) * frac + table[iLeft];
}

void FlyByWireModelClass::step()
{
  real_T Theta;
  real_T Phi;
  real_T result[3];
  real_T result_0[3];
  real_T rtb_Limiterxi;
  real_T rtb_Limiterxi1;
  real_T rtb_Limitereta;
  real_T rtb_Product3;
  real_T rtb_Product4;
  real_T rtb_Gainpk4;
  real_T rtb_GainTheta;
  real_T rtb_GainPhi;
  real_T rtb_Gainqk;
  real_T rtb_Gain;
  real_T rtb_Gainpk;
  boolean_T rtb_NOT;
  int32_T rtb_on_ground;
  int32_T rtb_BusAssignment_a_sim_data_computed_tracking_mode_on;
  real_T rtb_BusAssignment_a_sim_input_delta_zeta_pos;
  int32_T rtb_in_flare;
  real_T rtb_nz_limit_up_g;
  int32_T rtb_nz_limit_lo_g;
  boolean_T rtb_eta_trim_deg_should_freeze;
  real_T rtb_eta_trim_deg_reset_deg;
  boolean_T rtb_eta_trim_deg_should_write;
  real_T rtb_eta_trim_deg_rate_limit_lo_deg_s;
  real_T rtb_BusAssignment_c_pitch_data_computed_delta_eta_deg;
  real_T rtb_BusAssignment_m_pitch_output_eta_deg;
  int32_T rtb_in_flight;
  real_T rtb_BusAssignment_p_roll_data_computed_delta_xi_deg;
  real_T rtb_BusAssignment_p_roll_data_computed_delta_zeta_deg;
  real_T rtb_ManualSwitch;
  real_T rtb_Gain1_b;
  real_T rtb_Switch_c;
  real_T result_tmp;
  real_T tmp[9];
  FlyByWire_DWork.Delay_DSTATE += FlyByWire_U.in.time.dt;
  rtb_GainTheta = FlyByWire_P.GainTheta_Gain * FlyByWire_U.in.data.Theta_deg;
  rtb_GainPhi = FlyByWire_P.GainPhi_Gain * FlyByWire_U.in.data.Phi_deg;
  rtb_Gainqk = FlyByWire_P.Gain_Gain_n * FlyByWire_U.in.data.q_rad_s * FlyByWire_P.Gainqk_Gain;
  rtb_Gain = FlyByWire_P.Gain_Gain_l * FlyByWire_U.in.data.r_rad_s;
  rtb_Gainpk = FlyByWire_P.Gain_Gain_a * FlyByWire_U.in.data.p_rad_s * FlyByWire_P.Gainpk_Gain;
  Theta = 0.017453292519943295 * rtb_GainTheta;
  Phi = 0.017453292519943295 * rtb_GainPhi;
  result_tmp = std::tan(Theta);
  rtb_Gainpk4 = std::sin(Phi);
  Phi = std::cos(Phi);
  tmp[0] = 1.0;
  tmp[3] = rtb_Gainpk4 * result_tmp;
  tmp[6] = Phi * result_tmp;
  tmp[1] = 0.0;
  tmp[4] = Phi;
  tmp[7] = -rtb_Gainpk4;
  tmp[2] = 0.0;
  rtb_Limiterxi1 = 1.0 / std::cos(Theta);
  tmp[5] = rtb_Limiterxi1 * rtb_Gainpk4;
  tmp[8] = rtb_Limiterxi1 * Phi;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result[rtb_on_ground] = tmp[rtb_on_ground + 6] * rtb_Gain + (tmp[rtb_on_ground + 3] * rtb_Gainqk + tmp[rtb_on_ground]
      * rtb_Gainpk);
  }

  Theta = 0.017453292519943295 * rtb_GainTheta;
  Phi = 0.017453292519943295 * rtb_GainPhi;
  result_tmp = std::tan(Theta);
  rtb_Gainpk4 = std::sin(Phi);
  Phi = std::cos(Phi);
  tmp[0] = 1.0;
  tmp[3] = rtb_Gainpk4 * result_tmp;
  tmp[6] = Phi * result_tmp;
  tmp[1] = 0.0;
  tmp[4] = Phi;
  tmp[7] = -rtb_Gainpk4;
  tmp[2] = 0.0;
  rtb_Limiterxi1 = 1.0 / std::cos(Theta);
  tmp[5] = rtb_Limiterxi1 * rtb_Gainpk4;
  tmp[8] = rtb_Limiterxi1 * Phi;
  rtb_Gainpk4 = FlyByWire_P.Gain_Gain_nm * FlyByWire_U.in.data.p_dot_rad_s2 * FlyByWire_P.Gainpk1_Gain;
  Theta = FlyByWire_P.Gain_Gain_e * FlyByWire_U.in.data.q_dot_rad_s2 * FlyByWire_P.Gainqk1_Gain;
  result_tmp = FlyByWire_P.Gain_Gain_aw * FlyByWire_U.in.data.r_dot_rad_s2;
  for (rtb_on_ground = 0; rtb_on_ground < 3; rtb_on_ground++) {
    result_0[rtb_on_ground] = tmp[rtb_on_ground + 6] * result_tmp + (tmp[rtb_on_ground + 3] * Theta + tmp[rtb_on_ground]
      * rtb_Gainpk4);
  }

  rtb_Gainpk4 = FlyByWire_P.Gainpk4_Gain * FlyByWire_U.in.data.eta_pos;
  rtb_Product3 = FlyByWire_P.Gainpk2_Gain * FlyByWire_U.in.data.eta_trim_deg;
  rtb_Switch_c = FlyByWire_P.Gain1_Gain_h * FlyByWire_U.in.data.gear_animation_pos_1 - FlyByWire_P.Constant_Value_g;
  if (rtb_Switch_c > FlyByWire_P.Saturation1_UpperSat_g) {
    rtb_Switch_c = FlyByWire_P.Saturation1_UpperSat_g;
  } else {
    if (rtb_Switch_c < FlyByWire_P.Saturation1_LowerSat_j) {
      rtb_Switch_c = FlyByWire_P.Saturation1_LowerSat_j;
    }
  }

  rtb_Limitereta = FlyByWire_P.Gain2_Gain_a * FlyByWire_U.in.data.gear_animation_pos_2 - FlyByWire_P.Constant_Value_g;
  if (rtb_Limitereta > FlyByWire_P.Saturation2_UpperSat_b) {
    rtb_Limitereta = FlyByWire_P.Saturation2_UpperSat_b;
  } else {
    if (rtb_Limitereta < FlyByWire_P.Saturation2_LowerSat_g) {
      rtb_Limitereta = FlyByWire_P.Saturation2_LowerSat_g;
    }
  }

  FlyByWire_Y.out.sim.data.gear_strut_compression_1 = rtb_Switch_c;
  FlyByWire_Y.out.sim.data.gear_strut_compression_2 = rtb_Limitereta;
  Theta = FlyByWire_P.Gaineta_Gain * FlyByWire_U.in.input.delta_eta_pos;
  if (FlyByWire_DWork.is_active_c1_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c1_FlyByWire = 1U;
    FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
    rtb_on_ground = 1;
  } else if (FlyByWire_DWork.is_c1_FlyByWire == FlyByWire_IN_InAir) {
    if ((rtb_Switch_c > 0.1) || (rtb_Limitereta > 0.1)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_OnGround;
      rtb_on_ground = 1;
    } else {
      rtb_on_ground = 0;
    }
  } else {
    if ((rtb_Switch_c == 0.0) && (rtb_Limitereta == 0.0)) {
      FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_InAir;
      rtb_on_ground = 0;
    } else {
      rtb_on_ground = 1;
    }
  }

  rtb_NOT = ((FlyByWire_U.in.data.autopilot_master_on != 0.0) || (FlyByWire_U.in.data.slew_on != 0.0) ||
             (FlyByWire_U.in.data.pause_on != 0.0) || (FlyByWire_U.in.data.tracking_mode_on_override != 0.0));
  FlyByWire_Y.out.sim.data.eta_trim_deg = rtb_Product3;
  result_tmp = FlyByWire_P.Gainpk3_Gain * FlyByWire_U.in.data.zeta_trim_pos;
  Phi = FlyByWire_P.Gainxi_Gain * FlyByWire_U.in.input.delta_xi_pos;
  rtb_BusAssignment_a_sim_input_delta_zeta_pos = FlyByWire_P.Gainxi1_Gain * FlyByWire_U.in.input.delta_zeta_pos;
  rtb_BusAssignment_a_sim_data_computed_tracking_mode_on = ((FlyByWire_U.in.data.autopilot_master_on != 0.0) ||
    (FlyByWire_U.in.data.slew_on != 0.0) || (FlyByWire_U.in.data.pause_on != 0.0) ||
    (FlyByWire_U.in.data.tracking_mode_on_override != 0.0));
  if (FlyByWire_DWork.is_active_c3_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c3_FlyByWire = 1U;
    FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground_p;
    FlyByWire_B.in_flight = 0.0;
  } else {
    switch (FlyByWire_DWork.is_c3_FlyByWire) {
     case FlyByWire_IN_Flight:
      if ((rtb_on_ground == 1) && (rtb_GainTheta < 2.5)) {
        FlyByWire_DWork.on_ground_time = FlyByWire_U.in.time.simulation_time;
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_FlightToGroundTransition;
      } else {
        FlyByWire_B.in_flight = 1.0;
      }
      break;

     case FlyByWire_IN_FlightToGroundTransition:
      if (FlyByWire_U.in.time.simulation_time - FlyByWire_DWork.on_ground_time >= 5.0) {
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Ground_p;
        FlyByWire_B.in_flight = 0.0;
      } else {
        if ((rtb_on_ground == 0) || (rtb_GainTheta >= 2.5)) {
          FlyByWire_DWork.on_ground_time = 0.0;
          FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight;
          FlyByWire_B.in_flight = 1.0;
        }
      }
      break;

     default:
      if (((rtb_on_ground == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.H_radio_ft > 400.0)) {
        FlyByWire_DWork.on_ground_time = 0.0;
        FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_Flight;
        FlyByWire_B.in_flight = 1.0;
      } else {
        FlyByWire_B.in_flight = 0.0;
      }
      break;
    }
  }

  rtb_Limiterxi = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1;
  rtb_Limiterxi1 = rtb_Limiterxi + FlyByWire_P.Constant_Value_f;
  FlyByWire_DWork.Delay1_DSTATE = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_f - rtb_Limiterxi) *
    FlyByWire_DWork.Delay1_DSTATE + (rtb_GainTheta + FlyByWire_DWork.Delay_DSTATE_b) * (rtb_Limiterxi / rtb_Limiterxi1);
  if (FlyByWire_P.ManualSwitch_CurrentSetting == 1) {
    rtb_ManualSwitch = FlyByWire_P.Constant1_Value;
  } else {
    rtb_ManualSwitch = FlyByWire_P.Constant_Value_j;
  }

  if (FlyByWire_DWork.is_active_c2_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c2_FlyByWire = 1U;
    FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Ground;
    rtb_in_flare = 0;
    FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
    FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
  } else {
    switch (FlyByWire_DWork.is_c2_FlyByWire) {
     case FlyByWire_IN_Flare_Reduce_Theta_c:
      if (FlyByWire_B.in_flight == 0.0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Ground;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else if ((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_ManualSwitch == 0.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 1;
        FlyByWire_B.flare_Theta_c_deg = -2.0;
      }
      break;

     case FlyByWire_IN_Flare_Set_Rate:
      if (FlyByWire_P.ManualSwitch1_CurrentSetting == 1) {
        rtb_Limiterxi1 = FlyByWire_P.Constant1_Value;
      } else {
        rtb_Limiterxi1 = FlyByWire_P.Constant_Value_j;
      }

      if ((FlyByWire_U.in.data.H_radio_ft <= 30.0) || (rtb_Limiterxi1 == 1.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Reduce_Theta_c;
        rtb_in_flare = 1;
        FlyByWire_B.flare_Theta_c_deg = -2.0;
      } else if ((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_ManualSwitch == 0.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 1;
      }
      break;

     case FlyByWire_IN_Flare_Store_Theta_c_deg:
      if ((FlyByWire_U.in.data.H_radio_ft > 50.0) && (rtb_ManualSwitch == 0.0)) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        FlyByWire_B.flare_Theta_c_rate_deg_s = -(FlyByWire_DWork.Delay1_DSTATE + 2.0) / 8.0;
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Set_Rate;
        rtb_in_flare = 1;
      }
      break;

     case FlyByWire_IN_Flight_High:
      if ((FlyByWire_U.in.data.H_radio_ft <= 50.0) || (rtb_ManualSwitch == 1.0)) {
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flare_Store_Theta_c_deg;
        rtb_in_flare = 1;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;

     case FlyByWire_IN_Flight_Low:
      if (FlyByWire_U.in.data.H_radio_ft > 50.0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_High;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;

     default:
      if (FlyByWire_B.in_flight == 1.0) {
        FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_Flight_Low;
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      } else {
        rtb_in_flare = 0;
        FlyByWire_B.flare_Theta_c_deg = FlyByWire_DWork.Delay1_DSTATE;
        FlyByWire_B.flare_Theta_c_rate_deg_s = -3.0;
      }
      break;
    }
  }

  if (FlyByWire_B.in_flight > FlyByWire_P.Saturation_UpperSat_er) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_UpperSat_er;
  } else if (FlyByWire_B.in_flight < FlyByWire_P.Saturation_LowerSat_a) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_LowerSat_a;
  } else {
    rtb_Limiterxi1 = FlyByWire_B.in_flight;
  }

  rtb_Limiterxi1 -= FlyByWire_DWork.Delay_DSTATE_e;
  rtb_Product4 = FlyByWire_P.RateLimiterVariableTs_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_e += rtb_Limiterxi1;
  if (FlyByWire_DWork.is_active_c7_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c7_FlyByWire = 1U;
    FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
    rtb_ManualSwitch = 0.7;
    rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
    rtb_nz_limit_up_g = 2.0;
    rtb_nz_limit_lo_g = 0;
  } else {
    switch (FlyByWire_DWork.is_c7_FlyByWire) {
     case FlyByWire_IN_flight_clean:
      if (FlyByWire_U.in.data.flaps_handle_index != 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_ManualSwitch = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else if ((FlyByWire_B.in_flight == 0.0) && (FlyByWire_U.in.data.flaps_handle_index == 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_ManualSwitch = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_ManualSwitch = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      }
      break;

     case FlyByWire_IN_flight_flaps:
      if (FlyByWire_U.in.data.flaps_handle_index == 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_ManualSwitch = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      } else if (FlyByWire_B.in_flight == 0.0) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_ground;
        rtb_ManualSwitch = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_ManualSwitch = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      }
      break;

     default:
      if ((FlyByWire_B.in_flight != 0.0) && (FlyByWire_U.in.data.flaps_handle_index == 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_clean;
        rtb_ManualSwitch = 0.3;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.3;
        rtb_nz_limit_up_g = 2.5;
        rtb_nz_limit_lo_g = -1;
      } else if ((FlyByWire_B.in_flight != 0.0) && (FlyByWire_U.in.data.flaps_handle_index != 0.0)) {
        FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_flight_flaps;
        rtb_ManualSwitch = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      } else {
        rtb_ManualSwitch = 0.7;
        rtb_eta_trim_deg_rate_limit_lo_deg_s = -0.7;
        rtb_nz_limit_up_g = 2.0;
        rtb_nz_limit_lo_g = 0;
      }
      break;
    }
  }

  if (FlyByWire_DWork.is_active_c9_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c9_FlyByWire = 1U;
    FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
    rtb_eta_trim_deg_should_freeze = false;
  } else if (FlyByWire_DWork.is_c9_FlyByWire == FlyByWire_IN_frozen) {
    if ((rtb_in_flare == 0) && (FlyByWire_U.in.data.nz_g < 1.25) && (FlyByWire_U.in.data.nz_g > 0.5) && (std::abs
         (rtb_GainPhi) <= 30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_running;
      rtb_eta_trim_deg_should_freeze = false;
    } else {
      rtb_eta_trim_deg_should_freeze = true;
    }
  } else {
    if ((rtb_in_flare != 0) || (FlyByWire_U.in.data.nz_g >= 1.25) || (FlyByWire_U.in.data.nz_g <= 0.5) || (std::abs
         (rtb_GainPhi) > 30.0)) {
      FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_frozen;
      rtb_eta_trim_deg_should_freeze = true;
    } else {
      rtb_eta_trim_deg_should_freeze = false;
    }
  }

  if (FlyByWire_DWork.is_active_c8_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c8_FlyByWire = 1U;
    FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
    rtb_NOT = true;
    rtb_eta_trim_deg_reset_deg = rtb_Product3;
    rtb_eta_trim_deg_should_write = false;
  } else {
    switch (FlyByWire_DWork.is_c8_FlyByWire) {
     case FlyByWire_IN_automatic:
      if (FlyByWire_B.in_flight == 0.0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_reset;
        rtb_NOT = true;
        rtb_eta_trim_deg_reset_deg = 0.0;
        rtb_eta_trim_deg_should_write = true;
      } else if (rtb_NOT) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_tracking;
        rtb_NOT = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3;
        rtb_eta_trim_deg_should_write = false;
      } else {
        rtb_NOT = false;
        rtb_eta_trim_deg_reset_deg = rtb_Product3;
        rtb_eta_trim_deg_should_write = true;
      }
      break;

     case FlyByWire_IN_manual:
      if (FlyByWire_B.in_flight != 0.0) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_automatic;
        rtb_NOT = false;
        rtb_eta_trim_deg_reset_deg = rtb_Product3;
        rtb_eta_trim_deg_should_write = true;
      } else {
        rtb_NOT = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3;
        rtb_eta_trim_deg_should_write = false;
      }
      break;

     case FlyByWire_IN_reset:
      if ((FlyByWire_B.in_flight == 0.0) && (rtb_Product3 == 0.0)) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_manual;
        rtb_NOT = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3;
        rtb_eta_trim_deg_should_write = false;
      } else {
        rtb_NOT = true;
        rtb_eta_trim_deg_reset_deg = 0.0;
        rtb_eta_trim_deg_should_write = true;
      }
      break;

     default:
      if (!rtb_NOT) {
        FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_automatic;
        rtb_NOT = false;
        rtb_eta_trim_deg_reset_deg = rtb_Product3;
        rtb_eta_trim_deg_should_write = true;
      } else {
        rtb_NOT = true;
        rtb_eta_trim_deg_reset_deg = rtb_Product3;
        rtb_eta_trim_deg_should_write = false;
      }
      break;
    }
  }

  rtb_Limiterxi1 = FlyByWire_B.flare_Theta_c_deg - FlyByWire_DWork.Delay_DSTATE_i;
  rtb_Product4 = std::abs(FlyByWire_B.flare_Theta_c_rate_deg_s) * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_B.flare_Theta_c_rate_deg_s;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_i += rtb_Limiterxi1;
  rtb_BusAssignment_c_pitch_data_computed_delta_eta_deg = FlyByWire_P.Gain_Gain_d * Theta;
  rtb_Limitereta = FlyByWire_P.Gain1_Gain_j * result[1] * look1_binlxpw(FlyByWire_U.in.data.V_tas_kn,
    FlyByWire_P.uDLookupTable_bp01Data, FlyByWire_P.uDLookupTable_tableData, 4U) + FlyByWire_U.in.data.nz_g;
  rtb_Limiterxi1 = FlyByWire_U.in.data.autopilot_custom_Theta_c_deg - FlyByWire_DWork.Delay_DSTATE_g;
  rtb_Product4 = FlyByWire_P.RateLimiterVariableTs1_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs1_lo;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_g += rtb_Limiterxi1;
  rtb_Limiterxi1 = Theta - FlyByWire_DWork.Delay_DSTATE_k;
  rtb_Product4 = FlyByWire_P.RateLimiterVariableTs_up_f * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_f;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_k += rtb_Limiterxi1;
  if (FlyByWire_U.in.data.autopilot_custom_on > FlyByWire_P.Switch1_Threshold) {
    rtb_Switch_c = (FlyByWire_DWork.Delay_DSTATE_g - rtb_GainTheta) * FlyByWire_P.Gain4_Gain;
  } else {
    rtb_Gain1_b = FlyByWire_P.Gain1_Gain * rtb_GainTheta;
    if (rtb_in_flare > FlyByWire_P.Switch_Threshold) {
      rtb_Switch_c = (FlyByWire_DWork.Delay_DSTATE_i - rtb_GainTheta) * FlyByWire_P.Gain_Gain;
      if (rtb_Switch_c > FlyByWire_P.Saturation_UpperSat) {
        rtb_Switch_c = FlyByWire_P.Saturation_UpperSat;
      } else {
        if (rtb_Switch_c < FlyByWire_P.Saturation_LowerSat) {
          rtb_Switch_c = FlyByWire_P.Saturation_LowerSat;
        }
      }
    } else {
      rtb_Switch_c = FlyByWire_P.Constant_Value_m;
    }

    rtb_Product3 = FlyByWire_P.Gain2_Gain * FlyByWire_P.Theta_max1_Value - rtb_Gain1_b;
    if (rtb_Product3 > FlyByWire_P.Saturation1_UpperSat) {
      rtb_Product3 = FlyByWire_P.Saturation1_UpperSat;
    } else {
      if (rtb_Product3 < FlyByWire_P.Saturation1_LowerSat) {
        rtb_Product3 = FlyByWire_P.Saturation1_LowerSat;
      }
    }

    if (FlyByWire_DWork.Delay_DSTATE_k <= rtb_Product3) {
      rtb_Product3 = FlyByWire_P.Gain3_Gain * FlyByWire_P.Theta_max3_Value - rtb_Gain1_b;
      if (rtb_Product3 > FlyByWire_P.Saturation2_UpperSat) {
        rtb_Product3 = FlyByWire_P.Saturation2_UpperSat;
      } else {
        if (rtb_Product3 < FlyByWire_P.Saturation2_LowerSat) {
          rtb_Product3 = FlyByWire_P.Saturation2_LowerSat;
        }
      }

      if (FlyByWire_DWork.Delay_DSTATE_k >= rtb_Product3) {
        rtb_Product3 = FlyByWire_DWork.Delay_DSTATE_k;
      }
    }

    rtb_Product3 = look1_binlxpw(rtb_Product3, FlyByWire_P.Loaddemand_bp01Data, FlyByWire_P.Loaddemand_tableData, 2U);
    rtb_Switch_c += rtb_Product3;
  }

  if (rtb_GainPhi > FlyByWire_P.Saturation_UpperSat_d) {
    rtb_Gain1_b = FlyByWire_P.Saturation_UpperSat_d;
  } else if (rtb_GainPhi < FlyByWire_P.Saturation_LowerSat_p) {
    rtb_Gain1_b = FlyByWire_P.Saturation_LowerSat_p;
  } else {
    rtb_Gain1_b = rtb_GainPhi;
  }

  rtb_Switch_c += std::cos(FlyByWire_P.Gain1_Gain_p * rtb_GainTheta) / std::cos(FlyByWire_P.Gain1_Gain_b * rtb_Gain1_b);
  if (rtb_Switch_c > rtb_nz_limit_up_g) {
    rtb_Switch_c = rtb_nz_limit_up_g;
  } else {
    if (rtb_Switch_c < rtb_nz_limit_lo_g) {
      rtb_Switch_c = rtb_nz_limit_lo_g;
    }
  }

  rtb_Limiterxi = rtb_Limitereta - rtb_Switch_c;
  rtb_Gain1_b = rtb_Limiterxi * look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.DLUT_bp01Data,
    FlyByWire_P.DLUT_tableData, 1U) * FlyByWire_P.DiscreteDerivativeVariableTs_Gain;
  rtb_Limiterxi = (rtb_Gain1_b - FlyByWire_DWork.Delay_DSTATE_ev) / FlyByWire_U.in.time.dt + rtb_Limiterxi *
    look1_binlxpw(FlyByWire_U.in.data.V_tas_kn, FlyByWire_P.PLUT_bp01Data, FlyByWire_P.PLUT_tableData, 1U);
  if (rtb_Limiterxi > FlyByWire_P.Saturation_UpperSat_j) {
    rtb_Limiterxi = FlyByWire_P.Saturation_UpperSat_j;
  } else {
    if (rtb_Limiterxi < FlyByWire_P.Saturation_LowerSat_c) {
      rtb_Limiterxi = FlyByWire_P.Saturation_LowerSat_c;
    }
  }

  FlyByWire_Y.out.pitch.law_normal.nz_c_g = rtb_Switch_c;
  FlyByWire_Y.out.pitch.law_normal.Cstar_g = rtb_Limitereta;
  rtb_Limitereta = FlyByWire_P.DiscreteTimeIntegratorVariableTs_Gain * rtb_Limiterxi * FlyByWire_U.in.time.dt;
  if ((FlyByWire_DWork.Delay_DSTATE_e == 0.0) || (rtb_BusAssignment_a_sim_data_computed_tracking_mode_on != 0)) {
    FlyByWire_DWork.icLoad = 1U;
  }

  if (FlyByWire_DWork.icLoad != 0) {
    if (FlyByWire_B.in_flight > FlyByWire_P.Switch_Threshold_d) {
      rtb_Limiterxi1 = rtb_Gainpk4;
    } else {
      rtb_Limiterxi1 = rtb_BusAssignment_c_pitch_data_computed_delta_eta_deg;
    }

    FlyByWire_DWork.Delay_DSTATE_f = rtb_Limiterxi1 - rtb_Limitereta;
  }

  rtb_Limitereta += FlyByWire_DWork.Delay_DSTATE_f;
  if (rtb_Limitereta > FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit) {
    FlyByWire_DWork.Delay_DSTATE_f = FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit;
  } else if (rtb_Limitereta < FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit) {
    FlyByWire_DWork.Delay_DSTATE_f = FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit;
  } else {
    FlyByWire_DWork.Delay_DSTATE_f = rtb_Limitereta;
  }

  if (FlyByWire_DWork.Delay_DSTATE_e > FlyByWire_P.Saturation_UpperSat_n) {
    rtb_Limitereta = FlyByWire_P.Saturation_UpperSat_n;
  } else if (FlyByWire_DWork.Delay_DSTATE_e < FlyByWire_P.Saturation_LowerSat_b) {
    rtb_Limitereta = FlyByWire_P.Saturation_LowerSat_b;
  } else {
    rtb_Limitereta = FlyByWire_DWork.Delay_DSTATE_e;
  }

  rtb_Switch_c = FlyByWire_DWork.Delay_DSTATE_f * rtb_Limitereta;
  rtb_Limitereta = FlyByWire_P.Constant_Value_n - rtb_Limitereta;
  rtb_Limitereta *= rtb_BusAssignment_c_pitch_data_computed_delta_eta_deg;
  rtb_BusAssignment_m_pitch_output_eta_deg = rtb_Switch_c + rtb_Limitereta;
  if (rtb_eta_trim_deg_should_freeze) {
    rtb_Limitereta = FlyByWire_P.Constant_Value;
  } else {
    rtb_Limitereta = FlyByWire_DWork.Delay_DSTATE_f;
  }

  rtb_Switch_c = FlyByWire_P.Gain_Gain_ip * rtb_Limitereta * FlyByWire_P.DiscreteTimeIntegratorVariableTs_Gain_g *
    FlyByWire_U.in.time.dt;
  if (rtb_NOT) {
    FlyByWire_DWork.icLoad_b = 1U;
  }

  if (FlyByWire_DWork.icLoad_b != 0) {
    FlyByWire_DWork.Delay_DSTATE_m = rtb_eta_trim_deg_reset_deg - rtb_Switch_c;
  }

  rtb_Switch_c += FlyByWire_DWork.Delay_DSTATE_m;
  if (rtb_Switch_c > FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_j) {
    FlyByWire_DWork.Delay_DSTATE_m = FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_j;
  } else if (rtb_Switch_c < FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_g) {
    FlyByWire_DWork.Delay_DSTATE_m = FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_g;
  } else {
    FlyByWire_DWork.Delay_DSTATE_m = rtb_Switch_c;
  }

  rtb_Limiterxi1 = FlyByWire_DWork.Delay_DSTATE_m - FlyByWire_DWork.Delay_DSTATE_c;
  rtb_Product4 = rtb_ManualSwitch * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * rtb_eta_trim_deg_rate_limit_lo_deg_s;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_c += rtb_Limiterxi1;
  FlyByWire_Y.out.pitch.law_normal.eta_dot_deg_s = rtb_Limiterxi;
  FlyByWire_Y.out.pitch.vote.eta_dot_deg_s = rtb_Limiterxi;
  if (FlyByWire_DWork.is_active_c5_FlyByWire == 0U) {
    FlyByWire_DWork.is_active_c5_FlyByWire = 1U;
    FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
    rtb_in_flight = 0;
  } else if (FlyByWire_DWork.is_c5_FlyByWire == FlyByWire_IN_FlightMode) {
    if (rtb_on_ground == 1) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_GroundMode;
      rtb_in_flight = 0;
    } else {
      rtb_in_flight = 1;
    }
  } else {
    if (((rtb_on_ground == 0) && (rtb_GainTheta > 8.0)) || (FlyByWire_U.in.data.H_radio_ft > 400.0)) {
      FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_FlightMode;
      rtb_in_flight = 1;
    } else {
      rtb_in_flight = 0;
    }
  }

  if (rtb_in_flight > FlyByWire_P.Saturation_UpperSat_p) {
    rtb_Limitereta = FlyByWire_P.Saturation_UpperSat_p;
  } else if (rtb_in_flight < FlyByWire_P.Saturation_LowerSat_h) {
    rtb_Limitereta = FlyByWire_P.Saturation_LowerSat_h;
  } else {
    rtb_Limitereta = rtb_in_flight;
  }

  rtb_Limiterxi1 = rtb_Limitereta - FlyByWire_DWork.Delay_DSTATE_bh;
  rtb_Product4 = FlyByWire_P.RateLimiterVariableTs_up_k * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_fs;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_bh += rtb_Limiterxi1;
  rtb_BusAssignment_p_roll_data_computed_delta_xi_deg = FlyByWire_P.Gain_Gain_c * Phi;
  rtb_BusAssignment_p_roll_data_computed_delta_zeta_deg = FlyByWire_P.Gain1_Gain_jh *
    rtb_BusAssignment_a_sim_input_delta_zeta_pos;
  rtb_Limiterxi1 = FlyByWire_P.Gain1_Gain_m * Phi + look1_binlxpw(rtb_GainPhi, FlyByWire_P.BankAngleProtection_bp01Data,
    FlyByWire_P.BankAngleProtection_tableData, 6U);
  if (rtb_Limiterxi1 > FlyByWire_P.Saturation_UpperSat_nq) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_UpperSat_nq;
  } else {
    if (rtb_Limiterxi1 < FlyByWire_P.Saturation_LowerSat_o) {
      rtb_Limiterxi1 = FlyByWire_P.Saturation_LowerSat_o;
    }
  }

  rtb_Limitereta = rtb_Limiterxi1 * FlyByWire_DWork.Delay_DSTATE_bh;
  rtb_Limiterxi1 = FlyByWire_U.in.data.autopilot_custom_Phi_c_deg - FlyByWire_DWork.Delay_DSTATE_h;
  rtb_Product4 = FlyByWire_P.RateLimiterVariableTs1_up_k * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs1_lo_j;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_h += rtb_Limiterxi1;
  rtb_Switch_c = FlyByWire_P.DiscreteTimeIntegratorVariableTs_Gain_d * rtb_Limitereta * FlyByWire_U.in.time.dt;
  if ((FlyByWire_DWork.Delay_DSTATE_bh == 0.0) || (rtb_BusAssignment_a_sim_data_computed_tracking_mode_on != 0) ||
      (FlyByWire_U.in.data.autopilot_custom_on != 0.0)) {
    FlyByWire_DWork.icLoad_m = 1U;
  }

  if (FlyByWire_DWork.icLoad_m != 0) {
    FlyByWire_DWork.Delay_DSTATE_ho = rtb_GainPhi - rtb_Switch_c;
  }

  rtb_Switch_c += FlyByWire_DWork.Delay_DSTATE_ho;
  if (rtb_Switch_c > FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_d) {
    FlyByWire_DWork.Delay_DSTATE_ho = FlyByWire_P.DiscreteTimeIntegratorVariableTs_UpperLimit_d;
  } else if (rtb_Switch_c < FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_k) {
    FlyByWire_DWork.Delay_DSTATE_ho = FlyByWire_P.DiscreteTimeIntegratorVariableTs_LowerLimit_k;
  } else {
    FlyByWire_DWork.Delay_DSTATE_ho = rtb_Switch_c;
  }

  if (FlyByWire_U.in.data.autopilot_custom_on > FlyByWire_P.Switch_Threshold_p) {
    rtb_Limiterxi1 = FlyByWire_DWork.Delay_DSTATE_h;
  } else {
    rtb_Limiterxi1 = FlyByWire_DWork.Delay_DSTATE_ho;
  }

  rtb_Limiterxi1 -= FlyByWire_DWork.Delay_DSTATE_fd;
  rtb_Product4 = FlyByWire_P.RateLimiterVariableTs2_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs2_lo;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_fd += rtb_Limiterxi1;
  rtb_Switch_c = (FlyByWire_DWork.Delay_DSTATE_fd - rtb_GainPhi) * FlyByWire_P.Gain2_Gain_i + FlyByWire_P.Gain1_Gain_mg *
    result[0] * FlyByWire_P.pKp_Gain;
  rtb_Limiterxi1 = rtb_BusAssignment_a_sim_input_delta_zeta_pos - FlyByWire_DWork.Delay_DSTATE_mv;
  rtb_Product4 = FlyByWire_P.RateLimiterVariableTs_up_m * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterVariableTs_lo_p;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_mv += rtb_Limiterxi1;
  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.LagFilter_C1_d;
  rtb_Limiterxi = rtb_Limiterxi1 + FlyByWire_P.Constant_Value_p;
  FlyByWire_DWork.Delay1_DSTATE_m = 1.0 / rtb_Limiterxi * (FlyByWire_P.Constant_Value_p - rtb_Limiterxi1) *
    FlyByWire_DWork.Delay1_DSTATE_m + (rtb_Gain + FlyByWire_DWork.Delay_DSTATE_ej) * (rtb_Limiterxi1 / rtb_Limiterxi);
  rtb_Limiterxi = FlyByWire_U.in.time.dt * FlyByWire_P.WashoutFilter_C1;
  rtb_Limiterxi1 = rtb_Limiterxi + FlyByWire_P.Constant_Value_o;
  rtb_Product3 = FlyByWire_P.Constant_Value_o / rtb_Limiterxi1;
  rtb_Product4 = FlyByWire_DWork.Delay_DSTATE_d * rtb_Product3;
  rtb_Product3 *= rtb_Gain;
  FlyByWire_DWork.Delay1_DSTATE_f = 1.0 / rtb_Limiterxi1 * (FlyByWire_P.Constant_Value_o - rtb_Limiterxi) *
    FlyByWire_DWork.Delay1_DSTATE_f + (rtb_Product3 - rtb_Product4);
  if (FlyByWire_U.in.data.V_tas_kn > FlyByWire_P.Saturation_UpperSat_l) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_UpperSat_l;
  } else if (FlyByWire_U.in.data.V_tas_kn < FlyByWire_P.Saturation_LowerSat_l) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation_LowerSat_l;
  } else {
    rtb_Limiterxi1 = FlyByWire_U.in.data.V_tas_kn;
  }

  rtb_Limiterxi1 = (FlyByWire_DWork.Delay1_DSTATE_m - std::sin(FlyByWire_P.Gain1_Gain_br *
    FlyByWire_DWork.Delay_DSTATE_fd) * FlyByWire_P.Constant2_Value * std::cos(FlyByWire_P.Gain1_Gain_c * rtb_GainTheta) /
                    (FlyByWire_P.Gain6_Gain * rtb_Limiterxi1) * FlyByWire_P.Gain_Gain_cd) * FlyByWire_P.Gain_Gain_h;
  rtb_Product3 = FlyByWire_P.Gain6_Gain_k * FlyByWire_DWork.Delay1_DSTATE_f;
  if (rtb_Limiterxi1 > FlyByWire_P.Saturation1_UpperSat_h) {
    rtb_Limiterxi1 = FlyByWire_P.Saturation1_UpperSat_h;
  } else {
    if (rtb_Limiterxi1 < FlyByWire_P.Saturation1_LowerSat_g) {
      rtb_Limiterxi1 = FlyByWire_P.Saturation1_LowerSat_g;
    }
  }

  if (rtb_Product3 > FlyByWire_P.Saturation2_UpperSat_e) {
    rtb_Product3 = FlyByWire_P.Saturation2_UpperSat_e;
  } else {
    if (rtb_Product3 < FlyByWire_P.Saturation2_LowerSat_gp) {
      rtb_Product3 = FlyByWire_P.Saturation2_LowerSat_gp;
    }
  }

  rtb_Limiterxi = (FlyByWire_P.Gain5_Gain * FlyByWire_DWork.Delay_DSTATE_mv + rtb_Limiterxi1) *
    FlyByWire_DWork.Delay_DSTATE_bh + rtb_Product3;
  FlyByWire_Y.out.roll.law_normal.pk_c_deg_s = rtb_Limitereta;
  FlyByWire_Y.out.roll.law_normal.xi_deg = rtb_Switch_c;
  FlyByWire_Y.out.roll.law_normal.zeta_deg = rtb_Limiterxi;
  if (FlyByWire_DWork.Delay_DSTATE_bh > FlyByWire_P.Saturation_UpperSat_g) {
    rtb_Limitereta = FlyByWire_P.Saturation_UpperSat_g;
  } else if (FlyByWire_DWork.Delay_DSTATE_bh < FlyByWire_P.Saturation_LowerSat_j) {
    rtb_Limitereta = FlyByWire_P.Saturation_LowerSat_j;
  } else {
    rtb_Limitereta = FlyByWire_DWork.Delay_DSTATE_bh;
  }

  rtb_Switch_c *= rtb_Limitereta;
  rtb_Limitereta = FlyByWire_P.Constant_Value_b - rtb_Limitereta;
  rtb_Limitereta *= rtb_BusAssignment_p_roll_data_computed_delta_xi_deg;
  rtb_Switch_c += rtb_Limitereta;
  if (FlyByWire_DWork.Delay_DSTATE_bh > FlyByWire_P.Saturation_UpperSat_jo) {
    rtb_Limitereta = FlyByWire_P.Saturation_UpperSat_jo;
  } else if (FlyByWire_DWork.Delay_DSTATE_bh < FlyByWire_P.Saturation_LowerSat_jm) {
    rtb_Limitereta = FlyByWire_P.Saturation_LowerSat_jm;
  } else {
    rtb_Limitereta = FlyByWire_DWork.Delay_DSTATE_bh;
  }

  rtb_Limiterxi *= rtb_Limitereta;
  rtb_Limitereta = FlyByWire_P.Constant_Value_mg - rtb_Limitereta;
  rtb_Limitereta *= rtb_BusAssignment_p_roll_data_computed_delta_zeta_deg;
  rtb_Limiterxi += rtb_Limitereta;
  rtb_Limiterxi1 = rtb_BusAssignment_m_pitch_output_eta_deg - FlyByWire_DWork.Delay_DSTATE_id;
  rtb_Product4 = FlyByWire_P.RateLimitereta_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimitereta_lo;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_id += rtb_Limiterxi1;
  rtb_Limiterxi1 = rtb_Switch_c - FlyByWire_DWork.Delay_DSTATE_a;
  rtb_Product4 = FlyByWire_P.RateLimiterxi_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterxi_lo;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_a += rtb_Limiterxi1;
  rtb_Limiterxi1 = rtb_Limiterxi - FlyByWire_DWork.Delay_DSTATE_o;
  rtb_Product4 = FlyByWire_P.RateLimiterzeta_up * FlyByWire_U.in.time.dt;
  if (rtb_Limiterxi1 < rtb_Product4) {
    rtb_Product4 = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_U.in.time.dt * FlyByWire_P.RateLimiterzeta_lo;
  if (rtb_Product4 > rtb_Limiterxi1) {
    rtb_Limiterxi1 = rtb_Product4;
  }

  FlyByWire_DWork.Delay_DSTATE_o += rtb_Limiterxi1;
  FlyByWire_Y.out.sim.time.dt = FlyByWire_U.in.time.dt;
  FlyByWire_Y.out.sim.time.simulation_time = FlyByWire_U.in.time.simulation_time;
  FlyByWire_Y.out.sim.time.monotonic_time = FlyByWire_DWork.Delay_DSTATE;
  FlyByWire_Y.out.sim.data.nz_g = FlyByWire_U.in.data.nz_g;
  FlyByWire_Y.out.sim.data.Theta_deg = rtb_GainTheta;
  FlyByWire_Y.out.sim.data.Phi_deg = rtb_GainPhi;
  FlyByWire_Y.out.sim.data.q_deg_s = rtb_Gainqk;
  FlyByWire_Y.out.sim.data.r_deg_s = rtb_Gain;
  FlyByWire_Y.out.sim.data.p_deg_s = rtb_Gainpk;
  FlyByWire_Y.out.sim.data.qk_deg_s = result[1];
  FlyByWire_Y.out.sim.data.rk_deg_s = result[2];
  FlyByWire_Y.out.sim.data.pk_deg_s = result[0];
  FlyByWire_Y.out.sim.data.qk_dot_deg_s2 = result_0[1];
  FlyByWire_Y.out.sim.data.rk_dot_deg_s2 = result_0[2];
  FlyByWire_Y.out.sim.data.pk_dot_deg_s2 = result_0[0];
  FlyByWire_Y.out.sim.data.psi_magnetic_deg = FlyByWire_U.in.data.psi_magnetic_deg;
  FlyByWire_Y.out.sim.data.psi_true_deg = FlyByWire_U.in.data.psi_true_deg;
  FlyByWire_Y.out.sim.data.eta_deg = rtb_Gainpk4;
  FlyByWire_Y.out.sim.data.xi_deg = FlyByWire_P.Gainpk5_Gain * FlyByWire_U.in.data.xi_pos;
  FlyByWire_Y.out.sim.data.zeta_deg = FlyByWire_P.Gainpk6_Gain * FlyByWire_U.in.data.zeta_pos;
  FlyByWire_Y.out.sim.data.zeta_trim_deg = result_tmp;
  FlyByWire_Y.out.sim.data.alpha_deg = FlyByWire_U.in.data.alpha_deg;
  FlyByWire_Y.out.sim.data.beta_deg = FlyByWire_U.in.data.beta_deg;
  FlyByWire_Y.out.sim.data.beta_dot_deg_s = FlyByWire_U.in.data.beta_dot_deg_s;
  FlyByWire_Y.out.sim.data.V_ias_kn = FlyByWire_U.in.data.V_ias_kn;
  FlyByWire_Y.out.sim.data.V_tas_kn = FlyByWire_U.in.data.V_tas_kn;
  FlyByWire_Y.out.sim.data.V_mach = FlyByWire_U.in.data.V_mach;
  FlyByWire_Y.out.sim.data.H_ft = FlyByWire_U.in.data.H_ft;
  FlyByWire_Y.out.sim.data.H_ind_ft = FlyByWire_U.in.data.H_ind_ft;
  FlyByWire_Y.out.sim.data.H_radio_ft = FlyByWire_U.in.data.H_radio_ft;
  FlyByWire_Y.out.sim.data.CG_percent_MAC = FlyByWire_U.in.data.CG_percent_MAC;
  FlyByWire_Y.out.sim.data.total_weight_kg = FlyByWire_U.in.data.total_weight_kg;
  rtb_Limiterxi1 = FlyByWire_P.Gain_Gain_i * FlyByWire_U.in.data.gear_animation_pos_0 - FlyByWire_P.Constant_Value_g;
  if (rtb_Limiterxi1 > FlyByWire_P.Saturation_UpperSat_e) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_UpperSat_e;
  } else if (rtb_Limiterxi1 < FlyByWire_P.Saturation_LowerSat_e) {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = FlyByWire_P.Saturation_LowerSat_e;
  } else {
    FlyByWire_Y.out.sim.data.gear_strut_compression_0 = rtb_Limiterxi1;
  }

  FlyByWire_Y.out.sim.data.flaps_handle_index = FlyByWire_U.in.data.flaps_handle_index;
  FlyByWire_Y.out.sim.data.spoilers_left_pos = FlyByWire_U.in.data.spoilers_left_pos;
  FlyByWire_Y.out.sim.data.spoilers_right_pos = FlyByWire_U.in.data.spoilers_right_pos;
  FlyByWire_Y.out.sim.data.autopilot_master_on = FlyByWire_U.in.data.autopilot_master_on;
  FlyByWire_Y.out.sim.data.slew_on = FlyByWire_U.in.data.slew_on;
  FlyByWire_Y.out.sim.data.pause_on = FlyByWire_U.in.data.pause_on;
  FlyByWire_Y.out.sim.data.tracking_mode_on_override = FlyByWire_U.in.data.tracking_mode_on_override;
  FlyByWire_Y.out.sim.data.autopilot_custom_on = FlyByWire_U.in.data.autopilot_custom_on;
  FlyByWire_Y.out.sim.data.autopilot_custom_Theta_c_deg = FlyByWire_U.in.data.autopilot_custom_Theta_c_deg;
  FlyByWire_Y.out.sim.data.autopilot_custom_Phi_c_deg = FlyByWire_U.in.data.autopilot_custom_Phi_c_deg;
  FlyByWire_Y.out.sim.data.simulation_rate = FlyByWire_U.in.data.simulation_rate;
  FlyByWire_Y.out.sim.data.ice_structure_percent = FlyByWire_U.in.data.ice_structure_percent;
  FlyByWire_Y.out.sim.data.linear_cl_alpha_per_deg = FlyByWire_U.in.data.linear_cl_alpha_per_deg;
  FlyByWire_Y.out.sim.data.alpha_stall_deg = FlyByWire_U.in.data.alpha_stall_deg;
  FlyByWire_Y.out.sim.data.alpha_zero_lift_deg = FlyByWire_U.in.data.alpha_zero_lift_deg;
  FlyByWire_Y.out.sim.data.ambient_density_kg_per_m3 = FlyByWire_U.in.data.ambient_density_kg_per_m3;
  FlyByWire_Y.out.sim.data.ambient_pressure_mbar = FlyByWire_U.in.data.ambient_pressure_mbar;
  FlyByWire_Y.out.sim.data.ambient_temperature_celsius = FlyByWire_U.in.data.ambient_temperature_celsius;
  FlyByWire_Y.out.sim.data.ambient_wind_x_kn = FlyByWire_U.in.data.ambient_wind_x_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_y_kn = FlyByWire_U.in.data.ambient_wind_y_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_z_kn = FlyByWire_U.in.data.ambient_wind_z_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_velocity_kn = FlyByWire_U.in.data.ambient_wind_velocity_kn;
  FlyByWire_Y.out.sim.data.ambient_wind_direction_deg = FlyByWire_U.in.data.ambient_wind_direction_deg;
  FlyByWire_Y.out.sim.data.total_air_temperature_celsius = FlyByWire_U.in.data.total_air_temperature_celsius;
  FlyByWire_Y.out.sim.data.latitude_deg = FlyByWire_U.in.data.latitude_deg;
  FlyByWire_Y.out.sim.data.longitude_deg = FlyByWire_U.in.data.longitude_deg;
  FlyByWire_Y.out.sim.data_computed.on_ground = rtb_on_ground;
  FlyByWire_Y.out.sim.data_computed.tracking_mode_on = rtb_BusAssignment_a_sim_data_computed_tracking_mode_on;
  FlyByWire_Y.out.sim.input.delta_eta_pos = Theta;
  FlyByWire_Y.out.sim.input.delta_xi_pos = Phi;
  FlyByWire_Y.out.sim.input.delta_zeta_pos = rtb_BusAssignment_a_sim_input_delta_zeta_pos;
  FlyByWire_Y.out.pitch.data_computed.delta_eta_deg = rtb_BusAssignment_c_pitch_data_computed_delta_eta_deg;
  FlyByWire_Y.out.pitch.data_computed.in_flight = FlyByWire_B.in_flight;
  FlyByWire_Y.out.pitch.data_computed.in_flare = rtb_in_flare;
  FlyByWire_Y.out.pitch.data_computed.in_flight_gain = FlyByWire_DWork.Delay_DSTATE_e;
  FlyByWire_Y.out.pitch.data_computed.nz_limit_up_g = rtb_nz_limit_up_g;
  FlyByWire_Y.out.pitch.data_computed.nz_limit_lo_g = rtb_nz_limit_lo_g;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_should_freeze = rtb_eta_trim_deg_should_freeze;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_reset = rtb_NOT;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_reset_deg = rtb_eta_trim_deg_reset_deg;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_should_write = rtb_eta_trim_deg_should_write;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_rate_limit_up_deg_s = rtb_ManualSwitch;
  FlyByWire_Y.out.pitch.data_computed.eta_trim_deg_rate_limit_lo_deg_s = rtb_eta_trim_deg_rate_limit_lo_deg_s;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_deg = FlyByWire_DWork.Delay1_DSTATE;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_c_deg = FlyByWire_DWork.Delay_DSTATE_i;
  FlyByWire_Y.out.pitch.data_computed.flare_Theta_c_rate_deg_s = FlyByWire_B.flare_Theta_c_rate_deg_s;
  FlyByWire_Y.out.pitch.integrated.eta_deg = FlyByWire_DWork.Delay_DSTATE_f;
  FlyByWire_Y.out.pitch.output.eta_deg = rtb_BusAssignment_m_pitch_output_eta_deg;
  FlyByWire_Y.out.pitch.output.eta_trim_deg = FlyByWire_DWork.Delay_DSTATE_c;
  FlyByWire_Y.out.roll.data_computed.delta_xi_deg = rtb_BusAssignment_p_roll_data_computed_delta_xi_deg;
  FlyByWire_Y.out.roll.data_computed.delta_zeta_deg = rtb_BusAssignment_p_roll_data_computed_delta_zeta_deg;
  FlyByWire_Y.out.roll.data_computed.in_flight = rtb_in_flight;
  FlyByWire_Y.out.roll.data_computed.in_flight_gain = FlyByWire_DWork.Delay_DSTATE_bh;
  FlyByWire_Y.out.roll.law_normal.Phi_c_deg = FlyByWire_DWork.Delay_DSTATE_fd;
  FlyByWire_Y.out.roll.output.xi_deg = rtb_Switch_c;
  FlyByWire_Y.out.roll.output.zeta_deg = rtb_Limiterxi;
  rtb_Limiterxi1 = FlyByWire_P.Gaineta_Gain_d * FlyByWire_DWork.Delay_DSTATE_id;
  if (rtb_Limiterxi1 > FlyByWire_P.Limitereta_UpperSat) {
    FlyByWire_Y.out.output.eta_pos = FlyByWire_P.Limitereta_UpperSat;
  } else if (rtb_Limiterxi1 < FlyByWire_P.Limitereta_LowerSat) {
    FlyByWire_Y.out.output.eta_pos = FlyByWire_P.Limitereta_LowerSat;
  } else {
    FlyByWire_Y.out.output.eta_pos = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_P.GainiH_Gain * FlyByWire_DWork.Delay_DSTATE_c;
  if (rtb_Limiterxi1 > FlyByWire_P.LimiteriH_UpperSat) {
    FlyByWire_Y.out.output.eta_trim_deg = FlyByWire_P.LimiteriH_UpperSat;
  } else if (rtb_Limiterxi1 < FlyByWire_P.LimiteriH_LowerSat) {
    FlyByWire_Y.out.output.eta_trim_deg = FlyByWire_P.LimiteriH_LowerSat;
  } else {
    FlyByWire_Y.out.output.eta_trim_deg = rtb_Limiterxi1;
  }

  FlyByWire_Y.out.output.eta_trim_deg_should_write = rtb_eta_trim_deg_should_write;
  rtb_Limiterxi1 = FlyByWire_P.Gainxi_Gain_n * FlyByWire_DWork.Delay_DSTATE_a;
  if (rtb_Limiterxi1 > FlyByWire_P.Limiterxi_UpperSat) {
    FlyByWire_Y.out.output.xi_pos = FlyByWire_P.Limiterxi_UpperSat;
  } else if (rtb_Limiterxi1 < FlyByWire_P.Limiterxi_LowerSat) {
    FlyByWire_Y.out.output.xi_pos = FlyByWire_P.Limiterxi_LowerSat;
  } else {
    FlyByWire_Y.out.output.xi_pos = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_P.Gainxi1_Gain_e * FlyByWire_DWork.Delay_DSTATE_o;
  if (rtb_Limiterxi1 > FlyByWire_P.Limiterxi1_UpperSat) {
    FlyByWire_Y.out.output.zeta_pos = FlyByWire_P.Limiterxi1_UpperSat;
  } else if (rtb_Limiterxi1 < FlyByWire_P.Limiterxi1_LowerSat) {
    FlyByWire_Y.out.output.zeta_pos = FlyByWire_P.Limiterxi1_LowerSat;
  } else {
    FlyByWire_Y.out.output.zeta_pos = rtb_Limiterxi1;
  }

  rtb_Limiterxi1 = FlyByWire_P.Gainxi2_Gain * result_tmp;
  if (rtb_Limiterxi1 > FlyByWire_P.Limiterxi2_UpperSat) {
    FlyByWire_Y.out.output.zeta_trim_pos = FlyByWire_P.Limiterxi2_UpperSat;
  } else if (rtb_Limiterxi1 < FlyByWire_P.Limiterxi2_LowerSat) {
    FlyByWire_Y.out.output.zeta_trim_pos = FlyByWire_P.Limiterxi2_LowerSat;
  } else {
    FlyByWire_Y.out.output.zeta_trim_pos = rtb_Limiterxi1;
  }

  FlyByWire_DWork.Delay_DSTATE_b = rtb_GainTheta;
  FlyByWire_DWork.Delay_DSTATE_ev = rtb_Gain1_b;
  FlyByWire_DWork.icLoad = 0U;
  FlyByWire_DWork.icLoad_b = 0U;
  FlyByWire_DWork.icLoad_m = 0U;
  FlyByWire_DWork.Delay_DSTATE_ej = rtb_Gain;
  FlyByWire_DWork.Delay_DSTATE_d = rtb_Gain;
}

void FlyByWireModelClass::initialize()
{
  (void) std::memset((static_cast<void *>(&FlyByWire_B)), 0,
                     sizeof(BlockIO_FlyByWire_T));
  (void) std::memset(static_cast<void *>(&FlyByWire_DWork), 0,
                     sizeof(D_Work_FlyByWire_T));
  FlyByWire_U.in = FlyByWire_rtZfbw_input;
  FlyByWire_Y.out = FlyByWire_rtZfbw_output;
  FlyByWire_DWork.Delay_DSTATE = FlyByWire_P.Delay_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_b = FlyByWire_P.Delay_InitialCondition_g;
  FlyByWire_DWork.Delay1_DSTATE = FlyByWire_P.Delay1_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_e = FlyByWire_P.RateLimiterVariableTs_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_i = FlyByWire_P.RateLimiterDynamicVariableTs_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_g = FlyByWire_P.RateLimiterVariableTs1_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_k = FlyByWire_P.RateLimiterVariableTs_InitialCondition_c;
  FlyByWire_DWork.Delay_DSTATE_ev = FlyByWire_P.DiscreteDerivativeVariableTs_InitialCondition;
  FlyByWire_DWork.icLoad = 1U;
  FlyByWire_DWork.Delay_DSTATE_c = FlyByWire_P.RateLimiterDynamicVariableTs_InitialCondition_i;
  FlyByWire_DWork.icLoad_b = 1U;
  FlyByWire_DWork.Delay_DSTATE_bh = FlyByWire_P.RateLimiterVariableTs_InitialCondition_f;
  FlyByWire_DWork.Delay_DSTATE_fd = FlyByWire_P.RateLimiterVariableTs2_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_h = FlyByWire_P.RateLimiterVariableTs1_InitialCondition_c;
  FlyByWire_DWork.icLoad_m = 1U;
  FlyByWire_DWork.Delay_DSTATE_mv = FlyByWire_P.RateLimiterVariableTs_InitialCondition_fc;
  FlyByWire_DWork.Delay_DSTATE_ej = FlyByWire_P.Delay_InitialCondition_gj;
  FlyByWire_DWork.Delay1_DSTATE_m = FlyByWire_P.Delay1_InitialCondition_i;
  FlyByWire_DWork.Delay_DSTATE_d = FlyByWire_P.Delay_InitialCondition_p;
  FlyByWire_DWork.Delay1_DSTATE_f = FlyByWire_P.Delay1_InitialCondition_k;
  FlyByWire_DWork.Delay_DSTATE_id = FlyByWire_P.RateLimitereta_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_a = FlyByWire_P.RateLimiterxi_InitialCondition;
  FlyByWire_DWork.Delay_DSTATE_o = FlyByWire_P.RateLimiterzeta_InitialCondition;
  FlyByWire_DWork.is_active_c1_FlyByWire = 0U;
  FlyByWire_DWork.is_c1_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c3_FlyByWire = 0U;
  FlyByWire_DWork.is_c3_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c2_FlyByWire = 0U;
  FlyByWire_DWork.is_c2_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c7_FlyByWire = 0U;
  FlyByWire_DWork.is_c7_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c9_FlyByWire = 0U;
  FlyByWire_DWork.is_c9_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c8_FlyByWire = 0U;
  FlyByWire_DWork.is_c8_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
  FlyByWire_DWork.is_active_c5_FlyByWire = 0U;
  FlyByWire_DWork.is_c5_FlyByWire = FlyByWire_IN_NO_ACTIVE_CHILD;
}

void FlyByWireModelClass::terminate()
{
}

FlyByWireModelClass::FlyByWireModelClass()
{
}

FlyByWireModelClass::~FlyByWireModelClass()
{
}
