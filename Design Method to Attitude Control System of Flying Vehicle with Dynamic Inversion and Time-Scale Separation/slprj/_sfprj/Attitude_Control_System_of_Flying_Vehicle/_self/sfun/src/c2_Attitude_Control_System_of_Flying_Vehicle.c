/* Include files */

#include "Attitude_Control_System_of_Flying_Vehicle_sfun.h"
#include "c2_Attitude_Control_System_of_Flying_Vehicle.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "Attitude_Control_System_of_Flying_Vehicle_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c2_debug_family_names[13] = { "omega", "A", "angle",
  "nargin", "nargout", "omega_x", "omega_y", "omega_z", "theta", "gamma",
  "dtheta", "dphi", "dgamma" };

/* Function Declarations */
static void initialize_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void initialize_params_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void enable_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void disable_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void c2_update_debugger_state_c2_Attitude_Control_System_of_Flying_Ve
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static const mxArray *get_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void set_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_st);
static void finalize_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void sf_gateway_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void mdl_start_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void initSimStructsc2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber);
static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData);
static real_T c2_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_b_dgamma, const char_T *c2_identifier);
static real_T c2_b_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_c_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3]);
static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static void c2_d_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9]);
static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData);
static int32_T c2_e_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData);
static uint8_T c2_f_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_b_is_active_c2_Attitude_Control_System_of_Flying_Vehicle,
   const char_T *c2_identifier);
static uint8_T c2_g_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static void init_dsm_address_info
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);
static void init_simulink_io_address
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc2_Attitude_Control_System_of_Flying_Vehicle(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c2_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_is_active_c2_Attitude_Control_System_of_Flying_Vehicle = 0U;
}

static void initialize_params_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void enable_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_update_debugger_state_c2_Attitude_Control_System_of_Flying_Ve
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  real_T c2_hoistedGlobal;
  const mxArray *c2_b_y = NULL;
  real_T c2_b_hoistedGlobal;
  const mxArray *c2_c_y = NULL;
  real_T c2_c_hoistedGlobal;
  const mxArray *c2_d_y = NULL;
  uint8_T c2_d_hoistedGlobal;
  const mxArray *c2_e_y = NULL;
  c2_st = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(4, 1), false);
  c2_hoistedGlobal = *chartInstance->c2_dgamma;
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", &c2_hoistedGlobal, 0, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_b_hoistedGlobal = *chartInstance->c2_dphi;
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y", &c2_b_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  c2_c_hoistedGlobal = *chartInstance->c2_dtheta;
  c2_d_y = NULL;
  sf_mex_assign(&c2_d_y, sf_mex_create("y", &c2_c_hoistedGlobal, 0, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c2_y, 2, c2_d_y);
  c2_d_hoistedGlobal =
    chartInstance->c2_is_active_c2_Attitude_Control_System_of_Flying_Vehicle;
  c2_e_y = NULL;
  sf_mex_assign(&c2_e_y, sf_mex_create("y", &c2_d_hoistedGlobal, 3, 0U, 0U, 0U,
    0), false);
  sf_mex_setcell(c2_y, 3, c2_e_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_st)
{
  const mxArray *c2_u;
  chartInstance->c2_doneDoubleBufferReInit = true;
  c2_u = sf_mex_dup(c2_st);
  *chartInstance->c2_dgamma = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("dgamma", c2_u, 0)), "dgamma");
  *chartInstance->c2_dphi = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("dphi", c2_u, 1)), "dphi");
  *chartInstance->c2_dtheta = c2_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell("dtheta", c2_u, 2)), "dtheta");
  chartInstance->c2_is_active_c2_Attitude_Control_System_of_Flying_Vehicle =
    c2_f_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(
    "is_active_c2_Attitude_Control_System_of_Flying_Vehicle", c2_u, 3)),
    "is_active_c2_Attitude_Control_System_of_Flying_Vehicle");
  sf_mex_destroy(&c2_u);
  c2_update_debugger_state_c2_Attitude_Control_System_of_Flying_Ve(chartInstance);
  sf_mex_destroy(&c2_st);
}

static void finalize_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  real_T c2_hoistedGlobal;
  real_T c2_b_hoistedGlobal;
  real_T c2_c_hoistedGlobal;
  real_T c2_d_hoistedGlobal;
  real_T c2_e_hoistedGlobal;
  real_T c2_b_omega_x;
  real_T c2_b_omega_y;
  real_T c2_b_omega_z;
  real_T c2_b_theta;
  real_T c2_b_gamma;
  uint32_T c2_debug_family_var_map[13];
  real_T c2_omega[3];
  real_T c2_A[9];
  real_T c2_angle[3];
  real_T c2_nargin = 5.0;
  real_T c2_nargout = 3.0;
  real_T c2_b_dtheta;
  real_T c2_b_dphi;
  real_T c2_b_dgamma;
  real_T c2_c_omega_x[3];
  int32_T c2_i0;
  int32_T c2_i1;
  real_T c2_x;
  real_T c2_b_x;
  real_T c2_c_x;
  real_T c2_d_x;
  real_T c2_e_x;
  real_T c2_f_x;
  real_T c2_g_x;
  real_T c2_h_x;
  real_T c2_b_A;
  real_T c2_B;
  real_T c2_i_x;
  real_T c2_y;
  real_T c2_j_x;
  real_T c2_b_y;
  real_T c2_c_y;
  real_T c2_k_x;
  real_T c2_l_x;
  real_T c2_m_x;
  real_T c2_n_x;
  real_T c2_c_A;
  real_T c2_b_B;
  real_T c2_o_x;
  real_T c2_d_y;
  real_T c2_p_x;
  real_T c2_e_y;
  real_T c2_f_y;
  real_T c2_q_x;
  real_T c2_r_x;
  real_T c2_s_x;
  real_T c2_t_x;
  real_T c2_u_x;
  real_T c2_v_x;
  real_T c2_w_x;
  real_T c2_x_x;
  int32_T c2_i2;
  int32_T c2_i3;
  real_T c2_a[9];
  int32_T c2_i4;
  real_T c2_b[3];
  int32_T c2_i5;
  int32_T c2_i6;
  int32_T c2_i7;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_gamma, 4U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_theta, 3U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_omega_z, 2U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_omega_y, 1U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_omega_x, 0U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  chartInstance->c2_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  c2_hoistedGlobal = *chartInstance->c2_omega_x;
  c2_b_hoistedGlobal = *chartInstance->c2_omega_y;
  c2_c_hoistedGlobal = *chartInstance->c2_omega_z;
  c2_d_hoistedGlobal = *chartInstance->c2_theta;
  c2_e_hoistedGlobal = *chartInstance->c2_gamma;
  c2_b_omega_x = c2_hoistedGlobal;
  c2_b_omega_y = c2_b_hoistedGlobal;
  c2_b_omega_z = c2_c_hoistedGlobal;
  c2_b_theta = c2_d_hoistedGlobal;
  c2_b_gamma = c2_e_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 13U, 13U, c2_debug_family_names,
    c2_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_omega, 0U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_A, 1U, c2_c_sf_marshallOut,
    c2_c_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c2_angle, 2U, c2_b_sf_marshallOut,
    c2_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargin, 3U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_nargout, 4U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_omega_x, 5U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_omega_y, 6U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_omega_z, 7U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_theta, 8U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c2_b_gamma, 9U, c2_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_dtheta, 10U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_dphi, 11U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c2_b_dgamma, 12U, c2_sf_marshallOut,
    c2_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 4);
  c2_c_omega_x[0] = c2_b_omega_x;
  c2_c_omega_x[1] = c2_b_omega_y;
  c2_c_omega_x[2] = c2_b_omega_z;
  for (c2_i0 = 0; c2_i0 < 3; c2_i0++) {
    c2_omega[c2_i0] = c2_c_omega_x[c2_i0];
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 5);
  for (c2_i1 = 0; c2_i1 < 9; c2_i1++) {
    c2_A[c2_i1] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 6);
  c2_x = c2_b_gamma;
  c2_b_x = c2_x;
  c2_b_x = muDoubleScalarSin(c2_b_x);
  c2_c_x = c2_b_gamma;
  c2_d_x = c2_c_x;
  c2_d_x = muDoubleScalarCos(c2_d_x);
  c2_A[0] = 0.0;
  c2_A[3] = c2_b_x;
  c2_A[6] = c2_d_x;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 7);
  c2_e_x = c2_b_gamma;
  c2_f_x = c2_e_x;
  c2_f_x = muDoubleScalarCos(c2_f_x);
  c2_g_x = c2_b_theta;
  c2_h_x = c2_g_x;
  c2_h_x = muDoubleScalarCos(c2_h_x);
  c2_b_A = c2_f_x;
  c2_B = c2_h_x;
  c2_i_x = c2_b_A;
  c2_y = c2_B;
  c2_j_x = c2_i_x;
  c2_b_y = c2_y;
  c2_c_y = c2_j_x / c2_b_y;
  c2_k_x = c2_b_gamma;
  c2_l_x = c2_k_x;
  c2_l_x = muDoubleScalarSin(c2_l_x);
  c2_m_x = c2_b_theta;
  c2_n_x = c2_m_x;
  c2_n_x = muDoubleScalarCos(c2_n_x);
  c2_c_A = -c2_l_x;
  c2_b_B = c2_n_x;
  c2_o_x = c2_c_A;
  c2_d_y = c2_b_B;
  c2_p_x = c2_o_x;
  c2_e_y = c2_d_y;
  c2_f_y = c2_p_x / c2_e_y;
  c2_A[1] = 0.0;
  c2_A[4] = c2_c_y;
  c2_A[7] = c2_f_y;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 8);
  c2_q_x = c2_b_theta;
  c2_r_x = c2_q_x;
  c2_r_x = muDoubleScalarTan(c2_r_x);
  c2_s_x = c2_b_gamma;
  c2_t_x = c2_s_x;
  c2_t_x = muDoubleScalarCos(c2_t_x);
  c2_u_x = c2_b_theta;
  c2_v_x = c2_u_x;
  c2_v_x = muDoubleScalarTan(c2_v_x);
  c2_w_x = c2_b_gamma;
  c2_x_x = c2_w_x;
  c2_x_x = muDoubleScalarSin(c2_x_x);
  c2_A[2] = 0.0;
  c2_A[5] = -c2_r_x * c2_t_x;
  c2_A[8] = c2_v_x * c2_x_x;
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 10);
  for (c2_i2 = 0; c2_i2 < 9; c2_i2++) {
    c2_a[c2_i2] = c2_A[c2_i2];
  }

  for (c2_i3 = 0; c2_i3 < 3; c2_i3++) {
    c2_b[c2_i3] = c2_omega[c2_i3];
  }

  for (c2_i4 = 0; c2_i4 < 3; c2_i4++) {
    c2_angle[c2_i4] = 0.0;
  }

  for (c2_i5 = 0; c2_i5 < 3; c2_i5++) {
    c2_angle[c2_i5] = 0.0;
    c2_i6 = 0;
    for (c2_i7 = 0; c2_i7 < 3; c2_i7++) {
      c2_angle[c2_i5] += c2_a[c2_i6 + c2_i5] * c2_b[c2_i7];
      c2_i6 += 3;
    }
  }

  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 12);
  c2_b_dtheta = c2_angle[0];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 13);
  c2_b_dphi = c2_angle[1];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, 14);
  c2_b_dgamma = c2_angle[2];
  _SFD_EML_CALL(0U, chartInstance->c2_sfEvent, -14);
  _SFD_SYMBOL_SCOPE_POP();
  *chartInstance->c2_dtheta = c2_b_dtheta;
  *chartInstance->c2_dphi = c2_b_dphi;
  *chartInstance->c2_dgamma = c2_b_dgamma;
  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 1U, chartInstance->c2_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY
    (_Attitude_Control_System_of_Flying_VehicleMachineNumber_,
     chartInstance->chartNumber, chartInstance->instanceNumber);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_dtheta, 5U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_dphi, 6U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c2_dgamma, 7U, 1U, 0U,
                        chartInstance->c2_sfEvent, false);
}

static void mdl_start_c2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc2_Attitude_Control_System_of_Flying_Vehicle
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c2_machineNumber, uint32_T
  c2_chartNumber, uint32_T c2_instanceNumber)
{
  (void)c2_machineNumber;
  (void)c2_chartNumber;
  (void)c2_instanceNumber;
}

static const mxArray *c2_sf_marshallOut(void *chartInstanceVoid, void *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  real_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(real_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static real_T c2_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_b_dgamma, const char_T *c2_identifier)
{
  real_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dgamma),
    &c2_thisId);
  sf_mex_destroy(&c2_b_dgamma);
  return c2_y;
}

static real_T c2_b_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  real_T c2_y;
  real_T c2_d0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_d0, 1, 0, 0U, 0, 0U, 0);
  c2_y = c2_d0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_dgamma;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y;
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_b_dgamma = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_dgamma),
    &c2_thisId);
  sf_mex_destroy(&c2_b_dgamma);
  *(real_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_b_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i8;
  const mxArray *c2_y = NULL;
  real_T c2_u[3];
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  for (c2_i8 = 0; c2_i8 < 3; c2_i8++) {
    c2_u[c2_i8] = (*(real_T (*)[3])c2_inData)[c2_i8];
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_c_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[3])
{
  real_T c2_dv0[3];
  int32_T c2_i9;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv0, 1, 0, 0U, 1, 0U, 1, 3);
  for (c2_i9 = 0; c2_i9 < 3; c2_i9++) {
    c2_y[c2_i9] = c2_dv0[c2_i9];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_angle;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[3];
  int32_T c2_i10;
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_angle = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_angle), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_angle);
  for (c2_i10 = 0; c2_i10 < 3; c2_i10++) {
    (*(real_T (*)[3])c2_outData)[c2_i10] = c2_y[c2_i10];
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

static const mxArray *c2_c_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_i11;
  int32_T c2_i12;
  const mxArray *c2_y = NULL;
  int32_T c2_i13;
  real_T c2_u[9];
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_i11 = 0;
  for (c2_i12 = 0; c2_i12 < 3; c2_i12++) {
    for (c2_i13 = 0; c2_i13 < 3; c2_i13++) {
      c2_u[c2_i13 + c2_i11] = (*(real_T (*)[9])c2_inData)[c2_i13 + c2_i11];
    }

    c2_i11 += 3;
  }

  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", c2_u, 0, 0U, 1U, 0U, 2, 3, 3), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static void c2_d_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId, real_T c2_y[9])
{
  real_T c2_dv1[9];
  int32_T c2_i14;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_dv1, 1, 0, 0U, 1, 0U, 2, 3, 3);
  for (c2_i14 = 0; c2_i14 < 9; c2_i14++) {
    c2_y[c2_i14] = c2_dv1[c2_i14];
  }

  sf_mex_destroy(&c2_u);
}

static void c2_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_A;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  real_T c2_y[9];
  int32_T c2_i15;
  int32_T c2_i16;
  int32_T c2_i17;
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_A = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_A), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_A);
  c2_i15 = 0;
  for (c2_i16 = 0; c2_i16 < 3; c2_i16++) {
    for (c2_i17 = 0; c2_i17 < 3; c2_i17++) {
      (*(real_T (*)[9])c2_outData)[c2_i17 + c2_i15] = c2_y[c2_i17 + c2_i15];
    }

    c2_i15 += 3;
  }

  sf_mex_destroy(&c2_mxArrayInData);
}

const mxArray
  *sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_eml_resolved_functions_info
  (void)
{
  const mxArray *c2_nameCaptureInfo = NULL;
  c2_nameCaptureInfo = NULL;
  sf_mex_assign(&c2_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c2_nameCaptureInfo;
}

static const mxArray *c2_d_sf_marshallOut(void *chartInstanceVoid, void
  *c2_inData)
{
  const mxArray *c2_mxArrayOutData;
  int32_T c2_u;
  const mxArray *c2_y = NULL;
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_mxArrayOutData = NULL;
  c2_mxArrayOutData = NULL;
  c2_u = *(int32_T *)c2_inData;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_create("y", &c2_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c2_mxArrayOutData, c2_y, false);
  return c2_mxArrayOutData;
}

static int32_T c2_e_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  int32_T c2_y;
  int32_T c2_i18;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_i18, 1, 6, 0U, 0, 0U, 0);
  c2_y = c2_i18;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void c2_d_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c2_mxArrayInData, const char_T *c2_varName, void *c2_outData)
{
  const mxArray *c2_b_sfEvent;
  const char_T *c2_identifier;
  emlrtMsgIdentifier c2_thisId;
  int32_T c2_y;
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    chartInstanceVoid;
  c2_b_sfEvent = sf_mex_dup(c2_mxArrayInData);
  c2_identifier = c2_varName;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_e_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_sfEvent),
    &c2_thisId);
  sf_mex_destroy(&c2_b_sfEvent);
  *(int32_T *)c2_outData = c2_y;
  sf_mex_destroy(&c2_mxArrayInData);
}

static uint8_T c2_f_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_b_is_active_c2_Attitude_Control_System_of_Flying_Vehicle,
   const char_T *c2_identifier)
{
  uint8_T c2_y;
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = c2_identifier;
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_g_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_Attitude_Control_System_of_Flying_Vehicle), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_Attitude_Control_System_of_Flying_Vehicle);
  return c2_y;
}

static uint8_T c2_g_emlrt_marshallIn
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance,
   const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_y;
  uint8_T c2_u0;
  (void)chartInstance;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_u0, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_u0;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static void init_dsm_address_info
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address
  (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance)
{
  chartInstance->c2_omega_x = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c2_dtheta = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_omega_y = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c2_omega_z = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_dphi = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c2_dgamma = (real_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c2_theta = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c2_gamma = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_check_sum(mxArray *
  plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(1437682065U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(2191542217U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1658232278U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(849998990U);
}

mxArray* sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_post_codegen_info
  (void);
mxArray
  *sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("84tplOCPNkmE4ZmY8yoaPB");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,5,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,4,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,4,"type",mxType);
    }

    mxSetField(mxData,4,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,3,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo =
      sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c2_Attitude_Control_System_of_Flying_Vehicle_third_party_uses_info
  (void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c2_Attitude_Control_System_of_Flying_Vehicle_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray
  *sf_c2_Attitude_Control_System_of_Flying_Vehicle_updateBuildInfo_args_info
  (void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_post_codegen_info
  (void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray
  *sf_get_sim_state_info_c2_Attitude_Control_System_of_Flying_Vehicle(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x4'type','srcId','name','auxInfo'{{M[1],M[9],T\"dgamma\",},{M[1],M[8],T\"dphi\",},{M[1],M[5],T\"dtheta\",},{M[8],M[0],T\"is_active_c2_Attitude_Control_System_of_Flying_Vehicle\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 4, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance =
      (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _Attitude_Control_System_of_Flying_VehicleMachineNumber_,
           2,
           1,
           1,
           0,
           8,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation
          (_Attitude_Control_System_of_Flying_VehicleMachineNumber_,
           chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,
             _Attitude_Control_System_of_Flying_VehicleMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _Attitude_Control_System_of_Flying_VehicleMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"omega_x");
          _SFD_SET_DATA_PROPS(1,1,1,0,"omega_y");
          _SFD_SET_DATA_PROPS(2,1,1,0,"omega_z");
          _SFD_SET_DATA_PROPS(3,1,1,0,"theta");
          _SFD_SET_DATA_PROPS(4,1,1,0,"gamma");
          _SFD_SET_DATA_PROPS(5,2,0,1,"dtheta");
          _SFD_SET_DATA_PROPS(6,2,0,1,"dphi");
          _SFD_SET_DATA_PROPS(7,2,0,1,"dgamma");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,0,0,0,0,0,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",11,-1,473);
        _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(5,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(6,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
        _SFD_SET_DATA_COMPILED_PROPS(7,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c2_sf_marshallOut,(MexInFcnForType)c2_sf_marshallIn);
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _Attitude_Control_System_of_Flying_VehicleMachineNumber_,
        chartInstance->chartNumber,chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance =
      (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, chartInstance->c2_omega_x);
        _SFD_SET_DATA_VALUE_PTR(5U, chartInstance->c2_dtheta);
        _SFD_SET_DATA_VALUE_PTR(1U, chartInstance->c2_omega_y);
        _SFD_SET_DATA_VALUE_PTR(2U, chartInstance->c2_omega_z);
        _SFD_SET_DATA_VALUE_PTR(6U, chartInstance->c2_dphi);
        _SFD_SET_DATA_VALUE_PTR(7U, chartInstance->c2_dgamma);
        _SFD_SET_DATA_VALUE_PTR(3U, chartInstance->c2_theta);
        _SFD_SET_DATA_VALUE_PTR(4U, chartInstance->c2_gamma);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "s9DmYBHW33YRnCxJelKoAYH";
}

static void sf_opaque_initialize_c2_Attitude_Control_System_of_Flying_Vehicle
  (void *chartInstanceVar)
{
  chart_debug_initialization
    (((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
      chartInstanceVar)->S,0);
  initialize_params_c2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
     chartInstanceVar);
  initialize_c2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_enable_c2_Attitude_Control_System_of_Flying_Vehicle(void
  *chartInstanceVar)
{
  enable_c2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_disable_c2_Attitude_Control_System_of_Flying_Vehicle(void *
  chartInstanceVar)
{
  disable_c2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
     chartInstanceVar);
}

static void sf_opaque_gateway_c2_Attitude_Control_System_of_Flying_Vehicle(void *
  chartInstanceVar)
{
  sf_gateway_c2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
     chartInstanceVar);
}

static const mxArray*
  sf_opaque_get_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle(SimStruct*
  S)
{
  return get_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
     sf_get_chart_instance_ptr(S));    /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle
  (SimStruct* S, const mxArray *st)
{
  set_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
     sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c2_Attitude_Control_System_of_Flying_Vehicle
  (void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S =
      ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
       chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_Attitude_Control_System_of_Flying_Vehicle_optimization_info();
    }

    finalize_c2_Attitude_Control_System_of_Flying_Vehicle
      ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
       chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_Attitude_Control_System_of_Flying_Vehicle
    ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
     chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_Attitude_Control_System_of_Flying_Vehicle
  (SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_Attitude_Control_System_of_Flying_Vehicle
      ((SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*)
       sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c2_Attitude_Control_System_of_Flying_Vehicle
  (SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssSetStatesModifiedOnlyInUpdate(S, 1);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct =
      load_Attitude_Control_System_of_Flying_Vehicle_optimization_info
      (sim_mode_is_rtw_gen(S), sim_mode_is_modelref_sim(S), sim_mode_is_external
       (S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,2);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,2,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 2);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,2);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 4, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,2,5);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,2,3);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=3; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 5; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,2);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(2164788187U));
  ssSetChecksum1(S,(735181838U));
  ssSetChecksum2(S,(1220945499U));
  ssSetChecksum3(S,(3227228729U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c2_Attitude_Control_System_of_Flying_Vehicle(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c2_Attitude_Control_System_of_Flying_Vehicle(SimStruct *S)
{
  SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *chartInstance;
  chartInstance = (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct *)
    utMalloc(sizeof(SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof
         (SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway =
    sf_opaque_gateway_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.terminateChart =
    sf_opaque_terminate_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.enableChart =
    sf_opaque_enable_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.disableChart =
    sf_opaque_disable_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW =
    mdlRTW_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.mdlStart =
    mdlStart_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.mdlSetWorkWidths =
    mdlSetWorkWidths_c2_Attitude_Control_System_of_Flying_Vehicle;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c2_Attitude_Control_System_of_Flying_Vehicle(chartInstance);
}

void c2_Attitude_Control_System_of_Flying_Vehicle_method_dispatcher(SimStruct *S,
  int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c2_Attitude_Control_System_of_Flying_Vehicle(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_Attitude_Control_System_of_Flying_Vehicle(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_Attitude_Control_System_of_Flying_Vehicle(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_Attitude_Control_System_of_Flying_Vehicle_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
