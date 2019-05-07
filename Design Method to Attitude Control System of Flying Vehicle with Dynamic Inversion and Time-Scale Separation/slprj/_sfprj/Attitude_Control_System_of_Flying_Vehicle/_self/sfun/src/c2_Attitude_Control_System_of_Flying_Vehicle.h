#ifndef __c2_Attitude_Control_System_of_Flying_Vehicle_h__
#define __c2_Attitude_Control_System_of_Flying_Vehicle_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct
#define typedef_SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c2_sfEvent;
  boolean_T c2_doneDoubleBufferReInit;
  uint8_T c2_is_active_c2_Attitude_Control_System_of_Flying_Vehicle;
  real_T *c2_omega_x;
  real_T *c2_dtheta;
  real_T *c2_omega_y;
  real_T *c2_omega_z;
  real_T *c2_dphi;
  real_T *c2_dgamma;
  real_T *c2_theta;
  real_T *c2_gamma;
} SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct;

#endif                                 /*typedef_SFc2_Attitude_Control_System_of_Flying_VehicleInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray
  *sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_eml_resolved_functions_info
  (void);

/* Function Definitions */
extern void sf_c2_Attitude_Control_System_of_Flying_Vehicle_get_check_sum
  (mxArray *plhs[]);
extern void c2_Attitude_Control_System_of_Flying_Vehicle_method_dispatcher
  (SimStruct *S, int_T method, void *data);

#endif
