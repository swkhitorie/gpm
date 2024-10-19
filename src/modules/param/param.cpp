#include "parameters.hpp"
#include <string.h>

using namespace UParams;
param_value_t board_params[LIST_NUM];
param_t UParams::param_handle(parnodetype p) {  return (param_t)p; }
param_value_t* UParams::get_head() { return &board_params[0]; }
int UParams::param_list_size() { return LIST_NUM; }

static constexpr uint16_t param_info_count = LIST_NUM;
static constexpr bool handle_in_range(param_t param) { return (param < param_info_count); }

static bool strcmp_pri(const char *p1, const char *p2)
{
    for (int i = 0; i < 16; i++) {
        if (p1[i] != p2[i])
            return true;
    }
    return false;
}

param_t UParams::param_find(const char *name)
{
	param_t middle;
	param_t front = 0;
	param_t last = param_info_count;

	/* perform a binary search of the known parameters */

//	while (front <= last) {
//		middle = front + (last - front) / 2;
//		int ret = strcmp(name, param_name(middle));

//		if (ret == 0) {
//            
//			return middle;

//		} else if (middle == front) {
//			/* An end point has been hit, but there has been no match */
//			break;

//		} else if (ret < 0) {
//			last = middle;

//		} else {
//			front = middle;
//		}
//	}

	/* not found */
	return PARAM_INVALID;
}

const char *UParams::param_name(param_t param)
{
	return handle_in_range(param) ? board_params[param].param_id : nullptr;
}

void UParams::board_paramlist_init()
{
    UPARAMS_DEFINE( FRAME_CLASS, 1, MAV_PARAM_TYPE_INT32)			//rover
    UPARAMS_DEFINE( FRAME_TYPE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RCMAP_ROLL, 1, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RCMAP_PITCH, 2, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RCMAP_YAW, 4, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RCMAP_THROTTLE, 3, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FLTMODE6, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FLTMODE5, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FLTMODE4, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FLTMODE3, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FLTMODE2, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FLTMODE1, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( TUNE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( TUNE_MIN, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( TUNE_MAX, 100, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FS_THR_ENABLE, 1, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( AHRS_ORIENTATION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( INS_ACCOFFS_X, -0.073, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_ACCOFFS_Y, -0.021, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_ACCOFFS_Z, -0.139, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_ACCSCAL_X, 0.998, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_ACCSCAL_Y, 1.001, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_ACCSCAL_Z, 1.000, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_ACC_ID, 1246218, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( INS_ACCEL_FILTER, 10, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_GYROFFS_X, 0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_GYROFFS_Y, 0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_GYROFFS_Z, 0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_GYRO_FILTER, 50, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_GYRO_RATE, 1000, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( INS_GYR_ID, 2163722, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DIA_X, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DIA_Y, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DIA_Z, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS_X, 41, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS_Y, -153, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS_Z, 51, MAV_PARAM_TYPE_REAL32)
    
    UPARAMS_DEFINE( COMPASS_FILTER, 20, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DEC, 0.0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ENABLE, 1, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_SCALE, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_SCALE2, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_SCALE3, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_AUTODEC, 1, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_CAL_FIT, 16, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID, 658945, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID2, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID3, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID4, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID5, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID6, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID7, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DEV_ID8, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_DIA2_X, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DIA2_Y, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DIA2_Z, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DIA3_X, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DIA3_Y, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_DIA3_Z, 1, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_EXTERN2, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_EXTERN3, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_EXTERNAL, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_LEARN, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_MOT_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT2_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT2_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT2_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT3_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT3_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOT3_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_MOTCT, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_ODI2_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI2_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI2_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI3_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI3_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_ODI3_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFFS_MAX, 1800, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS2_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS2_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS2_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS3_X, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS3_Y, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OFS3_Z, 0, MAV_PARAM_TYPE_REAL32)
	UPARAMS_DEFINE( COMPASS_OPTIONS, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_ORIENT, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_ORIENT2, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_ORIENT3, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_PMOT_EN, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_PRIO1_ID, 658945, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_PRIO2_ID, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_PRIO3_ID, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_TYPEMASK, 0, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_USE, 1, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_USE2, 1, MAV_PARAM_TYPE_INT32)
	UPARAMS_DEFINE( COMPASS_USE3, 1, MAV_PARAM_TYPE_INT32)
    
    UPARAMS_DEFINE( BATT_MONITOR, 3, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( ARMING_CHECK, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( LAND_SPEED, 0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( RTL_ALT_FINAL, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RTL_LOIT_TIME, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RTL_ALT, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FS_THR_VALUE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FS_GCS_ENABLE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( AUTOTUNE_AXES, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FENCE_MARGIN, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FENCE_ACTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FENCE_TYPE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FENCE_RADIUS, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FENCE_ALT_MAX, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( FENCE_ENABLE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( MOT_SPIN_MIN, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( MOT_SPIN_ARM, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( PSC_ACCZ_P, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( PSC_ACCZ_I, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( ATC_RAT_RLL_P, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( ATC_RAT_RLL_I, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( ATC_RAT_PIT_P, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( ATC_RAT_PIT_I, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC1_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC2_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC3_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC4_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC5_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC6_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC7_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC8_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC9_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC10_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC11_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC12_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC13_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC14_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC15_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC16_OPTION, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC_OPTIONS, 32, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC_OVERRIDE_TIME, 3.000, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( RC_PROTOCOLS, 1, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC1_DZ, 30, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC1_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC1_MIN, 1009, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC1_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC1_TRIM, 1501, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC2_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC2_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC2_MIN, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC2_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC2_TRIM, 1499, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC3_DZ, 30, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC3_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC3_MIN, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC3_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC3_TRIM, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC4_DZ, 30, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC4_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC4_MIN, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC4_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC4_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC5_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC5_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC5_MIN, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC5_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC5_TRIM, 1499, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC6_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC6_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC6_MIN, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC6_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC6_TRIM, 1499, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC7_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC7_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC7_MIN, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC7_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC7_TRIM, 1499, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC8_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC8_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC8_MIN, 999, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC8_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC8_TRIM, 1499, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC9_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC9_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC9_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC9_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC9_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC10_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC10_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC10_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC10_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC10_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC11_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC11_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC11_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC11_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC11_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC12_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC12_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC12_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC12_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC12_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC13_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC13_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC13_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC13_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC13_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC14_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC14_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC14_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC14_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC14_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC15_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC15_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC15_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC15_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC15_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC16_DZ, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC16_MAX, 2000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC16_MIN, 1000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC16_REVERSED, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( RC16_TRIM, 1500, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( SENS_FLOW_ROT, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( SENS_FLOW_MINHGT, 0.2, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( SENS_FLOW_MAXHGT, 4, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( SENS_FLOW_MAXR, 0.785, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( SENS_FLOW_RATE, 70, MAV_PARAM_TYPE_REAL32)
    
    UPARAMS_DEFINE( LPE_FUSION, 1, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( LPE_VXY_PUB, 0.3, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_Z_PUB, 1.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_SNR_Z, 0.05, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_SNR_OFF_Z, 0.00, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_LDR_Z, 0.03, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_LDR_OFF_Z, 0.00, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_ACC_XY, 0.012, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_ACC_Z, 0.02, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_BAR_Z, 3.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_GPS_DELAY, 0.29, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_GPS_XY, 1.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_GPS_Z, 3.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_GPS_VXY, 0.25, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_GPS_VZ, 0.25, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_EPH_MAX, 3.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_EPV_MAX, 5.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_VIS_XY, 0.1, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_VIS_Z, 0.5, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_VIS_DELAY, 0.1, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_VIC_P, 0.001, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_FLW_OFF_Z, 0.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_FLW_SCALE, 1.3, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_FLW_QMIN, 150, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( LPE_FLW_R, 7.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_FLW_RR, 7.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_LAND_Z, 0.03, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_LAND_VXY, 0.05, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_PN_P, 0.1, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_PN_V, 0.1, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_PN_B, 1e-3, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_PN_T, 0.001, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_T_MAX_GRADE, 1.0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_LT_COV, 0.0001, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LTEST_MODE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( LPE_FAKE_ORIGIN, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE( LPE_LAT, 47.397742, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE( LPE_LON, 8.545594, MAV_PARAM_TYPE_REAL32)
    
    UPARAMS_DEFINE(EKF2_MIN_OBS_DT, 20, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_MAG_DELAY, 0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_BARO_DELAY, 0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_DELAY, 110, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_OF_DELAY, 5, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_DELAY, 5, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ASP_DELAY, 100, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EV_DELAY, 175, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_AVEL_DELAY, 5, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_CHECK, 245, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_REQ_EPH, 3.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_REQ_EPV, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_REQ_SACC, 0.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_REQ_NSATS, 6, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_REQ_PDOP, 2.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_REQ_HDRIFT, 0.1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_REQ_VDRIFT, 0.2f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GYR_NOISE, 1.5e-2f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ACC_NOISE, 3.5e-1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GYR_B_NOISE, 1.0e-3f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ACC_B_NOISE, 3.0e-3f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAG_B_NOISE, 1.0e-4f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAG_E_NOISE, 1.0e-3f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_WIND_NOISE, 1.0e-1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_V_NOISE, 0.3f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_P_NOISE, 0.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_NOAID_NOISE, 10.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_BARO_NOISE, 3.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_HEAD_NOISE, 0.3f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAG_NOISE, 5.0e-2f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EAS_NOISE, 1.4f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_BETA_GATE, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_BETA_NOISE, 0.3f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAG_DECL, 0, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_HDG_GATE, 2.6f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAG_GATE, 3.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_DECL_TYPE, 7, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_MAG_TYPE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_MAG_ACCLIM, 0.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAG_YAWLIM, 0.25f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_BARO_GATE, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GND_EFF_DZ, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GND_MAX_HGT, 0.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_P_GATE, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_V_GATE, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_TAS_GATE, 3.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_AID_MASK, 1, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_HGT_MODE, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_TERR_MASK, 3, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_NOAID_TOUT, 5000000, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_RNG_NOISE, 0.1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_SFE, 0.05f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_GATE, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MIN_RNG, 0.1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EV_NOISE_MD, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_EVP_NOISE, 0.1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EVV_NOISE, 0.1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EVA_NOISE, 0.05f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_OF_N_MIN, 0.15f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_OF_N_MAX, 0.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_OF_QMIN, 1, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_OF_GATE, 3.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_TERR_NOISE, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_TERR_GRAD, 0.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_IMU_ID, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_IMU_POS_X, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_IMU_POS_Y, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_IMU_POS_Z, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_POS_X, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_POS_Y, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_POS_Z, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_POS_X, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_POS_Y, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_POS_Z, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_OF_POS_X, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_OF_POS_Y, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_OF_POS_Z, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EV_POS_X, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EV_POS_Y, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EV_POS_Z, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ARSP_THR, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_FUSE_BETA, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_TAU_VEL, 0.25f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_TAU_POS, 0.25f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GBIAS_INIT, 0.1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ABIAS_INIT, 0.2f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ANGERR_INIT, 0.1f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_PITCH, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAGBIAS_X, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAGBIAS_Y, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAGBIAS_Z, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAGBIAS_ID, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_MAGB_VREF, 2.5E-7f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAGB_K, 0.2f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_AID, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_RNG_A_VMAX, 1.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_A_HMAX, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_RNG_A_IGATE, 1.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EVV_GATE, 3.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_EVP_GATE, 5.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_DRAG_NOISE, 2.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_BCOEF_X, 25.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_BCOEF_Y, 25.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ASPD_MAX, 20.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_PCOEF_XP, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_PCOEF_XN, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_PCOEF_YP, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_PCOEF_YN, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_PCOEF_Z, 0.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ABL_LIM, 0.4f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ABL_ACCLIM, 25.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ABL_GYRLIM, 3.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_ABL_TAU, 0.5f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_GPS_MASK, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_GPS_TAU, 10.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MOVE_TEST, 1.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_REQ_GPS_H, 10.0f, MAV_PARAM_TYPE_REAL32)
    UPARAMS_DEFINE(EKF2_MAG_CHECK, 0, MAV_PARAM_TYPE_INT32)
    UPARAMS_DEFINE(EKF2_GSF_TAS, 15.0f, MAV_PARAM_TYPE_REAL32)
}

