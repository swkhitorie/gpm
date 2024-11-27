#pragma once

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
extern "C"
{
	#include "math/matrix.h"
}

/**
 * @brief the degree of SGFilter's polynomial
*/
#define SG_FILTER_ORDER		 		 (3)

/**
 * @brief the number of sample window, windows length is (2 * SG_FILTER_SAMPLE_TURN + 1)
*/
#define SG_FILTER_SAMPLE_TURN	 	 (5)

static float power(float num1, int num2)
{
	float result=1;
	int i;
	for(i=0;i<num2;i++)
		result*=num1;	
	return result;
}

class Savitzky_Golay_Filer
{
public:
	Savitzky_Golay_Filer() = default;
	~Savitzky_Golay_Filer() = default;
	
	void init()
	{
		unsigned char i, j;
		float32_t *X_ = X_f32;
		arm_mat_init_f32(&Data_Windows, (2 * SG_FILTER_SAMPLE_TURN + 1), 1, Data_Windows_f32);
		arm_mat_init_f32(&Data_output, (2 * SG_FILTER_SAMPLE_TURN + 1), 1, Data_output_f32);
		arm_mat_init_f32(&X, (2 * SG_FILTER_SAMPLE_TURN + 1), (SG_FILTER_ORDER + 1), X_f32);
		arm_mat_init_f32(&B, (2 * SG_FILTER_SAMPLE_TURN + 1), (2 * SG_FILTER_SAMPLE_TURN + 1), B_f32);
		arm_mat_init_f32(&XT, (SG_FILTER_ORDER + 1), (2 * SG_FILTER_SAMPLE_TURN + 1), XT_f32);
		arm_mat_init_f32(&XT_multi_X, (SG_FILTER_ORDER + 1), (SG_FILTER_ORDER + 1), XT_multi_X_f32);
		arm_mat_init_f32(&XT_multi_X_inverse, (SG_FILTER_ORDER + 1), (SG_FILTER_ORDER + 1), XT_multi_X_inverse_f32);
		arm_mat_init_f32(&tempMulti_1, (2 * SG_FILTER_SAMPLE_TURN + 1), (SG_FILTER_ORDER + 1), tempMulti_1_f32);
		arm_mat_zero_f32(&Data_Windows);
		arm_mat_zero_f32(&Data_output);
		arm_mat_zero_f32(&X);
		arm_mat_zero_f32(&B);
		arm_mat_zero_f32(&XT);
		arm_mat_zero_f32(&XT_multi_X);
		arm_mat_zero_f32(&XT_multi_X_inverse);
		arm_mat_zero_f32(&tempMulti_1);
		
		for (i = 0; i < (2 * SG_FILTER_SAMPLE_TURN + 1); i++) {
			for (j = 0; j < (SG_FILTER_ORDER + 1); j++) {
				X_[i * (SG_FILTER_ORDER + 1) + j] = power(-SG_FILTER_SAMPLE_TURN + i, j);
			}
		}
		/* Get Matrix XT */
		arm_mat_trans_f32(&X, &XT);
		/* Get Matrix XT*X */
		arm_mat_mult_f32(&XT, &X, &XT_multi_X);
		/* Get Matrix (XT*X)^-1 */
		arm_mat_inverse_f32(&XT_multi_X, &XT_multi_X_inverse);
		/* Get Matrix X*(XT*X)^(-1) */
		arm_mat_mult_f32(&X, &XT_multi_X_inverse, &tempMulti_1);
		/* Get Matrix X*(XT*X)^(-1)*XT */
		arm_mat_mult_f32(&tempMulti_1, &XT, &B);
	}
	
	/**
	 * @brief 	put data into Filter, and output the SG filter result
	 * @param input the data will be input to SG filter
	 * @param output the result of lpf butterworth 2nd filter
	 * @return the result of SGFilting's process
	 * @retval 1 -- the data windows has no enough data
	 * @retval 0 -- success 
	*/
	unsigned char update(float input,float &output)
	{
		/* arm_status result; */
		output = 0;
		input_sg = input;
		if (windows_index < (2 * SG_FILTER_SAMPLE_TURN + 1)) {
			Data_Windows_f32[windows_index++] = input_sg;
			output = 0;
			return 1;
		}
		if (windows_index >= (2 * SG_FILTER_SAMPLE_TURN + 1)) {
			windows_index = 0xFF;				
			for (unsigned char i = 0; i < 2 * SG_FILTER_SAMPLE_TURN + 1 - 1; i++) {
				Data_Windows_f32[i] = Data_Windows_f32[i+1];
			}
			Data_Windows_f32[2 * SG_FILTER_SAMPLE_TURN + 1 -1] = input_sg;
			for (unsigned char i = 0; i < 2 * SG_FILTER_SAMPLE_TURN + 1; i++) {
				output += B.pData[SG_FILTER_SAMPLE_TURN * (B.numCols) + i] * Data_Windows_f32[i];
			}
			/* result = arm_mat_mult_f32(&B,&Data_Windows,&Data_output); */
			output_sg = output;
			/* if(result == ARM_MATH_SUCCESS){
				output = Data_output_f32[SG_FILTER_SAMPLE_TURN]; 
				output_sg = output;
				return 2;
			}else{
				return 1;
			} */		
		}
		return 0;
	}
	
private:
	arm_matrix_instance_f32 Data_Windows;
	arm_matrix_instance_f32 Data_output;
	arm_matrix_instance_f32 X;
	arm_matrix_instance_f32 XT;
	arm_matrix_instance_f32 XT_multi_X;
	arm_matrix_instance_f32 XT_multi_X_inverse;
	arm_matrix_instance_f32 tempMulti_1;
	arm_matrix_instance_f32 B;
	float32_t Data_Windows_f32[ (2*SG_FILTER_SAMPLE_TURN+1) * 1];
	float32_t Data_output_f32[ (2*SG_FILTER_SAMPLE_TURN+1) * 1];
	float32_t X_f32[ (2*SG_FILTER_SAMPLE_TURN+1) * (SG_FILTER_ORDER+1)];
	float32_t XT_f32[ (SG_FILTER_ORDER+1) * (2*SG_FILTER_SAMPLE_TURN+1)];
	float32_t XT_multi_X_f32[ (SG_FILTER_ORDER+1) * (SG_FILTER_ORDER+1)];
	float32_t XT_multi_X_inverse_f32[ (SG_FILTER_ORDER+1) * (SG_FILTER_ORDER+1) ];
	float32_t tempMulti_1_f32[ (2*SG_FILTER_SAMPLE_TURN+1) * (SG_FILTER_ORDER+1) ];
	float32_t B_f32[ (2*SG_FILTER_SAMPLE_TURN+1) * (2*SG_FILTER_SAMPLE_TURN+1)];

	float input_sg;
	float output_sg;
	unsigned char windows_index;
	
};

