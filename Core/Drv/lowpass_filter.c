#include "lowpass_filter.h"


/* Public Functions -------------------------------------------------------------------------------*/\

/**
 * @name Lowpass_Filter_Update.
 * @brief This function used to filter the signal.
 * @param output Pointer to output.
 * @param input Pointer to input.
 * @param alpha This alpha factor should be in range [0 : 1].
 * @retval None.
 * @note Do not change output, otherwise output will be incorrect.
*/
void Lowpass_Filter_Update(float *output, float *input, float alpha) {
    *output = (*output * (1-alpha)) + (*input * alpha);
}
