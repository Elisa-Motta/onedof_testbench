#include "esmacat_utility.h"


/** @brief Generates a sigmoid curve using the input parameter
 *
 * Sigmoid f(x) = 1/(1+ exp(-x))
 * @param x used in f(x) for sigmoid function
 * @return f(x) computed output of the sigmoid function
 */
float sigmoid(float x)
{
    float one = static_cast<float>(1);
    float exponent = static_cast<float>(exp(static_cast<double>(x)));
    float output = static_cast<float>(one/(one + exponent));
    return output;
}

/** @brief Generates a sigmoid curve using the input parameter
 *
 * smooth start function when x=1.0 at scale =1.0, the output is 0.75
 * @param x used in f(x) for sigmoid function
 * @return f(x) computed output of the sigmoid function
 */
float smooth_start_func_tanh(float x, float scale)
{
    return tanh(x/scale);
}


/** @brief Converts uint16 to float upon receipt from EtherCAT
 *
 * Move the 16 bits of the uint16 into the most significant 16 bits of the
 * float parameter.
 */
float convert_ecat_format_to_float(uint16_t uint_number)
{
    // same memory is shared by all members of the union
    union
    {
        float f_number;
        uint16_t bytes[2];
    } union_for_conv;
    union_for_conv.f_number = 0;
    union_for_conv.bytes[1] = uint_number;
    return union_for_conv.f_number;
}

/** @brief Converts float to uint16 for transmission over EtherCAT
 *
 * Move the most significant 16 bits of the float into the uint16 type. This allows
 * for the sign, biased exponent and 7 bits of the mantissa to be used which is
 * sufficient accuracy for the computations. The uint16 is no longer representative of the same
 * number as the float unless converted back using the convert_ecat_format_to_float function, prior to
 * use in any computations.
 */
uint16_t convert_float_to_ecat_format(float float_number)
{
    union {
        float f_number;
        uint16_t bytes[2];
    } union_for_conv;

    union_for_conv.f_number = float_number;
    uint16_t a = union_for_conv.bytes[1];
    return a;
}

/** @brief Interprets the input floating point as a 32 bit unsigned integer
 * for transmission over ethercat */
uint32_t convert_float_for_tx(float float_number)
{
    union {
        float f_number;
        uint32_t bytes;
    } union_for_conv;

    union_for_conv.f_number = float_number;
    return union_for_conv.bytes;
}

/** @brief Interprets the input  32 bit unsigned integer as a floating point
 * for receipt over ethercat */
float convert_rx_to_float(uint32_t uint_number)
{
    union {
        float f_number;
        uint32_t bytes;
    } union_for_conv;

    union_for_conv.bytes = uint_number;
    return union_for_conv.f_number;
}


