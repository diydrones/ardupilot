#pragma once

#include "definitions.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

// Enable double only if the board type and the builder explicitely enable it
#if defined(DBL_MATH) && CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    #include "AP_Math_Double.h"
// Single precision is the absolute default and always active if not forced otherwise
#else
    //#include "AP_Math_Double.h"
    #include "AP_Math_Float.h"
#endif

#include "edc.h"
#include "location.h"
#include "matrix3.h"
#include "polygon.h"
#include "quaternion.h"
#include "vector2.h"
#include "vector3.h"

// define AP_Param types AP_Vector3f and Ap_Matrix3f
AP_PARAMDEFV(Vector3f, Vector3f, AP_PARAM_VECTOR3F);


inline float constrain_float(const float amt, const float low, const float high)
{
    return constrain_value(amt, low, high);
}

inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
{
    return constrain_value(amt, low, high);
}

inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
{
    return constrain_value(amt, low, high);
}

/*
 * Return the smaller number of both parameters
 */
template<typename A, typename B>
auto min(const A one, const B two) -> decltype(one < two ? one : two) 
{ 
    return one < two ? one : two;
}

/*
 * Return the bigger number of both parameters
 */
template<typename A, typename B>
auto max(const A one, const B two) -> decltype(one > two ? one : two) 
{
    return one > two ? one : two;
}

/*
 * Converter functions
 *  - Avoid zero divisions
 *  - Inheritss a float cast (because of PX4)
 */
template<class T>
T hz_to_nsec(const T freq) 
{
    return freq != 0 ? NSEC_PER_SEC / freq : 0;
}

template<class T>
T nsec_to_hz(const T nsec) 
{
    return nsec != 0 ? NSEC_PER_SEC / nsec : 0;
}

template<class T>
T usec_to_nsec(const T usec) 
{
    return usec * NSEC_PER_USEC;
}

template<class T>
T nsec_to_usec(const T nsec) 
{
    return nsec / NSEC_PER_USEC;
}

template<class T>
T hz_to_usec(const T freq) 
{
    return freq != 0 ? USEC_PER_SEC / freq : 0;
}

template<class T>
T usec_to_hz(const T usec) 
{
    return usec != 0 ? USEC_PER_SEC / usec : 0;
}

template <class T>
T linear_interpolate(const T low_output, const T high_output,
                     const T var_value,
                     const T var_low, const T var_high)
{
    if (var_value <= var_low) {
        return low_output;
    }
    if (var_value >= var_high) {
        return high_output;
    }
    // avoid zero divisions or zero like divisions
    auto var_diff = var_high - var_low;
    if(is_zero((float)var_diff)) {
        return low_output;
    }
    
    T p = (var_value - var_low) / var_diff;
    return low_output + p * (high_output - low_output);
}

/*
 * MATRIX
 */
//matrix algebra
bool    inverse(float x[], float y[], uint16_t dim);
// invOut is an inverted 4x4 matrix when returns true, otherwise matrix is Singular
bool    inverse3x3(float m[], float invOut[]);
// invOut is an inverted 3x3 matrix when returns true, otherwise matrix is Singular
bool    inverse4x4(float m[],float invOut[]);
// matrix multiplication of two NxN matrices
float   *mat_mul(float *A, float *B, uint8_t n);
//matrix algebra
bool    inverse(float x[], float y[], uint16_t dim);
//
void    mat_LU_decompose(float*, float*, float*, float*, uint8_t);
//
bool    mat_inverse(float*, float*, uint8_t);
//
void    mat_back_sub(float*, float*, uint8_t);
//
void    mat_forward_sub(float *L, float *out, uint8_t n);
//
void    mat_pivot(float* A, float* pivot, uint8_t n);

