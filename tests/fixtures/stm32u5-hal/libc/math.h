/*
 * Freestanding <math.h> for the STM32U5 HAL smoke firmware. Written for rsemu
 * (MIT). The toolchain this is built with may have no C library at all, and
 * ST's HAL includes <math.h> only for float_t (the RCC PLL fraction maths).
 */
#ifndef RSEMU_FREESTANDING_MATH_H
#define RSEMU_FREESTANDING_MATH_H
typedef float float_t;
typedef double double_t;
#define sqrtf(x) __builtin_sqrtf(x)
#endif
