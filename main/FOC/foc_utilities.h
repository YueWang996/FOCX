//
// Created by Justin on 02/02/2022.
//

#ifndef FOCX_FOC_UTILITIES_H
#define FOCX_FOC_UTILITIES_H

#define fast_cos(x)                     fast_sin(1.5707963f - x)
#define fast_sqrt(x)                    _sqrtApprox(x)
#define _sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3  1.0471975512f
#define _2PI_3 2.0943951024f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f

extern "C" {
    float fast_sin(float theta);
}

#endif //FOCX_FOC_UTILITIES_H
