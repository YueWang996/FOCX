//
// Created by Justin on 02/02/2022.
//

#include "foc_utilities.h"
/// fast sine function lookup table
const float sin_tab[] = {
        0, 0.012296f, 0.024589f, 0.036879f, 0.049164f, 0.061441f, 0.073708f, 0.085965f, 0.098208f, 0.11044f, 0.12265f,
        0.13484f, 0.14702f, 0.15917f, 0.17129f, 0.18339f, 0.19547f, 0.20751f, 0.21952f, 0.2315f, 0.24345f, 0.25535f,
        0.26722f, 0.27905f, 0.29084f, 0.30258f, 0.31427f, 0.32592f, 0.33752f, 0.34907f, 0.36057f, 0.37201f, 0.38339f,
        0.39472f, 0.40599f, 0.41719f, 0.42834f, 0.43941f, 0.45043f, 0.46137f, 0.47224f, 0.48305f, 0.49378f, 0.50443f,
        0.51501f, 0.52551f, 0.53593f, 0.54627f, 0.55653f, 0.5667f, 0.57679f, 0.58679f, 0.5967f, 0.60652f, 0.61625f,
        0.62589f, 0.63543f, 0.64488f, 0.65423f, 0.66348f, 0.67263f, 0.68167f, 0.69062f, 0.69946f, 0.70819f, 0.71682f,
        0.72534f, 0.73375f, 0.74205f, 0.75023f, 0.75831f, 0.76626f, 0.77411f, 0.78183f, 0.78944f, 0.79693f, 0.80429f,
        0.81154f, 0.81866f, 0.82566f, 0.83254f, 0.83928f, 0.84591f, 0.8524f, 0.85876f, 0.865f, 0.8711f, 0.87708f,
        0.88292f,
        0.88862f, 0.89419f, 0.89963f, 0.90493f, 0.9101f, 0.91512f, 0.92001f, 0.92476f, 0.92937f, 0.93384f, 0.93816f,
        0.94235f, 0.94639f, 0.95029f, 0.95405f, 0.95766f, 0.96113f, 0.96445f, 0.96763f, 0.97066f, 0.97354f, 0.97628f,
        0.97887f, 0.98131f, 0.9836f, 0.98574f, 0.98774f, 0.98958f, 0.99128f, 0.99282f, 0.99422f, 0.99546f, 0.99656f,
        0.9975f, 0.99829f, 0.99894f, 0.99943f, 0.99977f, 0.99996f, 1.0f, 0.99988f, 0.99962f, 0.9992f, 0.99863f,
        0.99792f,
        0.99705f, 0.99603f, 0.99486f, 0.99354f, 0.99207f, 0.99045f, 0.98868f, 0.98676f, 0.98469f, 0.98247f, 0.9801f,
        0.97759f, 0.97493f, 0.97212f, 0.96916f, 0.96606f, 0.96281f, 0.95941f, 0.95587f, 0.95219f, 0.94836f, 0.94439f,
        0.94028f, 0.93602f, 0.93162f, 0.92708f, 0.9224f, 0.91758f, 0.91263f, 0.90753f, 0.9023f, 0.89693f, 0.89142f,
        0.88579f, 0.88001f, 0.87411f, 0.86807f, 0.8619f, 0.8556f, 0.84917f, 0.84261f, 0.83593f, 0.82911f, 0.82218f,
        0.81512f, 0.80793f, 0.80062f, 0.7932f, 0.78565f, 0.77798f, 0.7702f, 0.7623f, 0.75428f, 0.74615f, 0.73791f,
        0.72956f,
        0.72109f, 0.71252f, 0.70384f, 0.69505f, 0.68616f, 0.67716f, 0.66806f, 0.65886f, 0.64956f, 0.64017f, 0.63067f,
        0.62108f, 0.6114f, 0.60162f, 0.59176f, 0.5818f, 0.57176f, 0.56163f, 0.55141f, 0.54111f, 0.53073f, 0.52027f,
        0.50973f, 0.49911f, 0.48842f, 0.47765f, 0.46682f, 0.45591f, 0.44493f, 0.43388f, 0.42277f, 0.4116f, 0.40036f,
        0.38906f, 0.37771f, 0.36629f, 0.35483f, 0.3433f, 0.33173f, 0.32011f, 0.30843f, 0.29671f, 0.28495f, 0.27314f,
        0.26129f, 0.2494f, 0.23748f, 0.22552f, 0.21352f, 0.20149f, 0.18943f, 0.17735f, 0.16523f, 0.15309f, 0.14093f,
        0.12875f, 0.11655f, 0.10432f, 0.092088f, 0.079838f, 0.067576f, 0.055303f, 0.043022f, 0.030735f, 0.018443f,
        0.0061479f, -0.0061479f, -0.018443f, -0.030735f, -0.043022f, -0.055303f, -0.067576f, -0.079838f, -0.092088f,
        -0.10432f, -0.11655f, -0.12875f, -0.14093f, -0.15309f, -0.16523f, -0.17735f, -0.18943f, -0.20149f, -0.21352f,
        -0.22552f, -0.23748f, -0.2494f, -0.26129f, -0.27314f, -0.28495f, -0.29671f, -0.30843f, -0.32011f, -0.33173f,
        -0.3433f, -0.35483f, -0.36629f, -0.37771f, -0.38906f, -0.40036f, -0.4116f, -0.42277f, -0.43388f, -0.44493f,
        -0.45591f, -0.46682f, -0.47765f, -0.48842f, -0.49911f, -0.50973f, -0.52027f, -0.53073f, -0.54111f, -0.55141f,
        -0.56163f, -0.57176f, -0.5818f, -0.59176f, -0.60162f, -0.6114f, -0.62108f, -0.63067f, -0.64017f, -0.64956f,
        -0.65886f, -0.66806f, -0.67716f, -0.68616f, -0.69505f, -0.70384f, -0.71252f, -0.72109f, -0.72956f, -0.73791f,
        -0.74615f, -0.75428f, -0.7623f, -0.7702f, -0.77798f, -0.78565f, -0.7932f, -0.80062f, -0.80793f, -0.81512f,
        -0.82218f, -0.82911f, -0.83593f, -0.84261f, -0.84917f, -0.8556f, -0.8619f, -0.86807f, -0.87411f, -0.88001f,
        -0.88579f, -0.89142f, -0.89693f, -0.9023f, -0.90753f, -0.91263f, -0.91758f, -0.9224f, -0.92708f, -0.93162f,
        -0.93602f, -0.94028f, -0.94439f, -0.94836f, -0.95219f, -0.95587f, -0.95941f, -0.96281f, -0.96606f, -0.96916f,
        -0.97212f, -0.97493f, -0.97759f, -0.9801f, -0.98247f, -0.98469f, -0.98676f, -0.98868f, -0.99045f, -0.99207f,
        -0.99354f, -0.99486f, -0.99603f, -0.99705f, -0.99792f, -0.99863f, -0.9992f, -0.99962f, -0.99988f, -1.0f,
        -0.99996f,
        -0.99977f, -0.99943f, -0.99894f, -0.99829f, -0.9975f, -0.99656f, -0.99546f, -0.99422f, -0.99282f, -0.99128f,
        -0.98958f, -0.98774f, -0.98574f, -0.9836f, -0.98131f, -0.97887f, -0.97628f, -0.97354f, -0.97066f, -0.96763f,
        -0.96445f, -0.96113f, -0.95766f, -0.95405f, -0.95029f, -0.94639f, -0.94235f, -0.93816f, -0.93384f, -0.92937f,
        -0.92476f, -0.92001f, -0.91512f, -0.9101f, -0.90493f, -0.89963f, -0.89419f, -0.88862f, -0.88292f, -0.87708f,
        -0.8711f, -0.865f, -0.85876f, -0.8524f, -0.84591f, -0.83928f, -0.83254f, -0.82566f, -0.81866f, -0.81154f,
        -0.80429f,
        -0.79693f, -0.78944f, -0.78183f, -0.77411f, -0.76626f, -0.75831f, -0.75023f, -0.74205f, -0.73375f, -0.72534f,
        -0.71682f, -0.70819f, -0.69946f, -0.69062f, -0.68167f, -0.67263f, -0.66348f, -0.65423f, -0.64488f, -0.63543f,
        -0.62589f, -0.61625f, -0.60652f, -0.5967f, -0.58679f, -0.57679f, -0.5667f, -0.55653f, -0.54627f, -0.53593f,
        -0.52551f, -0.51501f, -0.50443f, -0.49378f, -0.48305f, -0.47224f, -0.46137f, -0.45043f, -0.43941f, -0.42834f,
        -0.41719f, -0.40599f, -0.39472f, -0.38339f, -0.37201f, -0.36057f, -0.34907f, -0.33752f, -0.32592f, -0.31427f,
        -0.30258f, -0.29084f, -0.27905f, -0.26722f, -0.25535f, -0.24345f, -0.2315f, -0.21952f, -0.20751f, -0.19547f,
        -0.18339f, -0.17129f, -0.15917f, -0.14702f, -0.13484f, -0.12265f, -0.11044f, -0.098208f, -0.085965f, -0.073708f,
        -0.061441f, -0.049164f, -0.036879f, -0.024589f, -0.012296f, 0
};

float fast_sin(float theta) {
    /* congruence of angle theta to 2pi */
    while (true) {
        if (theta > 6.2831854f && theta > 0)
            theta = theta - 6.2831854f;
        else if (theta < 0)
            theta = theta + 6.2831854f;
        else
            break;
    }
    /* look up the table to obtain the sine value */
    return sin_tab[(int) (81.4873308f * theta)];
}
