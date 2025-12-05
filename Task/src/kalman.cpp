#include "kalman.h"

Kalman1D::Kalman1D(float Q_, float R_, float P_, float x0)
    : Q(Q_), R(R_), P(P_), x(x0)
{
}

void Kalman1D::init(float Q_, float R_, float P_, float x0)
{
    Q = Q_;
    R = R_;
    P = P_;
    x = x0;
}

float Kalman1D::update(float z)
{
    // prediction step: for scalar stationary model, prediction does not change x
    // but covariance increases by process noise
    P += Q;

    // Kalman gain
    float K = P / (P + R);

    // update estimate with measurement z
    x = x + K * (z - x);

    // update covariance
    P = (1.0f - K) * P;

    return x;
}
