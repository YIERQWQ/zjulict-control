#ifndef _KALMAN_H_
#define _KALMAN_H_

class Kalman1D {
public:
    // Q: process noise variance, R: measurement noise variance
    Kalman1D(float Q = 0.01f, float R = 1.0f, float P = 1.0f, float x0 = 0.0f);
    void init(float Q, float R, float P, float x0);
    // update with a new measurement z, returns the filtered state
    float update(float z);
    float state() const { return x; }
private:
    float Q; // process noise covariance
    float R; // measurement noise covariance
    float P; // estimation error covariance
    float x; // state estimate
};

#endif // _KALMAN_H_
