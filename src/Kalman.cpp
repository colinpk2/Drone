#include "kalman.h"

void Kalman::Initialize() {
}

void Kalman::priori() {
    // x_priori = (F @ x_k) + ((B @ u).T) #* For some reason doesnt work when B
    // or u = 0
    x_pred = (F * x);
    P_pred = (F * P * F.transpose()) + Q;
}

/**
 * @brief Update Kalman Gain
 *
 */
void Kalman::update() {
    Eigen::Matrix<float, 2, 2> temp = Eigen::Matrix<float, 2, 2>::Zero();
    temp = (((H * P_pred * H.transpose()) + R)).inverse();
    Eigen::Matrix<float, 3, 3> identity =
        Eigen::Matrix<float, 3, 3>::Identity();
    K = (P_pred * H.transpose()) * temp;

    // # Posteriori Update
    x = x_pred + K * (y - (H * x_pred));
    P = (identity - K * H) * P_pred;
}