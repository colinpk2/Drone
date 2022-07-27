#include "../Eigen/ArduinoEigenSparse.h"

using namespace Eigen;

class Kalman {

    void update();
    void Initialize();
    void priori();

    float dt = .01;
    Matrix<float, 9, 1> x{0,0,0,0,0,0,0,0,0};
    Matrix<float, 9, 9> F{{1,0,0,dt,0,0,.5*dt*dt,0,0}, 
                        {0,1,0,0,dt,0,0,.5*dt*dt,0}, 
                        {0,0,1,0,0,dt,0,0,.5*dt*dt}, 
                        {0,0,0,1,0,0,dt,0,0},
                        {0,0,0,0,1,0,0,dt,0},
                        {0,0,0,0,0,1,0,0,dt},
                        {0,0,0,0,0,0,1,0,0},
                        {0,0,0,0,0,0,0,1,0},
                        {0,0,0,0,0,0,0,0,1},};
    Matrix<float, 2, 3> H{{0,0,0}, {0,0,0}};
    Matrix<float, 9, 9> P{{0,0,0}, {0,0,0}, {0,0,0}};
    Matrix<float, 9, 9> Q{{dt*dt,0,0,0,0,0,0,0,0},
                        {0,dt*dt,0,0,0,0,0,0,0}, 
                        {0,0,dt*dt,0,0,0,0,0,0},
                        {0,0,0,dt,0,0,0,0,0},
                        {0,0,0,0,dt,0,0,0,0},
                        {0,0,0,0,0,dt,0,0,0},
                        {0,0,0,0,0,0,1,0,0},
                        {0,0,0,0,0,0,0,1,0},
                        {0,0,0,0,0,0,0,0,1},
                        };
    Matrix<float, 2, 2> R{{0,0}, {0,0}};
    Matrix<float, 3, 3> P_pred{{0,0,0}, {0,0,0}, {0,0,0}};
    Matrix<float, 9, 1> x_pred{0,0,0,0,0,0,0,0,0};
    Matrix<float, 3, 2> K{{0,0}, {0,0}, {0,0}};
    Matrix<float, 2, 1> y{{0,0}};
};