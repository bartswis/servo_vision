#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <matrix.h>

class ExtendKalmanFilter
{
public:

    ExtendKalmanFilter();
    double compute(double position);
    double predict();
    void reset();

private:

    double last_rostime_;
    
    double kal_Q_;
    double kal_R_;

    Matrix P_k_;
    Matrix x_k_;
    Matrix A_;
    Matrix G_;
    Matrix C_;
    
    Matrix x_k_pred_;

};

