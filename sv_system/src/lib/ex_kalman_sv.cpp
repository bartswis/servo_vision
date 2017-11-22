#include "ex_kalman_sv.h"

ExtendKalmanFilter::ExtendKalmanFilter()
{
    kal_Q_=100.0;
    kal_R_=0.01;

    P_k_ = Matrix::eye(3);
    x_k_ = Matrix(3,1);
    A_ = Matrix::eye(3);
    G_ = Matrix(3,1);
    double C_data[3] = {1,0,0};
    C_ = Matrix(1,3, C_data);
    
    last_rostime_ = ros::Time::now().toSec();
}

double ExtendKalmanFilter::compute(double position)
{
    ros::Time current_rostime = ros::Time::now();
    double dt = current_rostime.toSec() - last_rostime_;

    A_.element[0][1]=dt;
    A_.element[0][2]=(dt*dt)/2;
    A_.element[1][2]=dt;

    G_.element[0][0]=(dt*dt*dt)/3;
    G_.element[1][0]=(dt*dt)/2;
    G_.element[2][0]=dt;

    x_k_ = A_*x_k_;
    P_k_ = A_*P_k_*~A_ + G_*kal_Q_*~G_;

    double eps = position - (C_*x_k_).element[0][0];
    double S = (C_*P_k_*~C_).element[0][0] + kal_R_;
    Matrix K = P_k_*~C_/S;
    x_k_ = x_k_ + K*eps;
    P_k_ = P_k_ - K*S*~K;

    last_rostime_= current_rostime.toSec();

    return x_k_.element[0][0];
}

double ExtendKalmanFilter::predict()
{
    x_k_pred_ = A_*x_k_pred_;    
    return x_k_pred_.element[0][0];
}

void ExtendKalmanFilter::reset()
{
    x_k_pred_ = x_k_;

    double dt = 0.02;
    
    A_.element[0][1]=dt;
    A_.element[0][2]=(dt*dt)/2;
    A_.element[1][2]=dt;

    G_.element[0][0]=(dt*dt*dt)/3;
    G_.element[1][0]=(dt*dt)/2;
    G_.element[2][0]=dt;
}
