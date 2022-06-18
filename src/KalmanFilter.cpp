// Author : BorisVandermeer
// Zhejiang University
// Date : 2022.06.17

#include "KalmanFilter.h"

using namespace ToyFilter;

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R
) : A(A),H(H),Q(Q),R(R),m(H.rows()),n(A.rows()),x_hat(n)
{
    init_flag = false;
}

void  KalmanFilter::setA(const Eigen::MatrixXd& A)
{
    this->A = A;
}
void  KalmanFilter::setQ(const Eigen::MatrixXd& Q)
{
    this->Q = Q;
}

void KalmanFilter::setR(const Eigen::MatrixXd& R)
{
    this->R = R;
}

bool KalmanFilter::init(const Eigen::VectorXd& x0,const Eigen::MatrixXd& P0)
{
    x_hat = x0;
    P = P0;
    init_flag = true;
    return true;
}


bool KalmanFilter::update(const Eigen::VectorXd& y)
{
    x_hat= A * x_hat;
    P = A*P*A.transpose()+Q;
    K = P*H.transpose()*(H*P*H.transpose()+R).inverse();
    x_hat += K*(y-H*x_hat);
    Eigen::MatrixXd I(m,n);
    I.setIdentity();
    P = (I-K*H)*P;
    return true;
}

Eigen::VectorXd KalmanFilter::state()
{
    return x_hat;
}

KalmanFilter_Dynamic::KalmanFilter_Dynamic(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& B,
    const Eigen::MatrixXd& H,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R
) : A(A),B(B),H(H),Q(Q),R(R),m(H.rows()),n(A.rows()),x_hat(n)
{
    init_flag = false;
}

void KalmanFilter_Dynamic::setA(const Eigen::MatrixXd& A)
{
    this->A = A;
}

void KalmanFilter_Dynamic::setB(const Eigen::MatrixXd& B)
{
    this->B = B;
}

void KalmanFilter_Dynamic::setQ(const Eigen::MatrixXd& Q)
{
    this->Q = Q;
}

void KalmanFilter_Dynamic::setR(const Eigen::MatrixXd& R)
{
    this->R = R;
}

bool KalmanFilter_Dynamic::init(const Eigen::VectorXd& x0,const Eigen::MatrixXd& P0)
{
    x_hat = x0;
    P = P0;
    init_flag = true;
    return true;
}

bool KalmanFilter_Dynamic::update(const Eigen::VectorXd& y,const Eigen::VectorXd& u)
{
    if(!init_flag) return false;
    x_hat = A*x_hat+B*u;
    P = A*P*A.transpose()+Q;
    K = P*H.transpose()*(H*P*H.transpose()+R);
    x_hat += K*(y-H*x_hat);
    Eigen::MatrixXd I(m,n);
    I.setIdentity();
    P = (I-K*H)*P;
    return true;
}

bool KalmanFilter_Dynamic::update(const Eigen::VectorXd& y,const Eigen::VectorXd& u,Eigen::MatrixXd& A)
{
    if(!init_flag) return false;
    x_hat = A*x_hat+B*u;
    P = A*P*A.transpose()+Q;
    K = P*H.transpose()*(H*P*H.transpose()+R);
    x_hat += K*(y-H*x_hat);
    Eigen::MatrixXd I(m,n);
    I.setIdentity();
    P = (I-K*H)*P;
    return true;
}

Eigen::VectorXd KalmanFilter_Dynamic::state()
{
    return x_hat;
}
