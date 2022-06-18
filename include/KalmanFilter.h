// Author : BorisVandermeer
// Zhejiang University
// Date : 2022.06.17

#include<Eigen/Dense>

namespace ToyFilter
{
    // KalmanFilter without input u
    class KalmanFilter
    {
    public:
        KalmanFilter() = delete;
        KalmanFilter(
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R
        );
        // Change The
        void setA(const Eigen::MatrixXd& A);
        void setQ(const Eigen::MatrixXd& Q);
        void setR(const Eigen::MatrixXd& R);

        bool update(const Eigen::VectorXd& y);
        // void update(const Eigen::VectorXd& y,double dt);
        // void update(const Eigen::VectorXd& y,Eigen::MatrixXd& A);
        bool init(const Eigen::VectorXd& x0,const Eigen::MatrixXd& P0);
        Eigen::VectorXd state();



    private:
        Eigen::MatrixXd A,H,Q,R,P;
        Eigen::MatrixXd K;
        int m, n;
        bool init_flag;
        Eigen::VectorXd x_hat;
    };


    // KalmanFilter with input u
    class KalmanFilter_Dynamic
    {
    public:
        KalmanFilter_Dynamic() = delete;
        KalmanFilter_Dynamic(
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& H,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R
        );

        void setA(const Eigen::MatrixXd& A);
        void setB(const Eigen::MatrixXd& B);
        void setQ(const Eigen::MatrixXd& Q);
        void setR(const Eigen::MatrixXd& R);
        
        bool update(const Eigen::VectorXd& y,const Eigen::VectorXd& u);
        bool update(const Eigen::VectorXd& y,const Eigen::VectorXd& u,Eigen::MatrixXd& A);
        bool init(const Eigen::VectorXd& x0,const Eigen::MatrixXd& P0);
        Eigen::VectorXd state();



    private:
        Eigen::MatrixXd A,B,H,Q,R,P;
        Eigen::MatrixXd K;
        int m, n;
        bool init_flag;
        Eigen::VectorXd x_hat;
    };

    
} // namespace ToyFilter