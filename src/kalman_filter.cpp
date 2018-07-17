#include "kalman_filter.h"
#include <math.h>
#include <string>
#include <fstream>
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
     */
    x_=F_*x_;
    P_=F_*P_*F_.transpose()+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
    */
    //cout << "Laser: H_ = " << H_ << endl;
    MatrixXd y;
    MatrixXd S;
	MatrixXd K;
	MatrixXd I = MatrixXd::Identity(4, 4);

	y=z - H_ * x_;

	MatrixXd Htrans = H_.transpose();
	//cout << "H transposed" << endl;

	S=H_*P_*Htrans+R_;
    K=P_*Htrans * S.inverse();

    // new state
    x_=x_+(K*y);
    P_=(I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

    MatrixXd y;
    MatrixXd S;
	MatrixXd K;
	VectorXd h_of_x(3);

	MatrixXd I = MatrixXd::Identity(4, 4);

    double h1 = sqrt(x_(0)*x_(0)+x_(1)*x_(1));
    double h2;

    //avoid division by p_x=0
    if(x_(0)<0.00001 && x_(0)>-0.00001){
        h2 = 0;
    }
    else{
        h2 = atan2(x_(1), x_(0));
    }
    double h3 = (x_(0)*x_(2)+x_(1)*x_(3))/sqrt(x_(0)*x_(0)+x_(1)*x_(1));

    h_of_x<<h1,h2,h3;

	y=z - h_of_x;
	//Correction of angle:
	if(y(1)> M_PI){
        y(1)=y(1)-2*M_PI;
	}
	else if(y(1)< -M_PI){
        y(1)=y(1)+2*M_PI;
	}

	//cout << "Radar H_ = " << H_ << endl;
	MatrixXd Htrans = H_.transpose();
	//cout << "H transposed" << endl;
	S=H_*P_*Htrans+R_;
	//cout << "S" << S << endl;
    K=P_*Htrans * S.inverse();
    //cout << "K " << K << endl;
    // new state
    x_=x_+(K*y);
    //cout << "x " << x_ << endl;
    P_=(I-K*H_)*P_;
    //cout << "P " << P_ << endl;
}
