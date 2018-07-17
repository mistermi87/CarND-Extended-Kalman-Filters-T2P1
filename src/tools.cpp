#include <iostream>
#include "tools.h"

using namespace std;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
    if((estimations.size()==0) | (ground_truth.size()!=estimations.size())){
	        return rmse;
    }

    VectorXd squared_res(4);
    squared_res << 0,0,0,0;

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){
        // ... your code here
        VectorXd diff=estimations[i]-ground_truth[i];
        VectorXd fact=diff.array().square();
		 squared_res=squared_res+fact;
	}

	//calculate the mean
	// ... your code here
    VectorXd mean = squared_res.array()/estimations.size();
	//calculate the squared root
	// ... your code here
    rmse=mean.array().sqrt();
    //cout << "RSME = " << rmse << endl;
	//return the result
	return rmse;


}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
    float i_denom = px*px+py*py;
    if(i_denom<0.000001){
       //cout << "Jacobian: Division by zero" << endl;
       return Hj;
    }
	float s_denom = sqrt(i_denom);

    //compute the Jacobian matrix
    Hj<<px/s_denom,     py/s_denom, 0,0,
        -py/(i_denom),  px/(i_denom), 0, 0,
        py*(vx*py-vy*px)/(s_denom*s_denom*s_denom), px*(vy*px-vx*py)/(s_denom*s_denom*s_denom), px/s_denom, py/s_denom;
    //cout << "Jacobian = " << Hj << endl;
	return Hj;
}
