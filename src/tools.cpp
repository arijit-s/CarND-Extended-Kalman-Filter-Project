#include <iostream>
#include "tools.h"

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

    // TODO: YOUR CODE HERE

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if(estimations.size() == 0 || estimations.size() != ground_truth.size()){
		cout<<"Estimator Vector is size zero"<<endl;
		return rmse;
	}
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
		VectorXd c = estimations[i] - ground_truth[i];
		VectorXd c_square = c.array()*c.array();
		rmse = rmse + c_square;
	}

	//calculate the mean
	// ... your code here
	rmse = rmse/estimations.size();


	//calculate the squared root
	// ... your code here
	rmse = rmse.array().sqrt();

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

	//TODO: YOUR CODE HERE 

	//check division by zero
	if (px == 0 && py == 0){
        cout<<"Cannot divide by zero"<<endl;
        return Hj;
    }
	
    float pxsq_ysq = px*px + py*py;


	//compute the Jacobian matrix
    Hj <<   px/(sqrt(pxsq_ysq)),py/(sqrt(pxsq_ysq)),0,0,
            -py/(pxsq_ysq),px/(pxsq_ysq),0,0,
            (py*(vx*py-vy*px))/(pow(pxsq_ysq,1.5)),(px*(vy*px-vx*py))/(pow(pxsq_ysq,1.5)),
                                px/(sqrt(pxsq_ysq)),py/(sqrt(pxsq_ysq));
	return Hj;
}
