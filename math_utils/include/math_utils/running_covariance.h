#ifndef RUNNING_COVARIANCE_H
#define RUNNING_COVARIANCE_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <ros/ros.h>


class RunningCovariance{

private:
	int _n;
    Eigen::Vector3d _oldMean, _newMean,
                    _oldVarianceSum, _newVarianceSum,
                    _oldCovarianceSum, _newCovarianceSum;

public:

	RunningCovariance();

	Eigen::Vector3d Push(Eigen::Vector3d x);

	int NumDataValues();

	Eigen::Vector3d Mean();

	Eigen::Vector3d Variance();
};



#endif