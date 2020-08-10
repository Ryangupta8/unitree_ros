#include "math_utils/running_covariance.h"

RunningCovariance::RunningCovariance(){

	std::cout << "Created running covariance object" << std::endl;
}

Eigen::Vector3d RunningCovariance::Push(Eigen::Vector3d x)
    {
        _n++;
        if (_n == 1)
        {
            _oldMean = _newMean = x;
            _oldVarianceSum = Eigen::Vector3d(0, 0, 0);
            _oldCovarianceSum = Eigen::Vector3d(0, 0, 0);
        }
        else
        {
            //_newM = _oldM + (x - _oldM) / _n;
            _newMean = Eigen::Vector3d(
                _oldMean[0] + (x[0] - _oldMean[0]) / _n,
                _oldMean[1] + (x[1] - _oldMean[1]) / _n,
                _oldMean[2] + (x[2] - _oldMean[2]) / _n);

            //_newS = _oldS + (x - _oldM) * (x - _newM);
            _newVarianceSum = Eigen::Vector3d(
                _oldVarianceSum[0] + (x[0] - _oldMean[0]) * (x[0] - _newMean[0]),
                _oldVarianceSum[1] + (x[1] - _oldMean[1]) * (x[1] - _newMean[1]),
                _oldVarianceSum[2] + (x[2] - _oldMean[2]) * (x[2] - _newMean[2]));

            /* .X is X vs Y
             * .Y is Y vs Z
             * .Z is Z vs X
             */
            _newCovarianceSum = Eigen::Vector3d(
                _oldCovarianceSum[0] + (x[0] - _oldMean[0]) * (x[1] - _newMean[1]),
                _oldCovarianceSum[1] + (x[1] - _oldMean[1]) * (x[2] - _newMean[2]),
                _oldCovarianceSum[2] + (x[2] - _oldMean[2]) * (x[0] - _newMean[0]));

            // set up for next iteration
            _oldMean = _newMean;
            _oldVarianceSum = _newVarianceSum;
        }
        return (_newCovarianceSum / (_n - 1));
    }

int RunningCovariance::NumDataValues()
    {
        return _n;
    }

Eigen::Vector3d RunningCovariance::Mean()
    {
        return (_n > 0) ? _newMean : Eigen::Vector3d(0, 0, 0);
    }

Eigen::Vector3d RunningCovariance::Variance()
    {
        return _n <= 1 ? Eigen::Vector3d(0, 0, 0) : _newVarianceSum/(_n - 1);
    }


