#ifndef RUNNING_STATS_H
#define RUNNING_STATS_H

#include <ros/ros.h>

class RunningStats
{
	public:
	RunningStats(); 
	void Clear();

	void Push(double x);

	int NumDataValues() const;

	double Mean() const;

	double Variance() const;

	double StandardDeviation() const;
	

	private:
	int m_n;
	double m_oldM, m_newM, m_oldS, m_newS;

};



#endif 
