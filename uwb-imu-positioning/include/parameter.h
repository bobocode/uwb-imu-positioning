#ifndef _PARAMETER_H_
#define _PARAMETER_H_

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <fstream>

// extern double ACC_N, ACC_W;
// extern double GYR_N, GYR_W;

// extern Eigen::Vector3d G;

// extern double BIAS_ACC_THRESHOLD;
// extern double BIAS_GYR_THRESHOLD;


enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};



#endif