#ifndef _UWB_POSITION_FACTOR_H_
#define _UWB_POSITION_FACTOR_H_

#include <ros/assert.h>
#include <iostream>
#include <eigen3/Eigen/Dense>

#include "utility.h"
#include <ceres/ceres.h>

class UWBPositionFactor{
public:
    UWBPositionFactor(const double uwb_x, const double uwb_y, const double uwb_z,
                        const double covariance):
        m_covariance(covariance){
        m_uwb_position << uwb_x, uwb_y, uwb_z;
    }


    template <typename T>
    bool operator()(const T* const x, const T* const y, const T* const z,
                    T* residuals_ptr) const{
        const Eigen::Matrix<T, 3, 1> uwb = m_uwb_position.cast<T>();
        const Eigen::Matrix<T, 3, 1> mobile(*x, *y, *z);
        Eigen::Matrix<T, 3, 1> diff = uwb - mobile;
        if(diff(0,0)==T(0) && diff(1,0)==T(0) && diff(2,0)==T(0)){
            diff(0,0) = T(1e-6);
        }

        residuals_ptr[0] = ( ceres::abs( diff.norm() ) ) * T(1.0/std::sqrt(m_covariance));

        // avoid zero derivate for sqrt()
        const T threshold = T(1e-6);
        if(residuals_ptr[0]<threshold){
            residuals_ptr[0] = threshold;
        }

        return true;
    }

    static ceres::CostFunction* Create(const double uwb_x, const double uwb_y, const double uwb_z,
                                     const double covariance){
        return (new ceres::AutoDiffCostFunction<UWBPositionFactor, 1, 1, 1,1>
                ( new UWBPositionFactor(uwb_x, uwb_y, uwb_z, covariance ) )
                );
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector3d m_uwb_position;
    const double m_covariance;
};

#endif